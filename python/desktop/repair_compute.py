# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 为点云修复提供 OpenCL 计算会话，以及界面可见的设备和失败状态；GPU 缓冲区由会话持有，必要时可走既有 CPU 路径。
"""Optional OpenCL point-cloud kernels; CPU is always available.

Uses the system ICD through ctypes (no CUDA or extra Python packages). Only
GPU devices with double precision are eligible. All work is batched; a failed
GPU operation is discarded before its CPU equivalent is run by the caller.
"""
from contextlib import ExitStack
import ctypes as C
import math
import threading

import numpy as np


MIN_GPU_POINTS = 32768
BATCH = 262144
_LOCK = threading.RLock()
_ENGINE = None
_FAILURE = ''


# 为调用层标识GPU错误这一分支；调用方可单独捕获以区分正常结果与中止。
class GPUError(RuntimeError):
    pass


_SOURCE = r'''
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#pragma OPENCL FP_CONTRACT OFF
kernel void transform(global const float *q, global const double *m,
                      global float *out, uint n) {
    uint i=get_global_id(0); if(i>=n) return;
    double x=q[3*i], y=q[3*i+1], z=q[3*i+2];
    for(int k=0;k<3;k++)
        out[3*i+k]=(float)(x*m[4*k]+y*m[4*k+1]+z*m[4*k+2]+m[4*k+3]);
}
kernel void project(global const float *q, global const double *p,
                    global double *uv, uint n) {
    uint i=get_global_id(0); if(i>=n) return;
    float x=q[3*i], y=q[3*i+1], z=q[3*i+2];
    // Match NumPy's float32 projection, followed by float64 screen origin.
    float u=y*(float)p[0]-x*(float)p[1];
    float v=x*(float)p[0]+y*(float)p[1];
    uv[2*i]=(double)(u*(float)p[4])+p[5];
    uv[2*i+1]=(double)((v*(float)p[3]-z*(float)p[2])*(float)p[4])+p[6];
}
kernel void select_points(global const float *q, global const double *p,
                          global const double *g, global uchar *hit,
                          uint n, uint kind, uint count) {
    uint i=get_global_id(0); if(i>=n) return;
    float x=q[3*i]-(float)p[7], y=q[3*i+1]-(float)p[8], z=q[3*i+2]-(float)p[9];
    float u=y*(float)p[0]-x*(float)p[1];
    float v=x*(float)p[0]+y*(float)p[1];
    double sx=(double)(u*(float)p[4])+p[5];
    double sy=(double)((v*(float)p[3]-z*(float)p[2])*(float)p[4])+p[6];
    uchar h=0;
    if(kind==0) h=sx>=g[0] && sx<=g[2] && sy>=g[1] && sy<=g[3];
    else for(uint j=0;j<count;j++) {
        uint k=(j+1)%count;
        double ax=g[2*j], ay=g[2*j+1], bx=g[2*k], by=g[2*k+1];
        if(ay!=by && ((ay>sy)!=(by>sy)) && sx<(bx-ax)*(sy-ay)/(by-ay)+ax) h^=1;
    }
    hit[i]=h;
}
kernel void radius_count(global const float *q, global const ulong *keys,
                         global const ulong *starts, global const double *params,
                         global uchar *out, uint base, uint n, uint cells,
                         uint required) {
    uint row=get_global_id(0); if(row>=n) return;
    uint i=base+row;
    double x=q[3*i], y=q[3*i+1], z=q[3*i+2], width=params[3];
    long cx=(long)floor((x-params[0])/width);
    long cy=(long)floor((y-params[1])/width);
    long cz=(long)floor((z-params[2])/width);
    uint found=0, visited=0, uncertain=0;
    // Own cell first: dense scans typically finish after a few candidates.
    for(int cell=0;cell<27;cell++) {
        int raw=cell==0 ? 13 : (cell<=13 ? cell-1 : cell);
        long a=cx+(raw%3)-1, b=cy+((raw/3)%3)-1, c=cz+(raw/9)-1;
        if(a<0||b<0||c<0||a>=2097152||b>=2097152||c>=2097152) continue;
        ulong key=(ulong)a|((ulong)b<<21)|((ulong)c<<42);
        uint lo=0, hi=cells;
        while(lo<hi) {uint m=lo+(hi-lo)/2; if(keys[m]<key) lo=m+1; else hi=m;}
        if(lo==cells||keys[lo]!=key) continue;
        for(ulong j=starts[lo];j<starts[lo+1];j++) {
            if(j==i) continue;
            // Unresolved rows go through an exact global CPU KDTree query.
            if(++visited>4096) {out[row]=2;return;}
            double dx=x-(double)q[3*j],dy=y-(double)q[3*j+1],dz=z-(double)q[3*j+2];
            double d2=dx*dx+dy*dy+dz*dz;
            // Squared thresholds and CPU sqrt can round differently exactly
            // at r. Defer ambiguous rows to the original KDTree predicate.
            if(fabs(d2-params[4])<=7.105427357601002e-15*fmax(d2,params[4])) uncertain=1;
            else if(d2<params[4] && ++found>=required) {out[row]=1;return;}
        }
    }
    out[row]=uncertain ? 2 : 0;
}
'''


# 集中保存计算引擎、 handle 等缓冲区所需状态；调用端通过 read 访问它。
class _Buffer:
    # 为缓冲区建立初始状态；输入为 engine、array、nbytes，更新实例的 engine、handle，非法输入或处理失败时抛出异常。
    def __init__(self, engine, array=None, nbytes=None):
        self.engine = engine
        err = C.c_int()
        size = int(array.nbytes if array is not None else nbytes)
        self.handle = engine.cl.clCreateBuffer(engine.context, 1, max(1, size), None, C.byref(err))
        engine.check(err.value, '分配GPU缓冲区')
        try:
            if array is not None:
                array = np.ascontiguousarray(array)
                engine.check(engine.cl.clEnqueueWriteBuffer(engine.queue, self.handle, 1, 0, size,
                             C.c_void_p(array.ctypes.data), 0, None, None), '上传GPU数据')
        except BaseException:
            self.__exit__(None, None, None)
            raise

    # 进入上下文时交还当前对象；返回 self。
    def __enter__(self): return self

    # 在退出上下文或回收对象时释放其持有的资源；更新实例的 handle。
    def __exit__(self, *args):
        if self.handle:
            self.engine.cl.clReleaseMemObject(self.handle)
            self.handle = None

    # 读取当前缓冲区；输入为 out，调用 check、clEnqueueReadBuffer 处理。
    def read(self, out):
        self.engine.check(self.engine.cl.clEnqueueReadBuffer(self.engine.queue, self.handle, 1, 0,
                          out.nbytes, C.c_void_p(out.ctypes.data), 0, None, None), '读回GPU结果')


# 集中保存 context 、 kernels 等 OpenCL 所需状态；调用端通过 check、info、run 访问它。
class _OpenCL:
    # 为 OpenCL 建立初始状态；输入为 source，更新实例的 context、kernels、cl，非法输入或处理失败时抛出异常。
    def __init__(self, source=None):
        self.context = self.queue = self.program = None
        self.kernels = {}
        try:
            self.cl = C.WinDLL('OpenCL.dll')
        except (OSError, AttributeError) as exc:
            raise GPUError('系统未提供OpenCL运行时') from exc
        p, u, z, i = C.c_void_p, C.c_uint, C.c_size_t, C.c_int
        signatures = {
            'clGetPlatformIDs': (i, [u, C.POINTER(p), C.POINTER(u)]),
            'clGetDeviceIDs': (i, [p, C.c_ulonglong, u, C.POINTER(p), C.POINTER(u)]),
            'clGetDeviceInfo': (i, [p, u, z, p, C.POINTER(z)]),
            'clCreateContext': (p, [p, u, C.POINTER(p), p, p, C.POINTER(i)]),
            'clCreateCommandQueue': (p, [p, p, C.c_ulonglong, C.POINTER(i)]),
            'clCreateProgramWithSource': (p, [p, u, C.POINTER(C.c_char_p), C.POINTER(z), C.POINTER(i)]),
            'clBuildProgram': (i, [p, u, C.POINTER(p), C.c_char_p, p, p]),
            'clGetProgramBuildInfo': (i, [p, p, u, z, p, C.POINTER(z)]),
            'clCreateKernel': (p, [p, C.c_char_p, C.POINTER(i)]),
            'clCreateBuffer': (p, [p, C.c_ulonglong, z, p, C.POINTER(i)]),
            'clEnqueueWriteBuffer': (i, [p, p, u, z, z, p, u, p, p]),
            'clEnqueueReadBuffer': (i, [p, p, u, z, z, p, u, p, p]),
            'clSetKernelArg': (i, [p, u, z, p]),
            'clEnqueueNDRangeKernel': (i, [p, p, u, p, C.POINTER(z), p, u, p, p]),
        }
        for name in ('clReleaseMemObject', 'clReleaseKernel', 'clReleaseProgram',
                     'clReleaseCommandQueue', 'clReleaseContext'):
            signatures[name] = (i, [p])
        try:
            for name, (result, args) in signatures.items():
                fn = getattr(self.cl, name); fn.restype = result; fn.argtypes = args
            count = u()
            self.check(self.cl.clGetPlatformIDs(0, None, C.byref(count)), '枚举OpenCL平台')
            platforms = (p * count.value)()
            self.check(self.cl.clGetPlatformIDs(count, platforms, None), '枚举OpenCL平台')
            devices = []
            for platform in platforms:
                nd = u()
                if self.cl.clGetDeviceIDs(platform, 4, 0, None, C.byref(nd)) != 0: continue
                ds = (p * nd.value)()
                self.check(self.cl.clGetDeviceIDs(platform, 4, nd, ds, None), '枚举GPU')
                devices.extend(ds)
            suitable = [d for d in devices if b'cl_khr_fp64' in self.info(d, 0x1030).split()]
            if not suitable: raise GPUError('没有支持双精度计算的OpenCL GPU')
            self.device = p(suitable[0])
            self.name = self.info(self.device, 0x102B).rstrip(b'\0').decode(errors='replace')
            self.max_alloc = int.from_bytes(self.info(self.device, 0x1010), 'little')
            self.global_mem = int.from_bytes(self.info(self.device, 0x101F), 'little')
            err = i()
            self.context = self.cl.clCreateContext(None, 1, C.byref(self.device), None, None, C.byref(err))
            self.check(err.value, '创建GPU上下文')
            self.queue = self.cl.clCreateCommandQueue(self.context, self.device, 0, C.byref(err))
            self.check(err.value, '创建GPU队列')
            source = C.c_char_p((_SOURCE if source is None else source).encode())
            self.program = self.cl.clCreateProgramWithSource(self.context, 1, C.byref(source), None, C.byref(err))
            self.check(err.value, '创建GPU程序')
            code = self.cl.clBuildProgram(self.program, 1, C.byref(self.device), b'-cl-std=CL1.2', None, None)
            if code:
                length = z()
                self.cl.clGetProgramBuildInfo(self.program, self.device, 0x1183, 0, None, C.byref(length))
                log = C.create_string_buffer(length.value)
                self.cl.clGetProgramBuildInfo(self.program, self.device, 0x1183, length, log, None)
                raise GPUError('编译GPU内核失败：' + log.value.decode(errors='replace')[:1000])
        except BaseException:
            self.close()
            raise

    # 检查当前 OpenCL ；输入为 code、action，非法输入或处理失败时抛出异常，调用 GPUError 处理。
    @staticmethod
    def check(code, action):
        if code: raise GPUError(f'{action}失败（OpenCL {code}）')

    # 提供 info 信息；输入为 device、key，返回 buf.raw。
    def info(self, device, key):
        length = C.c_size_t()
        self.check(self.cl.clGetDeviceInfo(device, key, 0, None, C.byref(length)), '查询GPU')
        buf = C.create_string_buffer(length.value)
        self.check(self.cl.clGetDeviceInfo(device, key, length, buf, None), '查询GPU')
        return buf.raw

    # 根据请求执行实际操作；输入为 name、count，调用 c_int、clCreateKernel 处理。
    def run(self, name, count, *args):
        if name not in self.kernels:
            err = C.c_int()
            self.kernels[name] = self.cl.clCreateKernel(self.program, name.encode(), C.byref(err))
            self.check(err.value, '创建GPU内核')
        kernel = self.kernels[name]
        for j, arg in enumerate(args):
            value = C.c_void_p(arg.handle) if isinstance(arg, _Buffer) else C.c_uint(arg)
            self.check(self.cl.clSetKernelArg(kernel, j, C.sizeof(value), C.byref(value)), '设置GPU参数')
        size = C.c_size_t(count)
        self.check(self.cl.clEnqueueNDRangeKernel(self.queue, kernel, 1, None, C.byref(size),
                                                None, 0, None, None), '运行GPU内核')

    # 关闭当前 OpenCL ；调用 values、clReleaseKernel 处理。
    def close(self):
        for kernel in self.kernels.values():
            if kernel: self.cl.clReleaseKernel(kernel)
        self.kernels.clear()
        for attribute, release in [('program', 'clReleaseProgram'), ('queue', 'clReleaseCommandQueue'),
                                   ('context', 'clReleaseContext')]:
            value = getattr(self, attribute, None)
            if value: getattr(self.cl, release)(value); setattr(self, attribute, None)


# 提供计算引擎信息；返回 _ENGINE，非法输入或处理失败时抛出异常。
def _engine():
    global _ENGINE, _FAILURE
    if _FAILURE: raise GPUError(_FAILURE)
    if _ENGINE is None:
        try: _ENGINE = _OpenCL()
        except GPUError as exc:
            _FAILURE = str(exc)
            raise
    return _ENGINE


# 提供 device_info 信息；返回包含 available、reason 等字段的字典。
def device_info():
    with _LOCK:
        try:
            gpu = _engine()
            return {'available': True, 'name': gpu.name, 'api': 'OpenCL',
                    'max_allocation': gpu.max_alloc, 'global_memory': gpu.global_mem}
        except GPUError as exc:
            return {'available': False, 'reason': str(exc)}


# 集中保存模式、步骤、设备、 fallback_reason 等计算会话所需状态；调用端通过 summary、transform、select 访问它。
class ComputeSession:
    """Per-operation provenance. No GUI, network, or persistent data writes."""
    # 为计算会话建立初始状态；输入为 mode，更新实例的 mode、steps、device，非法输入或处理失败时抛出异常。
    def __init__(self, mode='auto'):
        if mode not in ('auto', 'cpu'): raise ValueError('计算后端须为auto或cpu')
        self.mode = mode
        self.steps = []
        self.device = ''
        self.fallback_reason = ''

    # 提供 eligible 信息；输入为 n，用布尔值反馈判断结果。
    def _eligible(self, n):
        if self.mode == 'cpu' or self.fallback_reason: return False
        return n >= MIN_GPU_POINTS

    # 处理 failed 相关状态；输入为 exc，更新实例的 fallback_reason。
    def _failed(self, exc):
        self.fallback_reason = str(exc)

    # 提供 summary 信息；返回包含 backend、mode、device 等字段的字典。
    def summary(self):
        gpu_steps = list(dict.fromkeys(self.steps))
        backend = 'gpu+cpu' if gpu_steps else 'cpu'
        text = ('GPU · ' + self.device + '（' + '、'.join(gpu_steps) + '）；其余步骤CPU') if gpu_steps else 'CPU'
        if self.fallback_reason: text += ' · GPU回退：' + self.fallback_reason
        elif not gpu_steps:
            text += ' · ' + ('用户选择' if self.mode == 'cpu' else '当前步骤使用CPU／小数据量')
        return {'backend': backend, 'mode': self.mode, 'device': self.device,
                'steps': gpu_steps, 'fallback_reason': self.fallback_reason, 'summary': text}

    # 提供变换信息；输入为 q、output、rotation，更新实例的 device，用布尔值反馈判断结果。
    def transform(self, q, output, rotation, shift, check, report):
        if not self._eligible(len(q)): return False
        try:
            with _LOCK:
                gpu = _engine()
                m = np.column_stack((rotation, shift)).astype(np.float64)
                with _Buffer(gpu, m) as matrix:
                    for start in range(0, len(q), BATCH):
                        check(); batch = np.ascontiguousarray(q[start:start+BATCH])
                        out = np.empty_like(batch)
                        with _Buffer(gpu, batch) as src, _Buffer(gpu, nbytes=out.nbytes) as dst:
                            gpu.run('transform', len(batch), src, matrix, dst, len(batch)); dst.read(out)
                        output[start:start+len(batch)] = out
                        report(.9 * (start+len(batch))/len(q))
                check(); self.device = gpu.name; self.steps.append('坐标变换')
                return True
        except (GPUError, MemoryError) as exc:
            self._failed(exc); return False

    # 选择当前计算会话；输入为 q、found、yaw，更新实例的 device，把输入统一为数组，用布尔值反馈判断结果。
    def select(self, q, found, yaw, pitch, scale, center, shape, geometry, check, origin=None):
        if shape not in ('rectangle', 'lasso') or not self._eligible(len(q)): return False
        try:
            with _LOCK:
                gpu = _engine()
                params = np.array([math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch),
                                   scale, *center, *(origin if origin is not None else (0., 0., 0.))], np.float64)
                if shape == 'rectangle':
                    g = np.asarray(geometry, np.float64).reshape(2, 2)
                    g = np.r_[g.min(0), g.max(0)]
                else: g = np.asarray(geometry, np.float64)
                with _Buffer(gpu, params) as p, _Buffer(gpu, g) as path:
                    for start in range(0, len(q), BATCH):
                        check(); batch = np.ascontiguousarray(q[start:start+BATCH])
                        out = np.empty(len(batch), np.uint8)
                        with _Buffer(gpu, batch) as src, _Buffer(gpu, nbytes=out.nbytes) as dst:
                            gpu.run('select_points', len(batch), src, p, path, dst, len(batch),
                                    int(shape == 'lasso'), len(g)); dst.read(out)
                        found[start:start+len(batch)] = out
                check(); self.device = gpu.name; self.steps.append('全量' + ('框选' if shape == 'rectangle' else '套索'))
                return True
        except (GPUError, MemoryError) as exc:
            self._failed(exc); return False

    # 提供半径信息；输入为 q、radius、required，更新实例的 device，构建 KD 树进行空间查询，返回 result，非法输入或处理失败时抛出异常。
    def radius(self, q, radius, required, allocate, check, report):
        """Exact uniform-grid radius counts. Sorting stays on CPU; no sampling."""
        if not self._eligible(len(q)): return None
        try:
            with _LOCK:
                gpu = _engine()
                from .repair_resources import available_memory
                n = len(q)
                if n >= 2**32 or n * 100 > available_memory() * .45:
                    raise GPUError('GPU空间索引的工作内存预算不足')
                if n * 12 > gpu.max_alloc or n * 32 > gpu.global_mem * .4:
                    raise GPUError('完整空间索引超过GPU缓冲区预算')
                origin = np.full(3, np.inf); maximum = np.full(3, -np.inf)
                for start in range(0, n, BATCH):
                    check(); batch = q[start:start+BATCH]
                    origin = np.minimum(origin, batch.min(0)); maximum = np.maximum(maximum, batch.max(0))
                width = radius * (1 + 1e-12)
                if not np.isfinite(width) or np.max((maximum-origin)/width) >= 2**21-1:
                    raise GPUError('坐标跨度超过GPU空间索引范围')
                keys = allocate(n, np.uint64)
                for start in range(0, n, BATCH):
                    check(); xyz = np.floor((q[start:start+BATCH].astype(np.float64)-origin)/width).astype(np.uint64)
                    keys[start:start+len(xyz)] = xyz[:, 0] | (xyz[:, 1] << np.uint64(21)) | (xyz[:, 2] << np.uint64(42))
                check(); report(.08)
                order = np.argsort(keys, kind='stable')
                check(); report(.15)
                ordered_keys = keys[order]
                starts = np.r_[0, np.flatnonzero(ordered_keys[1:] != ordered_keys[:-1])+1, n].astype(np.uint64)
                unique = ordered_keys[starts[:-1].astype(np.intp)]
                del ordered_keys, keys
                sorted_q = allocate(q.shape, np.float32)
                for start in range(0, n, BATCH):
                    check(); sorted_q[start:start+BATCH] = q[order[start:start+BATCH]]
                result = allocate(n, bool)
                unknown = []
                params = np.array([*origin, width, radius*radius], np.float64)
                with ExitStack() as stack:
                    src, cells, offsets, p = [stack.enter_context(_Buffer(gpu, a))
                                             for a in (sorted_q, unique, starts, params)]
                    for start in range(0, n, 16384):
                        check(); count = min(16384, n-start); out = np.empty(count, np.uint8)
                        with _Buffer(gpu, nbytes=count) as dst:
                            gpu.run('radius_count', count, src, cells, offsets, p, dst,
                                    start, count, len(unique), required); dst.read(out)
                        rows = order[start:start+count]
                        result[rows] = out == 1
                        if np.any(out == 2): unknown.append(rows[out == 2].copy())
                        report(.2+.65*(start+count)/n)
                if unknown:
                    from scipy.spatial import cKDTree
                    tree = cKDTree(q)
                    for rows in unknown:
                        check()
                        ds = tree.query(q[rows], k=[required+1],
                                        distance_upper_bound=np.nextafter(radius, np.inf))[0][:, 0]
                        result[rows] = ds <= radius
                check(); self.device = gpu.name; self.steps.append('半径邻点计数')
                return result
        except (GPUError, MemoryError) as exc:
            self._failed(exc); return None
