# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 被动 UDP 收包及预览缓存。主机单调时间衡量断流和窗口，设备时间用于与 IMU 对齐；二者不能互换。
"""T100独立UDP接收：36字节Livox SDK2式头，仅接收，不发送控制命令。

布局参考external/livox-sdk2/livox_lidar_def.h，T100兼容性仍需实测。
CRC32覆盖范围尚未由本机协议资料确认，不把结构校验标成CRC通过。
"""
from collections import deque
from dataclasses import dataclass
import math
import socket
import select
import struct
import threading
import time
import numpy as np

HIGH=np.dtype([('x','<i4'),('y','<i4'),('z','<i4'),('reflectivity','u1'),('tag','u1')])
LOW=np.dtype([('x','<i2'),('y','<i2'),('z','<i2'),('reflectivity','u1'),('tag','u1')])
SPHERICAL=np.dtype([('depth','<u4'),('theta','<u2'),('phi','<u2'),('reflectivity','u1'),('tag','u1')])


# 一个数据报的协议字段与有效数组。count 保留源样点数，point_indices 保留过滤前索引；timestamp 为设备纳秒。
@dataclass
class Packet:
    points: np.ndarray
    reflectivity: np.ndarray
    tags: np.ndarray
    count: int
    sequence: int
    frame: int
    data_type: int
    timestamp: int
    time_type: int
    interval: int
    crc32: int
    imu_values: tuple | None = None
    # 零回波剔除前的采样序号，用于按设备时间给每个点去畸变。
    point_indices: np.ndarray | None = None


# 按 36 字节头核对长度/类型；输出米制 XYZ、反射率和标签或六轴 IMU，未知格式与非有限 IMU 抛 ValueError。
def decode_packet(data):
    if len(data)<36:raise ValueError('不足36字节点云头')
    version,length,interval,count,sequence,frame,kind,clock=struct.unpack_from('<BHHHHBBB',data)
    if version not in (0,1):raise ValueError(f'未知点云头版本 {version}')
    if length!=len(data):raise ValueError(f'长度不符：头{length} / 实收{len(data)}')
    formats={1:HIGH,2:LOW,3:SPHERICAL,0:np.dtype(('u1',24))}
    if kind not in formats:raise ValueError(f'暂不支持数据类型 0x{kind:02x}')
    dtype=formats[kind]
    if length!=36+count*dtype.itemsize:raise ValueError('点数与记录长度不符')
    raw=np.frombuffer(data,dtype=dtype,count=count,offset=36)
    if kind==0:
        if count!=1:raise ValueError('IMU包需为一组6轴数据')
        imu_values=struct.unpack_from('<6f',data,36)
        if not all(math.isfinite(v) for v in imu_values):raise ValueError('IMU包含非有限值')
        points=np.empty((0,3),dtype=np.float32);strength=tags=np.empty(0,dtype=np.uint8)
    elif kind in (1,2):
        points=np.column_stack([raw[k] for k in ('x','y','z')]).astype(np.float32)
        points*=.001 if kind==1 else .01
        valid=np.any(points!=0,axis=1)
        points=points[valid];strength=raw['reflectivity'][valid].copy();tags=raw['tag'][valid].copy()
    else:
        # SDK2球坐标约定：theta为相对+Z的天顶角，phi为XY平面方位，0.01度。
        theta=raw['theta'].astype(np.float64)*(math.pi/18000)
        phi=raw['phi'].astype(np.float64)*(math.pi/18000)
        r=raw['depth'].astype(np.float64)*.001
        if np.any(raw['theta']>18000) or np.any(raw['phi']>36000):raise ValueError('球坐标角度越界')
        points=np.column_stack((r*np.sin(theta)*np.cos(phi),r*np.sin(theta)*np.sin(phi),r*np.cos(theta))).astype(np.float32)
        valid=r>0;points=points[valid];strength=raw['reflectivity'][valid].copy();tags=raw['tag'][valid].copy()
    return Packet(points,strength,tags,count,sequence,frame,kind,
                  struct.unpack_from('<Q',data,28)[0],clock,interval,struct.unpack_from('<I',data,24)[0],
                  imu_values if kind==0 else None,
                  np.flatnonzero(valid).astype(np.int32) if kind!=0 else None)


# 把同点数的高精度直角包按步长组成矩阵批量转换；结果与输入顺序一致，未走批量路径的条目为 None。
def decode_cloud_batch(payloads):
    """对同长度的高精度直角坐标包一次展开 XYZ；非匹配格式交还原解析器。"""
    result=[None]*len(payloads)
    groups={}
    for i,b in enumerate(payloads):
        if len(b)<36 or b[10]!=1 or b[0] not in (0,1):continue
        n=int.from_bytes(b[5:7],'little')
        if n and len(b)==36+n*HIGH.itemsize and int.from_bytes(b[1:3],'little')==len(b):
            groups.setdefault(n,[]).append(i)
    for n,ids in groups.items():
        buf=b''.join(payloads[i] for i in ids)
        values=np.ndarray((len(ids),n),dtype=HIGH,buffer=buf,offset=36,strides=(36+n*14,14))
        xyz=np.stack([values[k] for k in ('x','y','z')],axis=2).astype(np.float32)
        xyz*=.001
        valid=np.any(xyz!=0,axis=2)
        for j,i in enumerate(ids):
            b=payloads[i];_,_,interval,_,seq,frame,kind,clock=struct.unpack_from('<BHHHHBBB',b)
            mask=valid[j]
            result[i]=Packet(xyz[j,mask],values['reflectivity'][j,mask],values['tag'][j,mask],n,seq,frame,kind,
                struct.unpack_from('<Q',b,28)[0],clock,interval,struct.unpack_from('<I',b,24)[0],None,
                np.flatnonzero(mask).astype(np.int32))
    return result


# 持锁维护最近包、预览及收包计数；设备原包建图回调与本预览集合分别管理。
class StreamState:
    # 建立有界点云窗和陀螺窗，初始化不同流的最后到达时刻与计数。
    def __init__(self,max_points=300000):
        self.lock=threading.Lock();self.max_points=max_points;self.frames=deque();self.retained=0
        self.packets=0;self.cloud_packets=0;self.points=0;self.invalid=0;self.filtered=0;self.imu=0;self.evicted=0
        self.last_at=0.;self.last_cloud_at=0.;self.last_imu_at=0.
        self.last_packet=None;self.last_imu=None;self.error='';self.peer='';self.events=deque();self.gyros=deque(maxlen=400)

    # 解包后先把原字节及解析结果交回调；在锁内记主机到达时刻，IMU 单独记，旧预览按年龄或点数淘汰。解析错误返回 False。
    def ingest(self,data,peer,now=None,decoded_callback=None,decoded=None):
        now=time.monotonic() if now is None else now
        try:p=decode_packet(data) if decoded is None else decoded
        except ValueError as exc:
            with self.lock:self.invalid+=1;self.error=str(exc)
            return False
        if decoded_callback is not None:decoded_callback(data,p)
        with self.lock:
            self.packets+=1;self.last_at=now;self.peer=str(peer);self.error=''
            if p.data_type==0:
                self.gyros.append((p.timestamp*1e-9,p.time_type,*p.imu_values[:3]))
                self.imu+=1;self.last_imu_at=now;self.last_imu=p.imu_values;return True
            self.last_cloud_at=now;self.last_packet=p;self.cloud_packets+=1
            self.events.append((now,len(p.points)))
            while self.events and (now-self.events[0][0]>=1 or len(self.events)>20000):self.events.popleft()
            self.points+=len(p.points)
            if self.frames and now-self.frames[-1][0]>2.5:self.frames.clear();self.retained=0
            if len(p.points):self.frames.append((now,p.points,p.reflectivity,p.tags,p.timestamp*1e-9+p.interval*.5e-7,p.time_type));self.retained+=len(p.points)
            while self.frames and (now-self.frames[0][0]>10 or self.retained>self.max_points):
                old=self.frames.popleft();xyz=old[1];self.retained-=len(xyz);self.evicted+=len(xyz)
        return True

    # 按主机时间提取最近点，锁外合并；motion 路径提供包中心设备时间及陀螺供预览补偿，不返回反射率或标签。
    def snapshot(self,seconds=1,*,motion=False,preview_limit=None):
        now=time.monotonic()
        with self.lock:
            age=now-self.last_cloud_at if self.last_cloud_at else None
            # 复制数组引用后在锁外合并，避免绘图长期阻塞UDP接收。
            frames=[f for f in self.frames if now-f[0]<=seconds] if age is not None and age<2.5 else []
            recent=[e for e in self.events if now-e[0]<1]
            meta=dict(packets=self.packets,received_points=self.points,invalid=self.invalid,filtered=self.filtered,
                      imu=self.imu,evicted=self.evicted,udp_age=age,peer=self.peer,error=self.error,
                      imu_age=now-self.last_imu_at if self.last_imu_at else None,imu_values=self.last_imu,
                      pps=sum(e[1] for e in recent),packet_rate=len(recent),last=self.last_packet,cloud_packets=self.cloud_packets)
            if motion:
                gyros=tuple(self.gyros)
        if motion:
            meta['window_seconds']=seconds
            # 高点率时沿整个时间窗口均匀选包，保留每包各束点及其 IMU 时间对应关系。
            if preview_limit and sum(len(f[1]) for f in frames)>preview_limit:
                n=max(1,preview_limit//max(len(f[1]) for f in frames))
                indices=np.linspace(0,len(frames)-1,n,dtype=int) if n>1 else [len(frames)-1]
                frames=[frames[i] for i in indices]
            meta['preview_motion']=(tuple((f[4],len(f[1]),f[5]) for f in frames),gyros)
        xyz=np.concatenate([f[1] for f in frames]) if frames else np.empty((0,3),np.float32)
        if motion:return xyz,None,None,meta  # 此路径只供建图预览，不复制另外两层。
        refl=np.concatenate([f[2] for f in frames]) if frames else np.empty(0,np.uint8)
        tags=np.concatenate([f[3] for f in frames]) if frames else np.empty(0,np.uint8)
        return xyz,refl,tags,meta

    # 清掉当前窗和最近状态；累计收包计数继续保留。
    def clear(self):
        with self.lock:
            self.frames.clear();self.gyros.clear();self.events.clear();self.retained=0
            self.last_packet=None;self.last_imu=None;self.last_at=self.last_cloud_at=self.last_imu_at=0.


# 拥有一个被动 UDP 套接字和后台收包线程，向 StreamState 交付，不承担建图运算。
class Receiver:
    # 保存绑定地址、端口及源 IP 过滤项；回调由接收线程调用，应快速返回。
    def __init__(self,state,bind_ip='0.0.0.0',port=60001,source_ip='192.168.1.10',raw_callback=None,decoded_callback=None):
        self.state=state;self.bind_ip=bind_ip;self.port=port;self.source_ip=source_ip
        self.stop_event=threading.Event();self.sock=None;self.thread=None;self.running=False
        # 建图回调只允许有界队列入队，禁止在接收线程做匹配或磁盘导出。
        self.raw_callback=raw_callback
        self.decoded_callback=decoded_callback

    # 创建独占 UDP 监听及接收线程；绑定失败时关套接字并把异常返给调用方。
    def start(self):
        # 独占监听，避免另一个程序悄悄分走UDP包。
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            if hasattr(socket,'SO_EXCLUSIVEADDRUSE'):sock.setsockopt(socket.SOL_SOCKET,socket.SO_EXCLUSIVEADDRUSE,1)
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024)
            sock.bind((self.bind_ip,self.port));sock.settimeout(.2)
        except Exception:sock.close();raise
        self.port=sock.getsockname()[1];self.sock=sock;self.running=True
        self.thread=threading.Thread(target=self._run,name='t100-passive-udp',daemon=True);self.thread.start()

    # 按短等待和批量上限收包，过滤非目标源；原包回调仍拿到原字节。预期关闭不计为接收故障。
    def _run(self):
        try:
            while not self.stop_event.is_set():
                try:data,peer=self.sock.recvfrom(65535)
                except socket.timeout:continue
                except OSError:
                    if self.stop_event.is_set():break
                    raise
                batch=[(data,peer,time.monotonic())]
                # 最多攒3ms/16包作批量XYZ展开，降低小NumPy数组与Python调用开销。
                # 不跨端口等IMU，也不等待满批；设备逐点时间戳保持原样。
                deadline=time.monotonic()+.003
                if len(data)>10 and data[10]==1:
                    while len(batch)<16 and not self.stop_event.is_set():
                        left=deadline-time.monotonic()
                        if left<=0 or not select.select([self.sock],[],[],left)[0]:break
                        try:raw,address=self.sock.recvfrom(65535)
                        except socket.timeout:break
                        batch.append((raw,address,time.monotonic()))
                decoded=decode_cloud_batch([raw for raw,_,_ in batch]) if len(batch)>1 else [None]
                for (raw,address,arrived),packet in zip(batch,decoded):
                    if self.source_ip and address[0]!=self.source_ip:
                        with self.state.lock:self.state.filtered+=1
                        continue
                    self.state.ingest(raw,address[0],arrived,decoded_callback=self.decoded_callback,decoded=packet)
                    if self.raw_callback is not None:self.raw_callback(raw)
        except (OSError,ValueError) as exc:
            # 关闭时select/recv可能报无效套接字；主动断开不记作数据故障。
            if not self.stop_event.is_set():
                with self.state.lock:self.state.error=f'UDP接收失败：{exc}'
        finally:self.running=False

    # 设置停止标志、关闭套接字以打断等待，再短时等待线程退出。
    def close(self):
        self.stop_event.set()
        if self.sock:self.sock.close()
        if self.thread:self.thread.join(timeout=1)
