# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 提供地图剖面测距与轨迹首尾重访检查。地图分块按空间范围读取，端点对齐用于诊断，结果经后台线程送回界面。
"""Read-only offline T100 geometry and its independent Tk tools."""
from collections import deque
from dataclasses import dataclass, replace
from itertools import count
import json
from pathlib import Path
import queue
import sqlite3
import struct
import threading

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from radar_client import t100_stream


_ROW = np.dtype([('key', '<i8'), ('xyz', '<f4', (3,)), ('count', 'u1')])
_BIAS = 1 << 20
_MASK = (1 << 21) - 1
_AXES = {'x': (1, 2, 0), 'y': (0, 2, 1), 'z': (0, 1, 2)}
_MAGIC = b'T100REC1\n'


# 集中保存点集、数量、 resolution 、边界等剖面所需状态。
@dataclass(frozen=True)
class Section:
    points: np.ndarray
    count: int
    resolution: float
    bounds: np.ndarray  # exact point limits in projected U,V,normal coordinates


# 处理取消标记相关状态；输入为 cancel，检查协作取消标记。
def _cancelled(cancel):
    if cancel is not None and cancel.is_set():
        raise InterruptedError('操作已取消')


# 提供SQLite路径信息；输入为 archive，调用 Path 处理。
def _sqlite_path(archive):
    path = Path(archive)
    return path if path.name == 'blocks.sqlite' else path / 'blocks.sqlite'


# 打开 db ；输入为 archive，非法输入或处理失败时抛出异常，调用 resolve、_sqlite_path 处理。
def _open_db(archive):
    path = _sqlite_path(archive).resolve(strict=True)
    wal = path.with_name(path.name + '-wal')
    shm = path.with_name(path.name + '-shm')
    # A closed archive may have no sidecars. Immutable reading avoids creating
    # either file; when WAL contains data read a real SQLite snapshot via SHM.
    has_wal = wal.is_file() and wal.stat().st_size > 0
    if has_wal and not (shm.is_file() and shm.stat().st_size):
        raise ValueError('分块存档 WAL 未完成，请先在建图软件中正常结束会话')
    return sqlite3.connect(path.as_uri() + '?mode=ro' + ('' if has_wal else '&immutable=1'), uri=True)


# 读取剖面；输入为 archive、bounds、axis，非法输入或处理失败时抛出异常，调用 ValueError、full 处理。
def read_section(archive, bounds=None, axis='z', center=None, thickness=.2, limit=100000, cancel=None):
    """Read committed points in a rectangle (u0,v0,u1,v1) and optional slab.

    The projected axes are z→XY, y→XZ, x→YZ. Coordinates are metres.
    Exact clipping/counting precede deterministic uniform sampling for display.
    Call from a worker and use a stopped/flushed archive.
    """
    order = _AXES[axis]
    if limit < 1:
        raise ValueError('显示采样上限必须为正')
    lo, hi = np.full(3, -np.inf), np.full(3, np.inf)
    if bounds is not None:
        u0, v0, u1, v1 = bounds
        if not np.isfinite(bounds).all() or u0 >= u1 or v0 >= v1:
            raise ValueError('框选范围无效')
        lo[:2], hi[:2] = (u0, v0), (u1, v1)
    if center is not None:
        if not np.isfinite([center, thickness]).all() or thickness <= 0:
            raise ValueError('剖面中心或厚度无效')
        lo[2], hi[2] = center - thickness / 2, center + thickness / 2
    _cancelled(cancel)
    rng = np.random.default_rng(0)
    points, priorities = np.empty((0, 3), np.float32), np.empty(0)
    minimum, maximum, count = np.full(3, np.inf), np.full(3, -np.inf), 0
    # 提供 accept 信息；输入为 p，按轴拼接数组，非法输入或处理失败时抛出异常。
    def accept(p):
        nonlocal minimum, maximum, count, points, priorities
        _cancelled(cancel)
        if not np.isfinite(p).all():
            raise ValueError('点云包含非有限坐标')
        p = p[((p >= lo) & (p <= hi)).all(axis=1)]
        if not len(p):
            return
        minimum = np.minimum(minimum, p.min(axis=0))
        maximum = np.maximum(maximum, p.max(axis=0))
        count += len(p)
        points = np.concatenate((points, p))
        priorities = np.concatenate((priorities, rng.random(len(p))))
        if len(points) >= limit * 2:
            keep = np.argpartition(priorities, limit-1)[:limit]
            points, priorities = points[keep], priorities[keep]

    from .white_mesh_io import resolve_point_source, iter_exported_points
    source = Path(archive)
    mobile = source if source.name=='map.sqlite' else source/'map.sqlite'
    is_mobile = mobile.is_file() and not (source/'blocks.sqlite').is_file()
    if not is_mobile:source = resolve_point_source(archive)
    resolution = 0.  # Exported XYZ clouds have no declared voxel grid.
    if is_mobile:
        # 非商用独立工具版不包含完整建图集成的手机地图读取器。
        # 请在原软件中导出二进制XYZ PLY/PCD，再传给这些独立工具。
        raise ValueError('此独立工具版不读取手机map.sqlite，请先导出二进制XYZ PLY/PCD')
    elif source.is_file() and source.suffix.lower() in ('.ply', '.pcd'):
        for p in iter_exported_points(source):
            accept(p[:, order])
    else:
        db = _open_db(source)
        try:
            db.execute('BEGIN')
            settings = dict(db.execute('SELECT name,value FROM settings'))
            resolution = float(settings['resolution'])
            tile = int(settings.get('tile_cells', max(1, round(4 / resolution)))) * resolution
            for (key,) in db.execute('SELECT id FROM blocks ORDER BY id'):
                _cancelled(cancel)
                block = np.array([key >> 42, key >> 21 & _MASK, key & _MASK])[list(order)] - _BIAS
                if np.any((block + 1) * tile < lo - 1e-6) or np.any(block * tile > hi + 1e-6):
                    continue
                n, blob = db.execute('SELECT n,data FROM blocks WHERE id=?', (key,)).fetchone()
                p = np.frombuffer(blob, _ROW)['xyz'][:, order]
                if len(p) != n:
                    raise ValueError('分块地图点数与文件不一致')
                accept(p)
        finally:
            db.close()
    if len(points) > limit:
        points = points[np.argpartition(priorities, limit-1)[:limit]]
    return Section(points, count, resolution, np.array([minimum, maximum]))


# 集中保存已接受、原因、 archive_shift_m 、 archive_turn_deg 等 ReturnCheck 所需状态。
@dataclass(frozen=True)
class ReturnCheck:
    accepted: bool
    reason: str
    archive_shift_m: float
    archive_turn_deg: float
    revisit_shift_m: float | None = None
    revisit_turn_deg: float | None = None
    residual_shift_m: float | None = None
    residual_turn_deg: float | None = None
    overlap: float = 0.
    rmse_m: float | None = None
    condition: float = 0.
    timing: str = ''


# 提供 decimate 信息；输入为 points、voxel、limit，把输入统一为数组，返回对应切片或索引值 points[keep]。
def _decimate(points, voxel=.10, limit=6000):
    points = np.asarray(points, dtype=float)
    if not len(points):
        return points
    _, keep = np.unique(np.floor(points / voxel).astype(np.int64), axis=0, return_index=True)
    if len(keep) > limit:
        keep = np.random.default_rng(0).choice(keep, limit, replace=False)
    return points[keep]


# 提供 norm_deg 信息；输入为 rotation，调用 rad2deg、magnitude 处理。
def _norm_deg(rotation):
    return float(np.rad2deg(Rotation.from_matrix(rotation).magnitude()))


# 配准 endpoints ；输入为 start、end、rotation，分解实对称矩阵的特征值与特征向量。
def align_endpoints(start, end, rotation, translation, cancel=None):
    """Align last sensor frame to first, seeded by independently archived pose.

    Rotation/translation map end-frame points into start-frame coordinates.
    Output is withheld when bidirectional overlap or 6-DOF conditioning fails.
    """
    rotation, translation = np.asarray(rotation, float), np.asarray(translation, float)
    summary = (float(np.linalg.norm(translation)), _norm_deg(rotation))
    start, end = _decimate(start), _decimate(end)
    if min(len(start), len(end)) < 250:
        return ReturnCheck(False, '有效首尾点太少', *summary)
    tree = cKDTree(start)
    d, nbr = tree.query(start, k=12)
    local = start[nbr] - start[nbr].mean(axis=1, keepdims=True)
    eig, basis = np.linalg.eigh(np.einsum('nki,nkj->nij', local, local) / 12)
    normal = basis[:, :, 0]
    planar = (eig[:, 0] < .18 * eig[:, 1]) & (eig[:, 1] > 1e-5) & (d[:, -1] < .65)
    r, t = rotation.copy(), translation.copy()
    condition = 0.
    # 提供 correspond 信息；输入为 gate，调用 query、einsum 处理。
    def correspond(gate):
        q = end @ r.T + t
        distance, idx = tree.query(q)
        keep = (distance < gate) & planar[idx]
        q, p, n = q[keep], start[idx[keep]], normal[idx[keep]]
        return q, n, np.einsum('ij,ij->i', q-p, n)
    for iteration in range(27):
        _cancelled(cancel)
        gate = (.65, .36, .22)[iteration // 9]
        q, n, error = correspond(gate)
        if len(q) < max(100, .15 * len(end)):
            return ReturnCheck(False, '首尾重叠不足或缺少可匹配平面', *summary)
        weights = np.sqrt(np.minimum(1., .055 / np.maximum(np.abs(error), 1e-9)))
        reach = max(1., float(np.median(np.linalg.norm(q-t, axis=1))))
        a = np.column_stack((np.cross(q-t, n) / reach, n)) * weights[:, None]
        eigen = np.linalg.eigvalsh(a.T @ a)
        condition = float(max(0., eigen[0] / max(eigen[-1], 1e-9)))
        if condition < .001:
            return ReturnCheck(False, '场景几何退化，无法独立约束三维平移与旋转', *summary, condition=condition)
        delta = np.linalg.lstsq(a, -error * weights, rcond=None)[0]
        if np.linalg.norm(delta[3:]) > .35 or np.linalg.norm(delta[:3] / reach) > .22:
            return ReturnCheck(False, '首尾对应不稳定', *summary, condition=condition)
        r = Rotation.from_rotvec(delta[:3] / reach).as_matrix() @ r
        t += delta[3:]
    end_world = end @ r.T + t
    overlap = float(min(np.mean(tree.query(end_world)[0] < .22),
                        np.mean(cKDTree(end_world).query(start)[0] < .22)))
    q, n, error = correspond(.22)
    rmse = float(np.sqrt(np.mean(error**2))) if len(error) else None
    if overlap < .42 or rmse is None or rmse > .075 or len(q) < .20 * len(end):
        return ReturnCheck(False, '双向重叠不足或平面残差过大', *summary,
                           overlap=overlap, rmse_m=rmse, condition=condition)
    drift = float(np.linalg.norm(t-translation))
    angular_drift = _norm_deg(r @ rotation.T)
    if drift > .75 or angular_drift > 18:
        return ReturnCheck(False, '几何解偏离归档初值过大，可能发生错误对应', *summary,
                           overlap=overlap, rmse_m=rmse, condition=condition)
    return ReturnCheck(True, '通过局部重叠与几何约束检查', *summary,
                       float(np.linalg.norm(t)), _norm_deg(r), drift, angular_drift,
                       overlap, rmse, condition)


# 提供 endpoint_clouds 信息；输入为 capture、duration、startup，按轴拼接数组，返回 endpoints，非法输入或处理失败时抛出异常。
def _endpoint_clouds(capture, duration, startup, cancel, epoch=None, pose_clock=None):
    # Index only endpoint candidates; memory is independent of walk duration.
    head, tail = [], deque()
    with Path(capture).open('rb') as stream:
        if stream.read(len(_MAGIC)) != _MAGIC:
            raise ValueError('不是 T100 原始记录')
        size = Path(capture).stat().st_size
        first = previous = clock = None
        for i in count():
            if i % 512 == 0:
                _cancelled(cancel)
            header = stream.read(10)
            if not header:
                break
            if len(header) != 10:
                raise ValueError('记录包头不完整')
            n = struct.unpack_from('<H', header, 8)[0]
            offset = stream.tell()
            prefix = stream.read(36)
            if n < 36 or len(prefix) != 36 or offset + n > size:
                raise ValueError('记录包不完整')
            typ = prefix[10]
            if typ in (1, 2, 3):
                stamp = struct.unpack_from('<Q', prefix, 28)[0]
                if clock is None:
                    clock = prefix[11]
                if prefix[11] != clock or (previous is not None and stamp < previous - 100_000_000):
                    raise ValueError('记录点云设备时钟切换或回跳，无法定位首尾窗口')
                if previous is None or stamp > previous:
                    if first is None:
                        first = stamp
                    previous = stamp
                    elapsed = (stamp-first)*1e-9
                    record = (elapsed, offset, n)
                    if epoch is None:
                        if elapsed <= startup+.86:
                            head.append(record)
                        tail.append(record)
                        while tail and tail[0][0] < elapsed-.35:
                            tail.popleft()
                    else:
                        rel = stamp*1e-9-epoch
                        if -.28 <= rel <= 0.:
                            head.append(record)
                        if duration-.28 <= rel <= duration:
                            tail.append(record)
            stream.seek(offset+n)
        if previous is None:
            raise ValueError('记录没有有效点云包')
        elapsed = (previous-first)*1e-9
        if epoch is None:
            initial = elapsed - duration
            if not max(.45, startup - .65) <= initial <= startup + .85:
                raise ValueError('轨迹时长与原始记录不吻合，或会话尾部没有有效跟踪')
        else:
            initial = epoch - first*1e-9
            if (clock != pose_clock or not max(.20, startup - .85) <= initial <= startup + 1.5
                    or not -.01 <= elapsed-initial-duration <= .8):
                raise ValueError('归档首尾设备时间与配套记录不吻合')
        endpoints = []
        last = elapsed if epoch is None else initial + duration
        for candidates, finish in ((head, initial), (tail, last)):
            records = [record for record in candidates if finish-.281 <= record[0] <= finish+1e-7]
            times = [record[0] for record in records]
            if len(records) < 25 or times[-1]-times[0] < .18 or np.max(np.diff(times)) > .04:
                raise ValueError('首尾扫描窗口点云不连续')
            clouds = []
            for _, offset, length in records:
                stream.seek(offset)
                clouds.append(t100_stream.decode_packet(stream.read(length)).points)
            p = np.concatenate(clouds)
            d = np.linalg.norm(p, axis=1)
            endpoints.append(p[(d >= .5) & (d <= 15.)])
    return endpoints


# 检查重访；输入为 capture、archive、cancel，替换选定的字符串或目标项，非法输入或处理失败时抛出异常。
def check_return(capture, archive=None, cancel=None):
    """Read a paired recording and stopped archive. Never alters either."""
    capture = Path(capture)
    archive = Path(archive) if archive is not None else capture.with_suffix('.map')
    if archive.name == 'blocks.sqlite':
        archive = archive.parent
    if archive.stem.casefold() != capture.stem.casefold():
        raise ValueError('录包和分块存档须使用相同基本文件名')
    state = json.loads((archive/'checkpoint.json').read_text(encoding='utf-8'))
    trajectory = np.asarray(state.get('trajectory', []), float)
    if (trajectory.ndim != 2 or trajectory.shape[1] != 8 or len(trajectory) < 3
            or not np.isfinite(trajectory).all() or np.any(np.diff(trajectory[:, 0]) <= 0)):
        raise ValueError('分块存档中没有有效完整轨迹')
    params = state.get('parameters', {})
    first, last = trajectory[[0, -1]]
    a, b = Rotation.from_quat([first[4:], last[4:]]).as_matrix()
    rot, trans = a.T @ b, a.T @ (last[1:4] - first[1:4])
    if np.linalg.norm(trans) > 2.:
        return ReturnCheck(False, '归档终点距起点超过 2 m，未进入重访诊断范围',
                           float(np.linalg.norm(trans)), _norm_deg(rot))
    epoch, clock = state.get('trajectory_epoch'), state.get('pose_clock')
    if epoch is not None:
        if not np.isfinite(epoch) or not isinstance(clock, int):
            raise ValueError('归档设备时间或时钟类型无效')
        epoch = float(epoch) + first[0]
    points = _endpoint_clouds(capture, last[0] - first[0],
                              float(params.get('calibration_seconds', 2.)) +
                              float(params.get('initial_scan_seconds', .5)), cancel, epoch, clock)
    return replace(align_endpoints(*points, rot, trans, cancel),
                   timing='设备时间定位' if epoch is not None else '旧归档：按录包尾部和轨迹时长推算')

# 提供后台任务窗口信息；输入为 window，通过 Tk 定时回调续接操作，返回 work。
def _worker_window(window, delivered):
    """Only Tk's own thread ever calls widgets; cancellation is per generation."""
    reports = queue.SimpleQueue()
    current = threading.Event()
    epoch = 0
    closed = False

    # 处理 _worker_window 提交的后台工作；创建后台线程。
    def work(job):
        nonlocal current, epoch
        current.set()
        current = threading.Event()
        epoch += 1
        token, stop = epoch, current

        # 处理 work 提交的后台工作；调用 job、put 处理。
        def run():
            try:
                result = job(stop)
            except Exception as exc:
                reports.put((token, None, str(exc) or type(exc).__name__))
            else:
                reports.put((token, result, None))

        threading.Thread(target=run, daemon=True, name='t100-field-geometry').start()

    # 轮询当前操作；通过 Tk 定时回调续接操作。
    def poll():
        nonlocal after_id
        while not reports.empty():
            token, result, error = reports.get()
            if token == epoch:
                delivered(result, error)
        if not closed:
            after_id = window.after(50, poll)

    after_id = window.after(50, poll)

    # 处理当前操作；输入为 event，取消挂起的界面回调。
    def destroyed(event):
        nonlocal closed
        if event.widget is window:
            closed = True
            current.set()
            window.after_cancel(after_id)

    window.bind('<Destroy>', destroyed, add='+')
    return work


# 提供投影距离信息；输入为 a、b，把输入统一为数组。
def projection_distance(a, b):
    """Plane distance in metres between two projected coordinates."""
    return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))


# 打开 profile ；输入为 parent、archive、comparison，注册界面事件，返回 window。
def open_profile(parent, archive, comparison=None):
    """Open a projection/section tool for one or two compatible archives."""
    import tkinter as tk
    from tkinter import ttk

    window = tk.Toplevel(parent)
    window.title('地图剖面和测距' if comparison is None else '同坐标系剖面对比')
    window.geometry('900x670')
    window.minsize(710, 420)
    paths = (Path(archive),) if comparison is None else (Path(archive), Path(comparison))
    ttk.Label(window, text='A：' + paths[0].name + ('    B：' + paths[1].name + '（同坐标系叠加，未配准）' if comparison is not None else '')).pack(anchor='w', padx=10, pady=(7, 0))
    controls = ttk.Frame(window)
    controls.pack(fill='x', padx=8, pady=7)
    planes = {'水平 XY': 'z', '立面 XZ': 'y', '立面 YZ': 'x'}
    plane = tk.StringVar(value='水平 XY')
    center, thick, step = tk.StringVar(value='0.00'), tk.StringVar(value='0.20'), tk.StringVar(value='0.10')
    mode, status, distance = tk.StringVar(value='box'), tk.StringVar(), tk.StringVar()
    projection = ttk.Combobox(controls, values=list(planes), textvariable=plane, state='readonly', width=9)
    projection.pack(side='left')
    for label, variable in (('中心 m', center), ('厚度 m', thick), ('步长 m', step)):
        ttk.Label(controls, text=label).pack(side='left', padx=(6, 1))
        ttk.Entry(controls, textvariable=variable, width=6).pack(side='left')
    tools = ttk.Frame(window)
    tools.pack(fill='x', padx=8)
    ttk.Radiobutton(tools, text='框选剖面（拖动）', variable=mode, value='box').pack(side='left')
    ttk.Radiobutton(tools, text='投影测距（两点）', variable=mode, value='measure').pack(side='left')
    ttk.Label(window, textvariable=status, wraplength=780).pack(fill='x', padx=10, pady=(6, 1))
    ttk.Label(window, textvariable=distance, wraplength=780).pack(fill='x', padx=10, pady=(0, 4))
    canvas = tk.Canvas(window, background='#10202f', highlightthickness=0)
    canvas.pack(fill='both', expand=True)
    roi, shown, view_limits = None, (), None
    transform, image, corner, drag = None, None, None, None
    marks = []
    finish_axes = 'XY'
    resize_job = None

    # 绘制当前操作；在画布显示渲染图像。
    def draw():
        nonlocal transform, image, resize_job
        resize_job = None
        canvas.delete('all')
        if view_limits is None:
            return
        w, h = max(canvas.winfo_width(), 1), max(canvas.winfo_height(), 1)
        low, high = view_limits
        span = np.maximum(high - low, .01)
        scale = min(max(1, w-50) / span[0], max(1, h-60) / span[1])
        midpoint = (low+high)/2
        origin = np.array([w/2 - midpoint[0]*scale, h/2 + midpoint[1]*scale])
        transform = scale, origin
        picture = np.full((h, w, 3), [16, 32, 47], dtype=np.uint8)
        layers = []
        for layer, color in zip(shown, ([63, 193, 244], [255, 172, 70])):
            xy = np.rint(layer.points[:, :2] * [scale, -scale] + origin).astype(int)
            x, y = xy.T
            inside = (x >= 0) & (x < w) & (y >= 0) & (y < h)
            x, y = x[inside], y[inside]
            if layers:
                both = np.all(picture[y, x] == layers[0], axis=1)
                picture[y, x] = color
                picture[y[both], x[both]] = [244, 245, 247]
            else:
                picture[y, x] = color
            layers.append(color)
        data = f'P6\n{w} {h}\n255\n'.encode('ascii') + picture.tobytes()
        image = tk.PhotoImage(master=canvas, data=data, format='PPM')
        canvas.create_image(0, 0, anchor='nw', image=image)
        unit = 10 ** np.floor(np.log10(90 / scale))
        candidates = [unit*n for n in (1, 2, 5, 10) if unit*n*scale <= 120]
        metres = max(candidates, default=unit)
        canvas.create_line(19, h-20, 19 + metres*scale, h-20, fill='#f8fafc', width=3)
        canvas.create_text(19, h-24, text=f'{metres:g} m   |   横 {finish_axes[0]} / 纵 {finish_axes[1]}', anchor='sw', fill='#f8fafc')
        if marks:
            xy = np.asarray(marks) * [scale, -scale] + origin
            if len(xy) == 2:
                canvas.create_line(*xy[0], *xy[1], fill='#fbeb74', width=2)
            for x, y in xy:
                canvas.create_oval(x-4, y-4, x+4, y+4, fill='#fbeb74', outline='#282817')

    # 重绘 soon ；通过 Tk 定时回调续接操作，取消挂起的界面回调。
    def redraw_soon(_event):
        nonlocal resize_job
        if resize_job is not None:
            canvas.after_cancel(resize_job)
        resize_job = canvas.after(40, draw)

    # 清除尺寸变化；输入为 event，取消挂起的界面回调。
    def clear_resize(event):
        if event.widget is canvas and resize_job is not None:
            canvas.after_cancel(resize_job)

    # 载入当前操作；输入为 result、error，把输入统一为数组。
    def load(result, error):
        nonlocal shown, view_limits, transform, finish_axes
        if error:
            status.set('无法读取：' + error)
            shown, view_limits, transform = (), None, None
            draw()
            return
        projection_axis, box, level, shown = result
        limits = [s.bounds[:, :2] for s in shown if s.count]
        finish_axes = {'z': 'XY', 'y': 'XZ', 'x': 'YZ'}[projection_axis]
        if box is not None:
            view_limits = np.asarray(box).reshape(2, 2)
        elif limits:
            view_limits = np.array([np.min(np.array(limits)[:, 0], axis=0), np.max(np.array(limits)[:, 1], axis=0)])
        else:
            view_limits = None
        counts = ' · '.join(f'{letter} {s.count:,} 点，体素 {s.resolution:g} m' +
                            (f'（显示采样 {len(s.points):,} 点）' if s.count > len(s.points) else '')
                            for letter, s in zip('AB', shown))
        if level is None:
            status.set(f'{finish_axes} 全图投影 · {counts}。拖框后读取框内薄层。')
            available = [s.bounds[:, 2] for s in shown if s.count]
            if available:
                low = min(item[0] for item in available)
                high = max(item[1] for item in available)
                center.set(f'{min(high, max(low, 0.)):.3f}')
        else:
            status.set(f'{finish_axes} 剖面 · {projection_axis.upper()} {level:g} m · {counts}' +
                       ('。本层无点，试试平移切片。' if not limits else '。测距为投影平面距离。'))
        transform = None
        draw()

    work = _worker_window(window, load)

    # 提交当前操作；输入为 overview，非法输入或处理失败时抛出异常，调用 isfinite、clear 处理。
    def submit(overview=False):
        nonlocal roi, shown, view_limits, transform, corner, drag
        axis = planes[plane.get()]
        if overview:
            roi = None
            level, thickness = None, .2
        else:
            try:
                level, thickness = float(center.get()), float(thick.get())
                if not np.isfinite([level, thickness]).all() or thickness <= 0:
                    raise ValueError
            except ValueError:
                status.set('请输入有效中心和大于 0 的厚度。')
                return
        box = roi
        marks.clear()
        distance.set('')
        corner, drag = None, None
        shown, view_limits, transform = (), None, None
        canvas.delete('all')
        status.set('正在只读扫描存档分块…')
        work(lambda cancel: (axis, box, level, tuple(read_section(path, box, axis, level, thickness, 85000, cancel) for path in paths)))

    # 根据输入计算移动；输入为 direction，非法输入或处理失败时抛出异常，调用 isfinite 处理。
    def move(direction):
        try:
            amount = float(step.get())
            value = float(center.get()) + direction * amount
            if not np.isfinite(value) or not np.isfinite(amount) or amount <= 0:
                raise ValueError
        except ValueError:
            status.set('请输入有效中心和大于 0 的平移步长。')
            return
        center.set(f'{value:.6g}')
        submit()

    # 提供 to_world 信息；输入为 x、y，把输入统一为数组。
    def to_world(x, y):
        scale, origin = transform
        return (np.asarray((x, y)) - origin) / (scale * np.array([1., -1.]))

    # 处理按下事件；输入为 event，调用 create_rectangle、clear 处理。
    def press(event):
        nonlocal corner, drag
        if transform is None:
            return
        if mode.get() == 'box':
            corner = (event.x, event.y)
            drag = canvas.create_rectangle(event.x, event.y, event.x, event.y, outline='#fbeb74', width=2)
        else:
            if len(marks) == 2:
                marks.clear()
            marks.append(to_world(event.x, event.y))
            pos = marks[-1]
            if len(marks) == 1:
                distance.set(f'第一点 {pos[0]:.3f}, {pos[1]:.3f} m；请选择第二点。')
            else:
                span = projection_distance(*marks)
                distance.set(f'投影距离 {span:.3f} m ｜ 屏幕每像素 {1/transform[0]:.3f} m。墙厚需沿墙面法向手选两侧。')
            draw()

    # 处理指针移动；输入为 event，调用 coords 处理。
    def motion(event):
        if corner is not None and drag is not None:
            canvas.coords(drag, *corner, event.x, event.y)

    # 结束当前按下或持有状态；输入为 event，调用 delete、abs 处理。
    def release(event):
        nonlocal roi, corner, drag
        if corner is None:
            return
        start = corner
        corner = None
        canvas.delete(drag)
        drag = None
        if transform is not None and abs(start[0]-event.x) >= 4 and abs(start[1]-event.y) >= 4:
            a, b = to_world(*start), to_world(event.x, event.y)
            roi = tuple(np.r_[np.minimum(a,b), np.maximum(a,b)])
            submit()

    for name, callback in (('应用切片', submit), ('← 平移', lambda: move(-1)),
                           ('平移 →', lambda: move(1)), ('全图投影', lambda: submit(True))):
        ttk.Button(tools, text=name, command=callback).pack(side='left', padx=3)
    projection.bind('<<ComboboxSelected>>', lambda _e: submit(True))
    canvas.bind('<Configure>', redraw_soon)
    canvas.bind('<Destroy>', clear_resize)
    canvas.bind('<ButtonPress-1>', press)
    canvas.bind('<B1-Motion>', motion)
    canvas.bind('<ButtonRelease-1>', release)
    submit(True)
    return window


# 打开 return_check ；输入为 parent、capture、archive，返回 window。
def open_return_check(parent, capture, archive=None):
    """Open an independent read-only diagnostic for an archived capture."""
    import tkinter as tk
    from tkinter import ttk

    window = tk.Toplevel(parent)
    window.title('起终点对齐检查 · 只读')
    window.geometry('650x365')
    ttk.Label(window, text=Path(capture).name).pack(anchor='w', padx=13, pady=(12, 0))
    status = tk.StringVar(value='正在读取录包和配套轨迹，检查首尾扫描重叠…')
    ttk.Label(window, textvariable=status, justify='left', wraplength=610).pack(anchor='w', padx=13, pady=12)
    ttk.Label(window, text='以归档位姿作为局部配准初值。新归档按设备时间定位，旧归档按轨迹时长估算窗口。'
              '诊断适用于回到同一区域并在首尾短暂停留的录包；运动、遮挡和重复结构可影响结果。'
              '数值衡量首尾一致性，不能代表全图精度；不会闭环或改写文件。',
              justify='left', wraplength=610).pack(anchor='w', padx=13)
    ttk.Button(window, text='关闭', command=window.destroy).pack(anchor='e', padx=13, pady=12)

    # 接收已完成任务的输出并处理后续状态；输入为 result、error，调用 append 处理。
    def delivered(result, error):
        if error:
            status.set('无法检查：' + error)
            return
        output = [result.reason if result.accepted else '不出具重访结论：' + result.reason,
                  f'归档轨迹首尾：位移 {result.archive_shift_m:.3f} m，转角 {result.archive_turn_deg:.2f}°']
        if result.accepted:
            output += [f'首尾扫描配准：位移 {result.revisit_shift_m:.3f} m，转角 {result.revisit_turn_deg:.2f}°',
                       f'相对归档残差：平移 {result.residual_shift_m:.3f} m，旋转 {result.residual_turn_deg:.2f}°']
        if result.rmse_m is not None:
            output.append(f'双向重叠 {result.overlap:.0%}  ·  点到平面残差 RMS {result.rmse_m:.3f} m')
        if result.timing:
            output.append('窗口定位：' + result.timing)
        if result.condition:
            output.append(f'几何约束指标 {result.condition:.3f}（越接近 0 越退化）')
        status.set('\n'.join(output))

    work = _worker_window(window, delivered)
    work(lambda cancel: check_return(capture, archive, cancel))
    return window
