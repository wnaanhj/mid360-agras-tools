# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 对白模输入执行体素归并、法线、边和三角面处理；按任务阶段报告进度，网格后续由预览或导出模块消费。
"""Conservative surface triangulation of completed T100 maps, in metres."""
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .field_geometry import _ROW, _cancelled, _open_db, _sqlite_path


_CHUNK = 65536
_BATCH = 8192
_MAX_INPUT = 40_000_000
_MAX_VOXELS = 400_000
# Keep margin for looking up nearby cells without integer carries.
_LIMIT = (1 << 20) - 4
_STRIDES = np.array([1 << 42, 1 << 21, 1], dtype=np.int64)


# 报告当前操作；输入为 progress、message。
def _report(progress, message):
    if progress is not None:
        progress(message)


# 整理分块；输入为 db、cancel，逐批交付结果，调用方可按需迭代，非法输入或处理失败时抛出异常，调用 execute、_cancelled 处理。
def _archive_chunks(db, cancel):
    # Query only the lengths first; corrupt payloads cannot allocate large arrays.
    for key, n, size in db.execute('SELECT id,n,length(data) FROM blocks ORDER BY id'):
        _cancelled(cancel)
        if not isinstance(n, int) or n < 0 or size != n * _ROW.itemsize:
            raise ValueError('分块地图点数与文件不一致')
        if n > _MAX_INPUT:
            raise ValueError('存档分块超过白模读取上限')
        for start in range(0, n, _CHUNK):
            _cancelled(cancel)
            count = min(_CHUNK, n - start)
            (blob,) = db.execute('SELECT substr(data,?,?) FROM blocks WHERE id=?',
                                 (start * _ROW.itemsize + 1, count * _ROW.itemsize, key)).fetchone()
            yield np.frombuffer(blob, _ROW)['xyz']


# 按米制步长量化三维点；分块滤掉非法样本并聚合体素内的 XYZ，为网格提供均匀的代表点。
def _voxelize(chunks, step, cancel, progress):
    cells = {}
    total = invalid = 0
    for chunk in chunks:
        _cancelled(cancel)
        p = np.asarray(chunk, dtype=np.float64)
        total += len(p)
        if total > _MAX_INPUT:
            raise ValueError(f'输入超过 {_MAX_INPUT:,} 点的白模处理上限')
        keep = np.isfinite(p).all(axis=1)
        invalid += len(p) - int(keep.sum())
        p = p[keep]
        if not len(p):
            continue
        q = np.floor(p / step)
        if np.any(np.abs(q) > _LIMIT):
            raise ValueError('坐标超出白模网格范围，请增大步长')
        keys = q.astype(np.int64) @ _STRIDES
        key, inv, counts = np.unique(keys, return_inverse=True, return_counts=True)
        sums = np.column_stack([np.bincount(inv, weights=p[:, i]) for i in range(3)])
        for k, xyz, n in zip(key.tolist(), sums.tolist(), counts.tolist()):
            prev = cells.get(k)
            if prev is None:
                if len(cells) >= _MAX_VOXELS:
                    raise ValueError(f'白模网格超过 {_MAX_VOXELS:,} 个，请增大步长')
                cells[k] = xyz + [n]
            else:
                prev[0] += xyz[0]
                prev[1] += xyz[1]
                prev[2] += xyz[2]
                prev[3] += n
        if total // 500_000 != (total - len(chunk)) // 500_000:
            _report(progress, f'正在汇总点云：已读取 {total:,} 点')
    keys = np.array(sorted(cells), dtype=np.int64)
    xyz = np.array([cells[int(k)] for k in keys], dtype=np.float64).reshape(-1, 4)
    return keys, xyz[:, :3] / xyz[:, 3:], total, invalid


# 利用 KD 树取近邻，在局部协方差上估计法向和优势投影轴；步长用于限制邻域半径。
def _normals(points, step, cancel):
    normal = np.zeros_like(points)
    if len(points) < 5:
        return normal, np.full(len(points), -1, dtype=np.int8)
    tree = cKDTree(points)
    axis = np.full(len(points), -1, dtype=np.int8)
    for start in range(0, len(points), _BATCH):
        _cancelled(cancel)
        stop = min(start + _BATCH, len(points))
        p = points[start:stop]
        d, indices = tree.query(p, k=min(14, len(points)), distance_upper_bound=2.35 * step)
        used = np.isfinite(d)
        local = (points[np.where(used, indices, 0)] - p[:, None, :]) / step
        local[~used] = 0
        weights = np.where(used, np.exp(-.38 * (d / step) ** 2), 0)
        sumw = weights.sum(axis=1)
        mean = np.einsum('nk,nki->ni', weights, local) / sumw[:, None]
        centered = local - mean[:, None, :]
        covariance = np.einsum('nk,nki,nkj->nij', weights, centered, centered) / sumw[:, None, None]
        eig, basis = np.linalg.eigh(covariance)
        n = basis[:, :, 0]
        # Require enough support in two directions and reject mixed planes.
        valid = (used.sum(axis=1) >= 5) & (eig[:, 1] >= .075) & (eig[:, 0] <= .32 * eig[:, 1])
        a = np.argmax(np.abs(n), axis=1)
        s = n[np.arange(len(n)), a]
        normal[start:stop] = n * np.where(s >= 0, 1., -1.)[:, None]
        axis[start:stop] = np.where(valid, a, -1)
    return normal, axis


# 在各主轴投影的体素邻域中寻找可能连边；检查距离、走向和法线，避免跨空隙搭面。
def _edges(keys, p, n, step, axis, du, dv, cancel):
    """Best reciprocal neighbour in an adjacent projected cell."""
    # Build matches in both directions. This keeps a projected layer from
    # touching a different depth layer when that layer has a better match.
    u, v = (axis + 1) % 3, (axis + 2) % 3
    matches = []
    for sign in (1, -1):
        offset = sign * (du * _STRIDES[u] + dv * _STRIDES[v])
        out = np.full(len(keys) + 1, -1, dtype=np.int32)
        for start in range(0, len(keys), _CHUNK):
            _cancelled(cancel)
            stop = min(start + _CHUNK, len(keys))
            score = np.full(stop-start, np.inf)
            matched = out[start:stop]
            for depth in (0, -1, 1, -2, 2):
                target = keys[start:stop] + offset + depth * _STRIDES[axis]
                ids = np.minimum(np.searchsorted(keys, target), len(keys)-1)
                delta = (p[ids] - p[start:stop]) / step
                d2 = np.einsum('ni,ni->n', delta, delta)
                dev_a = np.abs(np.einsum('ni,ni->n', delta, n[start:stop]))
                dev_b = np.abs(np.einsum('ni,ni->n', delta, n[ids]))
                angle = np.einsum('ni,ni->n', n[start:stop], n[ids])
                loss = d2 + 3 * (dev_a**2 + dev_b**2)
                good = ((keys[ids] == target) & (d2 <= (1.88 if du and dv else 1.64)**2)
                        & (dev_a <= .34) & (dev_b <= .34) & (angle >= .82) & (loss < score))
                matched[good] = ids[good]
                score[good] = loss[good]
        matches.append(out)
    fwd, back = matches
    ok = (fwd[:-1] >= 0) & (back[fwd[:-1]] == np.arange(len(keys)))
    fwd[:-1][~ok] = -1
    present = np.flatnonzero(back[:-1] >= 0)
    back[present[fwd[back[present]] != present]] = -1
    return fwd, back


# 按主轴分组连接体素相邻点，筛除几何质量不足的三角面，输出带零基索引的网格面。
def _triangulate(keys, points, normals, axes, step, cancel, progress):
    triangles, quality = [], []
    for axis in range(3):
        _cancelled(cancel)
        indices = np.flatnonzero(axes == axis).astype(np.int32)
        if len(indices) < 3:
            continue
        _report(progress, f'正在连接 {"XYZ"[axis]} 法向的局部网格…')
        k, p, n = keys[indices], points[indices], normals[indices]
        e, w = _edges(k, p, n, step, axis, 1, 0, cancel)
        north, _ = _edges(k, p, n, step, axis, 0, 1, cancel)
        ne, _ = _edges(k, p, n, step, axis, 1, 1, cancel)
        nw, se = _edges(k, p, n, step, axis, -1, 1, cancel)

        # a,c are left corners, b,d are right corners. Candidate triangles
        # stay inside one cell and require all three validated local edges.
        a = np.arange(len(k), dtype=np.int32)
        b, c, d = e[:-1], north[:-1], ne[:-1]
        abc = (b >= 0) & (c >= 0) & (nw[b] == c)
        abd = (b >= 0) & (d >= 0) & (north[b] == d)
        acd = (c >= 0) & (d >= 0) & (e[c] == d)
        # Represent bcd by its right-bottom corner b since a may be absent.
        cb, db = nw[:-1], north[:-1]
        bcd = (cb >= 0) & (db >= 0) & (e[cb] == db)
        # If both diagonals are possible, prefer the split yielding two
        # triangles and break a complete tie by the shorter diagonal.
        competing = abc & (abd | acd)
        q = a[competing]
        other = bcd[b[q]]
        d_ad = np.sum((p[a[q]] - p[d[q]])**2, axis=1)
        d_bc = np.sum((p[b[q]] - p[c[q]])**2, axis=1)
        same_fourth = north[b[q]] == d[q]
        prefer_ad = abd[q] & acd[q] & (~other | (same_fourth & (d_ad <= d_bc)))
        # With two triangles present along b-c, never mix an overlapping a-d face.
        prefer_bc = ~prefer_ad
        abd[q[prefer_bc]] = False
        acd[q[prefer_bc]] = False
        abc[q[prefer_ad]] = False
        bcd[b[q[prefer_ad & same_fourth]]] = False
        # Also handle the bcd-versus-abd/acd pair if the a-b-c triangle
        # was rejected (e.g. inconsistent depth match).
        q = a[(abd | acd) & (b >= 0) & (c >= 0)]
        overlap = bcd[b[q]] & (nw[b[q]] == c[q]) & (north[b[q]] == d[q])
        bcd[b[q[overlap]]] = False
        # bcd and abd could overlap even when the a-c edge is missing.
        q = a[abd]
        overlap = bcd[b[q]] & (north[b[q]] == d[q])
        bcd[b[q[overlap]]] = False
        q = a[acd]
        rb = se[c[q]]
        overlap = bcd[np.maximum(rb, 0)] & (rb >= 0) & (north[rb] == d[q]) & (w[rb] == a[q])
        bcd[rb[overlap]] = False

        _cancelled(cancel)
        pidx = np.column_stack((a, b, c))[abc]
        ridx = np.column_stack((a, b, d))[abd]
        sidx = np.column_stack((a, d, c))[acd]
        tidx = np.column_stack((a, db, cb))[bcd]
        faces = np.concatenate((pidx, ridx, sidx, tidx), axis=0)
        va, vb, vc = (p[faces[:, i]] for i in range(3))
        cross = np.cross((vb - va) / step, (vc - va) / step)
        area = np.linalg.norm(cross, axis=1)
        local_normal = n[faces[:, 0]] + n[faces[:, 1]] + n[faces[:, 2]]
        facing = np.einsum('ni,ni->n', cross, local_normal)
        # Reject thin and badly tilted triangles. No artificial vertices.
        keep = (area >= .045) & (facing >= .78 * area * np.linalg.norm(local_normal, axis=1))
        triangles.append(indices[faces[keep]])
        # Prefer triangles well aligned to all three measured normals.
        quality.append(facing[keep] / (area[keep] * np.linalg.norm(local_normal[keep], axis=1)))
    if not triangles:
        return np.empty((0, 3), dtype=np.int32)
    faces = np.concatenate(triangles, axis=0)
    score = np.concatenate(quality)
    keep = np.ones(len(faces), dtype=bool)
    # Two triangles with equally directed winding on a shared edge
    # occupy the same side of it. Keep the better supported one.
    _cancelled(cancel)
    edge = (faces.astype(np.int64) * len(points) + np.roll(faces, -1, axis=1)).ravel()
    rank = np.lexsort((-np.repeat(score, 3), edge))
    conflict = edge[rank[1:]] == edge[rank[:-1]]
    keep[rank[1:][conflict] // 3] = False
    return faces[keep]


# 从停止写入的地图档案或 PLY/PCD 流式收集点并重建；step_m 为米制步长，返回紧凑顶点、面索引和可序列化统计。
def build_mesh(archive, step_m, cancel=None, progress=None):
    """Build a local triangle surface from a stopped map, PLY or PCD (metres).

    Returns compact XYZ vertices, zero-based triangle indices, and JSON-safe
    statistics. Unsupported/sparse regions remain open; no hole filling.
    Raises InterruptedError if the caller signals cancellation.
    """
    _cancelled(cancel)
    step = float(step_m)
    if not np.isfinite(step) or not .0001 <= step <= 1_000:
        raise ValueError('网格步长需为 0.0001 至 1000 米之间的有限数值')
    path = Path(archive)
    resolution = None
    _report(progress, '正在只读读取点云并汇总网格…')
    if path.suffix.lower() in ('.ply', '.pcd'):
        from .white_mesh_io import iter_exported_points
        chunks = iter_exported_points(path)
        try:
            keys, p, total, invalid = _voxelize(chunks, step, cancel, progress)
        finally:
            close = getattr(chunks, 'close', None)
            if close is not None:
                close()
    else:
        dbfile = _sqlite_path(path).resolve(strict=True)
        checkpoint = dbfile.with_name('checkpoint.json')
        if not checkpoint.is_file():
            raise ValueError('存档缺少结束记录，请先在建图软件中正常结束会话')
        state = json.loads(checkpoint.read_text(encoding='utf-8'))
        status = state.get('status', '') if isinstance(state, dict) else ''
        if not isinstance(status, str) or not (status.startswith('已结束') or status in (
                '建图已结束，地图已保留', '软件关闭，建图已结束', '离线重算结束', '结束保存')):
            raise ValueError('白模仅支持已正常结束的分块存档')
        db = _open_db(dbfile)
        try:
            db.set_progress_handler(lambda: int(cancel is not None and cancel.is_set()), 1000)
            db.execute('BEGIN')
            settings = dict(db.execute('SELECT name,value FROM settings'))
            try:
                resolution = float(settings['resolution'])
            except (KeyError, ValueError):
                raise ValueError('分块存档缺少有效的体素分辨率') from None
            if not np.isfinite(resolution) or resolution <= 0:
                raise ValueError('分块存档缺少有效的体素分辨率')
            keys, p, total, invalid = _voxelize(_archive_chunks(db, cancel), step, cancel, progress)
        except Exception:
            _cancelled(cancel)
            raise
        finally:
            db.close()
    _cancelled(cancel)
    return _finish_mesh(keys,p,total,invalid,step,resolution,cancel,progress)

# 用已经加载的 N×3 点集进入同一体素化、法向估计与连接管线，不再从档案重读。
def build_mesh_from_points(points, step_m, cancel=None, progress=None):
    # Preserve the exact retained subset; never reload the unfiltered source.
    step=float(step_m)
    if not np.isfinite(step) or not .0001 <= step <= 1000:raise ValueError('构面间距无效')
    chunks=(points[i:i+_CHUNK] for i in range(0,len(points),_CHUNK))
    keys,p,total,invalid=_voxelize(chunks,step,cancel,progress)
    return _finish_mesh(keys,p,total,invalid,step,None,cancel,progress)

# 对体素代表点计算法向与三角面并压缩未使用顶点，同时组装有效点、无效点和分辨率统计。
def _finish_mesh(keys,p,total,invalid,step,resolution,cancel,progress):
    _cancelled(cancel)
    _report(progress, f'正在估计局部表面：{len(p):,} 个网格点…')
    n, axis = _normals(p, step, cancel)
    faces = _triangulate(keys, p, n, axis, step, cancel, progress)
    _cancelled(cancel)
    used = np.unique(faces)
    vertices = p[used]
    faces = np.searchsorted(used, faces).astype(np.int32)
    stats = dict(units='m', step_m=step, source_resolution_m=resolution,
                 input_points=total, invalid_points=invalid, voxel_count=len(p),
                 planar_points=int((axis >= 0).sum()), vertex_count=len(vertices),
                 face_count=len(faces))
    _report(progress, f'白模网格完成：{len(vertices):,} 顶点，{len(faces):,} 三角面')
    _cancelled(cancel)
    return vertices, faces, stats
