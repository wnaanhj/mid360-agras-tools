# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 实现点云修复的几何算法：去噪、连通片、调平和填补等；按分块复查取消和进度，输出修复数据与说明供预览。
"""Offline geometric repair. A result always refers to the original input rows."""
from dataclasses import dataclass
from numbers import Real

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


from .repair_resources import memory_plan, require_memory


MAX_POINTS = np.iinfo(np.intp).max // 12  # addressable float32 XYZ rows
CHUNK = 8192
# Defaults, inclusive upper bounds, and whether zero is permitted.
_SPECS = {
    'radius': {'radius': (.1, 1000., False), 'min_neighbors': (3, 256, False)},
    'statistical': {'neighbors': (16, 128, False), 'std_ratio': (2., 20., True)},
    'components': {'distance': (.15, 1000., False), 'min_points': (30, MAX_POINTS, False),
                   'min_extent': (.1, 1e6, True)},
    'ground': {'distance': (.05, 100., False), 'max_slope_deg': (25., 85., True), 'seed': (0, 2**32-1, True)},
    'level': {'distance': (.05, 100., False), 'max_slope_deg': (35., 85., True), 'target_height': (0., 1e6, True)},
    'flatten': {'radius': (.3, 1000., False), 'strength': (.5, 1., True), 'max_move': (.08, 100., True),
                'edge_threshold': (.08, 100., False)},
    'planes': {'distance': (.05, 100., False), 'max_move': (.05, 100., True), 'strength': (.8, 1., True),
               'min_points': (100, MAX_POINTS, False), 'max_planes': (8, 32, False)},
    'fill': {'spacing': (.05, 100., False), 'max_hole': (.25, 100., False), 'plane_tolerance': (.03, 100., False),
             'max_added': (100000, 100000, True)},
}


# 为调用层标识 RepairCancelled 这一分支；调用方可单独捕获以区分正常结果与中止。
class RepairCancelled(RuntimeError):
    """Raised before any result is returned when cancellation is requested."""


# 记录修复返回的点、掩码或变换及诊断字段，由修复文档决定是否应用和持久化。
@dataclass
class RepairResult:
    points: np.ndarray
    keep: np.ndarray
    moved: np.ndarray
    added: np.ndarray
    stats: dict


# 提供参数信息；输入为 operation、parameters，返回 result，非法输入或处理失败时抛出异常。
def _parameters(operation, parameters):
    if not isinstance(operation, str) or operation not in _SPECS:
        raise ValueError('未知的点云修复操作')
    if parameters is None:
        parameters = {}
    if not hasattr(parameters, 'items') or set(parameters) - set(_SPECS[operation]):
        raise ValueError('修复参数不适用于当前操作')
    result = {}
    for key, (default, upper, allow_zero) in _SPECS[operation].items():
        value = parameters.get(key, default)
        if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not np.isfinite(value):
            raise ValueError(f'修复参数 {key} 必须是有限数字')
        minimum = -upper if key == 'target_height' else (0 if allow_zero else 1 if isinstance(default, int) else 1e-6)
        if not minimum <= value <= upper or (isinstance(default, int) and int(value) != value):
            raise ValueError(f'修复参数 {key} 超出允许范围')
        result[key] = type(default)(value)
    if operation == 'planes' and result['min_points'] < 3:
        raise ValueError('平面至少需要 3 个支持点')
    return result


# 按最大块长度生成数组切片，限制临时内存并留出进度/取消检查点。
def _blocks(n, size=CHUNK):
    for start in range(0, n, size):
        yield slice(start, min(start + size, n))


# 按形状和数据类型从传入分配器或默认 NumPy 路径申请数组，必要时分块填充默认值。
def _allocate(shape, dtype, fill, check, allocator):
    check()
    result = np.empty(shape, dtype=dtype) if allocator is None else allocator(shape, dtype)
    if fill is not None:
        for sl in _blocks(len(result)):
            check()
            result[sl] = fill
    return result


# 分块复制点属性；有索引时散写到目标行，每块前检查取消。
def _copy_into(dest, src, check, indices=None):
    for sl in _blocks(len(src)):
        check()
        dest[sl if indices is None else indices[sl]] = src[sl]


# 扫描布尔选区，将置位位置压缩成点行索引，避免对整云生成临时索引。
def _selected_rows(selection, count, allocate, check):
    indices = allocate(count, np.intp)
    pos = 0
    for sl in _blocks(len(selection)):
        check()
        rows = np.flatnonzero(selection[sl]) + sl.start
        indices[pos:pos + len(rows)] = rows
        pos += len(rows)
    return indices


# 用点的局部邻域密度或距离识别噪点，返回应保留的掩码；加速结果可用时直接采用，否则走 CPU 查询。
def _point_denoise(q, op, p, check, report, allocate, compute=None):
    n = len(q)
    k = p['min_neighbors' if op == 'radius' else 'neighbors']
    if n <= k:
        return allocate(n, bool, op != 'radius')
    if op == 'radius' and compute is not None:
        accelerated = compute.radius(q, p['radius'], k, allocate, check, report)
        if accelerated is not None:
            return accelerated
    tree = cKDTree(q)
    if op == 'radius':
        # The kth other point is sufficient; no unbounded neighbour lists.
        return _radius(tree, q, p['radius'], k, check, report, allocate)
    scores = allocate(n, np.float64)
    for sl in _blocks(n):
        check()
        scores[sl] = tree.query(q[sl], k=k + 1)[0][:, 1:].mean(axis=1)
        report(.85 * sl.stop / n)
    mean = scores.mean()
    kept = allocate(n, bool)
    deviations = allocate(n, np.float64)
    for sl in _blocks(n):
        check()
        diff = scores[sl] - mean
        deviations[sl] = diff * diff
    cutoff = mean + p['std_ratio'] * np.sqrt(deviations.sum() / n)
    for sl in _blocks(n):
        check()
        kept[sl] = scores[sl] <= cutoff
    return kept


# 分块查询指定米制半径内的邻居数量，以最少邻点阈值产生保留掩码，并更新进度。
def _radius(tree, q, radius, k, check, report, allocate):
    kept = allocate(len(q), bool)
    for sl in _blocks(len(q)):
        check()
        ds = tree.query(q[sl], k=[k + 1], distance_upper_bound=np.nextafter(radius, np.inf))[0]
        kept[sl] = ds[:, 0] <= radius
        report(.85 * sl.stop / len(q))
    return kept


# 将点划分为体素后查找连通片；用并查集合并近邻，按点数及组件约束构造保留结果。
def _components(q, p, check, report, allocate):
    n = len(q)
    cutoff = p['distance']
    # Every point in a voxel is connected to every other point in that voxel.
    # Only nearby occupied voxels need an edge, and each edge is verified in XYZ.
    cell = cutoff / np.sqrt(3)
    coords = (q.astype(np.float64) - q.min(axis=0)) / cell
    if np.max(coords) >= (2**53 - 8):
        raise ValueError('点云坐标跨度过大，无法建立连通体素')
    grid, inverse, counts = np.unique(np.floor(coords).astype(np.int64), return_inverse=True, return_counts=True, axis=0)
    del coords
    check()
    order = np.argsort(inverse, kind='stable')
    starts = np.r_[0, np.cumsum(counts)]
    # Sparse voxel neighbors are queried in batches; no point adjacency matrix.
    voxel_tree = cKDTree(grid.astype(np.float64))
    index_dtype = np.int32 if len(grid) <= np.iinfo(np.int32).max else np.intp
    parent = np.arange(len(grid), dtype=index_dtype)
    sizes = np.ones(len(grid), dtype=index_dtype)
    exact_trees = {}
    # 查询并查集中体素所属根节点，同时压缩访问路径，减少后续邻接合并开销。
    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for sl in _blocks(len(grid), 2048):
        check()
        candidates = voxel_tree.query_ball_point(grid[sl], np.sqrt(3) + 1, p=np.inf)
        edge_i, edge_j = [], []
        for i, others in enumerate(candidates, sl.start):
            edge_i.extend([i] * sum(j > i for j in others))
            edge_j.extend(j for j in others if j > i)
        if edge_i:
            aa, bb = np.asarray(edge_i), np.asarray(edge_j)
            delta = np.maximum(np.abs(grid[aa] - grid[bb]) - 1, 0)
            possible = (delta * delta).sum(axis=1) * cell * cell <= cutoff * cutoff
            aa, bb = aa[possible], bb[possible]
            both_single = (counts[aa] == 1) & (counts[bb] == 1)
            valid = np.zeros(len(aa), dtype=bool)
            diff = q[order[starts[aa[both_single]]]].astype(float) - q[order[starts[bb[both_single]]]]
            valid[both_single] = (diff * diff).sum(axis=1) <= cutoff * cutoff
            for k in np.flatnonzero(~both_single):
                if k % 256 == 0:
                    check()
                i, j = aa[k], bb[k]
                if root(i) == root(j):
                    continue
                if counts[i] > counts[j]:
                    i, j = j, i
                right = exact_trees.get(j)
                if right is None:
                    if len(exact_trees) >= 16:
                        exact_trees.pop(next(iter(exact_trees)))
                    right = exact_trees[j] = cKDTree(q[order[starts[j]:starts[j + 1]]])
                left = order[starts[i]:starts[i + 1]]
                for bs in _blocks(len(left)):
                    check()
                    if np.any(right.query(q[left[bs]], distance_upper_bound=np.nextafter(cutoff, np.inf))[0] <= cutoff):
                        valid[k] = True
                        break
            # Relabeling connected components of this batch in C avoids Python
            # work per edge in ordinary dense, single-point voxels.
            if np.count_nonzero(valid):
                src, dst = aa[valid], bb[valid]
                nodes, inv = np.unique(np.r_[src, dst], return_inverse=True)
                graph = coo_matrix((np.ones(len(src), dtype=np.int8),
                                    (inv[:len(src)], inv[len(src):])), shape=(len(nodes), len(nodes)))
                _, labels = connected_components(graph.tocsr(), directed=False)
                ix = np.argsort(labels, kind='stable')
                sn, ls = nodes[ix], labels[ix]
                for a, b in zip(sn[1:][ls[1:] == ls[:-1]], sn[:-1][ls[1:] == ls[:-1]]):
                    a, b = root(a), root(b)
                    if a != b:
                        if sizes[a] < sizes[b]:
                            a, b = b, a
                        parent[b] = a
                        sizes[a] += sizes[b]
        report(.85 * sl.stop / len(grid))
    while np.any(parent != parent[parent]):
        check()
        parent = parent[parent]
    labels = parent[inverse]
    size = np.bincount(labels, minlength=len(grid))
    kept = allocate(n, bool)
    for sl in _blocks(n):
        check()
        kept[sl] = size[labels[sl]] >= p['min_points']
    if p['min_extent']:
        if not kept.any():
            return kept
        lower = np.full((3, len(grid)), np.inf, dtype=np.float32)
        upper = np.full_like(lower, -np.inf)
        for sl in _blocks(n):
            check()
            active = np.flatnonzero(kept[sl]) + sl.start
            for col in range(3):
                np.minimum.at(lower[col], labels[active], q[active, col])
                np.maximum.at(upper[col], labels[active], q[active, col])
        extent_ok = allocate(len(grid), bool, False)
        for sl in _blocks(len(grid)):
            check()
            present = np.flatnonzero(size[sl] > 0) + sl.start
            extent_ok[present] = np.linalg.norm(upper[:, present].astype(float) - lower[:, present], axis=0) >= p['min_extent']
        for sl in _blocks(n):
            check()
            kept[sl] &= extent_ok[labels[sl]]
    return kept


# 在候选支持行中取有限规模样本，用协方差分解拟合平面；若支持点不足或几何退化则没有模型。
def _fit(q, indices, check):
    if len(indices) < 3:
        return None
    # Fit covariance through a bounded, deterministic sample.
    chosen = indices[np.linspace(0, len(indices) - 1, min(len(indices), 100000), dtype=int)]
    check()
    subset = q[chosen].astype(float)
    center = subset.mean(axis=0)
    sub = subset - center
    values, axes = np.linalg.eigh(sub.T @ sub / len(sub))
    if values[1] <= max(1e-14, 1e-9 * values[2]):
        return None
    normal = axes[:, 0]
    if normal[2] < 0:
        normal = -normal
    return normal, center


# 以法线方向距离与容差比较，分块把支持同一平面的索引写进给定缓冲区。
def _support(q, normal, center, tolerance, indices, check):
    count = 0
    for sl in _blocks(len(q)):
        check()
        distance = (q[sl] - center) @ normal
        rows = np.flatnonzero(np.abs(distance) <= tolerance) + sl.start
        indices[count:count + len(rows)] = rows
        count += len(rows)
    return indices[:count]


# 按水平网格提取当地低位代表点作为地面候选，减少建筑边与高处物体对调平的偏置。
def _low_envelope(q, tolerance, check, allocate):
    # Spatial votes are independent of point density. Unlike a uniform sample,
    # this scan cannot lose a thin yet well-observed floor below a dense roof.
    bins = 16
    xy_min = np.full(2, np.inf)
    xy_max = np.full(2, -np.inf)
    for sl in _blocks(len(q)):
        check()
        xy_min = np.minimum(xy_min, q[sl, :2].min(axis=0))
        xy_max = np.maximum(xy_max, q[sl, :2].max(axis=0))
    xy_span = np.maximum(xy_max - xy_min, tolerance)
    z_min = np.full(bins * bins, np.inf)
    keys = allocate(len(q), np.uint16)
    for sl in _blocks(len(q)):
        check()
        xy = np.minimum(bins - 1, np.floor((q[sl, :2] - xy_min) / xy_span * bins).astype(int))
        keys[sl] = xy[:, 0] * bins + xy[:, 1]
        np.minimum.at(z_min, keys[sl], q[sl, 2])
    # Take one measured point from each occupied cell, with ties not increasing its vote.
    indices = np.full(bins * bins, len(q), dtype=int)
    for sl in _blocks(len(q)):
        check()
        idx = np.flatnonzero(q[sl, 2] == z_min[keys[sl]]) + sl.start
        np.minimum.at(indices, keys[idx], idx)
    return q[indices[indices < len(q)]]


# 提供 envelope_evidence 信息；输入为 residuals、tolerance，按序返回 ok、on、below。
def _envelope_evidence(residuals, tolerance):
    on = (np.abs(residuals) <= tolerance).sum(axis=0)
    below = (residuals < -2 * tolerance).sum(axis=0)
    count = len(residuals)
    # A ceiling or shelf cannot win if coherent lower returns occur across
    # the scene. A few isolated low returns do not invalidate a true floor.
    ok = (on >= max(3, int(np.ceil(.25 * count)))) & (below <= max(1, int(.20 * count)))
    return ok, on, below


# 对候选点寻找满足支持点数和可选最大坡度的平面，并更新候选模型；分块检查取消和支持范围。
def _find_plane(q, tolerance, min_points, rng, check, allocate, slope=None, support_rows=None):
    n = len(q)
    if n < min_points:
        return None
    sample = q[rng.choice(n, min(n, 4096), replace=False)].astype(float)
    origin = sample.mean(axis=0)
    sample -= origin
    envelope = _low_envelope(q, tolerance, check, allocate) if slope is not None else None
    if slope is not None and len(envelope) < 3:
        return None
    seed_sample = envelope.astype(float) - origin if slope is not None else sample
    best = None
    best_score = -np.inf
    minimum_sample = max(3, int(np.ceil(min_points * len(sample) / n * .75)))
    cos_limit = np.cos(np.radians(slope)) if slope is not None else 0.
    for attempt in range(5):
        check()
        source = seed_sample if slope is not None and attempt < 4 else sample
        triples = source[rng.integers(len(source), size=(48, 3))]
        normals = np.cross(triples[:, 1] - triples[:, 0], triples[:, 2] - triples[:, 0])
        lengths = np.linalg.norm(normals, axis=1)
        valid = lengths > 1e-10
        if not np.any(valid):
            continue
        normals = normals[valid] / lengths[valid, None]
        normals *= np.where(normals[:, 2:3] < 0, -1, 1)
        refs = triples[valid, 0]
        valid = normals[:, 2] + 1e-12 >= cos_limit
        normals, refs = normals[valid], refs[valid]
        if not len(normals):
            continue
        offsets = (normals * refs).sum(axis=1)
        support = (np.abs(sample @ normals.T - offsets) <= tolerance).sum(axis=0)
        scores = support.astype(float)
        if slope is not None:
            consistent, on, below = _envelope_evidence(seed_sample @ normals.T - offsets, tolerance)
            scores = (on - below) / len(seed_sample) + .01 * support / len(sample)
            scores[~consistent] = -np.inf
        else:
            scores[support < minimum_sample] = -np.inf
        j = np.argmax(scores)
        if scores[j] > best_score:
            best_score = scores[j]
            best = normals[j], refs[j] + origin
    if best is None:
        return None
    normal, center = best
    if support_rows is None:
        support_rows = allocate(n, np.intp)
    support = _support(q, normal, center, tolerance, support_rows, check)
    for _ in range(2):
        if len(support) < min_points:
            return None
        model = _fit(q, support, check)
        if model is None:
            return None
        normal, center = model
        if normal[2] + 1e-12 < cos_limit:
            return None
        support = _support(q, normal, center, tolerance, support_rows, check)
    if len(support) < min_points:
        return None
    if slope is not None:
        consistent, _, _ = _envelope_evidence((envelope - center) @ normal, tolerance)
        if not consistent:
            return None
    return normal, center, support


# 从点集中逐步提取一组平面；设定支持点门槛与模型上限以限制误拟合和开销。
def _plane_models(q, tolerance, minimum, limit, check, allocate):
    if len(q) < minimum:
        return
    rng = np.random.default_rng(0)
    remaining = allocate(len(q), np.intp)
    support_rows = allocate(len(q), np.intp)
    source_rows = allocate(len(q), np.intp)
    keep = allocate(len(q), bool)
    work = None
    for sl in _blocks(len(q)):
        check()
        remaining[sl] = np.arange(sl.start, sl.stop, dtype=np.intp)
    q_active = q
    for i in range(limit):
        check()
        fit = _find_plane(q_active, tolerance, minimum, rng, check, allocate, support_rows=support_rows)
        if fit is None:
            break
        normal, center, support = fit
        for sl in _blocks(len(support)):
            check()
            source_rows[sl] = remaining[support[sl]]
        yield normal, center, source_rows[:len(support)]
        if i + 1 == limit or len(remaining) - len(support) < minimum:
            break
        for sl in _blocks(len(remaining)):
            check()
            keep[sl] = True
        for sl in _blocks(len(support)):
            check()
            keep[support[sl]] = False
        if work is None:
            work = allocate((len(remaining) - len(support), 3), np.float32)
        pos = 0
        for sl in _blocks(len(remaining)):
            check()
            rows = remaining[sl][keep[sl]]
            remaining[pos:pos + len(rows)] = rows
            work[pos:pos + len(rows)] = q[rows]
            pos += len(rows)
        remaining = remaining[:pos]
        q_active = work[:pos]


# 提供 flatten 信息；输入为 q、p、check，构建 KD 树进行空间查询，返回 output。
def _flatten(q, p, check, report, allocate):
    tree = cKDTree(q)
    output = allocate(q.shape, q.dtype)
    k = min(64, len(q))
    for sl in _blocks(len(q), 4096):
        check()
        z = q[sl, 2].astype(float)
        ds, ii = tree.query(q[sl], k=k, distance_upper_bound=np.nextafter(p['radius'], np.inf))
        vals = q[np.minimum(ii, len(q) - 1), 2].astype(float)
        dz = vals - z[:, None]
        # Work within a single height band; a curb/wall is not a smoothing target.
        valid = np.isfinite(ds) & (np.abs(dz) <= p['edge_threshold'])
        if k > 2:
            upward = np.sum(np.isfinite(ds) & (dz > p['edge_threshold']), axis=1)
            downward = np.sum(np.isfinite(ds) & (dz < -p['edge_threshold']), axis=1)
        else:
            upward = downward = np.zeros(len(z))
        weights = np.where(valid, np.exp(-2 * (ds / p['radius'])**2), 0.)
        count = valid.sum(axis=1)
        delta = (weights * dz).sum(axis=1) / np.maximum(weights.sum(axis=1), 1.)
        editable = (count >= 4) & ~((upward >= 2) & (downward >= 2))
        # Vertical structures also have sparse or nearly collinear XY support.
        center_xy = q[sl, :2].astype(float)
        xy = q[np.minimum(ii, len(q) - 1), :2].astype(float) - center_xy[:, None]
        cov_x = (weights * xy[:, :, 0]**2).sum(axis=1)
        cov_y = (weights * xy[:, :, 1]**2).sum(axis=1)
        cov_xy = (weights * xy[:, :, 0] * xy[:, :, 1]).sum(axis=1)
        trace = cov_x + cov_y
        editable &= (trace > 0) & ((cov_x * cov_y - cov_xy**2) > .02 * trace**2)
        delta = np.clip(delta * p['strength'], -p['max_move'], p['max_move'])
        output[sl, :2] = q[sl, :2]
        output[sl, 2] = z + np.where(editable, delta, 0.)
        report(.90 * sl.stop / len(q))
    return output


# 由拟合平面的法向、中心与目标高度构造 4×4 调平变换；旋转和平移将在应用阶段写给结果。
def _level_matrix(normal, center, target_height):
    # Rodrigues in a form stable for a normal already facing +Z.
    vertical = np.array([0., 0., 1.])
    x, y, z = np.cross(normal, vertical)
    k = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    rotation = np.eye(3) + k + k @ k / (1. + normal[2])
    # Rotate around the fitted reference center to avoid drifting large XY origins.
    shift = center - rotation @ center
    shift[2] += target_height - center[2]
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, shift
    return transform


# 沿平面局部网格寻找有边界证据的小孔，从米制间距与洞大小限制得到补点；超预算的网格跳过并计数。
def _fill(q, all_points, p, check, report, allocate, plan):
    spacing, max_hole = p['spacing'], p['max_hole']
    remaining = p['max_added']
    additions = []
    skipped_large_grids = 0
    measured = None
    uv_buffer = cells_buffer = None
    # A hole smaller than one grid cell has no defensible output location.
    if max_hole < spacing or not remaining:
        return additions, skipped_large_grids
    for plane_no, (normal, center, inds) in enumerate(_plane_models(
            q, p['plane_tolerance'], 20, 8, check, allocate)):
        check()
        # Release the prior grid before checking or allocating this plane's grid.
        occupied = labels = sizes = None
        # Pick a global axis least parallel to the normal, to avoid a degenerate basis.
        axis = np.eye(3)[np.argmin(np.abs(normal))]
        u = axis - normal * (axis @ normal)
        u /= np.linalg.norm(u)
        v = np.cross(normal, u)
        if uv_buffer is None:
            uv_buffer = allocate((len(q), 2), np.float64)
            cells_buffer = allocate((len(q), 2), np.intp)
        uv, cells = uv_buffer[:len(inds)], cells_buffer[:len(inds)]
        basis = np.column_stack((u, v))
        origin = np.full(2, np.inf)
        top = np.full(2, -np.inf)
        for sl in _blocks(len(inds)):
            check()
            uv[sl] = (q[inds[sl]] - center) @ basis
            origin = np.minimum(origin, uv[sl].min(axis=0))
            top = np.maximum(top, uv[sl].max(axis=0))
        # A grid with the original sample phase avoids shifting an exact regular grid.
        ratio = (top - origin) / spacing
        if np.any(ratio >= MAX_POINTS):
            skipped_large_grids += 1
            continue
        shape = tuple(int(x) + 3 for x in np.floor(ratio + .5))
        workspace = plan['estimated_bytes'] + (80 * len(all_points) if measured is not None else 0)
        # Flood, labels, bounding boxes and region counts all require RAM.
        grid_bytes = 96 * shape[0] * shape[1]
        require_memory('fill', workspace + grid_bytes, plan['budget_bytes'])
        occupied = np.zeros(shape, dtype=bool)
        for sl in _blocks(len(inds)):
            check()
            cells[sl] = np.floor((uv[sl] - origin) / spacing + .5).astype(np.intp) + 1
            occupied[cells[sl, 0], cells[sl, 1]] = True
        check()
        # 8-connected exterior flood prevents closing diagonal doorway gaps.
        holes = ndimage.binary_fill_holes(occupied, structure=np.ones((3, 3), bool)) & ~occupied
        labels, count = ndimage.label(holes, structure=np.ones((3, 3), bool))
        del holes
        if not count:
            report(.9 * (plane_no + 1) / 8)
            continue
        sizes = np.bincount(labels.ravel())
        for label, bounds in enumerate(ndimage.find_objects(labels), 1):
            if label % 256 == 1:
                check()
            if bounds is None or sizes[label] > remaining:
                continue
            spans = np.array([d.stop - d.start for d in bounds]) * spacing
            if np.max(spans) > max_hole or max_hole**2 < float(sizes[label]) * spacing**2:
                continue
            expanded = tuple(slice(d.start - 1, d.stop + 1) for d in bounds)
            patch = labels[expanded] == label
            # Direct neighboring measured cells must enclose the physical gap.
            ring = ndimage.binary_dilation(patch, structure=np.ones((3, 3), bool)) & ~patch
            if occupied[expanded][ring].mean() < .75:
                continue
            ar = np.column_stack(np.nonzero(patch)) + np.array([d.start for d in expanded])
            # Exact diameter of a convex enclosing rectangle is conservative.
            # For a single cell this is intentionally one cell, not a diagonal.
            if np.hypot(*(np.maximum(np.ptp(ar, axis=0), 0) * spacing)) + spacing > max_hole + 1e-9:
                continue
            xy = (ar - 1) * spacing + origin
            guess = (center + xy[:, :1] * u + xy[:, 1:] * v).astype(np.float32)
            if not np.all(np.isfinite(guess)):
                continue
            if all_points is not None:
                if measured is None:
                    require_memory('fill', workspace + grid_bytes + 80 * len(all_points), plan['budget_bytes'])
                    measured = cKDTree(all_points)
                # A selection omission does not make a hole in the source cloud.
                if np.any(measured.query(guess, distance_upper_bound=.5 * spacing)[0] < .5 * spacing):
                    continue
            additions.append(guess)
            remaining -= len(ar)
            if not remaining:
                return additions, skipped_large_grids
        report(.9 * (plane_no + 1) / 8)
    return additions, skipped_large_grids


# 校验操作、参数和选择掩码，按操作分派去噪、调平或补洞；分配、进度和取消经传入接口执行，输出供文档预览或应用。
def apply_operation(points, operation, parameters=None, selection=None, cancel=None, progress=None,
                    allocator=None, memory_budget=None, compute_mode='auto'):
    """Apply one repair, with a progress callback taking a fraction in [0, 1].

    Unselected points are never modified, except that level applies its fitted
    rigid transform to the entire cloud. Ground reports a mask without editing.
    The caller owns any arrays returned by allocator(shape, dtype, fill=None).
    memory_budget overrides the physical RAM estimate for residual workspace.
    """
    from .repair_compute import ComputeSession
    compute = ComputeSession(compute_mode)
    p = _parameters(operation, parameters)
    # 检查当前操作；检查协作取消标记。
    def check():
        if cancel is not None and (cancel.is_set() if hasattr(cancel, 'is_set') else cancel()):
            raise RepairCancelled('点云修复已取消')
    # 报告当前操作；输入为 value，调用 progress 处理。
    def report(value):
        if progress is not None:
            progress(float(value))
    check()
    src = np.asarray(points)
    if src.ndim != 2 or src.shape[1] != 3 or src.dtype != np.float32:
        raise ValueError('点云必须是 N×3 float32 坐标')
    n = len(src)
    # 提供 allocate 信息；输入为 shape、dtype、fill，调用 _allocate 处理。
    def allocate(shape, dtype, fill=None):
        return _allocate(shape, dtype, fill, check, allocator)
    if selection is not None:
        selection = np.asarray(selection)
        if selection.dtype != bool or selection.shape != (n,):
            raise ValueError('选区必须是一维布尔掩码，并与点云等长')
    selected_count = 0 if selection is not None else n
    for sl in _blocks(n):
        check()
        if not np.all(np.isfinite(src[sl])):
            raise ValueError('点云包含非有限坐标')
        if selection is not None:
            selected_count += np.count_nonzero(selection[sl])
    work_count = selected_count
    if ((operation == 'radius' and selected_count <= p['min_neighbors'])
            or (operation == 'statistical' and selected_count <= p['neighbors'])
            or (operation == 'flatten' and (selected_count < 4 or not p['strength'] or not p['max_move']))
            or (operation == 'planes' and selected_count < p['min_points'])
            or (operation == 'fill' and (selected_count < 20 or p['max_hole'] < p['spacing'] or not p['max_added']))):
        work_count = 0
    plan = memory_plan(operation, n, work_count, memory_budget)
    if allocator is None:
        disk_scratch = {'radius': 1, 'statistical': 17, 'components': 2, 'ground': 10,
                        'level': 10, 'flatten': 12, 'planes': 37, 'fill': 69}
        plan['estimated_bytes'] += ((14 + (operation == 'ground')) * n
                                    + (disk_scratch[operation] + (20 if selection is not None else 0))
                                    * selected_count)
    require_memory(operation, **plan)
    indices = None
    q = src
    if selection is not None:
        indices = _selected_rows(selection, selected_count, allocate, check)
        q = allocate((selected_count, 3), np.float32)
        for sl in _blocks(selected_count):
            check()
            q[sl] = src[indices[sl]]
    output = allocate(src.shape, np.float32)
    _copy_into(output, src, check)
    keep = allocate(n, bool, True)
    moved = allocate(n, bool)
    added = np.empty((0, 3), dtype=np.float32)
    stats = {'input_points': n, 'selected_points': len(q)}
    if operation in ('ground', 'level'):
        stats.update(ground_found=False, ground_points=0)
    if operation == 'planes':
        stats['planes_found'] = 0
    if operation == 'fill':
        stats['fill_skipped_large_grids'] = 0
    if operation == 'ground':
        stats['ground_mask'] = allocate(n, bool, False)
    elif operation == 'level':
        stats['transform'] = np.eye(4)
    report(0.)
    if len(q):
        if operation in ('radius', 'statistical'):
            _copy_into(keep, _point_denoise(q, operation, p, check, report, allocate, compute), check, indices)
        elif operation == 'components':
            _copy_into(keep, _components(q, p, check, report, allocate), check, indices)
        elif operation in ('ground', 'level'):
            slope = p['max_slope_deg']
            seed = p.get('seed', 0)
            model = _find_plane(q, p['distance'], max(3, min(30, int(np.ceil(.08 * len(q))))),
                                np.random.default_rng(seed), check, allocate, slope)
            stats['ground_found'] = model is not None
            if model is not None:
                normal, center, support = model
                stats.update(ground_points=len(support), ground_normal=normal,
                             ground_center=center)
                if operation == 'ground':
                    for sl in _blocks(len(support)):
                        check()
                        part = support[sl]
                        stats['ground_mask'][part if indices is None else indices[part]] = True
                else:
                    result = _level_matrix(normal, center, p['target_height'])
                    stats['transform'] = result
                    rotation, shift = result[:3, :3], result[:3, 3]
                    if not compute.transform(src, output, rotation, shift, check, report):
                        for sl in _blocks(n):
                            check()
                            output[sl] = src[sl] @ rotation.T + shift
                            report(.90 * sl.stop / n)
        elif operation == 'flatten':
            if len(q) >= 4 and p['strength'] and p['max_move']:
                _copy_into(output, _flatten(q, p, check, report, allocate), check, indices)
        elif operation == 'planes':
            for j, (normal, center, inds) in enumerate(_plane_models(
                    q, p['distance'], p['min_points'], p['max_planes'], check, allocate), 1):
                stats['planes_found'] = j
                for sl in _blocks(len(inds)):
                    check()
                    part = inds[sl]
                    original = q[part]
                    ds = np.clip(((original - center) @ normal) * p['strength'],
                                 -p['max_move'], p['max_move'])
                    output[part if selection is None else indices[part]] = original - ds[:, None] * normal
                report(.90 * j / p['max_planes'])
        elif operation == 'fill':
            parts, skipped = _fill(q, src if selection is not None else None, p, check, report, allocate, plan)
            if parts:
                added = allocate((sum(map(len, parts)), 3), np.float32)
                pos = 0
                for part in parts:
                    _copy_into(added[pos:pos + len(part)], part, check)
                    pos += len(part)
            stats['fill_skipped_large_grids'] = skipped
    kept_count = moved_count = 0
    for sl in _blocks(n):
        check()
        if not np.all(np.isfinite(output[sl])):
            raise ValueError('修复结果超出 float32 坐标范围')
        moved[sl] = keep[sl] & np.any(output[sl] != src[sl], axis=1)
        kept_count += np.count_nonzero(keep[sl])
        moved_count += np.count_nonzero(moved[sl])
    stats.update(removed_points=int(n - kept_count), moved_points=int(moved_count), added_points=len(added),
                 compute=compute.summary())
    check()
    report(1.)
    check()
    return RepairResult(output, keep, moved, added, stats)
