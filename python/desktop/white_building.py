# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 用点云结构推断建筑平面和面片，生成白模网格；在工作坐标中分析结构与孔洞，再交由网格导出层写文件。
"""Evidence-based planar building surfaces from cleaned Z-up map points, in metres."""
import numpy as np
from scipy import ndimage

from .white_mesh_core import _cancelled, _normals, _report, _voxelize

_MAX_RASTER = 1_000_000
_MAX_PLANES_PER_AXIS = 32
_MAX_MESH_FACES = 600_000


# 提供帧信息；输入为 normals、good，调用 abs、eye 处理。
def _frame(normals, good):
    """Fit one shared orthogonal wall frame, using only confidently upright normals."""
    n = normals[good & (np.abs(normals[:, 2]) < .29)]
    if len(n) < 12:
        return np.eye(3)
    # +/- normals and the two perpendicular wall directions have the same folded angle.
    folded = np.mod(np.arctan2(n[:, 1], n[:, 0]), np.pi / 2)
    bins = 90
    ids = (folded * (bins / (np.pi / 2))).astype(int) % bins
    hist = np.bincount(ids, minlength=bins)
    mode = int(np.argmax(sum(np.roll(hist, k) for k in range(-4, 5))))
    dist = (folded - (mode+.5) * (np.pi / 2 / bins) + np.pi/4) % (np.pi/2) - np.pi/4
    coherent = folded[np.abs(dist) <= np.deg2rad(9)]
    c, s = np.cos(4*coherent).mean(), np.sin(4*coherent).mean()
    angle = np.arctan2(s,c) / 4
    t = np.cos(angle)
    q = np.sin(angle)
    return np.array([[t, -q, 0.], [q, t, 0.], [0., 0., 1.]])


# 提供平面信息；输入为 p、aligned、axis，以中位数描述典型值，逐批交付结果，调用方可按需迭代。
def _planes(p, aligned, axis, step, cancel):
    """Vote by surface samples and separate supported parallel plane offsets."""
    local = p[aligned, axis]
    if len(local) < 20:
        return
    width = min(.065, max(.025, step * .45))
    gap = width * 2.1
    origin = float(np.min(local))
    ids = np.floor((local-origin) / width).astype(np.int64)
    # Sparse coordinates stay sparse even when a remnant is far from the room.
    key, counts = np.unique(ids, return_counts=True)
    votes = counts.copy()
    near = key[1:] == key[:-1] + 1
    votes[:-1] += near * counts[1:]
    votes[1:] += near * counts[:-1]
    offsets = []
    for j in np.argsort(-votes, kind='stable'):
        _cancelled(cancel)
        if votes[j] < 16 or len(offsets) == _MAX_PLANES_PER_AXIS:
            return
        center = origin + (key[j] + .5) * width
        if any(abs(center - old) <= gap for old in offsets):
            continue
        members = local[np.abs(local - center) <= width*1.25]
        if len(members) < 16:
            continue
        plane = float(np.median(members))
        if any(abs(plane - old) <= gap for old in offsets):
            continue
        offsets.append(plane)
        yield plane, min(.10, width*1.45)


# 补齐 small_holes ；输入为 mask，调用 pad、label 处理。
def _fill_small_holes(mask):
    """Close an interior gap only with surrounding evidence; never fill the exterior."""
    missing = ~np.pad(mask, 1, constant_values=False)
    labels, _ = ndimage.label(missing, structure=np.ones((3,3), dtype=int))
    counts = np.bincount(labels.ravel())
    fill = counts <= 3
    fill[0] = fill[labels[0,0]] = False
    return mask | fill[labels[1:-1,1:-1]]


# 提供矩形掩码信息；输入为 xy、evidence、spacing，逐批交付结果，调用方可按需迭代，非法输入或处理失败时抛出异常，调用 unique、astype 处理。
def _rect_masks(xy, evidence, spacing, horizontal, cancel, depth=0):
    """Rasterize the observed plane, returning only large connected surface regions."""
    cell = np.unique(np.floor(xy/spacing).astype(np.int64), axis=0)
    if len(cell) < 12:
        return
    lo, hi = cell.min(axis=0), cell.max(axis=0)
    shape = hi - lo + 1
    if int(shape[0])*int(shape[1]) > _MAX_RASTER:
        # Split widely separated regions before rasterizing, without densifying the gap.
        gaps = []
        for axis in range(2):
            ids = np.unique(cell[:,axis])
            dif = np.diff(ids)
            if len(dif):
                j = int(np.argmax(dif))
                if dif[j] > 3:
                    gaps.append((int(dif[j]),axis,int(ids[j])))
        if not gaps or depth >= 24:
            raise ValueError('建筑平面范围过大，请增大构面间距或先裁剪地图')
        _, a, t = max(gaps)
        b = xy[:,a] < (t+1)*spacing
        yield from _rect_masks(xy[b],evidence[evidence[:,a] < (t+1)*spacing],spacing,horizontal,cancel,depth+1)
        yield from _rect_masks(xy[~b],evidence[evidence[:,a] >= (t+1)*spacing],spacing,horizontal,cancel,depth+1)
        return
    _cancelled(cancel)
    mask = np.zeros(tuple(shape), dtype=bool)
    coords = cell-lo
    mask[coords[:,0],coords[:,1]] = True
    e=np.floor(evidence/spacing).astype(np.int64)-lo
    e=e[((e >= 0) & (e < shape)).all(axis=1)]
    direct=np.zeros_like(mask)
    direct[e[:,0],e[:,1]]=True
    # Collect strip coverage only close to points having the expected surface normal.
    mask &= ndimage.binary_dilation(direct,iterations=2,structure=np.ones((3,3)))
    mask = _fill_small_holes(mask)
    labels, n = ndimage.label(mask)
    if not n:
        return
    counts = np.bincount(labels.ravel())
    minimum = 1.8 if horizontal else 1.25
    for label, section in enumerate(ndimage.find_objects(labels), 1):
        _cancelled(cancel)
        if counts[label]*spacing**2 < minimum:
            continue
        span = [(s.stop-s.start)*spacing for s in section]
        if span[0] < .70 or span[1] < (.70 if horizontal else 1.05):
            continue
        region = labels[section] == label
        supported = int(np.count_nonzero(region & direct[section]))
        if supported < max(12, counts[label]*.20):
            continue
        yield np.argwhere(region) + lo + [s.start for s in section]


# 提供表面信息；输入为 local、normals、good，把输入统一为数组，返回 result。
def _surfaces(local, normals, good, raw_local, grid, sample, cancel, progress):
    result = []
    wall_vertical = []
    for axis in (0,1,2):
        _report(progress,f'正在拟合{"水平面" if axis==2 else "竖直墙面"}…')
        horizontal = axis == 2
        cols = [a for a in range(3) if a!=axis]
        aligned = good & (np.abs(normals[:,axis]) >= .943)
        for offset, tol in _planes(local, aligned, axis, sample, cancel):
            support = np.abs(raw_local[:,axis] - offset) <= tol
            if horizontal and wall_vertical:
                lower,upper = np.asarray(wall_vertical).T
                bottom,top = np.percentile(lower,20),np.percentile(upper,80)
                if min(abs(offset-bottom),abs(offset-top)) > .42:
                    continue
            direct = aligned & (np.abs(local[:,axis] - offset) <= tol)
            for cells in _rect_masks(raw_local[support][:,cols],local[direct][:,cols],grid,horizontal,cancel):
                result.append((axis,offset,cols,cells))
                if not horizontal:
                    wall_vertical.append((cells[:,1].min()*grid,(cells[:,1].max()+1)*grid))
    return result


# 提供网格信息；输入为 surfaces、grid、frame，按轴拼接数组，非法输入或处理失败时抛出异常。
def _mesh(surfaces, grid, frame, origin, cancel):
    vertices, faces = [], []
    count = total = 0
    # Snap a plane boundary towards a nearby orthogonal observed plane
    # only within 0.6 cell, and only where both measured patches overlap.
    bounds = []
    occupancy = [set(map(tuple, s[3].tolist())) for s in surfaces]
    for a, off, cols, cells in surfaces:
        lower, upper = np.zeros(3), np.zeros(3)
        lower[a] = upper[a] = off
        lower[cols] = cells.min(axis=0)*grid
        upper[cols] = (cells.max(axis=0)+1)*grid
        bounds.append((lower,upper))
    for i,(axis,offset,cols,cells) in enumerate(surfaces):
        _cancelled(cancel)
        total += 2*len(cells)
        if total>_MAX_MESH_FACES:
            raise ValueError('建筑网格超过面数上限，请增大构面间距')
        corners = cells[:,None,:] + [[0,0],[1,0],[1,1],[0,1]]
        unique, ix = np.unique(corners.reshape(-1,2),axis=0,return_inverse=True)
        ix = ix.reshape(-1,4)
        xyz = np.empty((len(unique),3))
        xyz[:,axis] = offset
        xyz[:,cols] = unique*grid
        # Only use measured neighbour patches for snaps, not their plane equations alone.
        original = xyz.copy()
        for local_index,a in enumerate(cols):
            other_in_plane = cols[1-local_index]
            # A raster can overhang a measured boundary by one cell. Include
            # its nearest grid line as well as outward boundary vertices.
            best = np.full(len(xyz), grid*.501)
            for j,(b,adjacent,other_cols,ncells) in enumerate(surfaces):
                if b != a:
                    continue
                low,high = bounds[j]
                if offset < low[axis]-grid*.58 or offset > high[axis]+grid*.58:
                    continue
                side = np.minimum(original[:,a]-bounds[i][0][a],bounds[i][1][a]-original[:,a]) <= grid*1.01
                avail = side & (np.abs(original[:,a]-adjacent)>1e-8) & (np.abs(original[:,a]-adjacent)<best)
                if not avail.any():
                    continue
                # Require the adjacent surface raster to cover the corner's
                # location within one cell. This avoids snapping across doorways.
                rows = np.flatnonzero(avail)
                desired = np.floor(original[rows][:,other_cols]/grid).astype(np.int64)
                occ = occupancy[j]
                supported = np.zeros(len(rows),bool)
                for d0 in (-1,0,1):
                    for d1 in (-1,0,1):
                        supported |= np.fromiter((tuple(y) in occ for y in desired+[d0,d1]),dtype=bool,count=len(rows))
                rows = rows[supported]
                xyz[rows,a] = adjacent
                best[rows] = np.abs(original[rows,a]-adjacent)
        pair = np.concatenate([ix[:,[0,1,2]],ix[:,[0,2,3]]])
        if axis == 1:
            pair = pair[:,[0,2,1]]
        vertices.append(xyz);faces.append(pair+count);count += len(xyz)
    if not faces:
        return np.empty((0,3)), np.empty((0,3),np.int32)
    p = np.concatenate(vertices)
    # Common intersection vertices share topology when the fitted coordinates agree.
    _, first, inverse = np.unique(np.round(p,6),axis=0,return_index=True,return_inverse=True)
    v = p[first]
    f = inverse[np.concatenate(faces)]
    area = np.cross(v[f[:,1]]-v[f[:,0]], v[f[:,2]]-v[f[:,0]])
    f = f[np.linalg.norm(area,axis=1)>1e-8]
    return v @ frame.T + origin, f.astype(np.int32)


# 构建建筑；输入为 points、step_m、cancel，以中位数描述典型值，按序返回 v、f、stats，非法输入或处理失败时抛出异常。
def build_building(points, step_m, cancel=None, progress=None):
    """Infer the dominant upright walls and observed horizontal building surfaces."""
    _cancelled(cancel)
    points=np.asarray(points)
    if points.ndim!=2 or points.shape[1]!=3:
        raise ValueError('建筑白模需要 N × 3 点云坐标')
    step=float(step_m)
    if not np.isfinite(step) or not .02 <= step <= .50:
        raise ValueError('建筑白模间距须为 2–50 cm')
    grid=max(.10,step)
    sample=max(.04,min(.14,step))
    _report(progress,'正在读取已保留点云并分析建筑主方向…')
    chunks=(points[i:i+65536] for i in range(0,len(points),65536))
    _, p, count, invalid=_voxelize(chunks,sample,cancel,progress)
    normals, confidence=_normals(p,sample,cancel)
    good=confidence>=0
    frame=_frame(normals,good)
    origin=np.median(p,axis=0) if len(p) else np.zeros(3)
    _, coverage, _, _ = _voxelize((points[i:i+65536] for i in range(0,len(points),65536)),
                                     max(.025,grid*.5),cancel,None)
    surfaces=_surfaces((p-origin) @ frame,normals @ frame,good,(coverage-origin) @ frame,grid,sample,cancel,progress)
    _report(progress,'正在合并连续建筑网格…')
    v,f=_mesh(surfaces,grid,frame,origin,cancel)
    _cancelled(cancel)
    stats=dict(mode='building',units='m',step_m=step,building_grid_m=grid,input_points=count,
               invalid_points=invalid,vertex_count=len(v),face_count=len(f),
               wall_surfaces=sum(a!=2 for a,*_ in surfaces),
               horizontal_surfaces=sum(a==2 for a,*_ in surfaces))
    return v,f,stats
