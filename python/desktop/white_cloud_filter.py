# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 从支持的地图或导出文件收集点，清理不合格或稀疏样本，并为预览或另存提供结果；不修改输入点云。
"""Remove locally unsupported points without changing the source."""
from contextlib import closing
from datetime import datetime
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from uuid import uuid4

import numpy as np
from scipy.spatial import cKDTree

from .field_geometry import _cancelled, _open_db, _sqlite_path
from .white_mesh_io import iter_exported_points

_MAX_POINTS = 5_000_000
_BATCH = 65_536
_SAMPLE_MAX = 15_000


# 载入点集；输入为 source、cancel，从磁盘读取文本，非法输入或处理失败时抛出异常。
def _load_points(source, cancel):
    from .white_mesh_core import _archive_chunks

    # 提供 collect 信息；输入为 chunks，按轴拼接数组，非法输入或处理失败时抛出异常。
    def collect(chunks):
        pieces = []
        total = valid = 0
        for chunk in chunks:
            _cancelled(cancel)
            total += len(chunk)
            if total > _MAX_POINTS:
                raise ValueError(f'孤点清理最多读取 {_MAX_POINTS:,} 点，请先裁剪点云。')
            xyz = np.asarray(chunk, dtype=np.float32)
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            valid += len(xyz)
            pieces.append(xyz)
        return np.concatenate(pieces) if pieces else np.empty((0, 3), np.float32), total, total-valid

    path = Path(source)
    if path.suffix.lower() in ('.ply', '.pcd'):
        with closing(iter_exported_points(path)) as stream:
            return collect(stream)

    dbfile = _sqlite_path(path).resolve(strict=True)
    checkpoint = dbfile.with_name('checkpoint.json')
    if not checkpoint.is_file():
        raise ValueError('存档缺少结束记录，请先在建图软件中正常结束会话')
    state = json.loads(checkpoint.read_text(encoding='utf-8'))
    status = state.get('status', '') if isinstance(state, dict) else ''
    if not isinstance(status, str) or not (status.startswith('已结束') or status in (
            '建图已结束，地图已保留', '软件关闭，建图已结束', '离线重算结束', '结束保存')):
        raise ValueError('孤点清理仅支持已正常结束的分块存档')
    with closing(_open_db(dbfile)) as db:
        db.set_progress_handler(lambda: int(cancel is not None and cancel.is_set()), 1000)
        try:
            db.execute('BEGIN')
            count = db.execute('SELECT COALESCE(SUM(n),0) FROM blocks').fetchone()[0]
            if count > _MAX_POINTS:
                raise ValueError(f'孤点清理最多读取 {_MAX_POINTS:,} 点，请先裁剪点云。')
            return collect(_archive_chunks(db, cancel))
        except Exception:
            _cancelled(cancel)
            raise


# 提供清理输入点云信息；输入为 source、radius_cm、min_neighbors，构建 KD 树进行空间查询，非法输入或处理失败时抛出异常。
def clean_source(source, radius_cm, min_neighbors, cancel=None, progress=None):
    """Return all retained XYZ, a sample of discarded XYZ, and counts.

    Radius uses centimetres; neighbours are input points other than the query
    point, with distance <= radius. Non-finite input is counted separately.
    """
    _cancelled(cancel)
    try:
        radius = float(radius_cm)
        count = float(min_neighbors)
    except (TypeError, ValueError, OverflowError):
        raise ValueError('请填写有效的搜索半径和最少邻点数') from None
    if not np.isfinite(radius) or not .001 <= radius <= 100_000:
        raise ValueError('搜索半径须为 0.001–100,000 厘米')
    if not count.is_integer() or not 0 <= count <= 100:
        raise ValueError('最少邻点数须为 0–100 的整数')
    k = int(count)
    notify = progress if progress is not None else lambda _: None
    notify('正在只读载入原始点云…')
    points, total, invalid = _load_points(source, cancel)
    _cancelled(cancel)
    sample = np.empty((0, 3), dtype=np.float32)
    n = len(points)
    removed = 0
    if k and n:
        keep = np.zeros(n, dtype=np.bool_)
        if n > k:
            notify(f'正在建立 {n:,} 点的邻域索引…')
            tree = cKDTree(points)
            r = radius / 100
            # Keep a single returned neighbour rank per input point and check
            # inclusive radius ourselves (SciPy's search bound is exclusive).
            hi = r + max(r*1e-12, np.finfo(np.float64).tiny)
            for start in range(0, n, _BATCH):
                _cancelled(cancel)
                stop = min(start+_BATCH, n)
                distances, _ = tree.query(points[start:stop], k=[k+1], distance_upper_bound=hi)
                keep[start:stop] = distances[:, 0] <= r
                notify(f'正在检查邻点：{stop:,} / {n:,}')
            del tree
        _cancelled(cancel)
        removed = n-int(keep.sum())
        if removed:
            dropped = np.flatnonzero(~keep)
            pick = np.linspace(0, removed-1, min(removed, _SAMPLE_MAX), dtype=np.intp)
            sample = points[dropped[pick]]
            points = points[keep]
    _cancelled(cancel)
    return points, sample, dict(input_points=total, invalid_points=invalid,
                                kept_points=len(points), removed_points=removed,
                                radius_cm=radius, min_neighbors=k)


# 导出 clean ；输入为 parent_dir、source、points，将文本写入磁盘，返回 result，非法输入或处理失败时抛出异常。
def export_clean(parent_dir, source, points, stats, cancel=None):
    """Publish filtered coordinate-only PLY and provenance as a new directory."""
    _cancelled(cancel)
    parent = Path(parent_dir).resolve()
    src = Path(source).resolve()
    protected = ([] if src.is_file() and src.name != 'blocks.sqlite' else
                 [src.parent if src.name == 'blocks.sqlite' else src])
    protected.extend(p for p in src.parents if p.suffix.lower() == '.map' or (p/'blocks.sqlite').is_file())
    if any(parent.is_relative_to(p) for p in protected):
        raise ValueError('不能把清理结果写入源地图目录，请另选位置。')
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points):
        raise ValueError('没有可导出的清理后点云。')
    header = ('ply\nformat binary_little_endian 1.0\n'
              f'element vertex {len(points)}\n'
              'property float x\nproperty float y\nproperty float z\nend_header\n')
    parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix='.clean_points_', dir=parent) as staging:
        staged = Path(staging)/'payload'
        staged.mkdir()
        with (staged/'cleaned.ply').open('wb') as out:
            out.write(header.encode('ascii'))
            for start in range(0, len(points), _BATCH):
                _cancelled(cancel)
                chunk = points[start:start+_BATCH].astype('<f4', copy=False)
                if not np.isfinite(chunk).all():
                    raise ValueError('导出点云存在非有限坐标。')
                out.write(chunk.tobytes(order='C'))
        meta = {'source': str(src), 'units': 'm', **stats}
        (staged/'filter.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2, allow_nan=False)+'\n',
                                          encoding='utf8')
        result = parent/f'clean_points_{datetime.now():%Y%m%d_%H%M%S}_{uuid4().hex}'
        _cancelled(cancel)
        if result.exists():
            raise FileExistsError(result)
        staged.rename(result)
    return result
