# SPDX-License-Identifier: LicenseRef-T100-NonCommercial-1.0
# 非商业源码使用条款见仓库 LICENSE；第三方依赖按其各自许可证。
# 解析地图与 PCD/PLY 点数据并将白模导出到 GLB 或 OBJ；集中管理轴向和格式字段，源地图只读。
"""Read this application's exported point clouds and write independent mesh results."""
from concurrent.futures import CancelledError
from dataclasses import asdict, is_dataclass
import json
import os
from pathlib import Path
import shutil
import struct
import tempfile
from datetime import datetime
from uuid import uuid4

import numpy as np


_CHUNK = 65536
_XYZ = ("property float x", "property float y", "property float z")
_RGB = ("property uchar red", "property uchar green", "property uchar blue")


# 处理文件头文本行相关状态；输入为 stream，逐批交付结果，调用方可按需迭代，非法输入或处理失败时抛出异常，调用 readline、endswith 处理。
def _header_lines(stream):
    remaining = 16384
    while remaining > 0:
        raw = stream.readline(remaining)
        remaining -= len(raw)
        if not raw.endswith(b"\n"):
            break
        try:
            yield raw.decode("ascii").strip()
        except UnicodeDecodeError as exc:
            raise ValueError("点云文件头不是 ASCII 文本") from exc
    raise ValueError("点云文件头不完整或过长")


# 提供点数量信息；输入为 value，返回 int(value)，非法输入或处理失败时抛出异常。
def _point_count(value):
    if len(value) > 20 or not value.isdecimal():
        raise ValueError("点云文件头中的点数无效")
    return int(value)


# 解析PLY；输入为 lines，非法输入或处理失败时抛出异常，调用 startswith、append 处理。
def _parse_ply(lines):
    head = []
    for line in lines:
        if line == "end_header":
            break
        if not line.startswith(("comment ", "obj_info ")):
            head.append(line)
    if (len(head) < 6 or head[:2] != ["ply", "format binary_little_endian 1.0"]
            or not head[2].startswith("element vertex ")
            or tuple(head[3:]) not in (_XYZ, _XYZ + _RGB)):
        raise ValueError("不支持此 PLY 格式；请选择本软件导出的二进制 XYZ 或 XYZ/RGB PLY")
    return _point_count(head[2].removeprefix("element vertex ")), 12 if tuple(head[3:]) == _XYZ else 15


# 解析PCD；输入为 lines，非法输入或处理失败时抛出异常，调用 startswith、split 处理。
def _parse_pcd(lines):
    head = {}
    for line in lines:
        if not line or line.startswith("#"):
            continue
        key, *value = line.split()
        if key in head:
            raise ValueError("PCD 文件头包含重复字段")
        head[key] = value
        if key == "DATA":
            break
    expected = {"VERSION": ["0.7"], "FIELDS": ["x", "y", "z"], "SIZE": ["4"] * 3,
                "TYPE": ["F"] * 3, "COUNT": ["1"] * 3, "DATA": ["binary"], "HEIGHT": ["1"]}
    if any(head.get(key) != value for key, value in expected.items()):
        raise ValueError("不支持此 PCD 格式；请选择本软件导出的二进制 XYZ PCD")
    if len(head.get("POINTS", [])) != 1 or head.get("WIDTH") != head.get("POINTS"):
        raise ValueError("PCD 文件头中的点数或宽度无效")
    return _point_count(head["POINTS"][0]), 12


# 确定点来源；输入为 path，返回 path，非法输入或处理失败时抛出异常。
def resolve_point_source(path):
    """Resolve an archive or exported result directory without falling back to its original map."""
    path = Path(path)
    if path.is_dir():
        if (path / 'blocks.sqlite').is_file():
            return path
        for name in ('cloud.ply', 'cloud.pcd', 'map.ply', 'map.pcd'):
            if (path / name).is_file():
                return path / name
        raise ValueError('地图目录中未找到 blocks.sqlite、cloud.ply / cloud.pcd 或 map.ply / map.pcd')
    if not path.is_file():
        raise FileNotFoundError(f'地图或点云不存在：{path}')
    if path.name != 'blocks.sqlite' and path.suffix.lower() not in ('.ply', '.pcd'):
        raise ValueError('请选择分块地图目录或二进制 PLY / PCD 点云')
    return path


# 处理 iter_exported_points 相关状态；输入为 path，逐批交付结果，调用方可按需迭代，非法输入或处理失败时抛出异常，调用 Path、lower 处理。
def iter_exported_points(path):
    """Yield independent float32 XYZ batches from a PLY/PCD export."""
    path = Path(path)
    parser = {".ply": _parse_ply, ".pcd": _parse_pcd}.get(path.suffix.lower())
    if parser is None:
        raise ValueError("只支持本软件导出的二进制 PLY 或 PCD 点云文件")
    with path.open("rb") as stream:
        count, stride = parser(_header_lines(stream))
        size = os.fstat(stream.fileno()).st_size - stream.tell()
        if size < count * stride:
            raise ValueError("点云文件已截断：实际数据少于文件头声明的点数")
        if size != count * stride:
            raise ValueError("点云数据大小与文件头声明的格式或点数不一致")
        for offset in range(0, count, _CHUNK):
            rows = min(count - offset, _CHUNK)
            raw = stream.read(rows * stride)
            if len(raw) != rows * stride:
                raise ValueError("点云文件在读取期间被截断")
            yield np.ndarray((rows, 3), dtype="<f4", buffer=raw,
                             strides=(stride, 4)).astype(np.float32, copy=True, order="C")


# 检查当前操作；输入为 cancel，检查协作取消标记。
def _check(cancel):
    if cancel is not None and (cancel() if callable(cancel) else cancel.is_set()):
        raise CancelledError("白模导出已取消")


# 处理分块相关状态；输入为 array、cancel，逐批交付结果，调用方可按需迭代，调用 _check 处理。
def _chunks(array, cancel):
    for offset in range(0, len(array), _CHUNK):
        _check(cancel)
        yield array[offset:offset + _CHUNK]


# 提供 y_up 信息；输入为 points，返回 out。
def _y_up(points):
    out = np.empty(points.shape, dtype="<f4")
    out[:, 0] = points[:, 0]
    out[:, 1] = points[:, 2]
    out[:, 2] = -points[:, 1]
    return out


_COLOR = (0.82, 0.82, 0.82)


# 写入GLB；输入为 path、vertices、faces，序列化为 JSON 文本，非法输入或处理失败时抛出异常。
def _write_glb(path, vertices, faces, lo, hi, cancel):
    xyz_len, index_len = vertices.size * 4, faces.size * 4
    document = {
        "asset": {"version": "2.0", "generator": "T100 White Mesh"},
        "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1, "material": 0}]}],
        "materials": [{"name": "White", "doubleSided": True, "pbrMetallicRoughness": {
            "baseColorFactor": [*_COLOR, 1.0], "metallicFactor": 0.0, "roughnessFactor": 0.9}}],
        "buffers": [{"byteLength": xyz_len + index_len}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": xyz_len, "target": 34962},
            {"buffer": 0, "byteOffset": xyz_len, "byteLength": index_len, "target": 34963}],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": len(vertices), "type": "VEC3",
             "min": [float(lo[0]), float(lo[2]), -float(hi[1])],
             "max": [float(hi[0]), float(hi[2]), -float(lo[1])]},
            {"bufferView": 1, "componentType": 5125, "count": faces.size, "type": "SCALAR"}],
    }
    encoded = json.dumps(document, separators=(",", ":"), allow_nan=False).encode("utf-8")
    encoded += b" " * (-len(encoded) % 4)
    total = 28 + len(encoded) + xyz_len + index_len
    if total > 0xffffffff:
        raise ValueError("模型超出 GLB 文件大小上限")
    with path.open("wb") as stream:
        stream.write(struct.pack("<4sIII4s", b"glTF", 2, total, len(encoded), b"JSON"))
        stream.write(encoded)
        stream.write(struct.pack("<I4s", xyz_len + index_len, b"BIN\0"))
        for batch in _chunks(vertices, cancel):
            stream.write(_y_up(batch).tobytes())
        for batch in _chunks(faces, cancel):
            stream.write(batch.astype("<u4", copy=False).tobytes())


# 写入OBJ；输入为 directory、vertices、faces，将文本写入磁盘。
def _write_obj(directory, vertices, faces, cancel):
    with (directory / "model.obj").open("w", encoding="ascii", newline="\n") as stream:
        stream.write("# Map coordinates: Z up, meters\nmtllib model.mtl\no WhiteMesh\n")
        for batch in _chunks(vertices, cancel):
            np.savetxt(stream, batch, fmt="v %.9g %.9g %.9g")
        stream.write("usemtl White\n")
        for batch in _chunks(faces, cancel):
            np.savetxt(stream, batch.astype(np.int64, copy=False) + 1, fmt="f %d %d %d")
    (directory / "model.mtl").write_text(
        "newmtl White\nKa 0.1 0.1 0.1\nKd 0.82 0.82 0.82\nKs 0 0 0\nd 1\nillum 1\n",
        encoding="ascii")


# 提供 json_value 信息；输入为 value，返回 asdict(value)，非法输入或处理失败时抛出异常。
def _json_value(value):
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return asdict(value)
    raise TypeError(f"无法保存白模统计字段：{type(value).__name__}")


# 导出模型；输入为 parent_dir、source、vertices，创建独立临时目录，返回 result，非法输入或处理失败时抛出异常。
def export_model(parent_dir, source, vertices, faces, stats, cancel=None):
    """Export a fresh mesh directory; publish it only after every file is ready."""
    _check(cancel)
    parent = Path(parent_dir).resolve()
    source = Path(source).resolve()
    is_file = source.is_file() or source.suffix.lower() in {".ply", ".pcd"}
    source_dirs = [] if is_file else [source]
    source_dirs.extend(folder for folder in source.parents
                       if folder.suffix.lower() == ".map" or (folder / "blocks.sqlite").is_file())
    if any(parent.is_relative_to(folder) for folder in source_dirs):
        raise ValueError("白模结果不能写入源地图目录，请选择其他目录")
    vertices = np.asarray(vertices)
    faces = np.asarray(faces)
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) == 0
            or vertices.dtype.kind not in "fiu"
            or faces.ndim != 2 or faces.shape[1] != 3 or len(faces) == 0
            or faces.dtype.kind not in "iu" or len(vertices) > 0xffffffff):
        raise ValueError("白模需要非空三维顶点及三角形整数索引")
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for batch in _chunks(vertices, cancel):
        if not np.isfinite(batch).all() or np.max(np.abs(batch.astype(np.float64))) > np.finfo(np.float32).max:
            raise ValueError("白模顶点包含非有限值或超出 GLB 坐标范围")
        lo = np.minimum(lo, batch.min(axis=0))
        hi = np.maximum(hi, batch.max(axis=0))
    # POSITION bounds describe the stored float32 values.
    lo, hi = lo.astype(np.float32), hi.astype(np.float32)
    for batch in _chunks(faces, cancel):
        if batch.min() < 0 or batch.max() >= len(vertices):
            raise ValueError("白模三角形索引超出顶点范围")
    _check(cancel)
    parent.mkdir(parents=True, exist_ok=True)
    staged = Path(tempfile.mkdtemp(prefix=".white_mesh_", dir=parent))
    try:
        _write_glb(staged / "model.glb", vertices, faces, lo, hi, cancel)
        _write_obj(staged, vertices, faces, cancel)
        meta = {"source": str(source), "units": "m", "glb_up_axis": "Y", "obj_up_axis": "Z",
                "glb_from_map": "(x, z, -y)", "vertex_count": len(vertices),
                "triangle_count": len(faces), "stats": stats}
        with (staged / "metadata.json").open("w", encoding="utf-8") as stream:
            json.dump(meta, stream, default=_json_value, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        result = parent / f"white_mesh_{datetime.now():%Y%m%d_%H%M%S}_{uuid4().hex}"
        _check(cancel)
        if result.exists():
            raise FileExistsError(result)
        staged.rename(result)
        return result
    finally:
        if staged.exists():
            shutil.rmtree(staged)
