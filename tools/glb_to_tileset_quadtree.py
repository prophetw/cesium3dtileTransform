#!/usr/bin/env python3
from __future__ import annotations

import argparse
import array
import json
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


GLB_MAGIC = b"glTF"
GLB_VERSION_SUPPORTED = 2

CHUNK_TYPE_JSON = 0x4E4F534A  # b"JSON"
CHUNK_TYPE_BIN = 0x004E4942  # b"BIN\0"

TRIANGLES_MODE = 4

TARGET_ARRAY_BUFFER = 34962
TARGET_ELEMENT_ARRAY_BUFFER = 34963

COMPONENT_TYPE_INT8 = 5120
COMPONENT_TYPE_UINT8 = 5121
COMPONENT_TYPE_INT16 = 5122
COMPONENT_TYPE_UINT16 = 5123
COMPONENT_TYPE_UINT32 = 5125
COMPONENT_TYPE_FLOAT32 = 5126

IMAGE_MIME_TO_EXT: dict[str, str] = {
    "image/png": ".png",
    "image/jpeg": ".jpg",
    "image/jpg": ".jpg",
    "image/webp": ".webp",
    "image/gif": ".gif",
    "image/ktx2": ".ktx2",
}

COMPONENT_TYPE_BYTE_SIZE: dict[int, int] = {
    COMPONENT_TYPE_INT8: 1,
    COMPONENT_TYPE_UINT8: 1,
    COMPONENT_TYPE_INT16: 2,
    COMPONENT_TYPE_UINT16: 2,
    COMPONENT_TYPE_UINT32: 4,
    COMPONENT_TYPE_FLOAT32: 4,
}

INDEX_COMPONENT_TYPES: dict[int, tuple[str, int]] = {
    COMPONENT_TYPE_UINT8: ("B", 1),
    COMPONENT_TYPE_UINT16: ("H", 2),
    COMPONENT_TYPE_UINT32: ("I", 4),
}

TYPE_COMPONENT_COUNT: dict[str, int] = {
    "SCALAR": 1,
    "VEC2": 2,
    "VEC3": 3,
    "VEC4": 4,
    "MAT2": 4,
    "MAT3": 9,
    "MAT4": 16,
}


class GlbError(RuntimeError):
    pass


@dataclass(frozen=True)
class Aabb:
    min_xyz: tuple[float, float, float]
    max_xyz: tuple[float, float, float]

    @staticmethod
    def empty() -> "Aabb":
        inf = float("inf")
        return Aabb((inf, inf, inf), (-inf, -inf, -inf))

    def is_empty(self) -> bool:
        return any(self.min_xyz[i] > self.max_xyz[i] for i in range(3))

    def union(self, other: "Aabb") -> "Aabb":
        if self.is_empty():
            return other
        if other.is_empty():
            return self
        return Aabb(
            (
                min(self.min_xyz[0], other.min_xyz[0]),
                min(self.min_xyz[1], other.min_xyz[1]),
                min(self.min_xyz[2], other.min_xyz[2]),
            ),
            (
                max(self.max_xyz[0], other.max_xyz[0]),
                max(self.max_xyz[1], other.max_xyz[1]),
                max(self.max_xyz[2], other.max_xyz[2]),
            ),
        )

    def corners(self) -> list[tuple[float, float, float]]:
        min_x, min_y, min_z = self.min_xyz
        max_x, max_y, max_z = self.max_xyz
        return [
            (min_x, min_y, min_z),
            (min_x, min_y, max_z),
            (min_x, max_y, min_z),
            (min_x, max_y, max_z),
            (max_x, min_y, min_z),
            (max_x, min_y, max_z),
            (max_x, max_y, min_z),
            (max_x, max_y, max_z),
        ]

    def center_and_half_extents(self, min_half_extent: float = 0.0) -> tuple[
        tuple[float, float, float], tuple[float, float, float]
    ]:
        min_x, min_y, min_z = self.min_xyz
        max_x, max_y, max_z = self.max_xyz
        center = ((min_x + max_x) / 2, (min_y + max_y) / 2, (min_z + max_z) / 2)
        half = ((max_x - min_x) / 2, (max_y - min_y) / 2, (max_z - min_z) / 2)
        if min_half_extent > 0:
            half = (
                max(half[0], min_half_extent),
                max(half[1], min_half_extent),
                max(half[2], min_half_extent),
            )
        return center, half


@dataclass(frozen=True)
class NodeInfo:
    node_index: int
    aabb: Aabb
    triangle_count: int
    center_xy: tuple[float, float]


@dataclass(frozen=True)
class NewAccessorBlob:
    accessor: dict[str, Any]
    data: bytes
    target: int | None


@dataclass
class QuadTile:
    depth: int
    x: int
    y: int
    bounds_xy: tuple[float, float, float, float]
    aabb: Aabb
    triangle_count: int
    children: list["QuadTile"]
    node_indices: list[int] | None
    content_uri: str | None = None

    def is_leaf(self) -> bool:
        return not self.children


def _mat4_identity() -> list[float]:
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


def _mat4_multiply(a: list[float], b: list[float]) -> list[float]:
    out = [0.0] * 16
    for col in range(4):
        for row in range(4):
            out[col * 4 + row] = (
                a[0 * 4 + row] * b[col * 4 + 0]
                + a[1 * 4 + row] * b[col * 4 + 1]
                + a[2 * 4 + row] * b[col * 4 + 2]
                + a[3 * 4 + row] * b[col * 4 + 3]
            )
    return out


def _mat4_translation(t: Iterable[float]) -> list[float]:
    tx, ty, tz = t
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, tx, ty, tz, 1]


def _mat4_scale(s: Iterable[float]) -> list[float]:
    sx, sy, sz = s
    return [sx, 0, 0, 0, 0, sy, 0, 0, 0, 0, sz, 0, 0, 0, 0, 1]


def _mat4_rotation_from_quaternion(q: Iterable[float]) -> list[float]:
    x, y, z, w = q

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    m00 = 1 - 2 * (yy + zz)
    m01 = 2 * (xy + wz)
    m02 = 2 * (xz - wy)

    m10 = 2 * (xy - wz)
    m11 = 1 - 2 * (xx + zz)
    m12 = 2 * (yz + wx)

    m20 = 2 * (xz + wy)
    m21 = 2 * (yz - wx)
    m22 = 1 - 2 * (xx + yy)

    return [
        m00,
        m10,
        m20,
        0,
        m01,
        m11,
        m21,
        0,
        m02,
        m12,
        m22,
        0,
        0,
        0,
        0,
        1,
    ]


def _mat4_from_node(node: dict[str, Any]) -> list[float]:
    if "matrix" in node:
        matrix = node["matrix"]
        if not (isinstance(matrix, list) and len(matrix) == 16):
            raise GlbError("Invalid node.matrix, expected 16 numbers")
        return [float(v) for v in matrix]

    translation = node.get("translation", [0, 0, 0])
    rotation = node.get("rotation", [0, 0, 0, 1])
    scale = node.get("scale", [1, 1, 1])

    if len(translation) != 3 or len(rotation) != 4 or len(scale) != 3:
        raise GlbError("Invalid node TRS fields")

    t = _mat4_translation([float(v) for v in translation])
    r = _mat4_rotation_from_quaternion([float(v) for v in rotation])
    s = _mat4_scale([float(v) for v in scale])
    return _mat4_multiply(t, _mat4_multiply(r, s))


def _mat4_transform_point(m: list[float], p: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = p
    tx = m[0] * x + m[4] * y + m[8] * z + m[12]
    ty = m[1] * x + m[5] * y + m[9] * z + m[13]
    tz = m[2] * x + m[6] * y + m[10] * z + m[14]
    tw = m[3] * x + m[7] * y + m[11] * z + m[15]
    if tw not in (0.0, 1.0):
        tx /= tw
        ty /= tw
        tz /= tw
    return (tx, ty, tz)


def read_glb(glb_path: Path) -> tuple[dict[str, Any], bytes]:
    data = glb_path.read_bytes()
    if len(data) < 12:
        raise GlbError("Invalid GLB: file too small")

    magic, version, total_length = struct.unpack_from("<4sII", data, 0)
    if magic != GLB_MAGIC:
        raise GlbError("Invalid GLB: bad magic")
    if version != GLB_VERSION_SUPPORTED:
        raise GlbError(f"Unsupported GLB version: {version} (expected {GLB_VERSION_SUPPORTED})")
    if total_length != len(data):
        raise GlbError("Invalid GLB: length mismatch")

    json_chunk: bytes | None = None
    bin_chunk: bytes | None = None

    offset = 12
    while offset < total_length:
        if offset + 8 > total_length:
            raise GlbError("Invalid GLB: truncated chunk header")
        chunk_length, chunk_type = struct.unpack_from("<II", data, offset)
        offset += 8
        if offset + chunk_length > total_length:
            raise GlbError("Invalid GLB: truncated chunk data")
        chunk_data = data[offset : offset + chunk_length]
        offset += chunk_length

        if chunk_type == CHUNK_TYPE_JSON and json_chunk is None:
            json_chunk = chunk_data
        elif chunk_type == CHUNK_TYPE_BIN and bin_chunk is None:
            bin_chunk = chunk_data

    if json_chunk is None:
        raise GlbError("Invalid GLB: missing JSON chunk")
    if bin_chunk is None:
        raise GlbError("Invalid GLB: missing BIN chunk")

    try:
        gltf = json.loads(json_chunk.decode("utf-8"))
    except Exception as exc:  # noqa: BLE001 - surface parse failure as GlbError
        raise GlbError(f"Invalid GLB JSON chunk: {exc}") from exc

    if not isinstance(gltf, dict):
        raise GlbError("Invalid GLB: JSON root is not an object")

    return gltf, bin_chunk


def write_glb(glb_path: Path, gltf: dict[str, Any], bin_chunk: bytes) -> None:
    json_bytes = json.dumps(gltf, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
    json_padding = (4 - len(json_bytes) % 4) % 4
    if json_padding:
        json_bytes += b" " * json_padding

    bin_padding = (4 - len(bin_chunk) % 4) % 4
    if bin_padding:
        bin_chunk += b"\x00" * bin_padding

    total_length = 12 + 8 + len(json_bytes) + 8 + len(bin_chunk)
    header = struct.pack("<4sII", GLB_MAGIC, GLB_VERSION_SUPPORTED, total_length)
    json_header = struct.pack("<II", len(json_bytes), CHUNK_TYPE_JSON)
    bin_header = struct.pack("<II", len(bin_chunk), CHUNK_TYPE_BIN)

    glb_path.write_bytes(header + json_header + json_bytes + bin_header + bin_chunk)


def _read_accessor_aabb_from_min_max(accessor: dict[str, Any]) -> Aabb | None:
    if "min" not in accessor or "max" not in accessor:
        return None
    min_v = accessor["min"]
    max_v = accessor["max"]
    if not (isinstance(min_v, list) and isinstance(max_v, list) and len(min_v) == 3 and len(max_v) == 3):
        return None
    return Aabb((float(min_v[0]), float(min_v[1]), float(min_v[2])), (float(max_v[0]), float(max_v[1]), float(max_v[2])))


def _read_accessor_vec3_f32_aabb(
    *,
    accessor_index: int,
    gltf: dict[str, Any],
    bin_chunk: bytes,
) -> Aabb:
    accessors = gltf.get("accessors", [])
    buffer_views = gltf.get("bufferViews", [])

    if not (0 <= accessor_index < len(accessors)):
        raise GlbError(f"Accessor index out of range: {accessor_index}")
    accessor = accessors[accessor_index]

    if accessor.get("type") != "VEC3":
        raise GlbError(f"Unsupported accessor.type for POSITION: {accessor.get('type')} (expected VEC3)")
    if accessor.get("componentType") != COMPONENT_TYPE_FLOAT32:
        raise GlbError(
            f"Unsupported accessor.componentType for POSITION: {accessor.get('componentType')} (expected {COMPONENT_TYPE_FLOAT32})"
        )
    if "sparse" in accessor:
        raise GlbError("Sparse accessors are not supported in V3")

    count = accessor.get("count")
    if not isinstance(count, int) or count <= 0:
        raise GlbError("Invalid accessor.count for POSITION")

    buffer_view_index = accessor.get("bufferView")
    if not isinstance(buffer_view_index, int):
        raise GlbError("Accessor.bufferView missing for POSITION (and no min/max provided)")
    if not (0 <= buffer_view_index < len(buffer_views)):
        raise GlbError(f"bufferView index out of range: {buffer_view_index}")
    buffer_view = buffer_views[buffer_view_index]

    buffer_index = buffer_view.get("buffer", 0)
    if buffer_index != 0:
        raise GlbError("Only buffer 0 (GLB BIN chunk) is supported in V3")

    element_size = TYPE_COMPONENT_COUNT["VEC3"] * 4
    stride = buffer_view.get("byteStride", element_size)
    if not isinstance(stride, int) or stride < element_size:
        raise GlbError("Invalid bufferView.byteStride for POSITION")

    base_offset = int(buffer_view.get("byteOffset", 0)) + int(accessor.get("byteOffset", 0))
    total_bytes_needed = base_offset + (count - 1) * stride + element_size
    if total_bytes_needed > len(bin_chunk):
        raise GlbError("Accessor points outside BIN chunk")

    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = -float("inf")

    for i in range(count):
        offset = base_offset + i * stride
        x, y, z = struct.unpack_from("<fff", bin_chunk, offset)
        min_x = min(min_x, x)
        min_y = min(min_y, y)
        min_z = min(min_z, z)
        max_x = max(max_x, x)
        max_y = max(max_y, y)
        max_z = max(max_z, z)

    return Aabb((min_x, min_y, min_z), (max_x, max_y, max_z))


def _read_indices(
    *,
    accessor_index: int,
    gltf: dict[str, Any],
    bin_chunk: bytes,
) -> tuple[list[int], int]:
    accessors = gltf.get("accessors", [])
    buffer_views = gltf.get("bufferViews", [])

    if not (0 <= accessor_index < len(accessors)):
        raise GlbError(f"Accessor index out of range: {accessor_index}")
    accessor = accessors[accessor_index]

    if accessor.get("type") != "SCALAR":
        raise GlbError(f"Unsupported indices accessor.type: {accessor.get('type')}")
    component_type = accessor.get("componentType")
    if component_type not in INDEX_COMPONENT_TYPES:
        raise GlbError(f"Unsupported indices componentType: {component_type}")
    if "sparse" in accessor:
        raise GlbError("Sparse accessors are not supported in V3")

    count = accessor.get("count")
    if not isinstance(count, int) or count <= 0:
        raise GlbError("Invalid indices accessor.count")

    buffer_view_index = accessor.get("bufferView")
    if not isinstance(buffer_view_index, int):
        raise GlbError("Indices accessor.bufferView missing")
    if not (0 <= buffer_view_index < len(buffer_views)):
        raise GlbError(f"bufferView index out of range: {buffer_view_index}")
    buffer_view = buffer_views[buffer_view_index]

    buffer_index = buffer_view.get("buffer", 0)
    if buffer_index != 0:
        raise GlbError("Only buffer 0 (GLB BIN chunk) is supported in V3")

    fmt, element_size = INDEX_COMPONENT_TYPES[component_type]
    stride = buffer_view.get("byteStride", element_size)
    if not isinstance(stride, int) or stride < element_size:
        raise GlbError("Invalid bufferView.byteStride for indices")

    base_offset = int(buffer_view.get("byteOffset", 0)) + int(accessor.get("byteOffset", 0))
    total_bytes_needed = base_offset + (count - 1) * stride + element_size
    if total_bytes_needed > len(bin_chunk):
        raise GlbError("Indices accessor points outside BIN chunk")

    indices: list[int] = []
    unpack_fmt = "<" + fmt
    for i in range(count):
        offset = base_offset + i * stride
        value = struct.unpack_from(unpack_fmt, bin_chunk, offset)[0]
        indices.append(int(value))

    return indices, component_type


def _sample_triangles(indices: list[int], ratio: float) -> list[int]:
    triangle_count = len(indices) // 3
    if triangle_count <= 1 or ratio >= 1.0:
        return indices
    target = max(1, int(triangle_count * ratio))
    if target >= triangle_count:
        return indices

    out: list[int] = []
    step = triangle_count / target
    last_tri = -1
    for i in range(target):
        tri = int(i * step)
        if tri <= last_tri:
            tri = last_tri + 1
        if tri >= triangle_count:
            tri = triangle_count - 1
        last_tri = tri
        start = tri * 3
        out.extend(indices[start : start + 3])
    return out


def _pack_indices(indices: list[int], component_type: int) -> bytes:
    if component_type == COMPONENT_TYPE_UINT8:
        typecode = "B"
    elif component_type == COMPONENT_TYPE_UINT16:
        typecode = "H"
    elif component_type == COMPONENT_TYPE_UINT32:
        typecode = "I"
    else:
        raise GlbError(f"Unsupported indices componentType: {component_type}")

    arr = array.array(typecode, indices)
    if sys.byteorder != "little":
        arr.byteswap()
    return arr.tobytes()


def _choose_index_component_type(max_index: int) -> int:
    if max_index <= 0xFF:
        return COMPONENT_TYPE_UINT8
    if max_index <= 0xFFFF:
        return COMPONENT_TYPE_UINT16
    return COMPONENT_TYPE_UINT32


def _accessor_element_size(accessor: dict[str, Any]) -> int:
    component_type = accessor.get("componentType")
    if not isinstance(component_type, int):
        raise GlbError("Accessor.componentType missing")
    component_size = COMPONENT_TYPE_BYTE_SIZE.get(component_type)
    if component_size is None:
        raise GlbError(f"Unsupported accessor.componentType: {component_type}")
    type_name = accessor.get("type")
    if type_name not in TYPE_COMPONENT_COUNT:
        raise GlbError(f"Unsupported accessor.type: {type_name}")
    return TYPE_COMPONENT_COUNT[type_name] * component_size


def _extract_accessor_elements_bytes(
    *,
    accessor_index: int,
    element_indices: list[int],
    gltf: dict[str, Any],
    bin_chunk: bytes,
) -> bytes:
    accessors = gltf.get("accessors", [])
    buffer_views = gltf.get("bufferViews", [])

    if not element_indices:
        return b""
    if not (0 <= accessor_index < len(accessors)):
        raise GlbError(f"Accessor index out of range: {accessor_index}")
    accessor = accessors[accessor_index]
    if "sparse" in accessor:
        raise GlbError("Sparse accessors are not supported in V3")

    count = accessor.get("count")
    if not isinstance(count, int) or count < 0:
        raise GlbError("Invalid accessor.count")
    max_index = max(element_indices)
    if max_index >= count:
        raise GlbError("Accessor element index out of range")

    buffer_view_index = accessor.get("bufferView")
    if not isinstance(buffer_view_index, int):
        raise GlbError("Accessor.bufferView missing")
    if not (0 <= buffer_view_index < len(buffer_views)):
        raise GlbError(f"bufferView index out of range: {buffer_view_index}")
    buffer_view = buffer_views[buffer_view_index]

    buffer_index = buffer_view.get("buffer", 0)
    if buffer_index != 0:
        raise GlbError("Only buffer 0 (GLB BIN chunk) is supported in V3")

    element_size = _accessor_element_size(accessor)
    stride = buffer_view.get("byteStride", element_size)
    if not isinstance(stride, int) or stride < element_size:
        raise GlbError("Invalid bufferView.byteStride for accessor")

    base_offset = int(buffer_view.get("byteOffset", 0)) + int(accessor.get("byteOffset", 0))
    total_bytes_needed = base_offset + max_index * stride + element_size
    if total_bytes_needed > len(bin_chunk):
        raise GlbError("Accessor points outside BIN chunk")

    out = bytearray()
    for element_index in element_indices:
        offset = base_offset + element_index * stride
        out.extend(bin_chunk[offset : offset + element_size])
    return bytes(out)


def _vec3_f32_min_max(data: bytes) -> tuple[list[float], list[float]]:
    if len(data) % 12 != 0:
        raise GlbError("Invalid VEC3 float32 buffer length")
    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = -float("inf")
    for offset in range(0, len(data), 12):
        x, y, z = struct.unpack_from("<fff", data, offset)
        min_x = min(min_x, x)
        min_y = min(min_y, y)
        min_z = min(min_z, z)
        max_x = max(max_x, x)
        max_y = max(max_y, y)
        max_z = max(max_z, z)
    return [min_x, min_y, min_z], [max_x, max_y, max_z]


def _compact_vertex_indices(indices: list[int]) -> tuple[list[int], list[int]]:
    vertex_indices: list[int] = []
    remap: dict[int, int] = {}
    remapped: list[int] = []
    for idx in indices:
        new_idx = remap.get(idx)
        if new_idx is None:
            new_idx = len(vertex_indices)
            remap[idx] = new_idx
            vertex_indices.append(idx)
        remapped.append(new_idx)
    return vertex_indices, remapped


def _sniff_image_ext(data: bytes) -> str:
    if data.startswith(b"\x89PNG\r\n\x1a\n"):
        return ".png"
    if data.startswith(b"\xff\xd8\xff"):
        return ".jpg"
    if data.startswith(b"RIFF") and len(data) >= 12 and data[8:12] == b"WEBP":
        return ".webp"
    if data.startswith(b"GIF87a") or data.startswith(b"GIF89a"):
        return ".gif"
    if data.startswith(b"\xabKTX 20\xbb\r\n\x1a\n"):
        return ".ktx2"
    return ".bin"


def _sanitize_filename(value: str) -> str:
    keep: list[str] = []
    for ch in value.strip():
        if ch.isalnum() or ch in ("-", "_", "."):
            keep.append(ch)
        elif ch.isspace():
            keep.append("_")
    out = "".join(keep).strip("._")
    return out[:64] if out else ""


def extract_glb_images(
    gltf: dict[str, Any],
    bin_chunk: bytes,
    *,
    out_dir: Path,
    uri_prefix: str,
) -> dict[int, str]:
    images = gltf.get("images", [])
    buffer_views = gltf.get("bufferViews", [])

    out_dir.mkdir(parents=True, exist_ok=True)
    mapping: dict[int, str] = {}

    for image_index, image in enumerate(images):
        if not isinstance(image, dict):
            raise GlbError("Invalid glTF image entry")
        if "uri" in image:
            continue
        buffer_view_index = image.get("bufferView")
        if buffer_view_index is None:
            continue
        if not isinstance(buffer_view_index, int) or not (0 <= buffer_view_index < len(buffer_views)):
            raise GlbError("Invalid image.bufferView")
        buffer_view = buffer_views[buffer_view_index]
        if buffer_view.get("buffer", 0) != 0:
            raise GlbError("Only buffer 0 (GLB BIN chunk) is supported in V3")

        byte_offset = int(buffer_view.get("byteOffset", 0))
        byte_length = int(buffer_view.get("byteLength", 0))
        if byte_offset + byte_length > len(bin_chunk):
            raise GlbError("image bufferView points outside BIN chunk")
        data = bin_chunk[byte_offset : byte_offset + byte_length]

        mime_type = image.get("mimeType")
        if isinstance(mime_type, str) and mime_type in IMAGE_MIME_TO_EXT:
            ext = IMAGE_MIME_TO_EXT[mime_type]
        else:
            ext = _sniff_image_ext(data)

        name = ""
        if isinstance(image.get("name"), str):
            name = _sanitize_filename(image["name"])
        filename = f"img_{image_index:03d}{'_' + name if name else ''}{ext}"
        (out_dir / filename).write_bytes(data)
        mapping[image_index] = f"{uri_prefix}{filename}"

    return mapping


def compute_mesh_aabb(
    *,
    mesh_index: int,
    gltf: dict[str, Any],
    bin_chunk: bytes,
    accessor_aabb_cache: dict[int, Aabb],
    mesh_aabb_cache: dict[int, Aabb],
) -> Aabb:
    meshes = gltf.get("meshes", [])

    if mesh_index in mesh_aabb_cache:
        return mesh_aabb_cache[mesh_index]
    if not (0 <= mesh_index < len(meshes)):
        raise GlbError(f"Mesh index out of range: {mesh_index}")
    mesh = meshes[mesh_index]
    primitives = mesh.get("primitives", [])
    if not primitives:
        raise GlbError(f"Mesh {mesh_index} has no primitives")

    def get_accessor_aabb(accessor_index: int) -> Aabb:
        if accessor_index in accessor_aabb_cache:
            return accessor_aabb_cache[accessor_index]
        accessors = gltf.get("accessors", [])
        if not (0 <= accessor_index < len(accessors)):
            raise GlbError(f"Accessor index out of range: {accessor_index}")
        accessor = accessors[accessor_index]
        aabb = _read_accessor_aabb_from_min_max(accessor)
        if aabb is None:
            aabb = _read_accessor_vec3_f32_aabb(accessor_index=accessor_index, gltf=gltf, bin_chunk=bin_chunk)
        accessor_aabb_cache[accessor_index] = aabb
        return aabb

    mesh_aabb = Aabb.empty()
    for primitive in primitives:
        attributes = primitive.get("attributes", {})
        position_accessor_index = attributes.get("POSITION")
        if position_accessor_index is None:
            continue
        if not isinstance(position_accessor_index, int):
            raise GlbError("Invalid POSITION accessor index")
        mesh_aabb = mesh_aabb.union(get_accessor_aabb(position_accessor_index))

    if mesh_aabb.is_empty():
        raise GlbError(f"Mesh {mesh_index} has no POSITION attributes")

    mesh_aabb_cache[mesh_index] = mesh_aabb
    return mesh_aabb


def compute_mesh_triangle_count(
    *,
    mesh_index: int,
    gltf: dict[str, Any],
    mesh_tri_cache: dict[int, int],
) -> int:
    meshes = gltf.get("meshes", [])
    accessors = gltf.get("accessors", [])

    if mesh_index in mesh_tri_cache:
        return mesh_tri_cache[mesh_index]
    if not (0 <= mesh_index < len(meshes)):
        raise GlbError(f"Mesh index out of range: {mesh_index}")
    mesh = meshes[mesh_index]
    primitives = mesh.get("primitives", [])
    if not primitives:
        raise GlbError(f"Mesh {mesh_index} has no primitives")

    total_tris = 0
    for primitive in primitives:
        mode = primitive.get("mode", TRIANGLES_MODE)
        if mode != TRIANGLES_MODE:
            continue
        indices_accessor_index = primitive.get("indices")
        if indices_accessor_index is not None:
            if not isinstance(indices_accessor_index, int):
                raise GlbError("Invalid indices accessor index")
            count = accessors[indices_accessor_index].get("count")
        else:
            pos_accessor_index = primitive.get("attributes", {}).get("POSITION")
            if not isinstance(pos_accessor_index, int):
                raise GlbError("Primitive has no POSITION for non-indexed triangles")
            count = accessors[pos_accessor_index].get("count")
        if not isinstance(count, int) or count <= 0:
            raise GlbError("Invalid accessor.count for triangles")
        total_tris += count // 3

    mesh_tri_cache[mesh_index] = total_tris
    return total_tris


def compute_scene_nodes(
    gltf: dict[str, Any],
    bin_chunk: bytes,
) -> tuple[list[NodeInfo], list[int], dict[int, int], set[int]]:
    scenes = gltf.get("scenes", [])
    nodes = gltf.get("nodes", [])

    if not scenes or not nodes:
        raise GlbError("No scenes/nodes in GLB")

    scene_index = gltf.get("scene", 0)
    if not isinstance(scene_index, int) or not (0 <= scene_index < len(scenes)):
        raise GlbError(f"Invalid gltf.scene: {scene_index}")

    scene = scenes[scene_index]
    root_node_indices = scene.get("nodes", [])
    if not root_node_indices:
        raise GlbError("Scene has no root nodes")

    parent_map: dict[int, int] = {}
    for node_index, node in enumerate(nodes):
        for child_index in node.get("children", []):
            if not isinstance(child_index, int):
                raise GlbError("Invalid child node index")
            parent_map[child_index] = node_index

    accessor_aabb_cache: dict[int, Aabb] = {}
    mesh_aabb_cache: dict[int, Aabb] = {}
    mesh_tri_cache: dict[int, int] = {}

    def traverse(node_index: int, parent_matrix: list[float]) -> Iterable[tuple[int, list[float]]]:
        if not (0 <= node_index < len(nodes)):
            raise GlbError(f"Node index out of range: {node_index}")
        node = nodes[node_index]
        local_matrix = _mat4_from_node(node)
        world_matrix = _mat4_multiply(parent_matrix, local_matrix)
        yield node_index, world_matrix
        for child_index in node.get("children", []):
            if not isinstance(child_index, int):
                raise GlbError("Invalid child node index")
            yield from traverse(child_index, world_matrix)

    node_infos: list[NodeInfo] = []
    scene_nodes: set[int] = set()

    for root_node_index in root_node_indices:
        if not isinstance(root_node_index, int):
            raise GlbError("Invalid root node index")
        for node_index, node_world_matrix in traverse(root_node_index, _mat4_identity()):
            scene_nodes.add(node_index)
            node = nodes[node_index]
            mesh_index = node.get("mesh")
            if mesh_index is None:
                continue
            if not isinstance(mesh_index, int):
                raise GlbError("Invalid node.mesh index")
            mesh_local_aabb = compute_mesh_aabb(
                mesh_index=mesh_index,
                gltf=gltf,
                bin_chunk=bin_chunk,
                accessor_aabb_cache=accessor_aabb_cache,
                mesh_aabb_cache=mesh_aabb_cache,
            )
            tri_count = compute_mesh_triangle_count(mesh_index=mesh_index, gltf=gltf, mesh_tri_cache=mesh_tri_cache)

            transformed_aabb = Aabb.empty()
            for corner in mesh_local_aabb.corners():
                p = _mat4_transform_point(node_world_matrix, corner)
                transformed_aabb = transformed_aabb.union(Aabb(p, p))
            if transformed_aabb.is_empty():
                continue
            center, _half = transformed_aabb.center_and_half_extents()
            node_infos.append(
                NodeInfo(
                    node_index=node_index,
                    aabb=transformed_aabb,
                    triangle_count=tri_count,
                    center_xy=(center[0], center[1]),
                )
            )

    if not node_infos:
        raise GlbError("No geometry found in scene (no meshes with POSITION)")

    return node_infos, root_node_indices, parent_map, scene_nodes


def _tile_bounds_from_aabb(aabb: Aabb) -> tuple[float, float, float, float]:
    min_x, min_y, _min_z = aabb.min_xyz
    max_x, max_y, _max_z = aabb.max_xyz
    width = max_x - min_x
    height = max_y - min_y
    size = max(width, height)
    cx = (min_x + max_x) / 2
    cy = (min_y + max_y) / 2
    half = size / 2
    return (cx - half, cy - half, cx + half, cy + half)


def build_quadtree(
    nodes: list[NodeInfo],
    bounds_xy: tuple[float, float, float, float],
    *,
    depth: int,
    x: int,
    y: int,
    max_tris: int,
    max_depth: int,
) -> QuadTile:
    min_x, min_y, max_x, max_y = bounds_xy
    total_tris = sum(info.triangle_count for info in nodes)
    tile_aabb = Aabb.empty()
    for info in nodes:
        tile_aabb = tile_aabb.union(info.aabb)

    if depth >= max_depth or total_tris <= max_tris or len(nodes) <= 1:
        return QuadTile(
            depth=depth,
            x=x,
            y=y,
            bounds_xy=bounds_xy,
            aabb=tile_aabb,
            triangle_count=total_tris,
            children=[],
            node_indices=[info.node_index for info in nodes],
        )

    if min_x == max_x or min_y == max_y:
        return QuadTile(
            depth=depth,
            x=x,
            y=y,
            bounds_xy=bounds_xy,
            aabb=tile_aabb,
            triangle_count=total_tris,
            children=[],
            node_indices=[info.node_index for info in nodes],
        )

    mid_x = (min_x + max_x) / 2
    mid_y = (min_y + max_y) / 2

    quadrants: list[list[NodeInfo]] = [[], [], [], []]
    for info in nodes:
        cx, cy = info.center_xy
        east = cx >= mid_x
        north = cy >= mid_y
        if not east and not north:
            quadrants[0].append(info)  # SW
        elif east and not north:
            quadrants[1].append(info)  # SE
        elif not east and north:
            quadrants[2].append(info)  # NW
        else:
            quadrants[3].append(info)  # NE

    child_bounds = [
        (min_x, min_y, mid_x, mid_y),  # SW
        (mid_x, min_y, max_x, mid_y),  # SE
        (min_x, mid_y, mid_x, max_y),  # NW
        (mid_x, mid_y, max_x, max_y),  # NE
    ]
    child_coords = [
        (x * 2, y * 2),
        (x * 2 + 1, y * 2),
        (x * 2, y * 2 + 1),
        (x * 2 + 1, y * 2 + 1),
    ]

    children: list[QuadTile] = []
    for quad_nodes, quad_bounds, (cx, cy) in zip(quadrants, child_bounds, child_coords, strict=True):
        if not quad_nodes:
            continue
        child = build_quadtree(
            quad_nodes,
            quad_bounds,
            depth=depth + 1,
            x=cx,
            y=cy,
            max_tris=max_tris,
            max_depth=max_depth,
        )
        children.append(child)

    if not children:
        return QuadTile(
            depth=depth,
            x=x,
            y=y,
            bounds_xy=bounds_xy,
            aabb=tile_aabb,
            triangle_count=total_tris,
            children=[],
            node_indices=[info.node_index for info in nodes],
        )

    return QuadTile(
        depth=depth,
        x=x,
        y=y,
        bounds_xy=bounds_xy,
        aabb=tile_aabb,
        triangle_count=total_tris,
        children=children,
        node_indices=None,
    )


def collect_leaf_tiles(root: QuadTile) -> list[QuadTile]:
    leaves: list[QuadTile] = []
    stack = [root]
    while stack:
        tile = stack.pop()
        if tile.children:
            stack.extend(tile.children)
        else:
            leaves.append(tile)
    return leaves


def collect_nodes_with_ancestors(nodes: Iterable[int], parent_map: dict[int, int]) -> set[int]:
    keep = set(nodes)
    for node_index in list(keep):
        parent = parent_map.get(node_index)
        while parent is not None and parent not in keep:
            keep.add(parent)
            parent = parent_map.get(parent)
    return keep


def collect_root_nodes(nodes: set[int], parent_map: dict[int, int]) -> list[int]:
    roots = []
    for node_index in nodes:
        parent = parent_map.get(node_index)
        if parent is None or parent not in nodes:
            roots.append(node_index)
    return sorted(roots)


def build_subset_glb(
    gltf: dict[str, Any],
    bin_chunk: bytes,
    *,
    nodes_to_keep: set[int],
    mesh_nodes: set[int],
    root_nodes: list[int],
    simplify_ratio: float,
    rebuild_vertices: bool,
    externalize_textures: bool,
    external_image_uris: dict[int, str] | None,
) -> tuple[dict[str, Any], bytes]:
    if gltf.get("skins"):
        raise GlbError("Skins are not supported in V3")
    if gltf.get("animations"):
        raise GlbError("Animations are not supported in V3")

    buffers = gltf.get("buffers", [])
    if len(buffers) != 1:
        raise GlbError("Only single-buffer GLB is supported in V3")

    nodes = gltf.get("nodes", [])
    meshes = gltf.get("meshes", [])
    accessors = gltf.get("accessors", [])
    buffer_views = gltf.get("bufferViews", [])

    if not mesh_nodes.issubset(nodes_to_keep):
        raise GlbError("mesh_nodes must be a subset of nodes_to_keep")

    node_list = sorted(nodes_to_keep)
    node_map = {old: new for new, old in enumerate(node_list)}

    mesh_set = set()
    for node_index in sorted(mesh_nodes):
        mesh_index = nodes[node_index].get("mesh")
        if mesh_index is None:
            continue
        if not isinstance(mesh_index, int):
            raise GlbError("Invalid node.mesh index")
        mesh_set.add(mesh_index)

    mesh_list = sorted(mesh_set)
    mesh_map = {old: new for new, old in enumerate(mesh_list)}

    new_nodes: list[dict[str, Any]] = []
    for old_index in node_list:
        node = dict(nodes[old_index])
        if "children" in node:
            node["children"] = [node_map[c] for c in node["children"] if c in node_map]
            if not node["children"]:
                node.pop("children", None)
        if "mesh" in node:
            mesh_index = node.get("mesh")
            if old_index not in mesh_nodes:
                node.pop("mesh", None)
            elif mesh_index in mesh_map:
                node["mesh"] = mesh_map[mesh_index]
            else:
                node.pop("mesh", None)
        if "skin" in node:
            raise GlbError("Skins are not supported in V3")
        new_nodes.append(node)

    used_accessors: set[int] = set()
    new_accessor_blobs: list[NewAccessorBlob] = []

    def add_new_accessor_blob(*, accessor: dict[str, Any], data: bytes, target: int | None) -> tuple[str, int]:
        new_accessor_blobs.append(NewAccessorBlob(accessor=accessor, data=data, target=target))
        return ("__new__", len(new_accessor_blobs) - 1)

    new_meshes: list[dict[str, Any]] = []
    for mesh_index in mesh_list:
        mesh = meshes[mesh_index]
        primitives = mesh.get("primitives", [])
        if not primitives:
            raise GlbError(f"Mesh {mesh_index} has no primitives")

        new_mesh = dict(mesh)
        new_primitives: list[dict[str, Any]] = []
        for primitive in primitives:
            if "extensions" in primitive and "KHR_draco_mesh_compression" in primitive.get("extensions", {}):
                raise GlbError("KHR_draco_mesh_compression is not supported in V3")
            new_prim = dict(primitive)

            mode = primitive.get("mode", TRIANGLES_MODE)
            simplify_this = simplify_ratio < 1.0 and mode == TRIANGLES_MODE
            rebuild_this = rebuild_vertices and simplify_this

            attributes = primitive.get("attributes", {})
            if not isinstance(attributes, dict) or not attributes:
                raise GlbError("Primitive.attributes missing/invalid")

            indices: list[int] | None = None
            if simplify_this:
                if "indices" in primitive:
                    indices_accessor_index = primitive.get("indices")
                    if not isinstance(indices_accessor_index, int):
                        raise GlbError("Invalid indices accessor index")
                    indices, _component_type = _read_indices(
                        accessor_index=indices_accessor_index,
                        gltf=gltf,
                        bin_chunk=bin_chunk,
                    )
                else:
                    pos_accessor_index = attributes.get("POSITION")
                    if not isinstance(pos_accessor_index, int):
                        raise GlbError("Primitive has no POSITION for non-indexed triangles")
                    count = accessors[pos_accessor_index].get("count")
                    if not isinstance(count, int) or count <= 0:
                        raise GlbError("Invalid POSITION accessor.count")
                    indices = list(range(count))

                if len(indices) < 3:
                    raise GlbError("Triangle primitive has too few indices")
                indices = indices[: (len(indices) // 3) * 3]
                sampled = _sample_triangles(indices, simplify_ratio)

                if rebuild_this:
                    vertex_indices, remapped_indices = _compact_vertex_indices(sampled)

                    new_attrs: dict[str, Any] = {}
                    for name, acc_index in attributes.items():
                        if not isinstance(acc_index, int):
                            raise GlbError("Invalid attribute accessor index")
                        attr_data = _extract_accessor_elements_bytes(
                            accessor_index=acc_index,
                            element_indices=vertex_indices,
                            gltf=gltf,
                            bin_chunk=bin_chunk,
                        )
                        old_accessor = accessors[acc_index]
                        new_accessor = {
                            "componentType": old_accessor.get("componentType"),
                            "type": old_accessor.get("type"),
                            "count": len(vertex_indices),
                        }
                        if old_accessor.get("normalized"):
                            new_accessor["normalized"] = True
                        if name == "POSITION":
                            if old_accessor.get("componentType") != COMPONENT_TYPE_FLOAT32 or old_accessor.get("type") != "VEC3":
                                raise GlbError("POSITION must be VEC3 float32 for vertex rebuild")
                            min_v, max_v = _vec3_f32_min_max(attr_data)
                            new_accessor["min"] = min_v
                            new_accessor["max"] = max_v
                        new_attrs[name] = add_new_accessor_blob(accessor=new_accessor, data=attr_data, target=TARGET_ARRAY_BUFFER)
                    new_prim["attributes"] = new_attrs

                    if "targets" in primitive:
                        new_targets: list[dict[str, Any]] = []
                        for target in primitive["targets"]:
                            new_target: dict[str, Any] = {}
                            for name, acc_index in target.items():
                                if not isinstance(acc_index, int):
                                    raise GlbError("Invalid target accessor index")
                                target_data = _extract_accessor_elements_bytes(
                                    accessor_index=acc_index,
                                    element_indices=vertex_indices,
                                    gltf=gltf,
                                    bin_chunk=bin_chunk,
                                )
                                old_accessor = accessors[acc_index]
                                new_accessor = {
                                    "componentType": old_accessor.get("componentType"),
                                    "type": old_accessor.get("type"),
                                    "count": len(vertex_indices),
                                }
                                if old_accessor.get("normalized"):
                                    new_accessor["normalized"] = True
                                new_target[name] = add_new_accessor_blob(
                                    accessor=new_accessor,
                                    data=target_data,
                                    target=TARGET_ARRAY_BUFFER,
                                )
                            new_targets.append(new_target)
                        new_prim["targets"] = new_targets

                    max_index = max(remapped_indices) if remapped_indices else 0
                    component_type = _choose_index_component_type(max_index)
                    index_accessor = {
                        "componentType": component_type,
                        "type": "SCALAR",
                        "count": len(remapped_indices),
                    }
                    index_data = _pack_indices(remapped_indices, component_type)
                    new_prim["indices"] = add_new_accessor_blob(
                        accessor=index_accessor,
                        data=index_data,
                        target=TARGET_ELEMENT_ARRAY_BUFFER,
                    )
                else:
                    new_attrs = {}
                    for name, acc_index in attributes.items():
                        if not isinstance(acc_index, int):
                            raise GlbError("Invalid attribute accessor index")
                        used_accessors.add(acc_index)
                        new_attrs[name] = acc_index
                    new_prim["attributes"] = new_attrs

                    if "targets" in primitive:
                        new_targets = []
                        for target in primitive["targets"]:
                            new_target: dict[str, int] = {}
                            for name, acc_index in target.items():
                                if not isinstance(acc_index, int):
                                    raise GlbError("Invalid target accessor index")
                                used_accessors.add(acc_index)
                                new_target[name] = acc_index
                            new_targets.append(new_target)
                        new_prim["targets"] = new_targets

                    max_index = max(sampled) if sampled else 0
                    component_type = _choose_index_component_type(max_index)
                    index_accessor = {
                        "componentType": component_type,
                        "type": "SCALAR",
                        "count": len(sampled),
                    }
                    index_data = _pack_indices(sampled, component_type)
                    new_prim["indices"] = add_new_accessor_blob(
                        accessor=index_accessor,
                        data=index_data,
                        target=TARGET_ELEMENT_ARRAY_BUFFER,
                    )
            else:
                new_attrs = {}
                for name, acc_index in attributes.items():
                    if not isinstance(acc_index, int):
                        raise GlbError("Invalid attribute accessor index")
                    used_accessors.add(acc_index)
                    new_attrs[name] = acc_index
                new_prim["attributes"] = new_attrs

                if "targets" in primitive:
                    new_targets = []
                    for target in primitive["targets"]:
                        new_target: dict[str, int] = {}
                        for name, acc_index in target.items():
                            if not isinstance(acc_index, int):
                                raise GlbError("Invalid target accessor index")
                            used_accessors.add(acc_index)
                            new_target[name] = acc_index
                        new_targets.append(new_target)
                    new_prim["targets"] = new_targets

                if "indices" in primitive:
                    indices_accessor_index = primitive.get("indices")
                    if not isinstance(indices_accessor_index, int):
                        raise GlbError("Invalid indices accessor index")
                    used_accessors.add(indices_accessor_index)
                    new_prim["indices"] = indices_accessor_index
                else:
                    new_prim.pop("indices", None)

            new_primitives.append(new_prim)

        new_mesh["primitives"] = new_primitives
        new_meshes.append(new_mesh)

    used_accessors_sorted = sorted(used_accessors)
    new_accessors: list[dict[str, Any]] = []
    accessor_map: dict[int, int] = {}
    used_buffer_views: set[int] = set()
    for old_index in used_accessors_sorted:
        accessor = dict(accessors[old_index])
        if "sparse" in accessor:
            raise GlbError("Sparse accessors are not supported in V3")
        buffer_view_index = accessor.get("bufferView")
        if not isinstance(buffer_view_index, int):
            raise GlbError("Accessor.bufferView missing")
        used_buffer_views.add(buffer_view_index)
        accessor_map[old_index] = len(new_accessors)
        new_accessors.append(accessor)

    images = gltf.get("images", [])
    if not externalize_textures:
        for image in images:
            buffer_view_index = image.get("bufferView")
            if buffer_view_index is None:
                continue
            if not isinstance(buffer_view_index, int):
                raise GlbError("Invalid image.bufferView")
            used_buffer_views.add(buffer_view_index)

    used_buffer_views_sorted = sorted(used_buffer_views)
    buffer_view_map: dict[int, int] = {}
    new_buffer_views: list[dict[str, Any]] = []
    new_bin = bytearray()

    def append_aligned(data: bytes) -> int:
        pad = (4 - (len(new_bin) % 4)) % 4
        if pad:
            new_bin.extend(b"\x00" * pad)
        offset = len(new_bin)
        new_bin.extend(data)
        return offset

    for old_index in used_buffer_views_sorted:
        if not (0 <= old_index < len(buffer_views)):
            raise GlbError(f"bufferView index out of range: {old_index}")
        buffer_view = buffer_views[old_index]
        buffer_index = buffer_view.get("buffer", 0)
        if buffer_index != 0:
            raise GlbError("Only buffer 0 (GLB BIN chunk) is supported in V3")
        byte_offset = int(buffer_view.get("byteOffset", 0))
        byte_length = int(buffer_view.get("byteLength", 0))
        if byte_offset + byte_length > len(bin_chunk):
            raise GlbError("bufferView points outside BIN chunk")
        data = bin_chunk[byte_offset : byte_offset + byte_length]
        new_offset = append_aligned(data)
        new_buffer_view = dict(buffer_view)
        new_buffer_view["buffer"] = 0
        new_buffer_view["byteOffset"] = new_offset
        new_buffer_views.append(new_buffer_view)
        buffer_view_map[old_index] = len(new_buffer_views) - 1

    new_blob_buffer_views: list[int] = []
    for blob in new_accessor_blobs:
        new_offset = append_aligned(blob.data)
        buffer_view: dict[str, Any] = {
            "buffer": 0,
            "byteOffset": new_offset,
            "byteLength": len(blob.data),
        }
        if blob.target is not None:
            buffer_view["target"] = blob.target
        new_buffer_views.append(buffer_view)
        new_blob_buffer_views.append(len(new_buffer_views) - 1)

    for accessor in new_accessors:
        buffer_view_index = accessor.get("bufferView")
        if buffer_view_index is not None:
            accessor["bufferView"] = buffer_view_map[buffer_view_index]

    new_blob_base = len(new_accessors)
    for blob, buffer_view_index in zip(new_accessor_blobs, new_blob_buffer_views, strict=True):
        accessor = dict(blob.accessor)
        accessor["bufferView"] = buffer_view_index
        accessor["byteOffset"] = 0
        new_accessors.append(accessor)

    for mesh in new_meshes:
        for prim in mesh["primitives"]:
            attrs = prim.get("attributes", {})
            for name, acc_index in list(attrs.items()):
                if isinstance(acc_index, tuple) and acc_index[0] == "__new__":
                    attrs[name] = new_blob_base + acc_index[1]
                else:
                    attrs[name] = accessor_map[acc_index]
            if "targets" in prim:
                for target in prim["targets"]:
                    for name, acc_index in list(target.items()):
                        if isinstance(acc_index, tuple) and acc_index[0] == "__new__":
                            target[name] = new_blob_base + acc_index[1]
                        else:
                            target[name] = accessor_map[acc_index]
            if "indices" in prim:
                value = prim["indices"]
                if isinstance(value, tuple) and value[0] == "__new__":
                    prim["indices"] = new_blob_base + value[1]
                else:
                    prim["indices"] = accessor_map[value]

    new_images = [dict(image) for image in images]
    if externalize_textures:
        if external_image_uris is None:
            external_image_uris = {}
        for image_index, image in enumerate(new_images):
            if "uri" in image:
                continue
            buffer_view_index = image.get("bufferView")
            if buffer_view_index is None:
                continue
            if image_index not in external_image_uris:
                raise GlbError("Missing external URI for embedded image")
            image.pop("bufferView", None)
            image["uri"] = external_image_uris[image_index]
    else:
        for image in new_images:
            buffer_view_index = image.get("bufferView")
            if buffer_view_index is None:
                continue
            image["bufferView"] = buffer_view_map[buffer_view_index]

    new_gltf: dict[str, Any] = {
        "asset": dict(gltf.get("asset", {"version": "2.0"})),
        "scene": 0,
        "scenes": [{"nodes": [node_map[n] for n in root_nodes if n in node_map]}],
        "nodes": new_nodes,
        "meshes": new_meshes,
        "accessors": new_accessors,
        "bufferViews": new_buffer_views,
        "buffers": [{"byteLength": len(new_bin)}],
    }

    for key in ("materials", "textures", "samplers", "extensionsUsed", "extensionsRequired", "extensions", "extras"):
        if key in gltf:
            new_gltf[key] = gltf[key]
    if new_images:
        new_gltf["images"] = new_images

    return new_gltf, bytes(new_bin)


def aabb_to_tileset_box(aabb: Aabb, *, min_half_extent: float = 0.0) -> list[float]:
    center, half = aabb.center_and_half_extents(min_half_extent=min_half_extent)
    cx, cy, cz = center
    hx, hy, hz = half
    return [cx, cy, cz, hx, 0, 0, 0, hy, 0, 0, 0, hz]


def _finite_number(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Not a number: {value}") from exc
    if not math.isfinite(parsed):
        raise argparse.ArgumentTypeError(f"Not a finite number: {value}")
    return parsed


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a quadtree tileset with leaf tiles and a sampling-based root LOD from a GLB.",
    )
    parser.add_argument("input_glb", type=Path, help="Input .glb file")
    parser.add_argument("output_dir", type=Path, help="Output directory to write tileset.json and tiles")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("out"),
        help="Base output directory (default: out). Use '.' to keep legacy paths.",
    )

    parser.add_argument("--max-tris", type=int, default=200_000, help="Max triangles per leaf tile (default: 200000)")
    parser.add_argument("--max-depth", type=int, default=8, help="Max quadtree depth (default: 8)")
    parser.add_argument(
        "--simplify-ratio",
        type=_finite_number,
        default=0.15,
        help="Triangle sampling ratio for root LOD (default: 0.15)",
    )
    parser.add_argument(
        "--rebuild-vertices",
        default=True,
        action=argparse.BooleanOptionalAction,
        help="Rebuild vertex buffers for simplified primitives to drop unused vertices (default: True)",
    )
    parser.add_argument(
        "--external-textures",
        default=False,
        action=argparse.BooleanOptionalAction,
        help="Extract embedded images once and reference them as external files under tiles/textures (default: False)",
    )
    parser.add_argument("--refine", default="REPLACE", choices=["REPLACE", "ADD"], help="Tile refine mode (default: REPLACE)")
    parser.add_argument("--error-factor", type=_finite_number, default=0.1, help="Root geometricError = diagonal * factor (default: 0.1)")
    parser.add_argument("--geometric-error", type=_finite_number, default=None, help="Override root geometricError")
    parser.add_argument(
        "--tileset-geometric-error",
        type=_finite_number,
        default=None,
        help="Top-level tileset.geometricError (default: same as root)",
    )
    parser.add_argument(
        "--min-half-extent",
        type=_finite_number,
        default=0.0,
        help="Clamp each half-extent to at least this many meters (default: 0)",
    )
    parser.add_argument("--asset-version", default="1.1", help="tileset asset.version (default: 1.1)")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON (indent=2)")
    return parser.parse_args()


def _resolve_output_dir(output_dir: Path, output_root: Path | None) -> Path:
    if output_root is None or output_dir.is_absolute():
        return output_dir
    return output_root / output_dir


def _compute_diagonal(aabb: Aabb) -> float:
    dx = aabb.max_xyz[0] - aabb.min_xyz[0]
    dy = aabb.max_xyz[1] - aabb.min_xyz[1]
    dz = aabb.max_xyz[2] - aabb.min_xyz[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _geometric_error_for_tile(tile: QuadTile, root_error: float) -> float:
    if tile.is_leaf():
        return 0.0
    return root_error / (2**tile.depth)


def _tile_to_json(tile: QuadTile, *, root_error: float, refine: str, min_half_extent: float) -> dict[str, Any]:
    node = {
        "boundingVolume": {"box": aabb_to_tileset_box(tile.aabb, min_half_extent=min_half_extent)},
        "geometricError": _geometric_error_for_tile(tile, root_error),
        "refine": refine,
    }
    if tile.content_uri:
        node["content"] = {"uri": tile.content_uri}
    if tile.children:
        node["children"] = [_tile_to_json(child, root_error=root_error, refine=refine, min_half_extent=min_half_extent) for child in tile.children]
    return node


def main() -> int:
    args = parse_args()

    input_glb: Path = args.input_glb
    output_dir: Path = _resolve_output_dir(args.output_dir, args.output_root)

    if not input_glb.is_file():
        raise GlbError(f"Input not found: {input_glb}")
    if args.max_tris <= 0:
        raise GlbError("--max-tris must be > 0")
    if args.max_depth < 0:
        raise GlbError("--max-depth must be >= 0")
    if args.simplify_ratio <= 0:
        raise GlbError("--simplify-ratio must be > 0")

    output_dir.mkdir(parents=True, exist_ok=True)
    tiles_dir = output_dir / "tiles"
    tiles_dir.mkdir(parents=True, exist_ok=True)

    gltf, bin_chunk = read_glb(input_glb)
    node_infos, scene_root_nodes, parent_map, scene_nodes = compute_scene_nodes(gltf, bin_chunk)

    external_image_uris: dict[int, str] | None = None
    if args.external_textures:
        external_image_uris = extract_glb_images(
            gltf,
            bin_chunk,
            out_dir=tiles_dir / "textures",
            uri_prefix="textures/",
        )

    world_aabb = Aabb.empty()
    for info in node_infos:
        world_aabb = world_aabb.union(info.aabb)
    if world_aabb.is_empty():
        raise GlbError("No geometry found in scene")

    root_bounds_xy = _tile_bounds_from_aabb(world_aabb)
    root_tile = build_quadtree(
        node_infos,
        root_bounds_xy,
        depth=0,
        x=0,
        y=0,
        max_tris=args.max_tris,
        max_depth=args.max_depth,
    )

    leaves = collect_leaf_tiles(root_tile)
    for tile in leaves:
        tile_id = f"L{tile.depth}_X{tile.x}_Y{tile.y}"
        tile.content_uri = f"tiles/{tile_id}.glb"

        node_indices = tile.node_indices or []
        mesh_nodes = set(node_indices)
        nodes_keep = collect_nodes_with_ancestors(node_indices, parent_map)
        root_nodes = collect_root_nodes(nodes_keep, parent_map)
        subset_gltf, subset_bin = build_subset_glb(
            gltf,
            bin_chunk,
            nodes_to_keep=nodes_keep,
            mesh_nodes=mesh_nodes,
            root_nodes=root_nodes,
            simplify_ratio=1.0,
            rebuild_vertices=False,
            externalize_textures=args.external_textures,
            external_image_uris=external_image_uris,
        )
        write_glb(tiles_dir / f"{tile_id}.glb", subset_gltf, subset_bin)

    if args.simplify_ratio < 1.0:
        root_content = "tiles/root_simplified.glb"
        mesh_nodes = {info.node_index for info in node_infos}
        root_nodes = [n for n in scene_root_nodes if n in scene_nodes]
        subset_gltf, subset_bin = build_subset_glb(
            gltf,
            bin_chunk,
            nodes_to_keep=scene_nodes,
            mesh_nodes=mesh_nodes,
            root_nodes=root_nodes,
            simplify_ratio=args.simplify_ratio,
            rebuild_vertices=args.rebuild_vertices,
            externalize_textures=args.external_textures,
            external_image_uris=external_image_uris,
        )
        write_glb(output_dir / root_content, subset_gltf, subset_bin)
        root_tile.content_uri = root_content

    root_error = args.geometric_error
    if root_error is None:
        root_error = _compute_diagonal(root_tile.aabb) * args.error_factor

    tileset_geometric_error = args.tileset_geometric_error
    if tileset_geometric_error is None:
        tileset_geometric_error = root_error

    tileset = {
        "asset": {"version": args.asset_version},
        "geometricError": tileset_geometric_error,
        "root": _tile_to_json(
            root_tile,
            root_error=root_error,
            refine=args.refine,
            min_half_extent=args.min_half_extent,
        ),
    }

    tileset_path = output_dir / "tileset.json"
    with tileset_path.open("w", encoding="utf-8") as fp:
        json.dump(tileset, fp, ensure_ascii=True, indent=2 if args.pretty else None)
        fp.write("\n")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except GlbError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2)
