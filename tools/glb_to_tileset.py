#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import shutil
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


GLB_MAGIC = b"glTF"
GLB_VERSION_SUPPORTED = 2

CHUNK_TYPE_JSON = 0x4E4F534A  # b"JSON"
CHUNK_TYPE_BIN = 0x004E4942  # b"BIN\0"

COMPONENT_TYPE_FLOAT32 = 5126

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

    def expand(self, padding: float) -> "Aabb":
        if self.is_empty() or padding == 0:
            return self
        return Aabb(
            (
                self.min_xyz[0] - padding,
                self.min_xyz[1] - padding,
                self.min_xyz[2] - padding,
            ),
            (
                self.max_xyz[0] + padding,
                self.max_xyz[1] + padding,
                self.max_xyz[2] + padding,
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
        raise GlbError("Sparse accessors are not supported in V1")

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
        raise GlbError("Only buffer 0 (GLB BIN chunk) is supported in V1")

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


def compute_gltf_scene_aabb(gltf: dict[str, Any], bin_chunk: bytes) -> Aabb:
    scenes = gltf.get("scenes", [])
    nodes = gltf.get("nodes", [])
    meshes = gltf.get("meshes", [])

    if not scenes or not nodes:
        raise GlbError("No scenes/nodes in GLB")

    scene_index = gltf.get("scene", 0)
    if not isinstance(scene_index, int) or not (0 <= scene_index < len(scenes)):
        raise GlbError(f"Invalid gltf.scene: {scene_index}")

    scene = scenes[scene_index]
    root_node_indices = scene.get("nodes", [])
    if not root_node_indices:
        raise GlbError("Scene has no root nodes")

    accessor_aabb_cache: dict[int, Aabb] = {}
    mesh_aabb_cache: dict[int, Aabb] = {}

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

    def get_mesh_aabb(mesh_index: int) -> Aabb:
        if mesh_index in mesh_aabb_cache:
            return mesh_aabb_cache[mesh_index]
        if not (0 <= mesh_index < len(meshes)):
            raise GlbError(f"Mesh index out of range: {mesh_index}")
        mesh = meshes[mesh_index]
        primitives = mesh.get("primitives", [])
        if not primitives:
            raise GlbError(f"Mesh {mesh_index} has no primitives")

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

    world_aabb = Aabb.empty()
    for root_node_index in root_node_indices:
        if not isinstance(root_node_index, int):
            raise GlbError("Invalid root node index")
        for node_index, node_world_matrix in traverse(root_node_index, _mat4_identity()):
            node = nodes[node_index]
            mesh_index = node.get("mesh")
            if mesh_index is None:
                continue
            if not isinstance(mesh_index, int):
                raise GlbError("Invalid node.mesh index")
            mesh_local_aabb = get_mesh_aabb(mesh_index)

            transformed_aabb = Aabb.empty()
            for corner in mesh_local_aabb.corners():
                p = _mat4_transform_point(node_world_matrix, corner)
                transformed_aabb = transformed_aabb.union(Aabb(p, p))
            world_aabb = world_aabb.union(transformed_aabb)

    if world_aabb.is_empty():
        raise GlbError("No geometry found in scene (no meshes with POSITION)")

    return world_aabb


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
        description="Generate a minimal 3D Tiles tileset.json (single-tile) from a GLB by computing its scene AABB.",
    )
    parser.add_argument("input_glb", type=Path, help="Input .glb file")
    parser.add_argument("output_dir", type=Path, help="Output directory to write tileset.json (and copy .glb)")

    parser.add_argument("--out-glb", dest="out_glb", default=None, help="Output GLB filename inside output_dir (default: input basename)")
    parser.add_argument("--no-copy", action="store_true", help="Do not copy GLB into output_dir (tileset.json will still reference --out-glb)")
    parser.add_argument("--padding", type=_finite_number, default=0.0, help="Expand the computed AABB by this many meters (default: 0)")
    parser.add_argument(
        "--min-half-extent",
        type=_finite_number,
        default=0.0,
        help="Clamp each half-extent to at least this many meters to avoid degenerate boxes (default: 0)",
    )
    parser.add_argument("--refine", default="REPLACE", choices=["REPLACE", "ADD"], help="Tile refine mode (default: REPLACE)")
    parser.add_argument("--geometric-error", type=_finite_number, default=0.0, help="root.geometricError in meters (default: 0)")
    parser.add_argument(
        "--tileset-geometric-error",
        type=_finite_number,
        default=None,
        help="Top-level tileset.geometricError in meters (default: same as --geometric-error)",
    )
    parser.add_argument("--asset-version", default="1.1", help="tileset asset.version (default: 1.1)")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON (indent=2)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    input_glb: Path = args.input_glb
    output_dir: Path = args.output_dir

    if not input_glb.is_file():
        raise GlbError(f"Input not found: {input_glb}")

    output_dir.mkdir(parents=True, exist_ok=True)

    out_glb_name = args.out_glb or input_glb.name
    content_uri = out_glb_name
    out_glb_path = output_dir / out_glb_name

    gltf, bin_chunk = read_glb(input_glb)
    aabb = compute_gltf_scene_aabb(gltf, bin_chunk).expand(args.padding)

    tileset_geometric_error = args.tileset_geometric_error
    if tileset_geometric_error is None:
        tileset_geometric_error = args.geometric_error

    tileset = {
        "asset": {"version": args.asset_version},
        "geometricError": tileset_geometric_error,
        "root": {
            "boundingVolume": {"box": aabb_to_tileset_box(aabb, min_half_extent=args.min_half_extent)},
            "geometricError": args.geometric_error,
            "refine": args.refine,
            "content": {"uri": content_uri},
        },
    }

    tileset_path = output_dir / "tileset.json"
    with tileset_path.open("w", encoding="utf-8") as fp:
        json.dump(tileset, fp, ensure_ascii=False, indent=2 if args.pretty else None)
        fp.write("\n")

    if not args.no_copy:
        shutil.copyfile(input_glb, out_glb_path)

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except GlbError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2)
