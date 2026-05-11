#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from GeoJSONToGLB.geojson_to_glb import (  # noqa: E402
    DEFAULT_BASE_HEIGHT_PROPERTIES,
    DEFAULT_HEIGHT,
    DEFAULT_HEIGHT_PROPERTIES,
    DEFAULT_LEVEL_HEIGHT,
    DEFAULT_LEVEL_PROPERTIES,
    GeoJsonError,
    GeometryStats,
    build_geometry,
    choose_projector,
    collect_positions,
    iter_polygon_features,
    load_geojson,
    load_geojson_source_crs,
    parse_property_names,
    print_origin_and_bounds,
    warn,
)


BLENDER_EXPORT_SCRIPT = r"""
from __future__ import annotations

import json
import sys
import traceback
from pathlib import Path

import bpy


def fail(message: str) -> None:
    print(f"Error: {message}", file=sys.stderr)
    raise SystemExit(1)


def y_up_to_blender_z_up(position):
    x, y, z = position
    return [x, -z, y]


def y_up_normal_to_blender_z_up(normal):
    nx, ny, nz = normal
    return [nx, -nz, ny]


def main() -> int:
    try:
        separator_index = sys.argv.index("--")
    except ValueError:
        fail("Missing Blender script arguments")

    script_args = sys.argv[separator_index + 1 :]
    if len(script_args) != 2:
        fail("Expected mesh JSON path and output FBX path")

    mesh_json_path = Path(script_args[0])
    output_fbx_path = Path(script_args[1])

    with mesh_json_path.open("r", encoding="utf-8") as input_file:
        payload = json.load(input_file)

    source_positions = payload.get("positions") or []
    source_normals = payload.get("normals") or []
    source_indices = payload.get("indices") or []
    if not source_positions or not source_indices:
        fail("Mesh payload is empty")
    if len(source_indices) % 3 != 0:
        fail("Mesh index count is not divisible by 3")

    vertices = [y_up_to_blender_z_up(position) for position in source_positions]
    faces = [
        [source_indices[index], source_indices[index + 1], source_indices[index + 2]]
        for index in range(0, len(source_indices), 3)
    ]

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()

    mesh = bpy.data.meshes.new(payload.get("mesh_name") or output_fbx_path.stem)
    mesh.from_pydata(vertices, [], faces)
    mesh.update(calc_edges=True)

    obj = bpy.data.objects.new(payload.get("mesh_name") or output_fbx_path.stem, mesh)
    bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    if len(source_normals) == len(source_positions):
        loop_normals = []
        for polygon in mesh.polygons:
            for vertex_index in polygon.vertices:
                loop_normals.append(y_up_normal_to_blender_z_up(source_normals[vertex_index]))
        mesh.normals_split_custom_set(loop_normals)
        if hasattr(mesh, "use_auto_smooth"):
            mesh.use_auto_smooth = True

    material = bpy.data.materials.new(payload.get("material_name") or "DefaultMaterial")
    material.diffuse_color = (0.72, 0.72, 0.72, 1.0)
    obj.data.materials.append(material)

    output_fbx_path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.export_scene.fbx(
        filepath=str(output_fbx_path),
        use_selection=True,
        axis_forward="-Z",
        axis_up="Y",
        bake_anim=False,
    )

    if not output_fbx_path.exists() or output_fbx_path.stat().st_size <= 0:
        fail(f"FBX export did not create a non-empty file: {output_fbx_path}")

    return 0


try:
    raise SystemExit(main())
except SystemExit:
    raise
except Exception:
    traceback.print_exc(file=sys.stderr)
    raise SystemExit(1)
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert GeoJSON polygons to an extruded FBX via Blender")
    parser.add_argument("input", help="Input GeoJSON file")
    parser.add_argument("output", help="Output FBX file")
    parser.add_argument(
        "--coordinates",
        choices=["geographic", "planar"],
        default="geographic",
        help="Interpret GeoJSON positions as lon/lat or planar x/y (default: geographic)",
    )
    parser.add_argument(
        "--origin",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="Optional origin. Geographic mode expects lon lat; planar mode expects x y",
    )
    parser.add_argument(
        "--default-height",
        type=float,
        default=DEFAULT_HEIGHT,
        help=f"Extrusion height in meters when no height property exists (default: {DEFAULT_HEIGHT})",
    )
    parser.add_argument(
        "--level-height",
        type=float,
        default=DEFAULT_LEVEL_HEIGHT,
        help=f"Fallback meters per level for building:levels (default: {DEFAULT_LEVEL_HEIGHT})",
    )
    parser.add_argument(
        "--height-properties",
        default=",".join(DEFAULT_HEIGHT_PROPERTIES),
        help="Comma-separated height property names",
    )
    parser.add_argument(
        "--level-properties",
        default=",".join(DEFAULT_LEVEL_PROPERTIES),
        help="Comma-separated level property names",
    )
    parser.add_argument(
        "--base-height-properties",
        default=",".join(DEFAULT_BASE_HEIGHT_PROPERTIES),
        help="Comma-separated base height property names",
    )
    parser.add_argument(
        "--blender",
        default="blender",
        help="Blender executable used for FBX export (default: blender)",
    )
    return parser.parse_args()


def export_fbx_via_blender(
    positions: Sequence[tuple[float, float, float]],
    normals: Sequence[tuple[float, float, float]],
    indices: Sequence[int],
    filename: Path,
    blender: str,
) -> None:
    if not positions or not indices:
        raise GeoJsonError("No mesh was generated from the input GeoJSON")
    if len(indices) % 3 != 0:
        raise GeoJsonError("Mesh index count is not divisible by 3")

    blender_executable = shutil.which(blender) if not Path(blender).is_file() else blender
    if blender_executable is None:
        raise GeoJsonError(
            "Blender executable not found. Install Blender and make sure 'blender' is in PATH, "
            "or pass --blender /path/to/blender."
        )

    filename.parent.mkdir(parents=True, exist_ok=True)

    payload = {
        "mesh_name": filename.stem,
        "material_name": "DefaultMaterial",
        "positions": positions,
        "normals": normals,
        "indices": indices,
    }

    with tempfile.TemporaryDirectory(prefix="geojson_to_fbx_") as temp_dir:
        temp_path = Path(temp_dir)
        mesh_json_path = temp_path / "mesh.json"
        script_path = temp_path / "export_fbx.py"
        mesh_json_path.write_text(json.dumps(payload, ensure_ascii=True, separators=(",", ":")), encoding="utf-8")
        script_path.write_text(BLENDER_EXPORT_SCRIPT, encoding="utf-8")

        command = [
            blender_executable,
            "--background",
            "--python",
            str(script_path),
            "--",
            str(mesh_json_path),
            str(filename),
        ]
        try:
            result = subprocess.run(command, check=False, capture_output=True, text=True)
        except OSError as exc:
            raise GeoJsonError(f"Failed to launch Blender executable: {blender_executable}") from exc

    if result.returncode != 0:
        details = "\n".join(part.strip() for part in (result.stderr, result.stdout) if part.strip())
        if details:
            raise GeoJsonError(f"Blender FBX export failed with exit code {result.returncode}:\n{details}")
        raise GeoJsonError(f"Blender FBX export failed with exit code {result.returncode}")

    if not filename.exists() or filename.stat().st_size <= 0:
        details = "\n".join(part.strip() for part in (result.stderr, result.stdout) if part.strip())
        if details:
            raise GeoJsonError(f"FBX export did not create a non-empty file: {filename}\n{details}")
        raise GeoJsonError(f"FBX export did not create a non-empty file: {filename}")


def main() -> int:
    args = parse_args()

    height_names = parse_property_names(args.height_properties)
    level_names = parse_property_names(args.level_properties)
    base_height_names = parse_property_names(args.base_height_properties)

    geojson = load_geojson(Path(args.input))
    source_crs = load_geojson_source_crs(geojson) if args.coordinates == "planar" else None
    stats = GeometryStats()
    features = list(iter_polygon_features(geojson, stats))
    if not features:
        raise GeoJsonError("No Polygon or MultiPolygon geometry found in GeoJSON")

    source_positions = collect_positions(features)
    projector = choose_projector(
        coordinates=args.coordinates,
        origin=args.origin,
        positions=source_positions,
    )

    positions, normals, indices, emitted_polygon_count = build_geometry(
        features=features,
        projector=projector,
        default_height=args.default_height,
        level_height=args.level_height,
        height_names=height_names,
        level_names=level_names,
        base_height_names=base_height_names,
    )
    export_fbx_via_blender(positions, normals, indices, Path(args.output), args.blender)
    print_origin_and_bounds(
        coordinates=args.coordinates,
        projector=projector,
        source_positions=source_positions,
        source_crs=source_crs,
    )

    print(
        "Exported "
        f"{args.output}: {emitted_polygon_count} polygon(s), "
        f"{len(positions)} vertices, {len(indices) // 3} triangles."
    )
    if stats.unsupported_geometry_count:
        warn(f"Ignored {stats.unsupported_geometry_count} unsupported geometry object(s)")
    if stats.skipped_count:
        warn(f"Skipped {stats.skipped_count} invalid or degenerate polygon object(s)")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except GeoJsonError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
