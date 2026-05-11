#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import re
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Iterator, Sequence

try:
    import numpy as np
except ImportError:  # pragma: no cover - optional dependency
    np = None

try:
    import mapbox_earcut
except ImportError:  # pragma: no cover - optional dependency
    mapbox_earcut = None

try:
    from pyproj import CRS, Transformer
except ImportError:  # pragma: no cover - optional dependency
    CRS = None
    Transformer = None


EARTH_RADIUS = 6378137.0
DEFAULT_HEIGHT = 10.0
DEFAULT_LEVEL_HEIGHT = 3.0
WGS84_EPSG = 4326
DEFAULT_PLANAR_REPORT_EPSG = 4546

GLB_MAGIC = b"glTF"
GLB_VERSION_SUPPORTED = 2
CHUNK_TYPE_JSON = 0x4E4F534A  # b"JSON"
CHUNK_TYPE_BIN = 0x004E4942  # b"BIN\0"

ARRAY_BUFFER = 34962
ELEMENT_ARRAY_BUFFER = 34963
FLOAT32 = 5126
UINT32 = 5125
TRIANGLES = 4

DEFAULT_HEIGHT_PROPERTIES = [
    "height",
    "building:height",
    "building_height",
    "render_height",
    "extrudedHeight",
]
DEFAULT_LEVEL_PROPERTIES = [
    "building:levels",
    "levels",
    "render_levels",
]
DEFAULT_BASE_HEIGHT_PROPERTIES = [
    "min_height",
    "base_height",
    "building:min_height",
    "floor_height",
]

LENGTH_PATTERN = re.compile(
    r"^\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*([a-zA-Z\"']*)\s*$"
)
LENGTH_UNITS = {
    "": 1.0,
    "m": 1.0,
    "meter": 1.0,
    "meters": 1.0,
    "metre": 1.0,
    "metres": 1.0,
    "cm": 0.01,
    "mm": 0.001,
    "ft": 0.3048,
    "foot": 0.3048,
    "feet": 0.3048,
    "in": 0.0254,
    "inch": 0.0254,
    "inches": 0.0254,
    "\"": 0.0254,
    "'": 0.3048,
}


class GeoJsonError(RuntimeError):
    pass


@dataclass(frozen=True)
class RingPoint:
    x: float
    y: float
    z: float | None = None


@dataclass(frozen=True)
class PolygonFeature:
    rings: list[list[RingPoint]]
    properties: dict[str, Any]
    feature_id: str


@dataclass(frozen=True)
class Projector:
    coordinates: str
    origin_x: float
    origin_y: float

    def project(self, x: float, y: float) -> tuple[float, float]:
        if self.coordinates == "geographic":
            east, north = latlon_to_meters(
                lat=y,
                lon=x,
                origin_lat=self.origin_y,
                origin_lon=self.origin_x,
            )
            return east, -north
        return x - self.origin_x, -(y - self.origin_y)


@dataclass(frozen=True)
class Bounds2D:
    min_x: float
    min_y: float
    max_x: float
    max_y: float


@dataclass
class GeometryStats:
    polygon_count: int = 0
    skipped_count: int = 0
    unsupported_geometry_count: int = 0


def latlon_to_meters(lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple[float, float]:
    rad_lat = math.radians(origin_lat)
    x = math.radians(lon - origin_lon) * EARTH_RADIUS * math.cos(rad_lat)
    y = math.radians(lat - origin_lat) * EARTH_RADIUS
    return x, y


def warn(message: str) -> None:
    print(f"[geojson_to_glb] {message}", file=sys.stderr)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert GeoJSON polygons to an extruded GLB")
    parser.add_argument("input", help="Input GeoJSON file")
    parser.add_argument("output", help="Output GLB file")
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
    return parser.parse_args()


def parse_property_names(raw: str) -> list[str]:
    return [item.strip() for item in raw.split(",") if item.strip()]


def parse_length_meters(value: Any) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        if math.isfinite(float(value)):
            return float(value)
        return None
    if not isinstance(value, str):
        return None

    text = value.strip()
    if not text:
        return None

    match = LENGTH_PATTERN.match(text)
    if not match:
        return None

    number = float(match.group(1))
    unit = match.group(2).lower()
    factor = LENGTH_UNITS.get(unit)
    if factor is None:
        return number
    return number * factor


def first_property_value(properties: dict[str, Any], names: Sequence[str]) -> Any | None:
    for name in names:
        if name in properties:
            return properties[name]
    return None


def extract_base_and_height(
    properties: dict[str, Any],
    ring_points: Sequence[RingPoint],
    default_height: float,
    level_height: float,
    height_names: Sequence[str],
    level_names: Sequence[str],
    base_height_names: Sequence[str],
) -> tuple[float, float]:
    base_height = parse_length_meters(first_property_value(properties, base_height_names))
    if base_height is None:
        z_values = [point.z for point in ring_points if point.z is not None]
        base_height = float(z_values[0]) if z_values else 0.0

    height = parse_length_meters(first_property_value(properties, height_names))
    if height is None:
        levels = parse_length_meters(first_property_value(properties, level_names))
        if levels is not None:
            height = levels * level_height

    if height is None:
        height = default_height

    if height < 0:
        height = 0.0

    return base_height, base_height + height


def load_geojson(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise GeoJsonError(f"Input file not found: {path}") from exc
    except UnicodeDecodeError:
        # 尝试其他常见中文编码
        for encoding in ("gbk", "gb2312", "gb18030"):
            try:
                data = json.loads(path.read_text(encoding=encoding))
                break
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
        else:
            raise GeoJsonError(f"Cannot decode file with supported encodings: {path}") from None
    except json.JSONDecodeError as exc:
        raise GeoJsonError(f"Invalid GeoJSON: {exc}") from exc

    if not isinstance(data, dict):
        raise GeoJsonError("GeoJSON root must be an object")
    return data


def load_geojson_source_crs(geojson: dict[str, Any]) -> Any | None:
    raw_crs = geojson.get("crs")
    if raw_crs is None:
        return None

    if CRS is None:
        warn("GeoJSON contains crs metadata but pyproj is unavailable; CRS-aware lon/lat reporting is disabled.")
        return None

    candidates: list[str] = []
    if isinstance(raw_crs, str) and raw_crs.strip():
        candidates.append(raw_crs.strip())
    elif isinstance(raw_crs, dict):
        properties = raw_crs.get("properties")
        if isinstance(properties, dict):
            for key in ("name", "href"):
                value = properties.get(key)
                if isinstance(value, str) and value.strip():
                    candidates.append(value.strip())

    for candidate in candidates:
        try:
            return CRS.from_user_input(candidate)
        except Exception:  # pragma: no cover - pyproj error type varies by version
            continue

    warn("Ignored unsupported GeoJSON crs metadata while reporting planar lon/lat bounds.")
    return None


def describe_crs(source_crs: Any) -> str:
    if source_crs is None:
        return "unknown CRS"

    epsg = None
    try:
        epsg = source_crs.to_epsg()
    except Exception:  # pragma: no cover - defensive
        epsg = None

    if epsg:
        return f"EPSG:{epsg}"

    name = getattr(source_crs, "name", None)
    if name:
        return str(name)
    return "unknown CRS"


def iter_polygon_features(
    geojson: dict[str, Any],
    stats: GeometryStats,
) -> Iterator[PolygonFeature]:
    yield from _iter_geojson_object(geojson, stats, properties={}, feature_id="root")


def _iter_geojson_object(
    obj: Any,
    stats: GeometryStats,
    properties: dict[str, Any],
    feature_id: str,
) -> Iterator[PolygonFeature]:
    if not isinstance(obj, dict):
        return

    geojson_type = obj.get("type")

    if geojson_type == "FeatureCollection":
        for index, feature in enumerate(obj.get("features", [])):
            nested_id = _feature_id(feature, f"{feature_id}.feature{index}")
            yield from _iter_geojson_object(feature, stats, properties={}, feature_id=nested_id)
        return

    if geojson_type == "Feature":
        nested_properties = obj.get("properties") if isinstance(obj.get("properties"), dict) else {}
        nested_id = _feature_id(obj, feature_id)
        yield from _iter_geojson_object(
            obj.get("geometry"),
            stats,
            properties=nested_properties,
            feature_id=nested_id,
        )
        return

    if geojson_type == "GeometryCollection":
        for index, geometry in enumerate(obj.get("geometries", [])):
            yield from _iter_geojson_object(
                geometry,
                stats,
                properties=properties,
                feature_id=f"{feature_id}.geometry{index}",
            )
        return

    if geojson_type == "Polygon":
        rings = parse_polygon_rings(obj.get("coordinates"))
        if rings:
            stats.polygon_count += 1
            yield PolygonFeature(rings=rings, properties=properties, feature_id=feature_id)
        else:
            stats.skipped_count += 1
        return

    if geojson_type == "MultiPolygon":
        polygons = obj.get("coordinates")
        if not isinstance(polygons, list):
            stats.skipped_count += 1
            return
        for index, polygon in enumerate(polygons):
            rings = parse_polygon_rings(polygon)
            if rings:
                stats.polygon_count += 1
                yield PolygonFeature(
                    rings=rings,
                    properties=properties,
                    feature_id=f"{feature_id}.polygon{index}",
                )
            else:
                stats.skipped_count += 1
        return

    if geojson_type is not None:
        stats.unsupported_geometry_count += 1


def _feature_id(feature: dict[str, Any], fallback: str) -> str:
    raw_id = feature.get("id")
    if raw_id is None:
        return fallback
    return str(raw_id)


def parse_polygon_rings(raw_rings: Any) -> list[list[RingPoint]]:
    if not isinstance(raw_rings, list):
        return []

    parsed_rings: list[list[RingPoint]] = []
    for raw_ring in raw_rings:
        ring = parse_ring(raw_ring)
        if len(ring) >= 3:
            parsed_rings.append(ring)
    return parsed_rings


def parse_ring(raw_ring: Any) -> list[RingPoint]:
    if not isinstance(raw_ring, list):
        return []

    points: list[RingPoint] = []
    for raw_position in raw_ring:
        if not isinstance(raw_position, (list, tuple)) or len(raw_position) < 2:
            continue
        x = float(raw_position[0])
        y = float(raw_position[1])
        z = float(raw_position[2]) if len(raw_position) >= 3 else None
        point = RingPoint(x=x, y=y, z=z)
        if points and same_xy(points[-1], point):
            continue
        points.append(point)

    if len(points) >= 2 and same_xy(points[0], points[-1]):
        points.pop()

    if len(points) < 3:
        return []

    return remove_collinear_points(points)


def same_xy(a: RingPoint, b: RingPoint, epsilon: float = 1e-9) -> bool:
    return abs(a.x - b.x) <= epsilon and abs(a.y - b.y) <= epsilon


def remove_collinear_points(points: Sequence[RingPoint], epsilon: float = 1e-12) -> list[RingPoint]:
    if len(points) < 3:
        return list(points)

    cleaned = list(points)
    changed = True
    while changed and len(cleaned) >= 3:
        changed = False
        next_points: list[RingPoint] = []
        for index, current in enumerate(cleaned):
            prev_point = cleaned[index - 1]
            next_point = cleaned[(index + 1) % len(cleaned)]
            if abs(cross_2d(prev_point, current, next_point)) <= epsilon:
                changed = True
                continue
            next_points.append(current)
        cleaned = next_points
    return cleaned if len(cleaned) >= 3 else []


def cross_2d(a: RingPoint, b: RingPoint, c: RingPoint) -> float:
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)


def collect_positions(features: Sequence[PolygonFeature]) -> list[tuple[float, float]]:
    coords: list[tuple[float, float]] = []
    for feature in features:
        for ring in feature.rings:
            coords.extend((point.x, point.y) for point in ring)
    return coords


def choose_projector(
    coordinates: str,
    origin: Sequence[float] | None,
    positions: Sequence[tuple[float, float]],
) -> Projector:
    if origin is not None:
        return Projector(coordinates=coordinates, origin_x=float(origin[0]), origin_y=float(origin[1]))

    if not positions:
        raise GeoJsonError("GeoJSON does not contain any polygon coordinates")

    xs = [position[0] for position in positions]
    ys = [position[1] for position in positions]
    origin_x = (min(xs) + max(xs)) / 2.0
    origin_y = (min(ys) + max(ys)) / 2.0
    return Projector(coordinates=coordinates, origin_x=origin_x, origin_y=origin_y)


def compute_bounds2d(positions: Sequence[tuple[float, float]]) -> Bounds2D:
    if not positions:
        raise GeoJsonError("Cannot compute bounds from empty coordinates")

    min_x = min_y = float("inf")
    max_x = max_y = float("-inf")
    for x, y in positions:
        min_x = min(min_x, x)
        min_y = min(min_y, y)
        max_x = max(max_x, x)
        max_y = max(max_y, y)
    return Bounds2D(min_x=min_x, min_y=min_y, max_x=max_x, max_y=max_y)


def build_wgs84_transformer(source_crs: Any | None) -> Any | None:
    if source_crs is None:
        return None
    if CRS is None or Transformer is None:
        return None

    try:
        return Transformer.from_crs(source_crs, CRS.from_epsg(WGS84_EPSG), always_xy=True)
    except Exception:  # pragma: no cover - pyproj error type varies by version
        warn(f"Failed to build CRS transformer from {describe_crs(source_crs)} to WGS84.")
        return None


def transform_bounds_to_lonlat(
    positions: Sequence[tuple[float, float]],
    transformer: Any | None,
) -> Bounds2D | None:
    if transformer is None:
        return None
    transformed = [transformer.transform(x, y) for x, y in positions]
    return compute_bounds2d(transformed)


def print_bounds_line(label: str, minimum: float, maximum: float, precision: int, unit: str = "") -> None:
    print(f"  {label} min/max: {minimum:.{precision}f} / {maximum:.{precision}f}{unit}")


def print_origin_and_bounds(
    coordinates: str,
    projector: Projector,
    source_positions: Sequence[tuple[float, float]],
    source_crs: Any | None,
) -> None:
    source_bounds = compute_bounds2d(source_positions)
    local_bounds = compute_bounds2d([projector.project(x, y) for x, y in source_positions])

    print("=" * 50)
    print("Origin Info (for 3D Tiles / Cesium configuration):")
    if coordinates == "geographic":
        print(f"  Longitude: {projector.origin_x:.8f}")
        print(f"  Latitude:  {projector.origin_y:.8f}")
        print("Bounds:")
        print(f"  West Longitude:  {source_bounds.min_x:.8f}")
        print(f"  East Longitude:  {source_bounds.max_x:.8f}")
        print(f"  South Latitude:  {source_bounds.min_y:.8f}")
        print(f"  North Latitude:  {source_bounds.max_y:.8f}")
        print_bounds_line("Local X", local_bounds.min_x, local_bounds.max_x, precision=2, unit=" m")
        print_bounds_line("Local Y", local_bounds.min_y, local_bounds.max_y, precision=2, unit=" m")
    else:
        print(f"  Projected X: {projector.origin_x:.6f}")
        print(f"  Projected Y: {projector.origin_y:.6f}")
        report_crs = source_crs
        report_crs_note = "from GeoJSON crs"
        if report_crs is None and CRS is not None:
            try:
                report_crs = CRS.from_epsg(DEFAULT_PLANAR_REPORT_EPSG)
                report_crs_note = f"fallback to EPSG:{DEFAULT_PLANAR_REPORT_EPSG}"
            except Exception:  # pragma: no cover - defensive
                report_crs = None

        if report_crs is not None:
            print(f"  Source CRS:  {describe_crs(report_crs)} ({report_crs_note})")

        transformer = build_wgs84_transformer(report_crs)
        lonlat_bounds = transform_bounds_to_lonlat(source_positions, transformer)
        if lonlat_bounds is not None and transformer is not None:
            origin_lon, origin_lat = transformer.transform(projector.origin_x, projector.origin_y)
            print(f"  Longitude:   {origin_lon:.8f}")
            print(f"  Latitude:    {origin_lat:.8f}")
            print("Bounds:")
            print(f"  West Longitude:  {lonlat_bounds.min_x:.8f}")
            print(f"  East Longitude:  {lonlat_bounds.max_x:.8f}")
            print(f"  South Latitude:  {lonlat_bounds.min_y:.8f}")
            print(f"  North Latitude:  {lonlat_bounds.max_y:.8f}")
        else:
            if source_crs is not None and (CRS is None or Transformer is None):
                print("  (Install pyproj for GeoJSON crs-based lon/lat conversion: pip install pyproj)")
            elif source_crs is None:
                print(f"  (GeoJSON has no readable crs; using EPSG:{DEFAULT_PLANAR_REPORT_EPSG} requires pyproj)")
            else:
                print("  (Unable to transform source CRS to WGS84)")
            print("Bounds:")
        print_bounds_line("Source X", source_bounds.min_x, source_bounds.max_x, precision=6)
        print_bounds_line("Source Y", source_bounds.min_y, source_bounds.max_y, precision=6)
        print_bounds_line("Local X", local_bounds.min_x, local_bounds.max_x, precision=6)
        print_bounds_line("Local Y", local_bounds.min_y, local_bounds.max_y, precision=6)
    print("=" * 50)


def ring_area(points: Sequence[tuple[float, float]]) -> float:
    area = 0.0
    for index, point in enumerate(points):
        next_point = points[(index + 1) % len(points)]
        area += point[0] * next_point[1] - next_point[0] * point[1]
    return area / 2.0


def ensure_ccw(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    pts = list(points)
    if ring_area(pts) < 0:
        pts.reverse()
    return pts


def ensure_cw(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    pts = list(points)
    if ring_area(pts) > 0:
        pts.reverse()
    return pts


def triangulate_polygon(
    outer_ring: Sequence[tuple[float, float]],
    holes: Sequence[Sequence[tuple[float, float]]],
) -> tuple[list[tuple[float, float]], list[int]]:
    if mapbox_earcut is not None and np is not None:
        flat_vertices: list[tuple[float, float]] = list(outer_ring)
        ring_end_indices = [len(flat_vertices)]
        for hole in holes:
            flat_vertices.extend(hole)
            ring_end_indices.append(len(flat_vertices))

        if len(flat_vertices) < 3:
            return [], []

        vertices_np = np.asarray(flat_vertices, dtype=np.float64)
        rings_np = np.asarray(ring_end_indices, dtype=np.uint32)
        indices = mapbox_earcut.triangulate_float64(vertices_np, rings_np)
        return flat_vertices, indices.astype(np.uint32).tolist()

    if holes:
        warn("Detected polygons with holes but mapbox_earcut/numpy is unavailable; holes will be ignored.")
    return list(outer_ring), triangulate_simple_polygon(outer_ring)


def triangulate_simple_polygon(points: Sequence[tuple[float, float]]) -> list[int]:
    if len(points) < 3:
        return []

    indices = list(range(len(points)))
    triangles: list[int] = []
    remaining_guard = len(indices) * len(indices)

    while len(indices) > 2 and remaining_guard > 0:
        remaining_guard -= 1
        ear_found = False
        for cursor, current_index in enumerate(indices):
            prev_index = indices[cursor - 1]
            next_index = indices[(cursor + 1) % len(indices)]

            a = points[prev_index]
            b = points[current_index]
            c = points[next_index]

            if area2(a, b, c) <= 1e-12:
                continue

            is_ear = True
            for other_index in indices:
                if other_index in (prev_index, current_index, next_index):
                    continue
                if point_in_triangle(points[other_index], a, b, c):
                    is_ear = False
                    break

            if not is_ear:
                continue

            triangles.extend([prev_index, current_index, next_index])
            indices.pop(cursor)
            ear_found = True
            break

        if not ear_found:
            break

    return triangles


def area2(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def point_in_triangle(
    p: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
) -> bool:
    denominator = ((b[1] - c[1]) * (a[0] - c[0])) + ((c[0] - b[0]) * (a[1] - c[1]))
    if abs(denominator) <= 1e-12:
        return False

    alpha = (((b[1] - c[1]) * (p[0] - c[0])) + ((c[0] - b[0]) * (p[1] - c[1]))) / denominator
    beta = (((c[1] - a[1]) * (p[0] - c[0])) + ((a[0] - c[0]) * (p[1] - c[1]))) / denominator
    gamma = 1.0 - alpha - beta
    return alpha >= 0 and beta >= 0 and gamma >= 0


def build_geometry(
    features: Sequence[PolygonFeature],
    projector: Projector,
    default_height: float,
    level_height: float,
    height_names: Sequence[str],
    level_names: Sequence[str],
    base_height_names: Sequence[str],
) -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float]], list[int], int]:
    positions: list[tuple[float, float, float]] = []
    normals: list[tuple[float, float, float]] = []
    indices: list[int] = []
    emitted_polygon_count = 0

    for feature in features:
        outer_ring_points = feature.rings[0]
        base_height, top_height = extract_base_and_height(
            properties=feature.properties,
            ring_points=outer_ring_points,
            default_height=default_height,
            level_height=level_height,
            height_names=height_names,
            level_names=level_names,
            base_height_names=base_height_names,
        )

        if top_height <= base_height:
            warn(f"Skip {feature.feature_id}: non-positive extrusion height")
            continue

        projected_outer = ensure_ccw([projector.project(point.x, point.y) for point in outer_ring_points])
        projected_holes = [
            ensure_cw([projector.project(point.x, point.y) for point in ring])
            for ring in feature.rings[1:]
        ]
        projected_holes = [ring for ring in projected_holes if len(ring) >= 3]
        usable_holes = projected_holes
        if projected_holes and (mapbox_earcut is None or np is None):
            warn(
                f"Feature {feature.feature_id} contains holes but mapbox_earcut/numpy "
                "is unavailable; inner rings will be ignored."
            )
            usable_holes = []

        if len(projected_outer) < 3 or abs(ring_area(projected_outer)) <= 1e-12:
            warn(f"Skip {feature.feature_id}: degenerate outer ring")
            continue

        surface_vertices, surface_triangles = triangulate_polygon(projected_outer, usable_holes)
        if not surface_triangles:
            warn(f"Skip {feature.feature_id}: triangulation failed")
            continue

        emitted_polygon_count += 1

        roof_base = len(positions)
        for x, z in surface_vertices:
            positions.append((x, top_height, z))
            normals.append((0.0, 1.0, 0.0))
        for tri_offset in range(0, len(surface_triangles), 3):
            a = surface_triangles[tri_offset]
            b = surface_triangles[tri_offset + 1]
            c = surface_triangles[tri_offset + 2]
            indices.extend([roof_base + a, roof_base + c, roof_base + b])

        floor_base = len(positions)
        for x, z in surface_vertices:
            positions.append((x, base_height, z))
            normals.append((0.0, -1.0, 0.0))
        for tri_offset in range(0, len(surface_triangles), 3):
            a = surface_triangles[tri_offset]
            b = surface_triangles[tri_offset + 1]
            c = surface_triangles[tri_offset + 2]
            indices.extend([floor_base + a, floor_base + b, floor_base + c])

        all_rings = [projected_outer, *usable_holes]
        for ring in all_rings:
            for index, p1 in enumerate(ring):
                p2 = ring[(index + 1) % len(ring)]
                add_wall_quad(positions, normals, indices, p1, p2, base_height, top_height)

    return positions, normals, indices, emitted_polygon_count


def add_wall_quad(
    positions: list[tuple[float, float, float]],
    normals: list[tuple[float, float, float]],
    indices: list[int],
    p1: tuple[float, float],
    p2: tuple[float, float],
    base_height: float,
    top_height: float,
) -> None:
    dx = p2[0] - p1[0]
    dz = p2[1] - p1[1]
    length = math.hypot(dx, dz)
    if length <= 1e-12:
        return

    nx = dz / length
    nz = -dx / length

    base_index = len(positions)
    positions.extend(
        [
            (p1[0], base_height, p1[1]),
            (p2[0], base_height, p2[1]),
            (p2[0], top_height, p2[1]),
            (p1[0], top_height, p1[1]),
        ]
    )
    normals.extend([(nx, 0.0, nz)] * 4)
    indices.extend(
        [
            base_index + 0,
            base_index + 2,
            base_index + 1,
            base_index + 0,
            base_index + 3,
            base_index + 2,
        ]
    )


def export_glb(
    positions: Sequence[tuple[float, float, float]],
    normals: Sequence[tuple[float, float, float]],
    indices: Sequence[int],
    filename: Path,
) -> None:
    if not positions or not indices:
        raise GeoJsonError("No mesh was generated from the input GeoJSON")

    position_blob = bytearray()
    normal_blob = bytearray()
    index_blob = bytearray()

    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = float("-inf")

    for x, y, z in positions:
        position_blob.extend(struct.pack("<fff", x, y, z))
        min_x = min(min_x, x)
        min_y = min(min_y, y)
        min_z = min(min_z, z)
        max_x = max(max_x, x)
        max_y = max(max_y, y)
        max_z = max(max_z, z)

    for nx, ny, nz in normals:
        normal_blob.extend(struct.pack("<fff", nx, ny, nz))

    for index in indices:
        index_blob.extend(struct.pack("<I", index))

    position_blob = pad_to_4(position_blob)
    normal_blob = pad_to_4(normal_blob)
    index_blob = pad_to_4(index_blob)

    binary_blob = position_blob + normal_blob + index_blob
    header = {
        "asset": {"version": "2.0", "generator": "GeoJSONToGLB/geojson_to_glb.py"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0, "name": filename.stem}],
        "materials": [
            {
                "name": "DefaultMaterial",
                "pbrMetallicRoughness": {
                    "baseColorFactor": [0.72, 0.72, 0.72, 1.0],
                    "metallicFactor": 0.0,
                    "roughnessFactor": 0.85,
                },
                "doubleSided": True,
            }
        ],
        "meshes": [
            {
                "primitives": [
                    {
                        "attributes": {"POSITION": 0, "NORMAL": 1},
                        "indices": 2,
                        "material": 0,
                        "mode": TRIANGLES,
                    }
                ]
            }
        ],
        "buffers": [{"byteLength": len(binary_blob)}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": len(position_blob), "target": ARRAY_BUFFER},
            {
                "buffer": 0,
                "byteOffset": len(position_blob),
                "byteLength": len(normal_blob),
                "target": ARRAY_BUFFER,
            },
            {
                "buffer": 0,
                "byteOffset": len(position_blob) + len(normal_blob),
                "byteLength": len(index_blob),
                "target": ELEMENT_ARRAY_BUFFER,
            },
        ],
        "accessors": [
            {
                "bufferView": 0,
                "componentType": FLOAT32,
                "count": len(positions),
                "type": "VEC3",
                "min": [min_x, min_y, min_z],
                "max": [max_x, max_y, max_z],
            },
            {
                "bufferView": 1,
                "componentType": FLOAT32,
                "count": len(normals),
                "type": "VEC3",
            },
            {
                "bufferView": 2,
                "componentType": UINT32,
                "count": len(indices),
                "type": "SCALAR",
            },
        ],
    }

    json_blob = json.dumps(header, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
    json_blob = bytes(pad_to_4(bytearray(json_blob), fill=b" "))

    filename.parent.mkdir(parents=True, exist_ok=True)
    total_length = 12 + 8 + len(json_blob) + 8 + len(binary_blob)

    with filename.open("wb") as output:
        output.write(GLB_MAGIC)
        output.write(struct.pack("<I", GLB_VERSION_SUPPORTED))
        output.write(struct.pack("<I", total_length))

        output.write(struct.pack("<I", len(json_blob)))
        output.write(struct.pack("<I", CHUNK_TYPE_JSON))
        output.write(json_blob)

        output.write(struct.pack("<I", len(binary_blob)))
        output.write(struct.pack("<I", CHUNK_TYPE_BIN))
        output.write(binary_blob)


def pad_to_4(blob: bytearray, fill: bytes = b"\x00") -> bytearray:
    padding = (-len(blob)) % 4
    if padding:
        blob.extend(fill * padding)
    return blob


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
    export_glb(positions, normals, indices, Path(args.output))
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
