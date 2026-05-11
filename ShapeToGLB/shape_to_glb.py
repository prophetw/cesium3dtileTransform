#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

try:
    import shapefile
except ImportError:  # pragma: no cover - optional dependency
    shapefile = None

try:
    from pyproj import CRS, Transformer
except ImportError:  # pragma: no cover - optional dependency
    CRS = None
    Transformer = None


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
    PolygonFeature,
    build_geometry,
    choose_projector,
    collect_positions,
    export_glb,
    parse_polygon_rings,
    parse_property_names,
    warn,
)


WGS84_EPSG = 4326


@dataclass(frozen=True)
class CoordinateResolution:
    coordinates: str
    transform_to_wgs84: bool
    source_crs: Any | None
    prj_path: Path | None
    description: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert ESRI Shapefile polygons to an extruded GLB")
    parser.add_argument("input", help="Input shapefile (.shp)")
    parser.add_argument("output", help="Output GLB file")
    parser.add_argument(
        "--coordinates",
        choices=["auto", "geographic", "planar"],
        default="auto",
        help=(
            "Coordinate handling mode. "
            "'auto' uses .prj when available, otherwise falls back to bbox inference."
        ),
    )
    parser.add_argument(
        "--origin",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="Optional origin. Geographic mode expects lon lat; planar mode expects x y",
    )
    parser.add_argument(
        "--prj",
        help="Optional .prj path. Defaults to the file next to the input .shp",
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
        help=f"Fallback meters per level for level-based height fields (default: {DEFAULT_LEVEL_HEIGHT})",
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


def ensure_dependency(name: str, module: Any) -> None:
    if module is None:
        raise GeoJsonError(f"Missing dependency: {name}")


def resolve_prj_path(shp_path: Path, explicit_prj: str | None) -> Path | None:
    if explicit_prj:
        prj_path = Path(explicit_prj)
        return prj_path

    sibling_prj = shp_path.with_suffix(".prj")
    if sibling_prj.exists():
        return sibling_prj
    return None


def load_source_crs(prj_path: Path | None) -> Any | None:
    if prj_path is None or not prj_path.exists():
        return None

    if CRS is None:
        raise GeoJsonError("pyproj is required to read .prj files")

    try:
        prj_text = prj_path.read_text(encoding="utf-8", errors="ignore")
    except OSError as exc:
        raise GeoJsonError(f"Unable to read PRJ file: {prj_path}") from exc

    try:
        return CRS.from_wkt(prj_text)
    except Exception as exc:  # pragma: no cover - pyproj error type varies by version
        raise GeoJsonError(f"Failed to parse PRJ file: {prj_path}") from exc


def crs_is_wgs84(source_crs: Any) -> bool:
    if source_crs is None or CRS is None:
        return False
    try:
        if source_crs.to_epsg() == WGS84_EPSG:
            return True
    except Exception:  # pragma: no cover - defensive
        pass
    try:
        return source_crs.equals(CRS.from_epsg(WGS84_EPSG))
    except Exception:  # pragma: no cover - defensive
        return False


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


def looks_like_geographic_bbox(bbox: Sequence[float] | None) -> bool:
    if not bbox or len(bbox) < 4:
        return False
    min_x, min_y, max_x, max_y = bbox[:4]
    return (
        -180.0 <= min_x <= 180.0
        and -180.0 <= max_x <= 180.0
        and -90.0 <= min_y <= 90.0
        and -90.0 <= max_y <= 90.0
    )


def resolve_coordinates(
    requested_mode: str,
    source_crs: Any | None,
    prj_path: Path | None,
    bbox: Sequence[float] | None,
) -> CoordinateResolution:
    if requested_mode == "planar":
        description = "planar mode requested explicitly"
        if source_crs is not None:
            description += f" ({describe_crs(source_crs)})"
        return CoordinateResolution(
            coordinates="planar",
            transform_to_wgs84=False,
            source_crs=source_crs,
            prj_path=prj_path,
            description=description,
        )

    if requested_mode == "geographic":
        needs_transform = bool(source_crs is not None and not crs_is_wgs84(source_crs))
        return CoordinateResolution(
            coordinates="geographic",
            transform_to_wgs84=needs_transform,
            source_crs=source_crs,
            prj_path=prj_path,
            description=(
                f"geographic mode requested explicitly"
                + (f" using {describe_crs(source_crs)}" if source_crs is not None else "")
                + ("; transform to WGS84" if needs_transform else "")
            ),
        )

    if source_crs is not None:
        if getattr(source_crs, "is_projected", False):
            return CoordinateResolution(
                coordinates="geographic",
                transform_to_wgs84=True,
                source_crs=source_crs,
                prj_path=prj_path,
                description=f"auto-detected projected CRS {describe_crs(source_crs)} from PRJ; reproject to WGS84",
            )
        if getattr(source_crs, "is_geographic", False):
            return CoordinateResolution(
                coordinates="geographic",
                transform_to_wgs84=not crs_is_wgs84(source_crs),
                source_crs=source_crs,
                prj_path=prj_path,
                description=(
                    f"auto-detected geographic CRS {describe_crs(source_crs)} from PRJ"
                    + ("; transform to WGS84" if not crs_is_wgs84(source_crs) else "")
                ),
            )
        warn(
            f"PRJ {prj_path} resolved to {describe_crs(source_crs)}, "
            "but it is neither projected nor geographic; fallback to planar coordinates."
        )
        return CoordinateResolution(
            coordinates="planar",
            transform_to_wgs84=False,
            source_crs=source_crs,
            prj_path=prj_path,
            description=f"fallback to planar because {describe_crs(source_crs)} is unsupported for auto mode",
        )

    if looks_like_geographic_bbox(bbox):
        warn("No PRJ file found; bbox looks like lon/lat, so auto mode will treat data as geographic.")
        return CoordinateResolution(
            coordinates="geographic",
            transform_to_wgs84=False,
            source_crs=None,
            prj_path=None,
            description="auto mode inferred geographic coordinates from bbox",
        )

    warn("No PRJ file found; auto mode will treat data as planar coordinates.")
    return CoordinateResolution(
        coordinates="planar",
        transform_to_wgs84=False,
        source_crs=None,
        prj_path=None,
        description="auto mode fell back to planar coordinates",
    )


def build_transformer(source_crs: Any | None, enabled: bool) -> Any | None:
    if not enabled:
        return None
    if source_crs is None:
        return None
    if CRS is None or Transformer is None:
        raise GeoJsonError("pyproj is required to transform shapefile coordinates to WGS84")
    try:
        return Transformer.from_crs(source_crs, CRS.from_epsg(WGS84_EPSG), always_xy=True)
    except Exception as exc:  # pragma: no cover - pyproj error type varies by version
        raise GeoJsonError(f"Failed to build CRS transformer from {describe_crs(source_crs)} to WGS84") from exc


def transform_position(position: Sequence[Any], transformer: Any | None) -> list[float]:
    x = float(position[0])
    y = float(position[1])
    if transformer is None:
        transformed_x = x
        transformed_y = y
    else:
        transformed_x, transformed_y = transformer.transform(x, y)

    if len(position) >= 3 and position[2] is not None:
        return [transformed_x, transformed_y, float(position[2])]
    return [transformed_x, transformed_y]


def transform_polygon_rings(raw_rings: Any, transformer: Any | None) -> list[list[list[float]]]:
    if not isinstance(raw_rings, (list, tuple)):
        return []

    transformed_rings: list[list[list[float]]] = []
    for raw_ring in raw_rings:
        if not isinstance(raw_ring, (list, tuple)):
            continue
        transformed_ring: list[list[float]] = []
        for position in raw_ring:
            if not isinstance(position, (list, tuple)) or len(position) < 2:
                continue
            transformed_ring.append(transform_position(position, transformer))
        transformed_rings.append(transformed_ring)
    return transformed_rings


def iter_polygon_features_from_geometry(
    geometry: Any,
    properties: dict[str, Any],
    feature_id: str,
    transformer: Any | None,
    stats: GeometryStats,
) -> Iterable[PolygonFeature]:
    if not isinstance(geometry, dict):
        stats.skipped_count += 1
        return []

    geometry_type = geometry.get("type")
    coordinates = geometry.get("coordinates")

    if geometry_type == "Polygon":
        rings = parse_polygon_rings(transform_polygon_rings(coordinates, transformer))
        if rings:
            stats.polygon_count += 1
            return [PolygonFeature(rings=rings, properties=properties, feature_id=feature_id)]
        stats.skipped_count += 1
        return []

    if geometry_type == "MultiPolygon":
        if not isinstance(coordinates, (list, tuple)):
            stats.skipped_count += 1
            return []
        features: list[PolygonFeature] = []
        for polygon_index, polygon_coords in enumerate(coordinates):
            rings = parse_polygon_rings(transform_polygon_rings(polygon_coords, transformer))
            if rings:
                stats.polygon_count += 1
                features.append(
                    PolygonFeature(
                        rings=rings,
                        properties=properties,
                        feature_id=f"{feature_id}.polygon{polygon_index}",
                    )
                )
            else:
                stats.skipped_count += 1
        return features

    stats.unsupported_geometry_count += 1
    return []


def load_shapefile_features(
    shp_path: Path,
    transformer: Any | None,
    stats: GeometryStats,
) -> tuple[list[PolygonFeature], Sequence[float] | None]:
    ensure_dependency("pyshp", shapefile)
    try:
        reader = shapefile.Reader(str(shp_path))
    except FileNotFoundError as exc:
        raise GeoJsonError(f"Input file not found: {shp_path}") from exc
    except shapefile.ShapefileException as exc:
        raise GeoJsonError(f"Failed to open shapefile: {shp_path}") from exc

    features: list[PolygonFeature] = []
    try:
        bbox = getattr(reader, "bbox", None)
        for index, shape_record in enumerate(reader.iterShapeRecords()):
            properties = shape_record.record.as_dict()
            geometry = shape_record.shape.__geo_interface__
            feature_id = f"record{index}"
            features.extend(
                iter_polygon_features_from_geometry(
                    geometry=geometry,
                    properties=properties,
                    feature_id=feature_id,
                    transformer=transformer,
                    stats=stats,
                )
            )
        return features, bbox
    finally:
        reader.close()


def main() -> int:
    args = parse_args()

    shp_path = Path(args.input)
    if shp_path.suffix.lower() != ".shp":
        raise GeoJsonError("Input must be a .shp file")

    prj_path = resolve_prj_path(shp_path, args.prj)
    source_crs = load_source_crs(prj_path)

    bbox: Sequence[float] | None = None
    if shapefile is not None:
        try:
            reader = shapefile.Reader(str(shp_path))
            try:
                bbox = getattr(reader, "bbox", None)
            finally:
                reader.close()
        except Exception:
            bbox = None

    coordinate_resolution = resolve_coordinates(
        requested_mode=args.coordinates,
        source_crs=source_crs,
        prj_path=prj_path,
        bbox=bbox,
    )
    transformer = build_transformer(
        source_crs=coordinate_resolution.source_crs,
        enabled=coordinate_resolution.transform_to_wgs84,
    )

    height_names = parse_property_names(args.height_properties)
    level_names = parse_property_names(args.level_properties)
    base_height_names = parse_property_names(args.base_height_properties)

    stats = GeometryStats()
    features, loaded_bbox = load_shapefile_features(shp_path, transformer, stats)
    if bbox is None:
        bbox = loaded_bbox
    if not features:
        raise GeoJsonError("No Polygon or MultiPolygon geometry found in shapefile")

    projector = choose_projector(
        coordinates=coordinate_resolution.coordinates,
        origin=args.origin,
        positions=collect_positions(features),
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

    if prj_path is not None and prj_path.exists():
        print(f"Source PRJ: {prj_path}")
    if source_crs is not None:
        print(f"Source CRS: {describe_crs(source_crs)}")
    print(f"Coordinate handling: {coordinate_resolution.description}")
    if coordinate_resolution.coordinates == "geographic":
        print(f"Origin (lon, lat): {projector.origin_x:.8f}, {projector.origin_y:.8f}")
    else:
        print(f"Origin (x, y): {projector.origin_x:.6f}, {projector.origin_y:.6f}")
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
