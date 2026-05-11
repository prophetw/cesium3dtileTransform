#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from datetime import date, datetime
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


WGS84_EPSG = 4326

SUPPORTED_GEOMETRY_TYPES = {
    "Point",
    "MultiPoint",
    "LineString",
    "MultiLineString",
    "Polygon",
    "MultiPolygon",
    "GeometryCollection",
}

CPG_ENCODING_ALIASES = {
    "65001": "utf-8",
    "utf8": "utf-8",
}


class ShapeToGeoJsonError(RuntimeError):
    pass


@dataclass(frozen=True)
class CoordinateResolution:
    coordinates: str
    transform_to_wgs84: bool
    source_crs: Any | None
    prj_path: Path | None
    description: str


@dataclass
class ExportStats:
    feature_count: int = 0
    empty_geometry_count: int = 0
    unsupported_geometry_count: int = 0
    geometry_counts: dict[str, int] = field(default_factory=dict)


@dataclass
class Bounds2D:
    min_x: float = math.inf
    min_y: float = math.inf
    max_x: float = -math.inf
    max_y: float = -math.inf

    def update(self, x: float, y: float) -> None:
        self.min_x = min(self.min_x, x)
        self.min_y = min(self.min_y, y)
        self.max_x = max(self.max_x, x)
        self.max_y = max(self.max_y, y)

    def to_bbox(self) -> list[float] | None:
        if math.isinf(self.min_x) or math.isinf(self.min_y):
            return None
        return [self.min_x, self.min_y, self.max_x, self.max_y]


def warn(message: str) -> None:
    print(f"[shape_to_geojson] {message}", file=sys.stderr)


def ensure_dependency(name: str, module: Any) -> None:
    if module is None:
        raise ShapeToGeoJsonError(f"Missing dependency: {name}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert ESRI Shapefile data to GeoJSON")
    parser.add_argument("input", help="Input shapefile (.shp)")
    parser.add_argument("output", help="Output GeoJSON file")
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
        "--prj",
        help="Optional .prj path. Defaults to the file next to the input .shp",
    )
    parser.add_argument(
        "--encoding",
        help="Optional DBF text encoding. Defaults to sibling .cpg when available.",
    )
    parser.add_argument(
        "--encoding-errors",
        default="strict",
        help="DBF decoding error handling passed to pyshp (default: strict)",
    )
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON (indent=2)")
    return parser.parse_args()


def resolve_prj_path(shp_path: Path, explicit_prj: str | None) -> Path | None:
    if explicit_prj:
        return Path(explicit_prj)

    sibling_prj = shp_path.with_suffix(".prj")
    if sibling_prj.exists():
        return sibling_prj
    return None


def load_source_crs(prj_path: Path | None) -> Any | None:
    if prj_path is None or not prj_path.exists():
        return None

    if CRS is None:
        raise ShapeToGeoJsonError("pyproj is required to read .prj files")

    try:
        prj_text = prj_path.read_text(encoding="utf-8", errors="ignore")
    except OSError as exc:
        raise ShapeToGeoJsonError(f"Unable to read PRJ file: {prj_path}") from exc

    try:
        return CRS.from_wkt(prj_text)
    except Exception as exc:  # pragma: no cover - pyproj error type varies by version
        raise ShapeToGeoJsonError(f"Failed to parse PRJ file: {prj_path}") from exc


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
                "geographic mode requested explicitly"
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
    if not enabled or source_crs is None:
        return None
    if CRS is None or Transformer is None:
        raise ShapeToGeoJsonError("pyproj is required to transform shapefile coordinates to WGS84")
    try:
        return Transformer.from_crs(source_crs, CRS.from_epsg(WGS84_EPSG), always_xy=True)
    except Exception as exc:  # pragma: no cover - pyproj error type varies by version
        raise ShapeToGeoJsonError(f"Failed to build CRS transformer from {describe_crs(source_crs)} to WGS84") from exc


def read_cpg_encoding(shp_path: Path) -> str | None:
    cpg_path = shp_path.with_suffix(".cpg")
    if not cpg_path.exists():
        return None

    try:
        raw_value = cpg_path.read_text(encoding="ascii", errors="ignore").strip().lstrip("\ufeff")
    except OSError:
        return None

    if not raw_value:
        return None

    alias = CPG_ENCODING_ALIASES.get(raw_value.lower())
    if alias:
        return alias
    return raw_value


def open_reader(shp_path: Path, encoding: str | None, encoding_errors: str) -> Any:
    ensure_dependency("pyshp", shapefile)

    selected_encoding = encoding or read_cpg_encoding(shp_path)
    reader_kwargs: dict[str, Any] = {"encodingErrors": encoding_errors}
    if selected_encoding:
        reader_kwargs["encoding"] = selected_encoding

    try:
        return shapefile.Reader(str(shp_path), **reader_kwargs)
    except FileNotFoundError as exc:
        raise ShapeToGeoJsonError(f"Input file not found: {shp_path}") from exc
    except LookupError as exc:
        raise ShapeToGeoJsonError(f"Unsupported DBF encoding: {selected_encoding}") from exc
    except shapefile.ShapefileException as exc:
        raise ShapeToGeoJsonError(f"Failed to open shapefile: {shp_path}") from exc


def is_position(value: Any) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) >= 2
        and not isinstance(value[0], (list, tuple))
        and not isinstance(value[1], (list, tuple))
    )


def transform_position(position: Sequence[Any], transformer: Any | None) -> list[float]:
    x = float(position[0])
    y = float(position[1])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ShapeToGeoJsonError("Encountered non-finite coordinate in source geometry")

    if transformer is None:
        transformed_x = x
        transformed_y = y
    else:
        transformed_x, transformed_y = transformer.transform(x, y)

    if not math.isfinite(transformed_x) or not math.isfinite(transformed_y):
        raise ShapeToGeoJsonError("Encountered non-finite coordinate while transforming geometry")

    output = [transformed_x, transformed_y]
    if len(position) >= 3 and position[2] is not None:
        z = float(position[2])
        if not math.isfinite(z):
            raise ShapeToGeoJsonError("Encountered non-finite Z value in source geometry")
        output.append(z)
    return output


def transform_nested_coordinates(value: Any, transformer: Any | None) -> Any:
    if is_position(value):
        return transform_position(value, transformer)
    if isinstance(value, (list, tuple)):
        transformed_items = []
        for item in value:
            transformed_item = transform_nested_coordinates(item, transformer)
            if transformed_item is not None:
                transformed_items.append(transformed_item)
        return transformed_items
    return None


def transform_geometry(geometry: Any, transformer: Any | None, stats: ExportStats) -> dict[str, Any] | None:
    if not isinstance(geometry, dict):
        return None

    geometry_type = geometry.get("type")
    if geometry_type in (None, "Null"):
        return None

    if geometry_type == "GeometryCollection":
        raw_geometries = geometry.get("geometries")
        if not isinstance(raw_geometries, list):
            return None
        transformed_geometries = []
        for raw_geometry in raw_geometries:
            transformed_geometry = transform_geometry(raw_geometry, transformer, stats)
            if transformed_geometry is not None:
                transformed_geometries.append(transformed_geometry)
        return {"type": "GeometryCollection", "geometries": transformed_geometries}

    if geometry_type not in SUPPORTED_GEOMETRY_TYPES:
        stats.unsupported_geometry_count += 1
        return None

    coordinates = transform_nested_coordinates(geometry.get("coordinates"), transformer)
    if coordinates is None:
        return None
    return {"type": geometry_type, "coordinates": coordinates}


def iter_positions_in_coordinates(value: Any) -> Iterable[Sequence[float]]:
    if is_position(value):
        yield value
        return
    if isinstance(value, (list, tuple)):
        for item in value:
            yield from iter_positions_in_coordinates(item)


def iter_positions_in_geometry(geometry: dict[str, Any] | None) -> Iterable[Sequence[float]]:
    if not geometry:
        return

    geometry_type = geometry.get("type")
    if geometry_type == "GeometryCollection":
        for nested_geometry in geometry.get("geometries", []):
            yield from iter_positions_in_geometry(nested_geometry)
        return

    yield from iter_positions_in_coordinates(geometry.get("coordinates"))


def normalize_json_value(value: Any) -> Any:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, str):
        return value
    if isinstance(value, (date, datetime)):
        return value.isoformat()
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    if isinstance(value, dict):
        return {str(key): normalize_json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [normalize_json_value(item) for item in value]
    return str(value)


def normalize_properties(properties: dict[str, Any]) -> dict[str, Any]:
    return {str(key): normalize_json_value(value) for key, value in properties.items()}


def record_geometry_type(stats: ExportStats, geometry: dict[str, Any] | None) -> None:
    if geometry is None:
        stats.empty_geometry_count += 1
        return

    geometry_type = geometry.get("type")
    if not isinstance(geometry_type, str):
        stats.empty_geometry_count += 1
        return
    stats.geometry_counts[geometry_type] = stats.geometry_counts.get(geometry_type, 0) + 1


def format_geometry_counts(geometry_counts: dict[str, int]) -> str:
    if not geometry_counts:
        return "no geometry"
    parts = [f"{name}={count}" for name, count in sorted(geometry_counts.items())]
    return ", ".join(parts)


def export_geojson(
    shp_path: Path,
    reader: Any,
    transformer: Any | None,
    output_path: Path,
    pretty: bool,
) -> ExportStats:
    stats = ExportStats()
    bounds = Bounds2D()
    features: list[dict[str, Any]] = []

    try:
        shape_records = reader.iterShapeRecords()
        for index, shape_record in enumerate(shape_records):
            properties = normalize_properties(shape_record.record.as_dict(date_strings=True))
            geometry = transform_geometry(shape_record.shape.__geo_interface__, transformer, stats)
            feature = {
                "type": "Feature",
                "id": index,
                "properties": properties,
                "geometry": geometry,
            }
            features.append(feature)

            stats.feature_count += 1
            record_geometry_type(stats, geometry)
            for position in iter_positions_in_geometry(geometry):
                bounds.update(float(position[0]), float(position[1]))
    except UnicodeDecodeError as exc:
        raise ShapeToGeoJsonError(
            "Failed to decode DBF records; try --encoding or check the sibling .cpg file."
        ) from exc
    except shapefile.ShapefileException as exc:
        raise ShapeToGeoJsonError(f"Failed to read shapefile records from {shp_path}") from exc

    collection: dict[str, Any] = {
        "type": "FeatureCollection",
        "features": features,
    }
    bbox = bounds.to_bbox()
    if bbox is not None:
        collection["bbox"] = bbox

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as fp:
        json.dump(
            collection,
            fp,
            ensure_ascii=False,
            indent=2 if pretty else None,
            allow_nan=False,
        )
        fp.write("\n")

    return stats


def main() -> int:
    args = parse_args()

    shp_path = Path(args.input)
    if shp_path.suffix.lower() != ".shp":
        raise ShapeToGeoJsonError("Input must be a .shp file")

    output_path = Path(args.output)
    prj_path = resolve_prj_path(shp_path, args.prj)
    source_crs = load_source_crs(prj_path)

    reader = open_reader(shp_path, encoding=args.encoding, encoding_errors=args.encoding_errors)
    try:
        bbox = getattr(reader, "bbox", None)
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

        stats = export_geojson(
            shp_path=shp_path,
            reader=reader,
            transformer=transformer,
            output_path=output_path,
            pretty=args.pretty,
        )
    finally:
        reader.close()

    if stats.feature_count == 0:
        raise ShapeToGeoJsonError("No records found in shapefile")

    if prj_path is not None and prj_path.exists():
        print(f"Source PRJ: {prj_path}")
    if source_crs is not None:
        print(f"Source CRS: {describe_crs(source_crs)}")
    print(f"Coordinate handling: {coordinate_resolution.description}")
    print(
        f"Exported {output_path}: {stats.feature_count} feature(s), "
        f"{format_geometry_counts(stats.geometry_counts)}."
    )
    if stats.empty_geometry_count:
        warn(f"Emitted {stats.empty_geometry_count} feature(s) with null geometry")
    if stats.unsupported_geometry_count:
        warn(f"Ignored {stats.unsupported_geometry_count} unsupported geometry object(s)")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ShapeToGeoJsonError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
