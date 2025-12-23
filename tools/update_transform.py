#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any


WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)


class GeorefError(RuntimeError):
    pass


def _deg_to_rad(value: float) -> float:
    return value * math.pi / 180.0


def _mat4_identity() -> list[float]:
    return [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]


def _mat4_mul(a: list[float], b: list[float]) -> list[float]:
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


def _mat4_scale(sx: float, sy: float, sz: float) -> list[float]:
    return [sx, 0.0, 0.0, 0.0, 0.0, sy, 0.0, 0.0, 0.0, 0.0, sz, 0.0, 0.0, 0.0, 0.0, 1.0]


def _mat4_rot_x(rad: float) -> list[float]:
    c = math.cos(rad)
    s = math.sin(rad)
    return [1.0, 0.0, 0.0, 0.0, 0.0, c, s, 0.0, 0.0, -s, c, 0.0, 0.0, 0.0, 0.0, 1.0]


def _mat4_rot_y(rad: float) -> list[float]:
    c = math.cos(rad)
    s = math.sin(rad)
    return [c, 0.0, -s, 0.0, 0.0, 1.0, 0.0, 0.0, s, 0.0, c, 0.0, 0.0, 0.0, 0.0, 1.0]


def _mat4_rot_z(rad: float) -> list[float]:
    c = math.cos(rad)
    s = math.sin(rad)
    return [c, s, 0.0, 0.0, -s, c, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]


def _ecef_from_enu_matrix(origin_lon_lat_h: tuple[float, float, float]) -> list[float]:
    lon_deg, lat_deg, height = origin_lon_lat_h
    lon = _deg_to_rad(lon_deg)
    lat = _deg_to_rad(lat_deg)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + height) * cos_lat * cos_lon
    y = (n + height) * cos_lat * sin_lon
    z = ((1.0 - WGS84_E2) * n + height) * sin_lat

    east = (-sin_lon, cos_lon, 0.0)
    north = (-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat)
    up = (cos_lat * cos_lon, cos_lat * sin_lon, sin_lat)

    return [
        east[0],
        east[1],
        east[2],
        0.0,
        north[0],
        north[1],
        north[2],
        0.0,
        up[0],
        up[1],
        up[2],
        0.0,
        x,
        y,
        z,
        1.0,
    ]


def _parse_unit_scale(unit: str) -> float:
    unit_norm = unit.strip().lower()
    scale_map = {
        "m": 1.0,
        "meter": 1.0,
        "meters": 1.0,
        "cm": 0.01,
        "centimeter": 0.01,
        "centimeters": 0.01,
        "mm": 0.001,
        "millimeter": 0.001,
        "millimeters": 0.001,
        "ft": 0.3048,
        "foot": 0.3048,
        "feet": 0.3048,
        "in": 0.0254,
        "inch": 0.0254,
        "inches": 0.0254,
    }
    if unit_norm not in scale_map:
        raise GeorefError(f"Unsupported unit: {unit}")
    return scale_map[unit_norm]


def _parse_float(value: Any, name: str) -> float:
    if isinstance(value, (int, float)):
        value = float(value)
    else:
        raise GeorefError(f"{name} must be a number")
    if not math.isfinite(value):
        raise GeorefError(f"{name} must be finite")
    return value


def _parse_origin(value: Any) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise GeorefError("origin must be [lon, lat, height]")
    lon = _parse_float(value[0], "origin[0]")
    lat = _parse_float(value[1], "origin[1]")
    h = _parse_float(value[2], "origin[2]")
    return (lon, lat, h)


def _parse_config(data: dict[str, Any]) -> dict[str, Any]:
    mode = str(data.get("mode", "enu")).lower()
    if mode not in {"enu", "local"}:
        raise GeorefError(f"Unsupported mode: {mode}")

    origin = None
    if mode == "enu":
        if "origin" not in data:
            raise GeorefError("mode=enu requires origin")
        origin = _parse_origin(data["origin"])

    axis = str(data.get("axis", "Z_UP")).upper()
    if axis not in {"Z_UP", "Y_UP"}:
        raise GeorefError(f"Unsupported axis: {axis} (expected Z_UP or Y_UP)")

    unit = str(data.get("unit", "m"))
    unit_scale = _parse_unit_scale(unit)
    scale = unit_scale
    if "scale" in data:
        scale *= _parse_float(data["scale"], "scale")

    heading = _parse_float(data.get("heading", 0.0), "heading")
    pitch = _parse_float(data.get("pitch", 0.0), "pitch")
    roll = _parse_float(data.get("roll", 0.0), "roll")

    return {
        "mode": mode,
        "origin": origin,
        "axis": axis,
        "scale": scale,
        "heading": heading,
        "pitch": pitch,
        "roll": roll,
    }


def _build_local_to_enu_matrix(cfg: dict[str, Any]) -> list[float]:
    axis = cfg["axis"]
    if axis == "Z_UP":
        axis_mat = _mat4_identity()
    else:
        axis_mat = _mat4_rot_x(math.radians(90.0))

    scale = cfg["scale"]
    scale_mat = _mat4_scale(scale, scale, scale)

    heading = _deg_to_rad(cfg["heading"])
    pitch = _deg_to_rad(cfg["pitch"])
    roll = _deg_to_rad(cfg["roll"])
    h_mat = _mat4_rot_z(heading)
    p_mat = _mat4_rot_y(pitch)
    r_mat = _mat4_rot_x(roll)
    hpr_mat = _mat4_mul(h_mat, _mat4_mul(p_mat, r_mat))

    return _mat4_mul(hpr_mat, _mat4_mul(axis_mat, scale_mat))


def _build_transform(cfg: dict[str, Any]) -> list[float]:
    local_to_enu = _build_local_to_enu_matrix(cfg)
    if cfg["mode"] == "local":
        return local_to_enu
    ecef_from_enu = _ecef_from_enu_matrix(cfg["origin"])
    return _mat4_mul(ecef_from_enu, local_to_enu)


def _load_json(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text("utf-8"))
    except Exception as exc:  # noqa: BLE001
        raise GeorefError(f"Failed to read JSON: {path} ({exc})") from exc
    if not isinstance(data, dict):
        raise GeorefError(f"JSON root must be an object: {path}")
    return data


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Update tileset.json root.transform using a georeference config.")
    parser.add_argument("tileset", type=Path, help="Input tileset.json path")
    parser.add_argument("georef", type=Path, help="Georeference config JSON path")
    parser.add_argument("--out", type=Path, default=None, help="Output tileset.json path (default: overwrite input)")
    parser.add_argument("--pretty", action="store_true", help="Pretty-print JSON (indent=2)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    tileset_path: Path = args.tileset
    georef_path: Path = args.georef
    out_path: Path = args.out or tileset_path

    if not tileset_path.is_file():
        raise GeorefError(f"tileset.json not found: {tileset_path}")
    if not georef_path.is_file():
        raise GeorefError(f"georef config not found: {georef_path}")

    tileset = _load_json(tileset_path)
    georef = _load_json(georef_path)
    cfg = _parse_config(georef)
    transform = _build_transform(cfg)

    root = tileset.get("root")
    if not isinstance(root, dict):
        raise GeorefError("tileset.root missing or invalid")
    root["transform"] = [round(v, 12) for v in transform]

    out_path.write_text(
        json.dumps(tileset, ensure_ascii=False, indent=2 if args.pretty else None) + "\n",
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except GeorefError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2)
