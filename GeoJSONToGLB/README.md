# GeoJSONToGLB

把 `GeoJSON` 里的 `Polygon` / `MultiPolygon` 挤出成一个 `GLB` 模型，输出坐标为局部坐标、`Y-up`。

适合先把建筑面数据转成单个 `glb`，再接仓库里的 `glb_to_tileset.py` 或 `glb_to_tileset_quadtree.py`。

## 已实现

- 支持 `FeatureCollection`、`Feature`、`GeometryCollection`
- 支持 `Polygon`、`MultiPolygon`
- 支持洞面（有 `numpy + mapbox_earcut` 时）
- 支持常见高度字段：
  - 高度：`height`、`building:height`、`building_height`、`render_height`、`extrudedHeight`
  - 楼层：`building:levels`、`levels`、`render_levels`
  - 底高：`min_height`、`base_height`、`building:min_height`、`floor_height`
- 支持 GeoJSON 三维坐标的第 3 维作为默认底高

## 用法

默认按标准 GeoJSON 解释为 `WGS84 lon/lat`：

```bash
python3 GeoJSONToGLB/geojson_to_glb.py input.geojson output.glb
```

如果输入是平面坐标（例如米制投影坐标），显式指定：

```bash
python3 GeoJSONToGLB/geojson_to_glb.py input.geojson output.glb \
  --coordinates planar
```

`planar` 模式下，脚本会优先读取 GeoJSON 根对象里的 `crs` 元数据来输出经纬度原点和边界；如果没有可读的 `crs`，会兼容回退到 `EPSG:4546`。

自定义高度字段和默认高度：

```bash
python3 GeoJSONToGLB/geojson_to_glb.py input.geojson output.glb \
  --default-height 18 \
  --height-properties height,my_height \
  --base-height-properties min_height,base_z
```

脚本会打印本次使用的局部原点和边界范围：

- `geographic` 模式输出：
  - 原点 `Longitude` / `Latitude`
  - 边界 `West Longitude` / `East Longitude`
  - 边界 `South Latitude` / `North Latitude`
  - 局部平面 `Local X min/max` / `Local Y min/max`
- `planar` 模式输出：
  - 原点 `Projected X` / `Projected Y`
  - `Source CRS`
  - 输入范围 `Source X min/max` / `Source Y min/max`
  - 局部平面 `Local X min/max` / `Local Y min/max`
  - 如果安装了 `pyproj`，还会按 `Source CRS` 额外输出转换后的经纬度边界

其中 `Local X/Y` 表示相对原点平移后的局部平面范围；`geographic` 模式下该范围单位为米。

后续如果要接 3D Tiles，可把这个原点用于定位。

## 参数

```bash
python3 GeoJSONToGLB/geojson_to_glb.py --help
```

主要参数：

- `--coordinates geographic|planar`
- `--origin X Y`
- `--default-height 10`
- `--level-height 3`
- `--height-properties ...`
- `--level-properties ...`
- `--base-height-properties ...`

## 输入示例

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "properties": {
        "name": "Building A",
        "height": 36
      },
      "geometry": {
        "type": "Polygon",
        "coordinates": [
          [
            [121.4737, 31.2304],
            [121.4740, 31.2304],
            [121.4740, 31.2301],
            [121.4737, 31.2301],
            [121.4737, 31.2304]
          ]
        ]
      }
    }
  ]
}
```

## 串到 3D Tiles

```bash
python3 GeoJSONToGLB/geojson_to_glb.py buildings.geojson buildings.glb
python3 tools/glb_to_tileset.py buildings.glb geojson_tiles/
```

## 限制

- 只处理面要素，不处理 `Point` / `LineString`
- 输出是单个 mesh、单个默认材质
- 没有 `numpy + mapbox_earcut` 时，带洞 `Polygon` 会退化为忽略洞
