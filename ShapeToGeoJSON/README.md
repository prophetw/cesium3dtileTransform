# ShapeToGeoJSON

把 `ESRI Shapefile` 转成一个 `GeoJSON FeatureCollection`。

实现文件：

- `ShapeToGeoJSON/shape_to_geojson.py`

适合先把 `shp` 数据统一转成 `GeoJSON`，后续再接仓库里的 `GeoJSONToGLB/geojson_to_glb.py`。

## 已实现

- 读取 `shp + shx + dbf + cpg`
- 支持 `.prj` 自动识别坐标系
- `.prj` 为投影坐标时，自动重投影到 `WGS84`
- 支持 `Point`、`LineString`、`Polygon`、`Multi*`、`GeometryCollection`
- 保留 `dbf` 属性到 `GeoJSON properties`
- 输出 `FeatureCollection.bbox`

## 用法

最常用：

```bash
python3 ShapeToGeoJSON/shape_to_geojson.py input.shp output.geojson
```

如果 `input.shp` 同目录下有 `input.prj`：

- 地理坐标系会直接按地理坐标输出
- 投影坐标系会自动重投影到 `WGS84`

如果没有 `.prj`，脚本会：

- bbox 看起来像经纬度时，按地理坐标输出
- 否则按平面坐标原样输出

强制按平面坐标输出：

```bash
python3 ShapeToGeoJSON/shape_to_geojson.py input.shp output.geojson \
  --coordinates planar
```

显式指定 `.prj` 路径：

```bash
python3 ShapeToGeoJSON/shape_to_geojson.py input.shp output.geojson \
  --prj path/to/input.prj
```

格式化输出 JSON：

```bash
python3 ShapeToGeoJSON/shape_to_geojson.py input.shp output.geojson --pretty
```

如果 `dbf` 不是 UTF-8，可显式指定编码：

```bash
python3 ShapeToGeoJSON/shape_to_geojson.py input.shp output.geojson \
  --encoding gbk
```

## 测试数据

仓库自带样例：

```bash
python3 ShapeToGeoJSON/shape_to_geojson.py ShapeToGLB/test/省级.shp out/province.geojson --pretty
```

## 参数

```bash
python3 ShapeToGeoJSON/shape_to_geojson.py --help
```

主要参数：

- `--coordinates auto|geographic|planar`
- `--prj path/to/file.prj`
- `--encoding utf-8`
- `--encoding-errors strict`
- `--pretty`

## 串到 GeoJSONToGLB

如果输出是标准经纬度 `GeoJSON`：

```bash
python3 ShapeToGeoJSON/shape_to_geojson.py input.shp buildings.geojson
python3 GeoJSONToGLB/geojson_to_glb.py buildings.geojson buildings.glb
```

如果输出是平面坐标 `GeoJSON`：

```bash
python3 ShapeToGeoJSON/shape_to_geojson.py input.shp buildings.geojson \
  --coordinates planar
python3 GeoJSONToGLB/geojson_to_glb.py buildings.geojson buildings.glb \
  --coordinates planar
```
