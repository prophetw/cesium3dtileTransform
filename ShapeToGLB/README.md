# ShapeToGLB

把 `ESRI Shapefile` 里的 `Polygon` / `MultiPolygon` 挤出成单个 `GLB` 模型，输出坐标为局部坐标、`Y-up`。

实现文件：

- `ShapeToGLB/shape_to_glb.py`

适合把行政区、地块、建筑面等 `shp` 数据先转成一个 `glb`，再串到仓库里的 `glb_to_tileset.py` 或 `glb_to_tileset_quadtree.py`。

## 已实现

- 读取 `shp + shx + dbf + cpg`
- 支持 `.prj` 自动识别坐标系
- `.prj` 为投影坐标时，自动重投影到 `WGS84`
- 支持 `Polygon`、`MultiPolygon`
- 支持复用 `GeoJSONToGLB` 的高度字段规则
- 输出局部坐标原点，便于后续做地理定位

## 用法

最常用：

```bash
python3 ShapeToGLB/shape_to_glb.py input.shp output.glb
```

如果 `input.shp` 同目录下有 `input.prj`：

- 地理坐标系（例如 `EPSG:4326`）会直接按经纬度处理
- 投影坐标系（例如 Web Mercator、UTM）会自动按 `.prj` 重投影到 `WGS84`

如果没有 `.prj`，脚本会：

- bbox 看起来像经纬度时，按地理坐标处理
- 否则按平面坐标处理

强制按平面坐标解释：

```bash
python3 ShapeToGLB/shape_to_glb.py input.shp output.glb \
  --coordinates planar
```

显式指定 `.prj` 路径：

```bash
python3 ShapeToGLB/shape_to_glb.py input.shp output.glb \
  --prj path/to/input.prj
```

自定义高度字段和默认高度：

```bash
python3 ShapeToGLB/shape_to_glb.py input.shp output.glb \
  --default-height 30 \
  --height-properties height,HEIGHT,H
```

## 测试数据

仓库自带样例：

```bash
python3 ShapeToGLB/shape_to_glb.py ShapeToGLB/test/省级.shp out/province.glb
```

该样例自带：

- `ShapeToGLB/test/省级.shp`
- `ShapeToGLB/test/省级.dbf`
- `ShapeToGLB/test/省级.prj`

其中 `.prj` 是 `WGS84 / EPSG:4326`，脚本会自动识别。

## 参数

```bash
python3 ShapeToGLB/shape_to_glb.py --help
```

主要参数：

- `--coordinates auto|geographic|planar`
- `--origin X Y`
- `--prj path/to/file.prj`
- `--default-height 10`
- `--level-height 3`
- `--height-properties ...`
- `--level-properties ...`
- `--base-height-properties ...`

## 串到 3D Tiles

```bash
python3 ShapeToGLB/shape_to_glb.py ShapeToGLB/test/省级.shp province.glb
python3 tools/glb_to_tileset.py province.glb province_tiles/
```
