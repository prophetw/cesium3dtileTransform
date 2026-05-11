# GeoJSONToFBX

把 `GeoJSON` 里的 `Polygon` / `MultiPolygon` 挤出成一个 `FBX` 模型，输出坐标为局部坐标、`Y-up`。

实现文件：

- `GeoJSONToFBX/geojson_to_fbx.py`

该脚本复用 `GeoJSONToGLB` 的解析、投影和挤出建模逻辑，只把最终导出层换成 Blender 后台 FBX 导出。

## 依赖

- Python 依赖同 `GeoJSONToGLB`
- Blender 命令行可执行文件

如果 `blender` 不在 `PATH`，可用 `--blender` 指定路径。

## 用法

默认按标准 GeoJSON 解释为 `WGS84 lon/lat`：

```bash
python3 GeoJSONToFBX/geojson_to_fbx.py input.geojson output.fbx
```

如果输入是平面坐标：

```bash
python3 GeoJSONToFBX/geojson_to_fbx.py input.geojson output.fbx \
  --coordinates planar --blender ~/.local/bin/blender    
```

指定 Blender 路径：

```bash
python3 GeoJSONToFBX/geojson_to_fbx.py input.geojson output.fbx \
  --blender /path/to/blender
```

自定义高度字段和默认高度：

```bash
python3 GeoJSONToFBX/geojson_to_fbx.py input.geojson output.fbx \
  --default-height 18 \
  --height-properties height,my_height \
  --base-height-properties min_height,base_z
```

## 限制

- 只处理面要素，不处理 `Point` / `LineString`
- 输出是单个 mesh、单个默认材质
- 依赖 Blender 的 FBX exporter，当前脚本不直接手写 FBX
