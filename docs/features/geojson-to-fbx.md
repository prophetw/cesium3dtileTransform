# GeoJSONToFBX

## 功能目标

把 GeoJSON 面数据转换成单个 FBX 模型，面要素会按高度字段挤出成立体建筑或地块体块。输出使用局部坐标、单位米、`Y-up`，与现有 `GeoJSONToGLB` 输出语义保持一致。

## 核心入口

- 命令入口：`GeoJSONToFBX/geojson_to_fbx.py`
- 常用命令：`python3 GeoJSONToFBX/geojson_to_fbx.py input.geojson output.fbx`
- Blender 路径参数：`--blender /path/to/blender`

## 架构关系

`GeoJSONToFBX` 复用 `GeoJSONToGLB.geojson_to_glb` 的 GeoJSON 读取、坐标投影、高度字段解析、面挤出和边界打印能力。新增部分只负责把 `positions / normals / indices` 写入临时 mesh JSON，并调用 Blender 后台脚本导出 FBX。

## 数据流

1. 读取 GeoJSON，并收集 `Polygon` / `MultiPolygon`。
2. 根据 `--coordinates` 和 `--origin` 选择局部投影原点。
3. 复用 `build_geometry()` 生成 `Y-up` 三角网格。
4. 写入临时 mesh JSON。
5. 调用 `blender --background --python <script> -- <mesh.json> <output.fbx>`。
6. Blender 脚本将 `Y-up` 坐标临时转成 Blender 内部 `Z-up`，再用 FBX exporter 以 `axis_up="Y"` 导出。
7. 校验输出 FBX 存在且非空，并打印原点、边界和统计信息。

## 关键数据结构

- `positions`: `[(x, y, z), ...]`，局部坐标，`Y` 为向上方向。
- `normals`: `[(nx, ny, nz), ...]`，与 `positions` 一一对应。
- `indices`: `[i0, i1, i2, ...]`，三角形索引，长度必须能被 3 整除。
- 临时 mesh JSON：包含 `mesh_name`、`material_name`、`positions`、`normals`、`indices`。

## 外部依赖或 API

- Blender 命令行：用于后台执行 Python 脚本。
- Blender Python API：
  - `bpy.data.meshes.new()`
  - `Mesh.from_pydata()`
  - `bpy.ops.export_scene.fbx()`
- 可选 Python 依赖沿用 `GeoJSONToGLB`：`numpy`、`mapbox_earcut`、`pyproj`。

## 异常路径

- GeoJSON 不存在、不可解码或没有面要素时，抛出 `GeoJsonError`。
- 挤出后没有生成 mesh，抛出 `GeoJsonError`。
- `indices` 数量不是 3 的倍数，抛出 `GeoJsonError`。
- 找不到 Blender 可执行文件时，提示安装 Blender 或传入 `--blender`。
- Blender 退出码非 0 时，回传 Blender 的 stdout/stderr。
- 输出 FBX 不存在或为空时，抛出 `GeoJsonError`。

## 测试验证方式

```bash
python3 GeoJSONToFBX/geojson_to_fbx.py --help
python3 GeoJSONToFBX/geojson_to_fbx.py sample.geojson out/sample.fbx
python3 GeoJSONToGLB/geojson_to_glb.py sample.geojson out/sample.glb
```

在未安装 Blender 的环境中，第二条命令应输出清晰的依赖错误。在安装 Blender 的环境中，应生成非空 FBX，并可导入 Blender 检查 mesh、材质、轴向和高度。

## 变更记录

- 2026-05-11：新增 `GeoJSONToFBX` 转换器设计与实现文档。
