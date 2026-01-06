# 3DTileTransformer（学习性质）

把一个 3D 模型转换成 3D Tiles：`tileset.json` +（后续可加）分块/LOD 内容。

## V1（第一版本：最简单可跑）

- 输入：单个 `*.glb`
- 输出：单个 tile 的 `tileset.json` + 原样复制的 `*.glb`
- 脚本：`tools/glb_to_tileset.py`（零依赖）

### 用法

```bash
python3 tools/glb_to_tileset.py path/to/model.glb out_tiles/
```

默认会把输出放到 `out/` 下，例如 `out/out_tiles/tileset.json`。如需保持旧路径可加 `--output-root .`。

输出目录：

- `out/out_tiles/tileset.json`
- `out/out_tiles/model.glb`

### 本地验证（静态服务器）

```bash
python3 -m http.server 8000 --directory out/out_tiles
```

然后在客户端加载 `http://localhost:8000/tileset.json`。

## V3（切块 + 采样 LOD，试验版）

- 输入：单个 `*.glb`
- 输出：quadtree tileset（叶子 tile + root LOD）
- 脚本：`tools/glb_to_tileset_quadtree.py`

### 用法

```bash
python3 tools/glb_to_tileset_quadtree.py path/to/model.glb out_tiles/ \
  --max-tris 200000 --simplify-ratio 0.15 --pretty
```

默认会把输出放到 `out/` 下，例如 `out/out_tiles/tileset.json`。如需保持旧路径可加 `--output-root .`。

输出目录：

- `out/out_tiles/tileset.json`
- `out/out_tiles/tiles/root_simplified.glb`（`--simplify-ratio < 1` 时）
- `out/out_tiles/tiles/L{depth}_X{x}_Y{y}.glb`
- `out/out_tiles/tiles/textures/`（`--external-textures` 时，纹理只导出一份供所有 tile 共享）

说明：

- 按节点 AABB 中心点做 quadtree 分块（不切 mesh）
- LOD 目前只在 root 生成（按三角形均匀采样，默认重建顶点缓冲；可用 `--no-rebuild-vertices` 关闭）
- 纹理/材质会被拷贝到每个 tile 内（未做共享）
- 如觉得输出体积过大，可启用 `--external-textures` 避免每个 GLB 内重复内嵌纹理

## V4（多层 LOD，父粗子细）

- 输入：单个 `*.glb`
- 输出：quadtree tileset（叶子 tile + 多层 LOD）
- 脚本：`tools/glb_to_tileset_quadtree.py`

### 用法

```bash
python3 tools/glb_to_tileset_quadtree.py path/to/model.glb out_tiles/ \
  --max-tris 200000 --simplify-ratio 0.15 --simplify-method meshopt \
  --lod-internal --lod-ratio-scale 2.0 --simplify-target-tris 200000 \
  --split-mesh --draco --ktx2 --ktx2-mode etc1s --pretty
```

输出目录（新增）：

- `out/out_tiles/tiles/N{depth}_X{x}_Y{y}.glb`（内部节点 LOD）

说明：

- 在每个内部节点生成采样 LOD（比 V3 多层）
- LOD 简化比例按深度递增：`simplify_ratio * (lod_ratio_scale ** depth)`，超过 1 则视为不生成
- root 仍输出为 `tiles/root_simplified.glb`
- 可用 `--simplify-target-tris` 限制每个 LOD tile 的目标三角形数，避免父级 glb 过大
- 可用 `--split-mesh` 把超大 leaf tile 按三角形中心拆分为更小子瓦片（仅支持 TRIANGLES，且 mesh 不可被多个 node 复用）
- 可用 `--draco` 对输出 GLB 进行 Draco 压缩（依赖 `gltf-transform` 或 `gltf-pipeline`，可用 `--draco-tool` 指定）
- 可用 `--ktx2` 把纹理压缩为 KTX2（ETC1S/UASTC），依赖 `gltf-transform` + KTX-Software (`toktx`)
- 可用 `--min-half-extent`/`--min-half-extent-factor`/`--min-half-extent-ratio` 增大 boundingVolume，避免近景角度出现裁剪
- `gltf-transform` 安装：`npm i -g @gltf-transform/cli`；`gltf-pipeline` 安装：`npm i -g gltf-pipeline`
- `toktx` 来自 KTX-Software（https://github.com/KhronosGroup/KTX-Software）
- `--simplify-method meshopt` 需要安装 `numpy` + `meshoptimizer`（`pip install meshoptimizer`）
- meshopt 会尽量利用 `NORMAL` / `TEXCOORD_0` 做保真简化（若存在），可用 `--meshopt-error` 调整误差目标；部分环境会自动回退到位置简化

## 地理定位（用户输入）

模型是“纯局部坐标”时，可以在导出后让用户输入地理坐标，直接修改 `root.transform`：

1) 准备 `georef.json`（可参考 `georef.example.json`）
2) 更新 tileset：

```bash
python3 tools/update_transform.py out/out_tiles/tileset.json georef.example.json --pretty
```

`georef.json` 字段说明（最小集）：

- `origin`: `[lon, lat, height]`（WGS84，经纬度/米）
- `heading/pitch/roll`: 右手系旋转角（度），按 `Z->Y->X` 顺序应用
- `axis`: `Z_UP` 或 `Y_UP`（模型上方向）
- `unit`: `m|cm|mm|ft|in`

提示：最好在导入阶段把模型统一到米 + Z_UP；如果还没做，可以通过 `axis/unit` 做基础修正。

### 运行时定位（CesiumJS 示例）

```js
const tileset = await Cesium.Cesium3DTileset.fromUrl(url);
const origin = Cesium.Cartesian3.fromDegrees(lon, lat, height);
const hpr = new Cesium.HeadingPitchRoll(
  Cesium.Math.toRadians(heading),
  Cesium.Math.toRadians(pitch),
  Cesium.Math.toRadians(roll)
);
tileset.modelMatrix = Cesium.Transforms.headingPitchRollToFixedFrame(origin, hpr);
```

## 限制（V1）

- 仅支持 `.glb`（GLB 2.0）
- `POSITION` 仅支持 `float32`，且不支持 sparse accessor

## 限制（V3/V4）

- 仅支持 `TRIANGLES` primitive
- 不支持 skin/animation/Draco 输入/sparse accessor
- 简化为三角形采样（不是 QEM 等真实简化），外观/拓扑不保证最优
- `--external-textures` 会让 GLB 依赖外部纹理文件（不再是单文件自包含）

## 文档

- `doc.md`：实现思路/术语/后续路线



## 命令
```bash

# osm pbf => glb
python3 tools/osm_to_glb.py shanghai-260105.osm.pbf shanghai.glb

# osm => glb
python3 tools/osm_to_glb.py shanghai-260105.osm shanghai.glb

# 
python3 tools/glb_to_tileset_quadtree.py shanghai.glb  out_shanghai_osm_tiles/ \
  --max-tris 100000 --max-depth 10 --split-mesh \
  --simplify-ratio 0.15 --simplify-method meshopt \
  --lod-internal --lod-ratio-scale 2.0 --simplify-target-tris 200000 \
  --external-textures --draco --draco-tool gltf-transform \
  --ktx2 --ktx2-mode etc1s --ktx2-quality 128 \
  --min-half-extent 0.5 --pretty \
  --y-up
```