# 3DTileTransformer（学习向）

目标：把 3D 模型转换成可流式加载的 3D Tiles（`tileset.json` + 分块内容）。

## V1（第一版本：最简单可跑）

- 输入：单个 `*.glb`（本地坐标、单位默认米）
- 输出：一个目录：`tileset.json`（单 root，无 children）+ `*.glb`（作为 `content.uri`）
- 不做：切块 / 网格简化 / implicit tiling / 地理定位（`transform` 默认 identity）
- 验收：在支持 3D Tiles 1.1 glTF content 的客户端（如 CesiumJS 新版）能正常加载显示

仓库里提供一个零依赖脚本 `tools/glb_to_tileset.py`：从 GLB 解析整体 AABB，自动生成 `boundingVolume.box`（轴对齐 OBB）。

快速跑通（V1）：

```bash
python3 tools/glb_to_tileset.py path/to/model.glb out_tiles/
python3 -m http.server 8000 --directory out_tiles
```

然后在客户端加载 `http://localhost:8000/tileset.json`。

一句话概括本项目：**把一个 3D 模型“切成一棵可流式加载的空间层级树（BVH）”，再用 `tileset.json` 把这棵树描述出来；LOD（细节层级）就是这棵树的“父粗子细” + `geometricError` 驱动的细化策略。**

下文按“自己实现一套转换器”的思路，先讲清关键概念与 `tileset.json` 字段，再给出实现路线（从 V1 → 逐步加入切块/LOD）。

---

## 1) 先把概念钉死：Tile 不是模型，Tile 是“节点”，模型是 content

在 3D Tiles 里：

* **Tile**：树上的一个节点。它有：

  * **`boundingVolume`**（包围体，用来裁剪/估距）
  * **`geometricError`**（几何误差，单位米）
  * **`transform`**（把 tile 局部坐标放到父级/世界坐标）
  * **`refine`**（细化方式：`REPLACE` 或 `ADD`）
  * **`children`**（子节点）
* **Tile content**：tile 指向的外部内容文件，比如 glTF/glb、b3dm 等，通过 `content.uri` 引用。
  Cesium 的实现文档也强调了“Tile vs content”的区别：tile 是 BVH 节点，`content.uri` 才是实际渲染资源。([Cesium][1])

---

## 2) LOD 在 3D Tiles 里是怎么工作的（核心机制）

LOD 并不是你在 tileset 里写个“LOD: 0/1/2”字段（没有这种字段），而是这样实现的：

1. 运行时会**深度优先遍历** tileset 的 BVH。([Cesium][1])
2. 对每个可见 tile，算法会判断：**当前 tile 的细节是否足够**（通过屏幕空间误差 SSE）。足够就渲染它；不够就 **refine**（尝试用子 tile 替代）。([Cesium][1])
3. SSE 的关键输入之一就是：

   * **`geometricError`**：表示 tile 的几何近似误差（米）。([Cesium][1])
   * **到 `boundingVolume` 的距离**：用来把“米级误差”换算成“屏幕像素级误差”。([Cesium][1])

所以：
**你要“有 LOD 能力”，本质就是：你生成一棵合理的 tile 树，并给每层设置递减的 `geometricError`，父节点内容更粗，子节点内容更细（或父节点无内容只做占位）。**

---

## 3) `tileset.json` 里哪些字段决定“能不能正确 LOD”

一个 tile 节点里，跟 LOD/裁剪直接相关的字段是：

### `boundingVolume`

必须**完全包住**这个 tile（以及它的 content）在空间中的范围，否则会被裁剪掉出现“模型缺块”。

`box` 的格式不是 min/max，而是 12 个数：

* 前 3 个：中心点 `(cx, cy, cz)`
* 后 9 个：3 个半轴向量（3×3 矩阵的三列），表示 OBB（可旋转包围盒）的三个方向和半长度。

### `geometricError`

越往下（更细的 tile），一般越小。否则会导致 refine 逻辑怪异；在 Cesium Native 的描述里，如果子 tile 的误差比父 tile 还高，甚至会触发一些“总是 refine”的特殊处理。([Cesium][1])

### `refine`

* **`"REPLACE"`**：最常见。父 tile 渲染粗模型；细化时用子 tile 替换父 tile（典型的 LOD）。
* **`"ADD"`**：叠加型（比如点云或一些需要叠加的内容）。

---

## 4) 你真正要实现的“转换器流水线”

我把实现拆成 6 个必做模块，你照着做就能跑起来：

### A. 模型标准化 → glTF/GLB

无论你输入是 FBX/OBJ/DAE/GLTF，最好先统一到 glTF 2.0（glb）：

* 统一坐标系（右手/左手、Y-up/Z-up）
* 材质尽量转 PBR
* 纹理整理（相对路径、去重）
* 可选：Draco / meshopt / KTX2（看目标平台）

### B. 地理定位（非常容易踩坑）

glTF 顶点是 float32，直接把顶点写成 ECEF（几百万米量级）会有精度灾难：抖动、裂缝、法线乱跳。

更推荐的做法是：

* **tile content（glb）内部用局部坐标**（比如以 tile 的局部原点为中心，坐标值在几百米/几千米以内）
* 用 tile 的 **`transform`** 把它放到世界/ECEF 上

这也是 3D Tiles 设计里 `transform` 的存在意义之一。

#### B1) 纯模型坐标 + 用户输入定位（V2 方案）

当源文件没有地理信息（FBX/OBJ/RVT 未设置共享坐标）时，可以先保持“纯局部坐标”，等加载进场景后由用户输入地理位置：

1) 模型切块/打包时只保证 `boundingVolume` 与 content 在**局部坐标**下正确  
2) 通过 `root.transform` 把整个 tileset 放到地球上  
3) 这个过程不需要重算 AABB / 不需要重打包 content

仓库提供 `tools/update_transform.py`，用 `georef.json` 写入 `root.transform`：

```bash
python3 tools/update_transform.py out_tiles/tileset.json georef.example.json --pretty
```

`georef.json` 最小字段：

```json
{
  "mode": "enu",
  "origin": [116.3913, 39.9075, 35.0],
  "heading": 0.0,
  "pitch": 0.0,
  "roll": 0.0,
  "axis": "Z_UP",
  "unit": "m"
}
```

说明：`heading/pitch/roll` 为右手系旋转（度），按 `Z->Y->X` 顺序应用；`axis` 仅处理上方向（Y_UP 会绕 X 轴 +90 度转成 Z_UP）。

### C. 空间分块（tiling）

你要选择一种“怎么切”的策略：

* **地形/倾斜摄影 integrated mesh**：通常用 **QUADTREE**（按 XY 切）
* **城市建筑群 / 一堆离散模型**：常用 **Octree / BVH 聚类**（按空间聚类）
* **单体建筑**：可以按“楼层/房间/空间块”切，或者按网格空间聚类切

目标是：每个 tile 的内容体量适中、包围体紧凑、子 tile 完全落在父包围体内。

### D. LOD 生成（“父粗子细”）

你需要对每个 tile（或每层）生成不同细节的几何：

* **简化（Decimation）**：QEM（Quadric Error Metrics）是经典路线
* **分层**：

  * L0：整块超粗（或者仅占位不带 content）
  * L1：切 2×2 或按聚类切成少量块，中等细节
  * L2/L3：更细切分，更高细节

### E. `geometricError` 的设定（实用套路）

理想情况：你能计算“原始 vs 简化”的 Hausdorff 距离（最大偏差）当 error。

工程上更常用的是“够用就行”的规则：

* 对规则四叉树：每下一级误差大致 **除以 2**
  例如 root=200，L1=100，L2=50…
* 或者用 tile 尺寸做比例：
  `geometricError ≈ tileDiagonal * k`（k 取 0.01~0.1 之间按质量调）

只要保证：**误差沿树往下递减**，运行时就能稳定触发 refine。

### F. 输出 `tileset.json` + 内容文件

把每个 tile 写成 JSON 节点，把内容写成 glb/b3dm 文件，并保证 URI 正确。

---

## 5) 显式 LOD 树 vs 隐式（Implicit）LOD 树：怎么选

### 方案 1：显式 tiling（最直观）

`tileset.json` 里把每个 tile、每个 child 都写出来。

优点：

* 实现最简单
* 好调试（你能直接看到树结构）

缺点：

* tile 数量很大时，`tileset.json` 会变得巨大（甚至多文件 tileset）

### 方案 2：Implicit Tiling（3D Tiles 1.1，适合海量规则树）

3D Tiles 1.1 支持“隐式定义一棵规则四叉树/八叉树”，`tileset.json` 不再枚举所有节点，而是用 `implicitTiling` + subtree 文件表达“哪些 tile/内容存在”。

OGC 的 3D Tiles 规范 PDF 里明确提到：**subtrees 可以存储可用 tiles/contents 的元数据，`implicitTiling` 可以加在 tileset JSON 的 tile 上**。([OGC Public Document Repository][2])
Cesium Native 也解释了：3D Tiles 1.1 开始 BVH 可以隐式定义，这对“结构固定的细分”更高效，并通过隐式四叉树/八叉树 loader 支持。([Cesium][1])

优点：

* 超大规模 tileset 的入口 JSON 仍然很小
* 对规则结构（地形/mesh quadtree）非常香

缺点：

* 你要生成 `.subtree`（以及 availability bitstream / constant availability 等），实现复杂度明显上升
* 调试成本更高（尤其 subtreeLevels/availableLevels 等组合）

如果你目标是“倾斜摄影/地形那种 5 层、10 层四叉树”，并且 tile 数量巨大，隐式 tiling 值得上；如果只是几千个 tile 以内，显式更省命。

---

## 6) 版本和内容格式：1.0 的 b3dm 还是 1.1 的 glb？

你有两条路：

* **3D Tiles 1.0（经典）**：content 多用 `b3dm/i3dm/pnts/cmpt`
* **3D Tiles 1.1（更现代）**：更推 glTF/glb 作为 tile content，并支持 implicit tiling、更多 metadata 能力

如果你要兼容更多老工具链，1.0 仍然很常见；如果你自己控制生产和消费端（CesiumJS/Cesium Native/支持 1.1 的客户端），1.1 更未来。

工程上很多人做法是：**内部全用 glb → 需要兼容时再封 b3dm**。

---

## 7) 现成工具能帮你省掉哪些坑（即使你“自己实现”也建议用）

### 3d-tiles-tools：处理/升级/封装内容

CesiumGS 的 `3d-tiles-tools` 能：

* `upgrade`：把 tileset 升到 1.0 或 1.1；会把 `url` 改成 `uri`、把 `refine` 规范成大写、升级 glTF 版本等。([GitHub][3])
* `glbToB3dm` / `glbToI3dm`：把 glb 封成 b3dm/i3dm。([GitHub][3])
* `convert`：把 tileset 打包成 `.3tz` 等格式。([GitHub][3])
* `analyze`：拆解内容便于调试。([GitHub][3])

这类工具非常适合你自己实现“切块 + LOD + tileset tree”的部分，然后把“封装/升级/校验/打包”交给成熟工具。

### 3d-tiles-validator：校验 tileset 是否规范

`3d-tiles-validator` 支持：

* 校验 `tileset.json` 的结构
* 校验 b3dm/i3dm/pnts/cmpt 以及 glTF
* 校验 implicit tilesets、metadata 等([GitHub][4])

强烈建议把 validator 当成 CI 的一环：不然你会在“为什么 Cesium 不加载”上浪费大量生命。

---

## 8) 一个“最小可用”的显式 LOD `tileset.json` 骨架（示意）

下面是概念示意（数值你要自己填），重点看结构与字段：

```json
{
  "asset": { "version": "1.1" },
  "geometricError": 200,
  "root": {
    "transform": [ /* 4x4，列主序，放到世界坐标 */ ],
    "boundingVolume": {
      "box": [
        0, 0, 0,
        50, 0, 0,
        0, 50, 0,
        0, 0, 20
      ]
    },
    "geometricError": 100,
    "refine": "REPLACE",
    "content": { "uri": "lod0.glb" },
    "children": [
      {
        "boundingVolume": { "box": [ /* ... */ ] },
        "geometricError": 50,
        "refine": "REPLACE",
        "content": { "uri": "lod1_0.glb" },
        "children": [
          {
            "boundingVolume": { "box": [ /* ... */ ] },
            "geometricError": 0,
            "refine": "REPLACE",
            "content": { "uri": "lod2_0.glb" }
          }
        ]
      }
    ]
  }
}
```

注意：

* `geometricError: 0` 通常表示叶子 tile（不再细化）
* `boundingVolume.box` 里的 3 个半轴向量决定 OBB 的大小和方向（上面示意的是轴对齐盒）

---

## 9) 用 Node.js 生成 tileset.json 的一个“实现骨架”（偏工程可落地）

你最终一定会写一个“tile tree builder”。下面给一个非常实用的骨架（不含网格简化/切块，只管组织 JSON）：

```js
import fs from "node:fs";
import path from "node:path";

/**
 * 把 AABB(min/max) 转成 3D Tiles 的 OBB box(12)。
 * 假设 box 轴对齐且 tile 本地坐标系不旋转（旋转用 transform 处理）。
 */
function aabbToBox(min, max) {
  const cx = (min[0] + max[0]) / 2;
  const cy = (min[1] + max[1]) / 2;
  const cz = (min[2] + max[2]) / 2;
  const hx = (max[0] - min[0]) / 2;
  const hy = (max[1] - min[1]) / 2;
  const hz = (max[2] - min[2]) / 2;

  // halfAxes = [hx,0,0, 0,hy,0, 0,0,hz]，按列向量展开
  return [cx, cy, cz,  hx, 0, 0,  0, hy, 0,  0, 0, hz];
}

function makeTile({ aabbMin, aabbMax, error, uri, children = [], refine = "REPLACE" }) {
  const tile = {
    boundingVolume: { box: aabbToBox(aabbMin, aabbMax) },
    geometricError: error,
    refine,
  };
  if (uri) tile.content = { uri };
  if (children.length) tile.children = children;
  return tile;
}

// 示例：你切块/简化后，把每个 tile 的 aabb/error/uri 填进来
const root = makeTile({
  aabbMin: [-100, -100, -20],
  aabbMax: [ 100,  100,  80],
  error: 100,
  uri: "lod0.glb",
  children: [
    makeTile({
      aabbMin: [-100, -100, -20],
      aabbMax: [   0,    0,  80],
      error: 50,
      uri: "lod1_0.glb",
      children: [
        makeTile({
          aabbMin: [-100, -100, -20],
          aabbMax: [ -50,  -50,  80],
          error: 0,
          uri: "lod2_0.glb",
        })
      ]
    })
  ]
});

const tileset = {
  asset: { version: "1.1" },
  geometricError: 200,
  root: {
    // 你自己的地理定位矩阵放这里（或 identity）
    transform: [1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1],
    ...root
  }
};

fs.writeFileSync(path.resolve("tileset.json"), JSON.stringify(tileset, null, 2), "utf8");
```

你接下来要补的“硬菜”就是：

* 如何从模型几何计算 AABB/OBB
* 如何切块（空间聚类 / quadtree / octree）
* 如何为每块生成简化 LOD 网格（输出 glb）
* 如何估算 `geometricError`

---

## 10) 最常见的坑（提前给你扫雷）

1. **顶点直接用 ECEF 大坐标** → 精度炸裂
   用 `transform` 放置，顶点保持局部小坐标。
2. **boundingVolume 不紧 / 不包含内容** → 被裁剪、穿帮、缺块
3. **child 不在 parent 包围体内** → 选择算法行为异常（各种“永远 refine”/“永远不加载”）
4. **tile 太碎** → 网络请求爆炸、CPU 管理开销爆炸
   需要权衡：块大小、数量、打包（比如 `.3tz`）、缓存策略。
5. **没校验就上 viewer** → 你会在黑屏里悟道
   用 validator + analyze 工具链把问题前置。([GitHub][4])

---

## 你可以怎么落地到一个“可交付”的实现路径

* **第一阶段（2-3 天能出结果）**：
  单个模型 → 转 glb → `createTilesetJson` 或你自己生成一个 root tile（无 LOD 或 2 级 LOD）先跑通加载。
* **第二阶段（进入真正 LOD）**：
  加入切块 + 简化 → 显式 tileset 树（`geometricError` 递减）
* **第三阶段（规模上去）**：
  需要时上 implicit tiling（subtree 文件），否则先别自虐。([Cesium][1])

---

后续可以按“模型类型”选择更具体的切块/LOD 策略模板：

- 倾斜摄影 integrated mesh（四叉树切块、每层 decimation 的经验参数）
- 城市建筑（按建筑/街区聚类 + 多级 LOD）
- 单体 BIM（楼层/构件层级 + 交互属性/metadata）

这些不同类型的 tileset，tile 树结构和 `geometricError` 设法会差很多，但核心机制都是上面这套。

[1]: https://cesium.com/learn/cesium-native/ref-doc/selection-algorithm-details.html "cesium-native: 3D Tiles Selection Algorithm"
[2]: https://docs.ogc.org/cs/22-025r4/22-025r4.pdf "3D Tiles Specification"
[3]: https://github.com/CesiumGS/3d-tiles-tools "GitHub - CesiumGS/3d-tiles-tools"
[4]: https://github.com/CesiumGS/3d-tiles-validator "CesiumGS/3d-tiles-validator"
