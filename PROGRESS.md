# 3DTileTransformer 进度文档

更新时间：2024-12-23

## 当前进度（阶段性结论）

V1 已完成并验证；V3（quadtree 切块 + 采样 LOD）进入可用试验阶段：

- V1：单个 `*.glb` → `tileset.json` + 原样 `*.glb`
- V3：新增 quadtree 切块脚本 `tools/glb_to_tileset_quadtree.py`
- V3 输出：`tileset.json` + `tiles/` 叶子 tile + `root_simplified.glb`
- 地理定位：`tools/update_transform.py` 可写入 `root.transform`

## 已完成

- GLB 2.0 基础解析（JSON/BIN chunk）
- 从 POSITION 计算 AABB 并生成 `boundingVolume.box`
- `tileset.json` 基本字段生成
- `georef.json` → `root.transform` 写入工具
- quadtree 切块（按节点 AABB 中心点）
- root 级采样 LOD（按三角形均匀采样）
- 子 tiles GLB 子集导出（按节点集合）

## 进行中

- LOD 仅在 root 生成，内部节点无内容
- 不切 mesh（仅按节点聚类）
- 纹理/材质在每个 tile 内重复

## 总体规划（路线图）

### V1：最简可跑（完成）

- 输入：单个 `*.glb`
- 输出：`tileset.json` + `*.glb`
- 目标：在 CesiumJS 中可加载显示

### V2：地理定位工作流（已具备基础）

- 保持内容局部坐标
- 通过 `root.transform` 放置到地球坐标
- 用户输入 `georef.json`

### V3：切块（Tiling）

- 已实现：四叉树切块（按节点 AABB 中心点）
- 待增强：支持八叉树 / BVH 聚类、切 mesh

### V4：LOD 生成（父粗子细）

- 已具备：root 采样 LOD（均匀采样）
- 待增强：多层 LOD、真实简化（QEM/meshopt）

### V5：隐式瓦片（Implicit Tiling）

- 适配 3D Tiles 1.1
- 生成 subtree 元数据

### V6：工程化优化（可选）

- Draco / meshopt / KTX2
- 大规模 tileset 的打包与发布
