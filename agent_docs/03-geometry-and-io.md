# 03 — 几何模块与 IO

## SimplicialComplex（核心几何类型）

头文件：`include/uipc/geometry/simplicial_complex.h`，实现：`src/geometry/simplicial_complex*.cpp`。

单形复形，四个维度槽位：
- `vertices()` — 0 维（点）
- `edges()` — 1 维（线段）
- `triangles()` — 2 维（三角形）
- `tetrahedra()` — 3 维（四面体）

组织方式：**AttributeCollection 属性集合**。每个槽位是 `AttributeCollection`（按元素索引的属性表，每属性是一列同构数据，类型安全模板访问 `find<T>(name)->view()`）；几何本身还有 `meta()`（全局元信息属性，如 `constitution_uid`）和 `instances()`（每实例属性，如 `is_fixed`、`transform`）。

属性系统的通用基类：`IAttribute` / `IAttributeSlot`（`include/uipc/geometry/attribute*.h`），支持延迟重建、resize、视图（`view()` 返回 span 风格只读/可写视图）。Geometry 抽象基类见 `include/uipc/geometry/geometry.h`，场景中的句柄是 `GeometrySlot` / `SimplicialComplexSlot`（含 rest 几何对偶：`rest_geo_slots()`）。

## 几何工厂函数（`include/uipc/geometry/factory.h` 等）

- `tetmesh(Vs, Ts)`、`trimesh(Vs, Fs)`、`linemesh(Vs, Es)`、`pointcloud(Vs)` 从数组构造。
- `merge(span<SimplicialComplex*>)` 合并多个网格。
- 隐式几何：`ground(height)`（HalfPlane）等，见 `docs/specification/implicit_geometry_uid.md`。

## 拓扑与几何算法（`src/geometry/`，依赖 libigl + TBB + octree）

常用函数（头文件同名于 `include/uipc/geometry/`）：
- `label_surface(sc)` — 标记表面顶点/边/三角形（`builtin::is_surface`），**开启接触必需**
- `extract_surface(sc)` — 提取表面 trimesh
- `label_triangle_orient(sc)` — 标记三角形朝向（`builtin::parent_id` 等），摩擦接触需要
- `label_open_edge` / `flip_inward_triangles`
- `compute_vertex_volume` / `compute_vertex_mass`
- `affine_body/compute_affine_body_*` — ABD 质量属性（dyadic mass 三元组等）
- BVH：`include/uipc/geometry/bvh.h`（CPU 侧层次包围盒）
- 距离/相交查询、最近点、octree 空间索引
- `implicit_geometry/`：HalfPlane 距离场等

## 内置属性名（`include/uipc/builtin/`）

`builtin::` 命名空间集中定义约定属性名，避免字符串散落：
- `builtin::is_fixed`（instance，是否固定）
- `builtin::is_surface`（vertex/edge/triangle，是否表面）
- `builtin::parent_id`、`builtin::orient`（表面三角形来源四面体与朝向）
- `builtin::is_constrained`、`aim_position`、`aim_transform`（动画约束）
- `builtin::constitution_uid`（meta）、`builtin::thickness`（codim 顶点厚度）等
- 完整清单见 `include/uipc/builtin/attribute_name.h`、`uid.h`、`constitution_uid.h`。

## IO（`include/uipc/io/` + `src/io/`）

| 类 | 能力 |
|---|---|
| `SimplicialComplexIO` | 按扩展名统一 `read/write`：读 `.msh`（tet）/`.obj`/`.ply`/`.stl`（tri/line/point），写 `.obj`/`.msh`；构造可传 `pre_transform`（Matrix4x4/Transform）对读入顶点预变换 |
| `SceneIO` | `write_surface(path, scene)` 导场景表面 `.obj`；`simplicial_surface(dim)` 取任意维表面；场景序列化 `load/save`（`.json`/`.bson`）；增量 `commit/update`；`to_json/from_json` |
| `SpreadSheetIO` | 把 Geometry 全部 attribute 导出为 JSON/CSV（每属性一张表），调试用 |
| `UrdfIO` / `UrdfController` | URDF 机器人读入为 Object；控制器提供 `move_root`、`rotate_to(joint, angle)`，暴露 revolute joints（ImplicitGeometrySlot）与 links（SimplicialComplexSlot）、`sync_visual_mesh` |
| `AttributeIO` | 从文件读单个命名 attribute 到 `IAttributeSlot` |
| glTF | `test_gltf` 实验接口（tinygltf） |

STL 解析在 `src/io/stl_reader.h`。

## 常见坑

- 接触仿真前**必须** `label_surface`；摩擦需要 `label_triangle_orient` 保证法向一致。
- `is_fixed=1` 的实例不参与动力学但仍参与接触。
- `apply_to` 写入的是**属性**，修改属性即修改材料参数；多实例几何注意属性在 instance 槽还是顶点槽。
- 读入网格建议用 `pre_transform` 统一缩放，而不是事后改顶点（rest 几何会不一致）。
