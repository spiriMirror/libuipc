# 01 — 项目概览

## 定位

**libuipc**（**Lib**rary of **U**nified **I**ncremental **P**otential **C**ontact）是跨平台现代 C++20 库，在 GPU 上统一仿真刚体、软体、布料、线程及其耦合，保证**无穿透的精确摩擦接触**，设计上支持可微仿真（Diff-Sim 规划中）。提供 C++ 与 Python（`pyuipc`）双 API，支持 Windows/Linux。

学术来源：
- GIPC 2024 (ACM TOG 43(2))：Gauss-Newton 优化 IPC 障碍能
- StiffGIPC 2025 (ACM TOG 44(3), Siggraph)：刚性仿射-变形耦合 GPU IPC
- 原型开源实现：[Stiff-GIPC](https://github.com/KemengHuang/Stiff-GIPC)（性能基准）

近期重要变更（README News）：
- 2026-03：Genesis AI 贡献 AL-IPC（Augmented Lagrangian IPC）接触、ExternalArticulationConstraint（最小坐标关节系统）、cibuildwheel CI 打包
- 2026-02：PyPI 发布 `pip install pyuipc`（Win/Linux, Python 3.10–3.13, CUDA 12.8）

## 顶层目录

| 目录 | 内容 |
|---|---|
| `src/` | 核心实现：`core/`（引擎，编译为 `libuipc_core`）、`geometry/`（几何算法）、`constitution/`（本构）、`backends/`（`common/`+`cuda/`+`none/`）、`io/`、`pybind/`、`sanity_check/`、`usd/`、`vdb/` |
| `include/uipc/` | 公开头文件，按模块分：`core/ geometry/ constitution/ io/ backend/ builtin/ common/ diff_sim/ usd/ vdb/`；聚合头 `uipc.h / core.h / geometry.h / io.h` |
| `apps/` | `tests/`（Catch2 测试）、`examples/`（3 个 C++ 示例）、`benchmarks/`、`app/`（测试工具库） |
| `python/` | Python 包源码 `src/uipc/`、`tests/`、示例；另有独立 `pyproject.toml`（开发用） |
| `docs/` | mkdocs 文档源：`tutorial/ specification/ development/ build_install/ media/` 等 |
| `external/` | 第三方源码依赖，主要是 `muda/`（CUDA 启动/线性代数封装库） |
| `scripts/` | 构建辅助（`gen_vcpkg_json.py`、`after_build_pyuipc.py`）、`symbol_calculation/`（~25 个符号推导 notebook）、`SymEigen/` |
| `assets/` | 仿真网格资产（`sim_data/{linemesh,tetmesh,trimesh}`） |
| `output/` | 仿真输出目录（gitignored） |
| `.cursor/` | AI 开发配置：`rules/`（C++ 风格、自我改进）+ `skills/`（17 个工作流 SKILL.md） |
| `agent_docs/` | 本目录，agent 导览 |
| `CMakeLists.txt` / `CMakePresets.json` / `pyproject.toml` | 构建入口 |

## 三大核心概念

```
Engine（算法 + 设备，如 "cuda"）
  └── World（仿真生命周期：init → advance → retrieve，frame() 查询当前帧）
        └── Scene（某一时刻的快照，驱动物理所需的全部信息）
```

Scene 的 5 大组成（详见 `docs/tutorial/concepts.md`）：

1. **Objects**：现实实体（如"一件 T 恤"），`scene.objects().create("name")`；一个 Object 可挂多个 geometry（`object.geometries().create(mesh)` 返回 GeometrySlot），支持多实例（instance）。
2. **Geometries**：场景持有的几何数据（核心是 `SimplicialComplex`）。
3. **Constitution Tabular**：本构模型集合。`constitution.apply_to(mesh, params...)` 把参数写入几何属性，用 **UID** 标识（官方 0~2³²-1，用户自定义 2³²~2⁶⁴-1，写入 meta 的 `constitution_uid` 属性）。
4. **Contact Tabular**：接触模型表。`contact_tabular.create("wood")` 建 ContactElement；`insert(ce1, ce2, friction, resistance)` 建成对模型 $C_{ij}=(\kappa,\mu,f)$；未定义对回退到 `default_model`（id=0）。另有 Subscene Tabular（子场景使能矩阵，默认单位阵）。
5. **Animator**：动画脚本。`animator.insert(obj, update_fn)` 每帧回调，`info.geo_slots()` 取绑定几何；需配合 Constraint（`SoftTransformConstraint` 驱动 affine body 的 `aim_transform`，`SoftPositionConstraint` 驱动顶点 `aim_position`），且几何需先设 `is_constrained=1`。

## 典型仿真流程（Hello Libuipc）

```cpp
Engine engine{"cuda", workspace};
World world{engine};
Scene scene{Scene::default_config()};
auto& ct = scene.contact_tabular();
ct.default_model(0.5, 1.0_GPa);            // 摩擦 0.5，接触刚度 1 GPa

auto mesh = tetmesh(Vs, Ts);               // 建四面体网格
abd.apply_to(mesh, 100.0_MPa);             // AffineBodyConstitution，kappa=100MPa
label_surface(mesh);                       // 接触必需：标记表面
label_triangle_orient(mesh);               // 标记三角形朝向
mesh.instances().find<IndexT>(builtin::is_fixed)->view()[0] = 0; // 可动

auto obj = scene.objects().create("tet");
obj->geometries().create(mesh);
world.init(scene);
world.advance();
world.retrieve();
scene_io.write_surface("out.obj", scene);  // 导出表面
```

Python 完全对应（`from uipc import Engine, World, Scene` 等）。

## 关键设计取舍

- **Data-Oriented**：几何/物理数据存于属性集合（AttributeCollection），系统间通过 RMR 传递视图，避免深 OOP 继承，利于 GPU cache 友好访问。
- **后端可插拔**：算法全部在 backend 里，core 只做数据结构与生命周期；新硬件只需实现新 backend 模块。
- **单位字面量**：`uipc::unit` 提供 `1.0_GPa`、`100.0_MPa`、`0.05_m/1.0_s` 等字面量，物理参数自带量纲。
- **双构建系统**：CMake（主，CI/wheel 用）+ XMake（备选）；源码目录平行维护 `xmake.lua`。
