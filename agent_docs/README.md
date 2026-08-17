# agent_docs — 给 AI Agent 的项目导览

本目录是对 libuipc 代码库的结构化总结，供新接入的 AI agent（或新开发者）在**不通读全部源码**的情况下快速建立准确的项目认知。所有内容基于对源码与 `docs/` 的实际阅读整理，标注了关键文件路径。

> 使用建议：先读本 README 与 `01-project-overview.md` 建立全局观，再按任务需要查阅对应专题文档。若总结与源码冲突，**以源码为准**。

## 文档索引

| 文件 | 内容 |
|---|---|
| `01-project-overview.md` | 项目定位、顶层目录、三大核心概念、典型仿真流程 |
| `02-core-architecture.md` | 三层架构、Engine/World/Scene 生命周期、RMR 模式、后端插件 ABI |
| `03-geometry-and-io.md` | SimplicialComplex、属性系统、几何算法、IO 类 |
| `04-constitutions.md` | 全部本构模型清单（UID、参数、物理意义）、Constraint 与关节 |
| `05-cuda-backend.md` | CUDA 后端子系统、advance 管线、muda kernel 命名、性能分析 |
| `06-python-api-and-packaging.md` | pybind 结构、Python 包布局、wheel 打包管线 |
| `07-build-test-workflow.md` | CMake/XMake 构建、测试体系、开发规范（.cursor 规则摘要） |

## 30 秒速览

- **libuipc** = C++20 跨平台 GPU 物理仿真库，实现 Unified IPC（增量势能接触），统一模拟刚体/软体/布料/绳索，无穿透摩擦接触。对应论文 GIPC 2024、StiffGIPC 2025 (Siggraph)。
- 用户 API 三层概念：**Engine**（算法+后端）→ **World**（`init/advance/retrieve`）→ **Scene**（快照：Objects/Geometries/Constitutions/Contacts/Animator）。
- 后端是**运行时动态加载的 MODULE 库**（`uipc_backend_cuda`、`uipc_backend_none`），通过 `uipc_init_module/uipc_create_engine/uipc_destroy_engine` 三个导出符号通信。
- 后端内部采用 **DOP + RMR（Reporter-Manager-Receiver）** 的 ECS 风格架构，所有仿真功能都是 `SimSystem` 派生类，靠 `REGISTER_SIM_SYSTEM` 宏自动注册。
- GPU kernel 全部通过 `external/muda` 库的 `muda::ParallelFor` 启动。
- 本构模型（constitution）共 40+ 个，能量/梯度/Hessian 由 `scripts/symbol_calculation/` 的 Jupyter 符号推导生成（SymEigen）。
- 双构建系统：CMake（主）+ XMake（备）；双 API：C++ + Python（pybind11，PyPI 包名 `pyuipc`）。
- 测试：Catch2（`apps/tests/`，含 94 个编号 sim_case 仿真用例）+ pytest（`python/tests/`）。

## 新 Agent 上手清单

1. 读本目录文档建立认知（约 15 分钟）。
2. 编码前必读 `.cursor/rules/cpp-format.mdc`（C++ 风格）与 `agent_docs/07-build-test-workflow.md` 的规范摘要。
3. 改 solver/约束/GPU kernel 前读 `.cursor/skills/simulation-dev/SKILL.md`（索引安全、NaN 检查、调试流程）。
4. 构建/测试命令见 `07-build-test-workflow.md`；GPU 性能优化见 `.cursor/skills/gpu-optimization/SKILL.md`。
5. 提交遵循 Conventional Commits（`07` 文档中有摘要）。

## 已知文档漂移提示（截至 2026-08）

- `docs/` 面向用户（mkdocs 站），`agent_docs/` 面向 agent，两者互补。
- `include/uipc_gui/` 与 `src/gui/` 已废弃（标注 "now not supported"）。
- `none` 后端是空实现（模板/调试用），**无 GPU 环境无法跑仿真**。
- Diff-Sim 标记 "Coming Soon"，相关 API 可能变动。
