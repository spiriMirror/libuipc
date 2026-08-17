# 05 — CUDA 后端

目录：`src/backends/cuda/`（编译为 MODULE 库 `uipc_backend_cuda`）。入口 `entrance.cpp` → `cuda::SimEngine`（`sim_engine.h` + `engine/*.cu`）。GPU id 取自 engine config `gpu/device`。

## 子系统目录

| 目录 | 职责 |
|---|---|
| `engine/` | 管线编排：`sim_engine_do_init/do_advance/do_retrieve/do_sync` 等 |
| `pipeline/` | 管线动作（SimAction）定义 |
| `global_geometry/` | 全局顶点管理（GlobalVertexManager）、包围盒 |
| `affine_body/` | ABD 动力学（97 文件）：雅可比、质量/能量组装、关节、state accessor |
| `finite_element/` | FEM（123 文件）：各本构的梯度/Hessian 组装 kernel（Fem3D/Codim2D/Codim1D 系列） |
| `collision_detection/` | StacklessBVH、simplex distance、global trajectory filter、DCD/CCD 候选检测 |
| `contact_system/` | IPC / AL-IPC 接触模型（symplectic/implicit）、法向/切向力、CFL 条件 |
| `distance_system/` | 距离计算 |
| `active_set_system/` | 接触激活集 |
| `dytopo_effect_system/` | 动态拓扑效应（摩擦等耦合项） |
| `inter_primitive_effect_system/` | 图元间效应（软缝合等） |
| `joint_dof_system/` | 关节自由度系统 |
| `coupling_system/` | 多体耦合 |
| `linear_system/` | 线性求解：GlobalLinearSystem、fused PCG、SpMV、预条件子 |
| `line_search/` | 线搜索能量评估 |
| `newton_tolerance/` | Newton 收敛判据 |
| `time_integrator/` | 时间积分与速度更新（BDF1 等） |
| `animator/` | 动画步进 |
| `external_force/` | 外力计算 |
| `diff_sim/` | 可微仿真 |
| `implicit_geometry/` | 隐式几何（HalfPlane 等） |
| `sanity_check/` | GPU 侧检查器 |
| `cuda_device/` | 设备管理 |
| `algorithm/`、`utils/` | 共享算法与工具 |
| 根级文件 | `sim_engine.h/.cpp`、`sim_system.h`、`sim_action*.h`、`sim_engine_state.h`、`energy_component_flags.*`、`kernel_cout.*`、`type_define.h` |

## advance 管线（`engine/sim_engine_do_advance.cu`）

```
Pipeline
├── Rebuild Scene                          （含条件分支 Update Diff Parm）
└── Simulation
    ├── Clear External Forces              （条件）
    ├── Step Animation                     （条件）
    ├── Compute External Force Accel.      （条件）
    ├── Detect DCD Candidates              （条件）
    ├── Newton Iteration                   （LOOP）
    │   ├── Detect DCD Candidates          （iter > 0）
    │   ├── Compute DyTopo Effect          （条件：Assemble → Convert → Distribute）
    │   ├── Solve Global Linear System     （Build → PCG：Apply Preconditioner + SpMV）
    │   └── Line Search
    │       ├── Detect Trajectory Cand.
    │       ├── Compute Energy             （初始 E0）
    │       ├── Filter CCD TOI
    │       ├── Compute CFL Condition
    │       └── Line Search Iteration      （LOOP：Filter Contact Cand. → Compute Energy）
    └── Update Velocity
```

注意：父计时器时长**包含**子计时器（如 Newton Iteration 占 80% 时 PCG/Line Search 已计入）。实际层级以运行输出的 `timer_frames.json` 为准（随启用的 feature 动态变化）。

`do_init/do_advance/do_retrieve/do_sync` 中各 sim system 通过 `SimActionCollection` 注册的回调（`on_init_scene/on_rebuild_scene/on_write_scene`）被依次调用。

## muda 与 kernel 命名

所有 kernel 经 `external/muda` 的 `muda::ParallelFor().apply(N, lambda)` 启动，编译产物为 `parallel_for_kernel<Lambda>`。NVCC 会把外层函数名嵌入符号：

```
parallel_for_kernel<StacklessBVH::calcExtNodeSplitMetrics()::lambda>
```

- 一个函数多个 `ParallelFor` 时追加 `#N`（N = 源码中第 N 个 `ParallelFor().apply()` 调用）。
- `buffer::kernel_fill<int>` 等是内存操作，通常不是优化目标。
- `python/src/uipc/profile/nsight.py` 的 `_shorten_kernel_name()` 负责提取可读名。

## GPU 代码规范（摘自 review-pr / simulation-dev）

- buffer 参数用 `muda::BufferView` / `muda::TripletMatrixView`，**禁裸指针**。
- kernel 体顶部先做索引守卫，越界立即 return。
- 调试用 `muda::debug_sync_all()` 做快速失败屏障；给关键 buffer/viewer 命名（`viewer().name("x")`）；launch 带 `file_line(__FILE__, __LINE__)` 元数据。
- 检查 `isfinite`；除零/负开方加保守守卫。
- 修改 solver/kernel 遵循 `.cursor/skills/simulation-dev/SKILL.md` 的调试闭环。

## 性能分析工具链

- `python -m uipc.cli.benchmark run/profile/analyze/compare`（CLI）与 `uipc.profile` / `uipc.profile.nsight`（Python API）。
- 流程：先 `run` 拿分阶段 wall-clock（`report/report.md` + `timer_frames.json`），再 `profile` 拿 per-kernel 指标（ncu），交叉定位"热点阶段 + 低效 kernel"；只占帧时间 <5% 的阶段不优化。
- 细节见 `.cursor/skills/gpu-optimization/SKILL.md`。

## none 后端

`src/backends/none/`：空实现（`none::SimEngine` 各 `do_*` 基本 no-op），用作新后端模板与 core 基础功能自检。**注意：无 GPU 环境没有可用的计算后端**。
