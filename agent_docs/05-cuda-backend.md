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

## kernel 与 kernel 命名

所有 kernel 都是**命名 `__global__` 函数**（匿名 namespace 内定义），经裸 `<<<grid, block, 0, stream>>>` 启动。kernel 名按 `<所属类>_<原函数>[_kN]_kernel` 约定命名，ncu 报告里直接可读：

```
InfoStacklessBVHSimplexTrajectoryFilter_detect_k1_kernel
FEMLineSearchReporter_step_forward_kernel
```

- 启动块大小经 `cuda_tool::best_block_dim(kernel)` 按占用率自动选择（与 muda 的 occupancy 自选一致，保证 FP 行为不变）；grid 由 `cuda_tool::best_grid_dim(n, kernel)` 计算。**块大小缓存以 kernel 函数地址为键**（`unordered_map<const void*, int>`），不能按函数指针类型做 `static` 缓存——同签名的不同 kernel 会共享缓存项，跨 kernel 串扰块大小会扰动 atomic 归约顺序，曾在全套件中引发 case 36 frame 17 line-search 失败（隔离单跑不现）。
- `buffer::kernel_fill<int>` 类内存操作统一走 `cuda_tool::BufferLaunch`（内部为命名模板 kernel）。
- `python/src/uipc/profile/nsight.py` 的 `_shorten_kernel_name()` 原用于从 `parallel_for_kernel<Lambda>` 提取外层函数名；命名 kernel 时代理不再需要（符号本身可读）。

## GPU 代码规范（摘自 review-pr / simulation-dev）

- buffer 参数用 `cuda_tool::BufferView` / `cuda_tool::TripletMatrixView` 等 view 类型，**禁裸指针**。
- kernel 体顶部先做索引守卫，越界立即 return。
- 调试用 `cuda_tool::debug_sync_all()` 做快速失败屏障；kernel 断言用 `UIPC_KERNEL_ASSERT`（随 `uipc::RUNTIME_CHECK` 开关）。
- 检查 `isfinite`；除零/负开方加保守守卫。
- 修改 solver/kernel 遵循 `.cursor/skills/simulation-dev/SKILL.md` 的调试闭环。

## 性能分析工具链

- `python -m uipc.cli.benchmark run/profile/analyze/compare`（CLI）与 `uipc.profile` / `uipc.profile.nsight`（Python API）。
- 流程：先 `run` 拿分阶段 wall-clock（`report/report.md` + `timer_frames.json`），再 `profile` 拿 per-kernel 指标（ncu），交叉定位"热点阶段 + 低效 kernel"；只占帧时间 <5% 的阶段不优化。
- 细节见 `.cursor/skills/gpu-optimization/SKILL.md`。

## cuda_tool（自研 raw-CUDA 工具库）

`src/backends/cuda/cuda_tool/`，命名空间 `uipc::backend::cuda_tool`。**后端唯一的设备工具库**（muda 依赖已全部移除，无 vendored 副本、无外部子模块；Eigen 保留）。

| 文件 | 内容 |
|---|---|
| `stream.h` | `CUDA_TOOL_CHECK` 错误检查、`default_stream`、`Stream`（`Stream::Default()`） |
| `view.h` / `view_nd.h` | `CBufferView/BufferView/VarView/CVarView`、`Dense/CDense`（标量 viewer）、`ViewerBase`、`Extent2D`、`Buffer2DView`、`Dense1D/Dense2D`（含 `make_dense_1d/2d`） |
| `launch.h` | `best_block_dim/best_grid_dim`（占用率自选块大小/grid 计算） |
| `buffer.h` | `DeviceVector/DeviceBuffer/DeviceVar/DeviceBuffer2D`、`BufferLaunch`（fill/copy/resize，内部命名模板 kernel） |
| `cub.h` | `DeviceReduce/Scan/Select/Partition/RadixSort/MergeSort/RunLengthEncode` 薄封装（指针形态）+ warp 级 cub 头 |
| `linear_system.h` + `linear_system/views.h` | `DeviceTripletMatrix/DoubletVector/DenseVector/BCOOMatrix/BSRMatrix/DenseMatrix` + 全套 view（Triplet/Doublet/DenseVector/BCOO）+ `LinearSystemContext`（cublas dot/norm） |
| `eigen.h` + `eigen/` | 设备端小矩阵数学（`eigen::evd/svd/pd/inverse/atomic_add`，自 muda ext/eigen 逐字移植，比特级一致） |
| `debug.h` | `debug_sync_all/check_finite` + `UIPC_KERNEL_ASSERT/ERROR/WARN` 宏族（随 `uipc::RUNTIME_CHECK`） |
| `logger.h` | `LoggerViewer`（kernel 内 `cout <<`，device printf）+ `KernelCout` |
| `atomic.h` | 标量 `atomic_add/atomic_exch` |

- 构建要点：需 `--extended-lambda --expt-relaxed-constexpr`，MSVC + CUDA≥13 需 `/Zc:preprocessor`；fmt 在 nvcc device pass 有 UTF-8 冲突，故 cuda_tool 用 `std::runtime_error`；Eigen `::arg` 需全局 shim（已内置 `type_define.h`）。
- **伞头 `cuda_tool.h` 不含 `cub.h`**：CCCL 设备算法头极重（每个 TU 多展开 ~16.5 万行），使用 `Device*` 封装的 ~23 个文件显式 `#include <cuda_tool/cub.h>`；`linear_system.h` 也不再传递包含 cub.h。
- **必须启用 RDC**（`CUDA_SEPARABLE_COMPILATION ON` + `CUDA_RESOLVE_DEVICE_SYMBOLS ON`）：`affine_body/utils.cu` 定义了 `UIPC_GENERIC` 自由函数（如 `q_to_transform`），被其他 TU 的 device 代码跨编译单元调用；关 RDC 会 `ptxas fatal: Unresolved extern`。xmake 侧对应 `add_cuflags("-rdc=true")`。
- **教训（2026-08-20 纠错）**：CMake+ninja 不跟踪编译旗标变化（纯 mtime 驱动）——切换 RDC 这类全局旗标后"全量构建"可能实际只重编了源文件有变动的 TU，陈旧对象会蒙混链接成功造成"验证通过"假象；改全局旗标必须手动清对象目录再验证。曾因此误判 RDC 可关（提交 d2f48087 中的 RDC 部分已纠正）。
- 编译耗时画像（32 核，CUDA 13.2，RTX 5090）：全量 ~4.7 min / ~8800 CPU·s，219 个 .cu 平均 ~35s；单 TU 预处理展开 ~163 万行，其中 CUDA 工具链头 ~86 万、WinSDK ~31 万、MSVC STL ~15 万、Eigen ~15 万、项目自身 <2 万——大头是工具链头，项目头已经砍无可砍。
- smoke test 保留为 `test_compile.cu.txt`。

## none 后端

`src/backends/none/`：空实现（`none::SimEngine` 各 `do_*` 基本 no-op），用作新后端模板与 core 基础功能自检。**注意：无 GPU 环境没有可用的计算后端**。
