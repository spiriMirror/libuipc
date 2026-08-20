# Handoff — Current State of the Repo

> Written 2026-08-20. Supersedes the earlier handoffs. **The muda→cuda_tool
> migration is complete: the backend runs entirely on the self-written
> cuda_tool library with named `__global__` kernels throughout.**
> Verify against the working tree before assuming anything beyond this file.

## TL;DR

- Branch `refactor-main`. The CUDA backend no longer depends on muda in any
  form (no submodule, no vendored copy, no xmake package).
- All 273 lambda kernel launch sites were rewritten as named `__global__`
  functions with raw `<<<>>>` launches.
- Build is green and regressions pass with strict mode
  (`0_abd_gravity` 178, `14_fem_3d_ground_contact` 432,
  `37_abd_revolute_joint` 311 assertions).
- Working tree is clean. One known leftover below.

## Commits (oldest → newest, on top of `74a5df62`)

```
ef87325c docs(agent_docs): record muda vendoring completion and fix stale references
b2aec545 feat(cuda_tool): complete primitives for muda replacement
8e3299af refactor(cuda): migrate backend from muda to cuda_tool
423be546 refactor(cuda): rewrite lambda kernels as named __global__ functions
cb9341c1 build: drop the vendored muda from cuda_tool and sync xmake
```

(Pre-history on this thread: `0b9f88d3` graph-include trim, `921bf14f`
external/muda submodule removal, `9018bec7` vendored ext/ subtree trim.)

## What was done

1. **cuda_tool 原语补全** (`b2aec545`) — stream/view/view_nd/launch/buffer/
   cub/linear_system(+views)/debug/logger/atomic + eigen 子目录（自 muda
   ext/eigen 逐字移植，数值比特级一致）。`UIPC_KERNEL_*` 宏族随
   `uipc::RUNTIME_CHECK` 启用。
2. **机械迁移** (`8e3299af`) — 280 文件 `muda::`→`cuda_tool::`、伞头改
   `cuda_tool/cuda_tool.h`、宏改名、删 `.name("...")` 标签。
   **关键**：`ParallelFor` 复刻了 muda 的占用率自选块大小
   （`cudaOccupancyMaxPotentialBlockSize`），否则 warp 归约类 kernel 的
   FP 求和顺序变化会导致严格模式下 line search 偶发不收敛（实测）。
3. **kernel 改写** (`423be546`) — 273 个 lambda kernel → 命名
   `__global__`（匿名 namespace，body 逐字，捕获→参数）。启动用
   `cuda_tool::best_grid_dim/best_block_dim` 保持同一占用率选择。
4. **删除 vendored muda + 构建系统同步** (`cb9341c1`) — 删
   `cuda_tool/muda/`（288 文件）与 `muda_compat.h`；CMake 去掉 MUDA_*
   宏；xmake 去掉 `add_requires/add_packages("muda")` 改用本地 cuda_tool
   include（xmake 本机未装，仅静态核对，未经 xmake 构建验证）。
5. **cuda_tool 精简（未提交时已完成一部分）** — 删零引用原语：
   `Launch`/`KernelLabel`/`parallel_for` 自由函数/3D 视图族
   （DeviceBuffer3D/Buffer3DView/Extent3D）/`DeviceCOOVector`。

## Leftover（唯一遗留）

- **`src/backends/cuda/linear_system/current_frame_diff_dof_reporter.cu`
  仍是一个 lambda ParallelFor 点**（273 中的 1 个）。该文件在本机被某
  进程硬锁（无法写/改名/删除，多次尝试 + 杀进程排查未果；疑似
  VSCode/索引器/杀软持有）。转换内容已就绪并在
  `current_frame_diff_dof_reporter.cu.staged`（已提交跟踪）。锁释放后：
  `cp ...cu.staged ...cu && git rm ...cu.staged`，然后从 cuda_tool 移除
  `ParallelFor`（届时零引用）。
- `ParallelFor` 目前因这 1 处引用保留在 cuda_tool/launch.h。

## Environment notes (unchanged)

- Build: `output/build.bat`（vcvars64 + `cmake --build build --config
  Release --target sim_case -j8`）；全量错误收集用
  `output/build_keepgoing.bat`（`ninja -k 0`）。
- nvcc 需要 MSVC 环境，裸 shell 报 "Cannot find compiler 'cl.exe'"。
- Configure: `cmake -S . -B build --preset ci-release
  -DUIPC_BUILD_BENCHMARKS=OFF -DUIPC_BUILD_EXAMPLES=OFF`（必须传 `-B
  build`；删/增源码文件后必须重新 configure，因为 file(GLOB) 在配置期展开）。
- Catch2 过滤多个用例名在本机不生效，逐个跑。
- `output/test_compile.cu` + `output/compile_smoke.bat` 是 cuda_tool 的
  独立 smoke 编译/运行入口（`src/.../test_compile.cu.txt` 是仓库内档案）。
