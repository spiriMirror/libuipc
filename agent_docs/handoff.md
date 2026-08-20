# Handoff — Current State of the Repo

> Written 2026-08-20 (evening). Supersedes the earlier handoffs. **The
> muda→cuda_tool migration is complete AND fully verified: all apps/tests
> pass, including the 95-case sim suite (2/2 runs, 14214 assertions — same
> count as the pre-migration baseline).**
> Verify against the working tree before assuming anything beyond this file.

## TL;DR

- Branch `refactor-main`. The CUDA backend no longer depends on muda in any
  form (no submodule, no vendored copy, no xmake package).
- All 273 lambda kernel launch sites were rewritten as named `__global__`
  functions with raw `<<<>>>` launches.
- **All tests green**: 6 fast binaries (common/core/geometry/sanity_check/
  backend_cuda/regression) + `uipc_test_sim_case.exe` full suite 95/95 cases,
  14214 assertions, run twice deterministically.
- One uncommitted fix batch remains in the working tree (see "Pending
  commit" below) — it is the root-cause fix for the full-suite failure.

## Commits (oldest → newest, on top of `74a5df62`)

```
ef87325c docs(agent_docs): record muda vendoring completion and fix stale references
b2aec545 feat(cuda_tool): complete primitives for muda replacement
8e3299af refactor(cuda): migrate backend from muda to cuda_tool
423be546 refactor(cuda): rewrite lambda kernels as named __global__ functions
cb9341c1 build: drop the vendored muda from cuda_tool and sync xmake
f6fd6bb3 refactor(cuda_tool): trim unused primitives and refresh agent docs
ee4bea1e refactor(cuda): convert the last lambda kernel and remove ParallelFor
2a8f78d7 refactor(cuda_tool): second trim of zero-reference helpers
```

## Fix commits on top of `2a8f78d7` (root-cause fix for the suite failure)

1. `fix(cuda_tool)` — `launch.h`: `best_block_dim` occupancy cache keyed by
   kernel function address (`std::unordered_map<const void*, int>`) instead
   of a `static thread_local int` per template instantiation (**ROOT-CAUSE
   FIX**, see below); `buffer.h`: `DeviceVector::resize(n)` value-initializes
   the grown tail (memset 0 for trivial types, `T{}` fill otherwise),
   matching thrust/muda resize semantics.
2. `test/build sync` — `apps/tests/backends/cuda/CMakeLists.txt`:
   `/Zc:preprocessor` (CUDA>=13 CCCL requires it),
   `--extended-lambda --expt-relaxed-constexpr` (test .cu use cuda_tool
   launch/dense math), nvcc diag-suppress list; 5 test .cu files gain
   global-scope `namespace cuda_tool = uipc::backend::cuda_tool;` alias
   fixes (`lbvh.cu` uses `copy_from` instead of rvalue copy-init); xmake
   parity (static check only, no local xmake):
   `apps/tests/backends/cuda/xmake.lua` gains the same three flags,
   `src/backends/cuda/xmake.lua` gains `-Xcompiler=/Zc:preprocessor`
   (public, windows block); agent_docs refreshed.

## The full-suite failure and its root cause (RESOLVED)

Symptom: after the migration, `uipc_test_sim_case.exe` (95 cases, one
process) failed deterministically 3/3 at case `36_no_surf_but_contact_on`
frame 17: `Line Search Exits with Max Iteration: 8`. Case 36 run in an
isolated process passed 6/6. Pre-migration baseline (`ef87325c`) full suite
passed 3/3 (14214 assertions).

Root cause: `best_block_dim(Kernel kernel)` cached the occupancy result in a
`static thread_local int` **per template instantiation**, i.e. per function
*pointer type*. Distinct kernels with identical signatures share one
pointer type, so whichever same-signature kernel was queried first set the
block size for all of them. muda's equivalent cache was keyed per unique
lambda type (= per call site), so no cross-kernel pollution existed. In the
full-suite process, 35+ engines ran before case 36 and poisoned shared cache
entries; wrong block sizes on atomic-accumulation assembly kernels perturbed
float atomic-reduction order, shifting the FP trajectory enough to push the
frame-17 line search over the iteration limit. In isolation fewer collisions
occurred, so the perturbation stayed below the threshold.

Fix: cache keyed by kernel address. Verified: full suite 2/2 green with the
same assertion count as baseline; case-36 isolation residual series matches
baseline except residual ULP-level noise (expected: two different binaries
have different address layouts → different atomic arrival order).

Ruled out during the hunt (do not re-open): compile-flag drift (none —
`git diff` of CMakeLists), wrapper-vs-raw occupancy difference (probe with
the verbatim `abd_linear_subsystem_assemble_reporters_k2` body: 256 == 256,
see `output/probe_occupancy2.cu`), eigen port drift (normalized diff vs muda
ext/eigen: macro/namespace renames only, math bodies identical), thrust
calls in bvh (verbatim from baseline), buffer fill/copy block sizes
(per-element ops, no FP effect), stream defaults, cub/cublas call shapes.

## What was done (migration recap)

1. **cuda_tool 原语补全** (`b2aec545`) — stream/view/view_nd/launch/buffer/
   cub/linear_system(+views)/debug/logger/atomic + eigen 子目录（自 muda
   ext/eigen 逐字移植，数值比特级一致）。`UIPC_KERNEL_*` 宏族随
   `uipc::RUNTIME_CHECK` 启用。
2. **机械迁移** (`8e3299af`) — 280 文件 `muda::`→`cuda_tool::`、伞头改
   `cuda_tool/cuda_tool.h`、宏改名、删 `.name("...")` 标签。
3. **kernel 改写** (`423be546` + `ee4bea1e`) — 273 个 lambda kernel → 命名
   `__global__`（匿名 namespace，body 逐字，捕获→参数）。启动用
   `cuda_tool::best_grid_dim/best_block_dim` 保持同一占用率选择；
   `ParallelFor` 机制随后从 cuda_tool 移除（业务 lambda kernel = 0）。
4. **删除 vendored muda + 构建系统同步** (`cb9341c1`) — 删
   `cuda_tool/muda/`（288 文件）与 `muda_compat.h`；CMake 去掉 MUDA_*
   宏；xmake 去掉 `add_requires/add_packages("muda")`。
5. **两轮 cuda_tool 精简** (`f6fd6bb3`, `2a8f78d7`) — 删零引用原语与
   helper。

## Environment notes (unchanged)

- Build: `output/build.bat`（vcvars64 + `cmake --build build --config
  Release --target sim_case -j8`）；全量错误收集用
  `output/build_keepgoing.bat`（`ninja -k 0`）；全测试目标
  `output/build_all_tests.bat`。
- nvcc 需要 MSVC 环境，裸 shell 报 "Cannot find compiler 'cl.exe'"。
- Configure: `cmake -S . -B build --preset ci-release
  -DUIPC_BUILD_BENCHMARKS=OFF -DUIPC_BUILD_EXAMPLES=OFF`（必须传 `-B
  build`；删/增源码文件后必须重新 configure，因为 file(GLOB) 在配置期展开）。
- Catch2 过滤多个用例名在本机不生效，逐个跑；单个名称过滤可用
  （如 `./uipc_test_sim_case.exe "36_no_surf_but_contact_on"`）。
- `output/test_compile.cu` + `output/compile_smoke.bat` 是 cuda_tool 的
  独立 smoke 编译/运行入口（`src/.../test_compile.cu.txt` 是仓库内档案）。
- 占用率探针：`output/probe_occupancy2.cu` + `.bat`（裸核 vs muda 包装核
  的 `cudaOccupancyMaxPotentialBlockSize` 对比模板）。
- 全套件日志：`output/test_sim_case_fix1.log` / `_run2.log`（迁移后，全过）；
  基线对照 `output/test_sim_case_baseline.log`。
