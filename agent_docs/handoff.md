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

## Cloth stiffness model update + strain_rate exposure (after `b7056879`)

- 布料刚度公式与 mas-pncg 对齐，膜元权重为三角形 **area**（非 volume，
  避免与厚度无关的 shear 被体积测度错误加权）：
  - stretch：`StrainLimitingBaraffWitkinShell` 的三角形属性 `"lambda"` 写
    `(λ+2μ)·t`（恒等 `E·t/(1-ν²)`）；后端 `strain_limiting_baraff_witkin_shell_2d.cu`
    两个能量 kernel 的测度已从 `area·2t` 改为纯 `rest_area`。
  - shear：属性 `"mu"` = `E/(2(1+ν))`，与厚度无关（面积测度下无 t）。
  - bend：`DiscreteShellBending` 系列（含 strain/stress plastic 变体）测度
    统一为面积 `V_bar = A`（3 个 function header）；raw `apply_to(sc, κ)`
    的 κ 语义变为"单位面积刚度"——**所有 raw 调用点已按 κ×t 迁移以保持
    物理一致**（libuipc 测试 33/82-87、regression 全部 t=0.001；samples
    11/24/26/33ext/34 按各自厚度）。公式重载 `apply_to(sc, E, ν)` 与
    静态帮助 `bending_stiffness(E,ν,t)` 写字面值 `E·t³/(12(1-ν²))`。
  - strainRate：不再硬编码 100——`apply_to(..., strain_rate=100)` 写
    三角形属性 `"strain_rate"`，后端读属性（旧场景缺失自动创建并回填
    100）；pybind 同步暴露。
  - **stretch/shear 材料参数分离**：`StrainLimitingBaraffWitkinShell::apply_to`
    双模量重载 `apply_to(sc, stretch_moduli, shear_moduli, ρ, t, strain_rate)`
    （stretch 用 `(λ_s+2μ_s)·t`、shear 用 `μ_sh`，各自独立 (E,ν)，对齐
    mas-pncg ClothMaterialConfig）；单模量旧重载保留兼容。samples
    11_bunny_cloth/34_cloth_stack 已改用分离参数（shear 按 100:1 软化），
    11 的 bend 同时改用公式重载演示；新增
    `apps/tests/core/strain_limiting_baraff_witkin_constitution.cpp`
    覆盖双模量属性布局。
  - **厚度单一来源**：DSB 公式重载为 `apply_to(sc, E, ν)`——弯曲是可选件、
    stretch 是必需件，故厚度只由膜本构设置（vertex `"thickness"` 属性），
    DSB 按边端点平均读取（天然支持非均匀厚度壳）；缺失时给出明确报错。
- 验证：Python 冒烟数值全对；DSB 相关 sim_case 33、82-87 全过；6 快速
  二进制全过；全套件通过；`11_bunny_cloth` headless OK。
- 注意：`ElasticModuli2D`/`EP_to_lame_2D` 与 NeoHookeanShell 等其他本构
  未动（scope 仅布料 BW 应变限制壳 + DSB）；BW 壳无套件/样例使用，
  测度改动零回归面。

## Hygiene & test-robustness batch (after `d2f48087`)

- **Duplicate-include sweep**: 17 files had identical `#include` lines
  (mostly migration-cruft `cuda_tool/cuda_tool.h`); deduped. The
  `geometry_export_types.inl` double-include in `geometry_factory.cpp` is an
  intentional X-macro pattern — do NOT dedupe it.
- **Catch2 v3.8 filter syntax (measured)**: multiple specs as separate argv
  are AND-intersected ("No tests ran"); OR requires comma-separated specs in
  ONE argv: `uipc_test_sim_case "0_abd_gravity,13_fem_3d_gravity"`.
  `--list-tests --verbosity quiet` prints one case name per line.
- **`file(GLOB ... CONFIGURE_DEPENDS)`** added to all 44 project CMake files
  (external/ untouched) — adding/removing sources no longer needs a manual
  re-configure.
- **Line-search diagnostics**: the "Line Search Exits with Max Iteration"
  warning and the strict-mode exception now carry
  `alpha_last / E0 / E_last / rel_E_increase / ccd_alpha / cfl_alpha`
  (`engine/advance_ipc.cu`, `engine/advance_al.cu`) so a threshold-crossing
  can be judged as real regression vs ULP jitter from the log alone.
- **Isolated suite runner**: `scripts/run_sim_case_isolated.py` runs each
  sim case in its own process (`--filter/--start-from/--timeout`) —
  complements the single-process suite to separate cross-case global-state
  pollution from case-local failures.
- (ccache integration was implemented and then reverted at user request;
  nothing ccache-related remains in the tree.)

## Build-time optimization (after `88965feb`)

- **Umbrella split**: `cuda_tool/cuda_tool.h` no longer includes `cub.h`
  (CCCL device-algorithm headers add ~165K preprocessed lines per TU); the
  ~23 files using `Device*` wrappers include `<cuda_tool/cub.h>` explicitly
  (found via API grep over `DeviceReduce(/DeviceScan(/...` + `cub::`).
  `linear_system.h` also dropped its (unused) cub.h include.
- **RDC 纠错过山车**：d2f48087 曾关闭 RDC 并声称"无跨 TU 设备符号"——**结论错误**。`affine_body/utils.cu` 的 `UIPC_GENERIC` 自由函数（`q_to_transform` 等）被其他 TU 的 kernel 调用，关 RDC 必现 `ptxas fatal: Unresolved extern`。当时未暴露是因为 CMake+ninja 不跟踪旗标变化，陈旧 RDC-on 对象混在链接里蒙混过关；pyuipc 构建触发大范围重编才现形。已恢复 `CUDA_SEPARABLE_COMPILATION/RESOLVE_DEVICE_SYMBOLS ON` 并给 xmake 补 `-rdc=true`。同时下条"全量重建 ~4.7 min"的数字也受影响（部分 TU 未重编），仅供参考。
- ~~**RDC off**~~（已纠正，见上）
- Measured (32-core, CUDA 13.2): full rebuild wall **~4.7 min** (was ~10 min
  perceived), CUDA TU CPU 8.3K→6.5K s, per-TU avg 38→35 s. Line-count
  attribution of a non-cub TU (~1.63M lines after -E): CUDA toolkit headers
  ~860K, WinSDK ~310K, MSVC STL ~150K, Eigen ~150K, project <20K — the
  remaining cost is toolchain headers, not project includes. Next lever if
  needed: ccache (`CMAKE_CUDA_COMPILER_LAUNCHER`).
- Verified after both changes: 6 fast binaries pass; full sim suite passes
  (14214 assertions / 95 cases).

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
