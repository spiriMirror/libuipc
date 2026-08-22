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

- **性能排查教训（6_wrecking_balls，619a5412 基线 A/B）**：
  1. 真正的回归根因：cuda_tool 初版 cub 封装逐次 `cudaMalloc/cudaFree`
     临时存储（每次 ~10-100µs 且隐式设备同步；每帧几十次 cub 调用）
     → 已修为 stream 级 workspace 缓存（`cub.h` details），帧耗时
     73.9→66.7ms，前 8 帧（接触前）中位数与基线持平。
  2. 排查陷阱 a：**构建争用会彻底污染计时**（sanity_check 曾被误判为
     ~112ms/帧元凶，实际空闲机上仅 ~2-5ms/帧）——计时实验必须空机。
  3. 排查陷阱 b：接触激活后的帧间相位对比无意义——两套二进制块大小
     不同→ULP 级差异→轨迹发散，后程帧已不在同一物理状态。
  4. 该场景 ~65ms/帧的下限构成：dump ~5-10ms + 每帧 Timer.report/日志
     打印 ~10-30ms + sanity_check ~2-5ms + 真管线 20-45ms（基线同样
     如此）——"感觉慢"主要是固有结构，不是回归。

## 场景对角线自适应参数（Stiff-GIPC 对齐，after `7cf19f21`）

- `GlobalVertexManager` init 时计算静止包围盒对角线 `scene_diagonal()`（日志
  打印；wrecking-ball 场景实测 28.93，与 Stiff-GIPC 的 √bboxDiagSize2=√834.9
  完全一致）。
- 新配置（默认 0 = 关闭，向后兼容）：
  - `contact/d_hat_relative`：>0 时 d_hat = 相对值 × 对角线（Stiff-GIPC 的
    relative_dhat 惯例；其 dHat 存平方故原文为 rel²·diag²）。
  - `newton/velocity_tol_relative`：>0 时牛顿退出阈值 = 相对值 × 对角线 × dt
    （MaxTranslationChecker，Stiff-GIPC 的 threshold×diag×dt 惯例）。
- 6_wrecking_balls 已全参数对齐 set_case3（mu 0.2、逐对象密度
  1000/7680、tol_rate 1e-4、相对 d_hat/tol）。**注意：此前记录的
  "advance 26-28ms/帧 ≈ Stiff 1.1×" 是接触前/污染 dll 下的误测；接触段
  真实表现曾崩溃至 3-18s/帧，根因与修复见下节。**
- 性能排查记录：cub workspace 修复 + 两个测量陷阱（构建争用、轨迹发散）
  见 05 文档与上文性能节。

## Stiff-GIPC 屏障对齐：log² 硬屏障（after `4294c1d5`）

- **根因链（wrecking ball 曾 3-18s/帧、牛顿撞 1024 上限）**：
  1. Stiff-GIPC 的增量势能中屏障项**不乘 dt²**（GIPC.cu `computeEnergy`），
     libuipc 屏障系数为 `kt2 = κ·dt²` → 同 κ 名义值下屏障强度差 1/dt²
     （dt=0.01 时 10⁴ 倍）。"两边都设 κ=1e4"的对齐因此是假对齐。
  2. Stiff-GIPC 的屏障是 RANK=2 的 log² 硬屏障 `κ(D-d̂²)²ln²(D/d̂²)`
     （GIPC.cu:28 `#define RANK 2`，`_d_EE` 返回平方距离）；深陷区力比
     经典 log 屏障强 ~2|ln(D/d̂²)| 倍，同载荷下平衡间隙宽 ~10×、平衡曲率
     低 ~10×，牛顿 3-5 次/帧收敛。libuipc 经典 log 屏障（且 κ_eff 弱 10⁴
     倍）深陷 D→0 病态区，CCD 把 α 压到 ~1e-3，牛顿线性爬行数百次。
- **改动**：
  - `sym/codim_ipc_contact.inl` 重新生成（生成器 notebook
    `scripts/symbol_calculation/codim_ipc_contact.ipynb` 同步更新）：
    ξ==0（体积/地面接触）走新生成的 log² 屏障 `KappaBarrierLog2` 系列；
    ξ>0（codim 壳）保持经典带厚度屏障。公开入口 `KappaBarrier`/
    `dKappaBarrierdD`/`ddKappaBarrierddD` 改为运行时 dispatcher，PT/EE/
    PE/PP、地面半平面、摩擦 normal_force 全部自动跟随，无调用点改动。
  - samples 6_wrecking_balls：κ=1e8（等效换算：libuipc κ = Stiff Kappa /
    dt² = 1e4/1e-4）。
- **验证**：探针 120 帧全收敛；牛顿 mean 4.54/max 22（Stiff 3.44/7）；
  min_alpha 0.15-0.72（此前 ~1e-3 爬行）；帧均 132ms（原 3-18s）；
  自由摆动段（f0-80）轨迹与 Stiff 平移对齐后差 <3mm；全套件
  95 用例/14214 断言全绿。
- **已知剩余差距（~5× 帧时）**：PCG 76 次/解 vs Stiff 15，按压帧 PCG
  在 15-50 与 150-565 尖峰间交替（Stiff 全程 20-60 均匀）；每迭代固定
  开销：DCD 检测 4.3ms + DyTopo 装配 3.7ms（接触对数两侧同量级：
  Stiff cpNum ~25k vs libuipc 30-50k）。
- **测量陷阱备忘**：
  1. post-build 只在 pyuipc 重链接时同步 dll —— 仅改后端 .cu 时
     site-packages 的 `uipc_backend_cuda.dll` 不会更新，必须手动同步
     `cp build/Release/bin/uipc_*.dll build/python/src/uipc/_native/` 与
     `.../site-packages/uipc/_native/`。
  2. pyuipc 的 `Scene(config)` 按值拷贝 config  dict，构造后修改 python
     侧 dict 静默无效 —— 所有 config 键必须在 `Scene(config)` 之前设好。
  3. Stiff-GIPC 参考副本 stdout 重定向时 printf 块缓冲，timeout 强杀会丢
     日志——插桩 printf 后必须跟 `fflush(stdout)`。

## 第二轮对齐（摩擦平滑 + CFL 语义，after `81e52fce`）

- **`contact/eps_velocity_relative`**（新配置，默认 0=关闭）：>0 时
  摩擦 C1 平滑阈值 eps_velocity = 相对值 × scene_diagonal（Stiff-GIPC 惯例：
  其每步滑移阈值为 sqrt(fDhat)·dt = 1e-2·diag·dt）。libuipc 原默认
  0.01 m/s 绝对值，该场景下摩擦 Hessian 曲率比 Stiff 硬 ~840 倍。
  实现于 `global_contact_manager.cu` Impl::init（与 d_hat_relative 同模式）。
  6_wrecking_balls 与探针已设 1e-2。效果：线搜 α 恢复 0.9-1.0 健康区间
  （此前 0.15-0.72），帧时持平。
- **CFL 语义修正**：原实现仅统计"已激活接触顶点"的位移（`vert_is_active_contact`
  掩码）——实测 wrecking ball 120 帧零触发，正在高速冲向接触的顶点恰好
  不在掩码内，可一步冲入深间隙。改为 Stiff-GIPC 设计：max|dx| 覆盖**全部
  表面顶点**（`GlobalSimplicialSurfaceManager::surf_vertices`，无表面管理器时
  回退全顶点）；且在 `advance_ipc.cu` 线搜中仅当 CCD 命中（ccd_alpha<1）才
  施加——避免自由飞行帧被无谓限步。本场景实测仍不触发（接触迭代内顶点
  步长多在 5cm 以下），属语义对齐，价值在高速冲击类场景。
- **剩余 ~5× 帧时差距的最终分解（实证）**：
  - PCG 尖峰（150-565 次/解 vs Stiff ≤67）：dump 线性系统（config
    `extras/debug/dump_linear_system=1`）后用 scipy+12×12 块 Jacobi 复算，
    尖峰矩阵本身即需 236-346 次——**是矩阵（状态）难，非求解器 bug**。
    与两侧接触段轨迹混沌发散耦合，无法逐帧对应。
  - PCG tol 放宽到 1e-3：迭代数降 36% 但帧时不变 → PCG 非帧时主因。
  - 每牛顿迭代固定开销 ~12ms：DCD 检测 4.3ms + 轨迹检测 4.1ms +
    DyTopo 装配 3.7ms（检测边际 = d_hat+thickness，不肥；对数两侧同量级
    ~25-50k）。Stiff 全迭代 7.1ms。此为 kernel 级吞吐差距（stackless BVH
    vs mlbvh、通用 triplet/BCOO 管线 vs 手工融合 kernel），需 nsight 级
    性能工程，非算法设计问题。
  - 日志开销实测可忽略（Info vs Warn：132.4 vs 130.9ms/帧）。
- **⚠ 对比口径纠正**：Stiff 日志的 "average time cost" = totalTime/totalNT
  （GIPC.cu:11262），是**每牛顿迭代**的 GPU 时间（totalNT/ totalTime/
  total_Frames 是文件级全局量，跨帧累积），不是每帧时间！用 PAIRCOUNT
  插桩日志按边际量重算：Stiff 帧时 mean 57.6ms / median 54.3ms（按压帧
  63-98ms）。即 libuipc 帧均 130.9ms 对 Stiff 是 **~2.3×（中位 1.45×）**，
  每迭代成本 29.3 vs 24.4ms（1.2×）——并非此前以为的 5×。剩余差距主要
  是按压帧牛顿数 4.5 vs 3.4（~1.3×）。

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
