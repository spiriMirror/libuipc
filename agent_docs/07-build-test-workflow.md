# 07 — 构建、测试与开发规范

## 依赖与环境

- CMake ≥ 3.26、Python ≥ 3.11、CUDA ≥ 12.4（wheel 用 12.8）、Vcpkg（设 `CMAKE_TOOLCHAIN_FILE`）。
- vcpkg 清单由 `scripts/gen_vcpkg_json.py` 生成。基础依赖：eigen3、catch2、libigl、spdlog、fmt（锁 10.2.1）、cppitertools、dylib、nlohmann-json、magic-enum、tinygltf、tbb、urdfdom、cpptrace、octree（私有 registry spiriMirror/vcpkg，同 registry 还有 ftetwild）；GUI 追加 imgui/glfw3/opengl/freeglut/bgfx；VDB 追加 openvdb。
- `external/muda/` 为源码内置的 CUDA 封装库（kernel 启动、buffer、线性代数）。

## CMake

**选项**（根 `CMakeLists.txt`）：`UIPC_USING_LOCAL_VCPKG`(ON)、`UIPC_BUILD_GUI`(OFF)、`UIPC_BUILD_PYBIND`(OFF)、`UIPC_BUILD_PYTHON_WHEEL`(OFF)、`UIPC_BUILD_EXAMPLES/TESTS/BENCHMARKS`(ON)、`UIPC_DEV_MODE`(OFF)、`UIPC_WITH_USD_SUPPORT`(OFF)、`UIPC_WITH_VDB_SUPPORT`(OFF)、`UIPC_WITH_CUDA_BACKEND`(ON，Apple 强制 OFF)、`UIPC_CUDA_ARCHITECTURES`("native")。WHEEL=ON ⇒ PYBIND=ON。

**Presets**（`CMakePresets.json` v6，Ninja 生成器）：
- `ci-release`：Release 默认构建
- `ci-build-wheel`：Release + PYBIND/WHEEL ON
- （无名为 `release` 的 preset）

```bash
cmake --preset ci-release && cmake --build --preset ci-release -j8
# 或手动：mkdir build && cd build && cmake -S .. -DUIPC_BUILD_PYBIND=ON && cmake --build . --config Release -j8
```

**输出目录**：Windows `<build>/<config>/bin`（runtime+library）与 `.../lib`；Linux `<build>/<CMAKE_BUILD_TYPE>/bin|lib`。聚合目标 `uipc::uipc`（= core+geometry+constitution+io+sanity_check）。

## XMake（备选）

```bash
xmake f -c                 # 配置（默认 release；-m releasedbg / -m debug）
xmake build -j8            # -j 别太高，NVCC 易 OOM
xmake run sim_case         # 跑测试目标
```
测试 target 名经 `uipc_test` 规则改写为二进制名 `uipc_test_<target>`；`xmake run --help` 列全部可跑目标。

## 测试体系

**C++（Catch2，`apps/tests/`）**，`uipc_add_test(<name>)` 生成 `uipc_test_<name>`，链接 `app` 工具库（`AssetDir` 等）：

| 目录 | 内容 |
|---|---|
| `common/` | range、run_length_encode 等基础设施 |
| `geometry/` | 网格算法 ~20 项（extract_surface、compute_volume、label_*、urdf_io 等） |
| `core/` | engine、scene、object、contact_model、scene_io、serializer 等 |
| `backends/cuda/` | CUDA 后端专项 |
| `sanity_check/` | 内置检查器（表面距离/交叉、半平面距离） |
| `regression/` | 回归测试 |
| `sim_case/` | **94 个编号仿真用例**：`abd_*`（仿射体/关节）、`fem_*`、`fem_mas_*`（线性系统）、`*_stitch`、`discrete_shell_bending*`、joint 系列（revolute/prismatic/spherical/fixed + limit/driving/external force） |
| `usd/`、`gui/` | 条件编译 / 未完成 |

运行：`./build/<config>/bin/uipc_test_<name> ["test name"] ['[tag]'] [--list-tests] [--log-level info]`。

**Python（pytest）**：`uv run --no-sync pytest python/tests` 或 `.venv/bin/pytest python/tests`。

**典型 sim_case 流程**（如 `0_abd_gravity.cpp`、`14_fem_3d_ground_contact.cpp`、`37_abd_revolute_joint.cpp`）：`Engine{"cuda"}` → 配 `Scene::default_config()`（gravity/contact/friction/line_search）→ 建几何 + `apply_to` 本构 → `world.init(scene)` → 循环 `advance/retrieve` + `SceneIO::write_surface` 每帧导 obj。`0_abd_gravity` 用两个 SECTION 对比 `ipc` 与 `al-ipc`。

## CI / 发布

- `.github/workflows/`：`cmake.yml`（PR 构建 Win/Ubuntu + CUDA 12.8）、`xmake.yml`、`clang-format.yml`（PR 变更 C++ 文件的格式检查，clang-format 18）、`python-wheels.yml`（cibuildwheel 跨平台 PyPI wheel，Win/Linux，Python 3.10–3.13，CUDA 12.8）、`docs.yml`、`hotfix_publish.yml`。
- `.github/PULL_REQUEST_TEMPLATE.md`：PR 审查清单（fast-fail、C++ 风格、GPU、本构、构建/绑定、测试），源自 review-pr skill。
- docker：`artifacts/` 提供 dev-cmake-cu128/cu130、dev-xmake 等 compose 服务。
- 版本 tag 与 release 流程见 `.cursor/skills/push-tag/SKILL.md`。

## 开发规范摘要（源自 .cursor/）

**C++ 风格**（`.cursor/rules/cpp-format.mdc`，对齐根目录 `.clang-format`）：
4 空格缩进、80 列、Allman 大括号、左对齐指针（`int* p`）、`if(cond)` 无空格、`namespace uipc::module {}` 内层缩进、参数一行一个（不 bin-pack）、不排序 include、`#pragma once`。

**代码准则**（review-pr 清单）：
- 内部不变量用 `UIPC_ASSERT`，用户输入用 `UIPC_ASSERT_THROW`（fast-fail）。
- 禁 C 风格代码与裸指针参数（用 `span<T>` / `muda::BufferView`）；禁 `const std::string&` 参数（用 `std::string_view`）；禁多重继承。
- 命名：类型 `CamelCase`，函数 `snake_case`，成员 `m_` 前缀。
- 新后端用 `uipc_add_backend(name)` 宏；新源码子目录同步 `file(GLOB)` 与 `xmake.lua`。

**提交**：Conventional Commits `<type>(<scope>): <summary>`；type ∈ feat/fix/refactor/perf/test/docs/build/ci/chore；scope 用模块名（geometry/core/cuda/io…）；summary 小写祈使句无句号；关联 issue 用 `Fixes #N`。

**仿真开发**（simulation-dev）：正确性 > 可调试 > 性能；边界处验证索引/形状；kernel 顶部索引守卫；关键检查点 `isfinite`；调试期 `muda::debug_sync_all()`；调试闭环：最小复现 → 假设验证 → 最小修复 → 原场景验证 → 回归测试。

**文档**（document skill）：specification 用 LaTeX（`$$`/`$`、公式后定义变量、编号小节 `## #12 XXX`）；tutorial 用 C++/Python 双 tab、完整可跑示例；相对路径互链。

## 常用工作流入口（.cursor/skills/）

构建测试 `cmake-workflow` / `xmake-workflow` / `run-tests`；提交/PR `commit-convention` / `commit` / `github-pr` / `fix-issue` / `fix-pr` / `review-pr` / `push-tag`；GPU 优化 `gpu-optimization`；仿真开发 `simulation-dev`；文档 `document`；仓库布局 `project-structure`；fork 远程 `repository-setup`（gh 命令统一 `--repo spiriMirror/libuipc`）。
