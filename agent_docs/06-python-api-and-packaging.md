# 06 — Python API 与打包

## pybind 结构（`src/pybind/pyuipc/`）

入口 `module.cpp` 的 `PYBIND11_MODULE(pyuipc, m)`：

- 子模块：`unit`、`geometry`、`constitution`、`diff_sim`、`core`、`backend`、`builtin`、`usd`（USD 仅 `UIPC_WITH_USD_SUPPORT` 时）。
- C++ 命名空间 `pyuipc::xxx` ↔ Python 子模块 `pyuipc.xxx` 一一对应；每个子目录有自己的 `module.cpp`，用 `PyXxx{m}` 构造器逐类绑定。
- **早期暴露**：主 `module.cpp` 先绑定核心数据结构（`PyFeature`、`PyBufferView`、`PyAttributeSlot`、`PyGeometry`、`PySimplicialComplex`、各类 Slot、`PyParameterCollection`），再调各子模块 `PyModule`（几何工具/IO 依赖 core 类型）。
- 顶层别名：`Engine`、`World`、`Scene`、`SceneIO`、`Animation` 提升到 `pyuipc` 顶层；另注册 `init`、`default_config`、`config`、`uipc::Exception`、`__version__`。
- 构建：`pybind11_add_module(pyuipc)` 链 `uipc::uipc` + `uipc::backends`，POST_BUILD 调 `scripts/after_build_pyuipc.py`（拷包、拷依赖库、pybind11_stubgen 生成 `.pyi`）。

## Python 包布局（`python/src/uipc/`）

- `__init__.py`：import 时先 `pyuipc.init(config)`（注入 `module_dir` 指向 `_native`），再 `from ._native.pyuipc import *`。
- `core.py / geometry.py / constitution.py / backend.py / builtin.py / diff_sim.py / unit.py`：每个仅一行 `from uipc._native.pyuipc.xxx import *`（纯转发原生模块）。
- 纯 Python 增强层：
  - `gui.py`（polyscope 可视化）
  - `profile/`（benchmark 计时 + `nsight.py` Nsight Compute 封装）
  - `cli/`（`benchmark`、`mesh_doctor`、`uid_info`）
  - `adapter/torch`、`adapter/warp`（ML 框架适配）
  - `assets/`（从 HuggingFace `MuGdxy/uipc-assets` 下载场景，每个资产有 `scene.py` 的 `build_scene(scene)`）
  - `stats.py`、`dev/`
- 示例：`python/examples/`（4 个 demo：shell 塑性折痕 ×2、prismatic/revolute joint limit GUI）。

## 打包管线

**正式发布（根 `pyproject.toml`，scikit-build-core）**：
- 版本由 `setuptools_scm`（release-branch-semver）动态生成。
- `wheel.packages = ["python/src/uipc"]`；CMake 定义 `UIPC_BUILD_PYBIND/WHEEL=ON`、`UIPC_CUDA_ARCHITECTURES=89`，关 tests/examples/benchmarks。
- `if(DEFINED SKBUILD)` 时 `UIPC_INSTALL_DIR = uipc/_native`：pyuipc 扩展 + vcpkg 运行时 DLL 全部装进包内 `_native/`；`.pyi` stub 装到包根。
- cibuildwheel：linux 下 auditwheel 排除全部 CUDA 库（依赖系统 CUDA 12.8）。

**开发模式（`python/pyproject.toml` + `python/setup.py`）**：
- CMake 构建时 `after_build_pyuipc.py` 把 `python/src/` + pyproject 拷到 `<build>/python/`，扩展与共享库拷进 `src/uipc/_native/`，生成 stub，非 wheel 模式直接 `pip install`。
- `setup.py` 的 `BuildPyCommand` 从 `build/vcpkg_installed/<triplet>/{bin,lib}` 与 `build/<config>/bin` 收集 dll/so。
- uv editable 开发：`uv run --no-sync pytest python/tests`（`--no-sync` 避免每次重构建）。

## Python 测试

`python/tests/`，pytest。前置：装好 pyuipc（CMake 构建或 `uv pip install -e .`）。

## 绑定扩展要点

- 新增 C++ 公开 API 时考虑同步 pybind：在对应子模块目录加 `PyXxx` 绑定文件并在该目录 `module.cpp` 注册；保持命名空间映射一致。
- 不要破坏 `__init__.py` 的 import 链（`pyuipc` → `init()` → `config["module_dir"]`）。
- 新绑定面需在 `python/tests/` 补测试。
