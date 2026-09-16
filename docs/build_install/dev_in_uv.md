# Development in UV

[Install UV](https://docs.astral.sh/uv/getting-started/installation/) if you haven't already.

Install vcpkg as [Windows](./windows.md) or [Linux](./linux.md) guides.

The root package continues to use scikit-build-core with Git-derived distribution
metadata. Its existing CMake editable support is available; there is no custom
`builder=xmake` backend or `python_editable` XMake option.

## Dependency setup

Create and activate a Python environment, then choose the build entry point.
For example:

```bash
uv venv --seed --python 3.12 .venv
source .venv/bin/activate
```

On Windows PowerShell, activate with `.\.venv\Scripts\Activate.ps1` instead.

- **Direct CMake configuration:** missing Python modules are installed
  automatically with the selected interpreter's `python -m pip`. Importable
  modules are reused. An installation or subsequent import failure stops
  configuration with an error.
- **Root pip/uv builds:** build isolation provisions the requirements declared
  in `pyproject.toml`. Scikit-build-managed CMake does not bootstrap pip or
  install extra packages inside that build. With `--no-build-isolation`, install
  the requirements into the chosen environment yourself.

These dependency checks do not change the existing uninstall-first package
staging workflow.

## CMake development

For the non-isolated command below, install all Python build requirements:

```bash
uv pip install scikit-build-core setuptools-scm pybind11 pybind11-stubgen numpy typing_extensions
```

Build the project and install the Python package in editable mode; specify the build directory to cache the CMake build results; and disable build isolation to reuse the prepared dependencies:

```bash
uv pip install -e . --config-settings=build-dir=build --no-build-isolation -v
```

Run the tests, with `--no-sync` to prevent uv from rebuilding the project every time you run the tests (which is slow):

```bash
uv run --no-sync pytest python/tests
```

The root wheel settings target several GPU architectures and require a sufficiently
new CUDA toolkit. For a local-GPU build, append
`--config-settings=cmake.define.UIPC_CUDA_ARCHITECTURES=native`.
Python source edits are visible through the editable install; native C++/CUDA
changes still require a rebuild and refreshed binaries.

## XMake

Use [xmake](https://xmake.io/) to build the project. The following steps configure xmake with Python binding support, create a virtual environment, build the project, and package the Python wheel.

```shell
uv venv --seed --python 3.12 .venv
source .venv/bin/activate
xmake f --pybind=true --python_system=true --python_version=3.12.x -c
xmake build -j8
xmake pack -v
```

`xmake pack` invokes the same full-package stub generator as CMake. If
pybind11-stubgen, numpy or typing_extensions is missing, packaging installs
them with `uv pip --python` targeting its chosen interpreter.
This is the normal XMake wheel workflow, not a new editable install backend.
