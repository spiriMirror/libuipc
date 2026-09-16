# Editable Python Development with CMake or XMake

These instructions build the native extension and install the Python package
in editable mode. Python files are imported from the checkout; rebuild the
native extension after C++ or CUDA changes. The examples use `.` when the
current directory is the repository root. From another directory, replace it
with the absolute checkout path, such as `/home/user/src/libuipc` or
`C:\\src\\libuipc`.

## Prerequisites

The examples use Python 3.12 in either a uv venv or a Conda environment.
Both backends need Git, Python development headers,
a C++20 compiler compatible with the CUDA toolkit, and network access for the
first dependency download. The default build enables CUDA, so install the
CUDA Toolkit (including `nvcc`) and a compatible NVIDIA driver.

| Backend | Required tools |
| ------- | -------------- |
| CMake | CMake >=3.26, Ninja, and bootstrapped vcpkg >=2025.7.25. Set `CMAKE_TOOLCHAIN_FILE` to vcpkg's `scripts/buildsystems/vcpkg.cmake`. |
| XMake | XMake >=3.0.5 on `PATH`. XMake resolves third-party packages itself and does not use the vcpkg toolchain. |

The [Linux build guide](./linux.md) documents CUDA >=12.4 for source builds.
The default wheel settings contain compute capability 12.0 and are intended
for CUDA 12.8; use the `native` override below for a local GPU build.

Create a venv with [uv](https://docs.astral.sh/uv/getting-started/installation/):

```bash
uv venv --seed --python 3.12 .venv
source .venv/bin/activate
which python
python --version
python -m pip --version
```

[`--seed`](https://docs.astral.sh/uv/reference/cli/#uv-venv--seed) supplies pip.
Alternatively, start in a shell without an activated venv and use Conda:

```bash
conda create -n uipc-dev -c conda-forge python=3.12 pip setuptools -y
conda activate uipc-dev
which python
python -m pip --version
```

Verify the executable as well as the prompt: an active venv can shadow
Conda's Python on `PATH`. Keep separate build directories when switching
interpreters. XMake writes native libraries into the checkout, so separate
venvs alone do not isolate xmake builds for different Python ABIs; use
separate checkouts for independent comparisons.

The root install examples use `python -m pip`. An in-tree `backend-path`
failure was observed with `uv pip` 0.12.5 for this checkout. uv can still
create environments and install individual dependencies.

## CMake editable install

Set the vcpkg toolchain to the actual installation on the machine:

```bash
export CMAKE_TOOLCHAIN_FILE="$HOME/Toolchain/vcpkg/scripts/buildsystems/vcpkg.cmake"
test -f "$CMAKE_TOOLCHAIN_FILE"
export VCPKG_DISABLE_METRICS=1
```

Install with the normal isolated PEP 517 build. The root `pyproject.toml`
declares the Python modules that CMake checks, so pip puts them in the
temporary build environment before configuration.

```bash
CMAKE_BUILD_PARALLEL_LEVEL=4 python -m pip install -v -e . \
  --config-settings=builder=cmake \
  --config-settings=build-dir=build/cmake-editable
```

The persistent build directory enables incremental rebuilds. `builder=cmake`
also overrides a shell setting of `UIPC_BUILDER=xmake`.
The CMake helper only checks modules. It does not install or upgrade Python
packages during configuration; missing modules produce an error with setup
instructions.

For repeated local builds, `--no-build-isolation` is also supported when the
same environment already contains the build requirements. Install these
modules before a direct `cmake` configuration with `UIPC_BUILD_PYBIND=ON` as
well; CMake no longer installs missing modules automatically.

```bash
python -m pip install setuptools scikit-build-core setuptools-scm ninja \
  pybind11 pybind11-stubgen numpy typing_extensions
python -m pip install -v -e . --no-build-isolation \
  --config-settings=builder=cmake \
  --config-settings=build-dir=build/cmake-editable
```

To compile only for the local GPU, add:

```bash
--config-settings=cmake.define.UIPC_CUDA_ARCHITECTURES=native
```

The CMake editable path reports the development version `0.9.0`, matching the
xmake editable path. This override applies only to editable CMake hooks;
non-editable wheels continue to derive their version from Git through
setuptools-scm. Re-run the same command after native code changes.

## XMake editable install

The xmake route is opt-in and uses the metadata under `python/` to create the
editable wheel:

```bash
python -m pip install -v -e . \
  --config-settings=builder=xmake \
  --config-settings=jobs=4
```

The backend runs `xmake f` and builds `pyuipc`, then installs the extension and
its dependency libraries into `python/src/uipc/_native`. Pip provisions
setuptools for this isolated route. For repeated rebuilds, `--no-build-isolation`
is also supported after installing the build requirements in the active
environment. To match an existing `releasedbg` build, append:

```bash
--config-settings=xmake-args="-m releasedbg"
```

The xmake backend defaults to `release` and stores its configuration in
`build/xmake-pep517`. A different manually configured xmake mode may therefore
recompile the CUDA translation units. This path reports the hardcoded version
in `python/pyproject.toml` (currently `0.9.0`), not the Git-derived CMake
version. Re-run the same command after native code changes.

## Verify either installation

```bash
python - <<'PY'
import importlib.metadata as metadata
import sys
import uipc
from uipc._native import pyuipc

print("Python:", sys.executable)
print("Version:", metadata.version("pyuipc"))
print("Package:", uipc.__file__)
print("Native:", pyuipc.__file__)
PY
python -m pip list --editable
```

For a CUDA runtime check, which initializes the GPU, run:

```bash
python -m uipc doctor --probe-cuda
```

An import alone does not verify that the CUDA backend can construct an
engine. The probe checks initialization, not a full simulation episode.

The manual xmake workflow (`xmake f --python_editable=true`, `xmake build`,
then `uv pip install -e python`) and stub generation details are documented in
the [XMake guide](./xmake.md#editable-install).
