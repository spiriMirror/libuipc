# Build on Windows

## Prerequisites

For editable Python development with either backend, see the complete
[CMake/XMake editable guide](./dev_in_uv.md). The Windows commands are also
listed below in PowerShell form.

The following dependencies are required to build the project.

| Name                                                | Version      | Usage           | Import         |
| --------------------------------------------------- | ------------ | --------------- | -------------- |
| [CMake](https://cmake.org/download/)                | >=3.26       | build system    | system install |
| [XMake](https://xmake.io/)                          | >=3.0.5      | build system    | system install |
| [Python](https://www.python.org/downloads/)         | >=3.11       | build system    | system install |
| [Cuda](https://developer.nvidia.com/cuda-downloads) | >=12.4       | GPU programming | system install |
| [Vcpkg](https://github.com/microsoft/vcpkg)         | >=2025.7.25  | package manager | git clone      |

Python development headers, a C++20 compiler (Visual Studio 2022 is
recommended), and network access for the first vcpkg/XMake dependency download
are also required. The default build enables CUDA, so install the CUDA Toolkit
with `nvcc` and a compatible NVIDIA driver.

## Install Vcpkg

If you haven't installed Vcpkg, you can clone the repository with the following command:

```shell
mkdir ~/Toolchain
cd ~/Toolchain
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.bat
```

The simplest way to let CMake detect Vcpkg is to set the **System Environment Variable** `CMAKE_TOOLCHAIN_FILE` to `~/Toolchain/vcpkg/scripts/buildsystems/vcpkg.cmake`

You can set the environment variable in the PowerShell:

```shell
# In PowerShell: Permanently set the environment variable
[System.Environment]::SetEnvironmentVariable("CMAKE_TOOLCHAIN_FILE", "~/Toolchain/vcpkg/scripts/buildsystems/vcpkg.cmake", "User")
```

## Build Libuipc

Clone the repository with the following command:

```shell
git clone https://github.com/spiriMirror/libuipc.git
```

### CMake-GUI

On Windows, you can use the `CMake-GUI` to **configure** the project and **generate** the Visual Studio solution file with only a few clicks.

- Toggling the `UIPC_BUILD_PYBIND` option to `ON` to enable the Python binding.

### CMake-CLI

Or, you can use the following commands to build the project.

```shell
cd libuipc; mkdir build; cd build
cmake -S .. -DUIPC_BUILD_PYBIND=1
cmake --build . --config <Release/RelWithDebInfo> -j8
```

The default CUDA build retains the V0, stackless, and linear-BVH comparison
filters. Add `-DUIPC_WITH_CUDA_LEGACY_COLLISION=OFF` to the configure command
for a lean build containing only the default broad-phase trajectory filter.

!!!NOTE
    Use multi-thread to speed up the build process as possible, becasue the NVCC compiler will take a lot of time.

## Build Libuipc with XMake

If you prefer XMake over CMake, use the following commands in PowerShell or `cmd`.

```shell
cd libuipc
xmake f -c
xmake build -j8
```

Enable Python bindings with the following configuration.

```shell
cd libuipc
xmake f --pybind=true --python_system=true --python_version=3.11.x -c
xmake build -j8
```

If you are building against another Python installation, replace `3.11.x` with the version you want XMake to resolve.

The build outputs are placed under `build/`, and the staged Python package is generated in `build/.xpack/pyuipc`.

## Run Project

Just run the executable files in `build/<Release/RelWithDebInfo>/bin` folder.

## Install Pyuipc 

With `UIPC_BUILD_PYBIND` option set to `ON`, the Python binding will be **built** and **installed** in the specified Python environment.

If some **errors** occur during the installation, you can try to **manually** install the Python binding.

```shell
cd build/python
pip install .
```

## Conda Environment (Alternative)

Create and activate a conda environment with the following command:

```shell
conda env create -f conda/env.yaml
conda activate uipc_env
```

Setup the `CMAKE_TOOLCHAIN_FILE` environment variable in the conda environment:

```shell
conda env config vars set CMAKE_TOOLCHAIN_FILE=~/Toolchain/vcpkg/scripts/buildsystems/vcpkg.cmake
```

Then, you can build the project with the same commands as above in the conda environment.

## Check Installation

You can run the `uipc_info.py` to check if the `Pyuipc` is installed correctly.

```shell
cd libuipc/python
python uipc_info.py
```

More samples are at [Pyuipc Samples](https://github.com/spiriMirror/libuipc-samples).

## Install in Any Python Venv

If you want to install the Pyuipc to any Python Venv (like [uv](https://docs.astral.sh/uv/)) after build, you can use the following command:

```shell
cmake -S .. -DUIPC_BUILD_PYBIND=1 -DUIPC_PYTHON_EXECUTABLE_PATH=<YOUR_PYTHON_EXECUTABLE_PATH>
cmake --build . --config <Release/RelWithDebInfo> -j8
```

## Editable Python Install with CMake or XMake

Use a Python 3.12 environment. `uv --seed` installs pip into the new venv;
Conda environments with Python and pip work as well.

```powershell
$Project = "C:\path\to\libuipc"
Set-Location $Project

uv venv --seed --python 3.12 .venv
.\.venv\Scripts\Activate.ps1

python --version
python -m pip --version
xmake --version
cmake --version
nvcc --version
```

For CMake, configure vcpkg in the current PowerShell session. Replace the
path if vcpkg is installed elsewhere:

```powershell
$env:CMAKE_TOOLCHAIN_FILE = "$HOME\Toolchain\vcpkg\scripts\buildsystems\vcpkg.cmake"
if (-not (Test-Path $env:CMAKE_TOOLCHAIN_FILE)) { throw "CMAKE_TOOLCHAIN_FILE not found" }
$env:VCPKG_DISABLE_METRICS = "1"
$env:CMAKE_BUILD_PARALLEL_LEVEL = "4"

python -m pip install -v -e . `
  --config-settings=builder=cmake `
  --config-settings=build-dir=build/cmake-editable
```

The CMake editable hook supplies the development version `0.9.0`. Its build
requirements (`pybind11`, `pybind11-stubgen`, `numpy`, and
`typing_extensions`) are installed into pip's isolated build environment, and
the CMake helper only checks them; it does not run `ensurepip` or install into
the target interpreter. Re-run the same command after native code changes.

For XMake, select the backend explicitly:

```powershell
$env:CMAKE_BUILD_PARALLEL_LEVEL = "4"

python -m pip install -v -e . `
  --config-settings=builder=xmake `
  --config-settings=jobs=4
```

XMake also reports the development version `0.9.0` from
`python/pyproject.toml`. For repeated local builds, `--no-build-isolation` is
optional; install the build requirements in the active environment first:

```powershell
python -m pip install "setuptools>=64" wheel
python -m pip install -v -e . `
  --config-settings=builder=xmake `
  --config-settings=jobs=4 `
  --no-build-isolation
```

`uv pip install -e .` currently misreads the repository's in-tree PEP 517
`backend-path`; use `python -m pip` for the root project. `uv` remains suitable
for creating the venv and installing individual packages.

Verify either installation:

```powershell
python -c "import importlib.metadata as m, uipc; from uipc._native import pyuipc; print(m.version('pyuipc')); print(uipc.__file__); print(pyuipc.__file__)"
python -m pip list --editable
python -m uipc doctor --probe-cuda
```

The doctor command initializes the CUDA backend. An import alone does not
verify that a CUDA engine can be constructed.
