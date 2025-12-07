# Build on Windows

## Prerequisites

The following dependencies are required to build the project.

| Name                                                | Version      | Usage           | Import         |
| --------------------------------------------------- | ------------ | --------------- | -------------- |
| [CMake](https://cmake.org/download/)                | >=3.26       | build system    | system install |
| [Python](https://www.python.org/downloads/)         | >=3.11       | build system    | system install |
| [Cuda](https://developer.nvidia.com/cuda-downloads) | >=12.4       | GPU programming | system install |
| [Vcpkg](https://github.com/microsoft/vcpkg)         | >=2025.7.25  | package manager | git clone      |

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

!!!NOTE
    Use multi-thread to speed up the build process as possible, becasue the NVCC compiler will take a lot of time.

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
