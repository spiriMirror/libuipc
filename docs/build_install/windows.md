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
cd libuipc; cd ..; mkdir CMakeBuild; cd CMakeBuild
cmake -S ../libuipc -DUIPC_BUILD_PYBIND=1 
cmake --build . --config <Release/RelWithDebInfo> -j8
```

!!!NOTE
    Use multi-thread to speed up the build process as possible, becasue the NVCC compiler will take a lot of time.

## Run Project

Just run the executable files in `CMakeBuild/<Release/RelWithDebInfo>/bin` folder.

## Install Pyuipc 

With `UIPC_BUILD_PYBIND` option set to `ON`, the Python binding will be **built** and **installed** in the specified Python environment.

If some **errors** occur during the installation, you can try to **manually** install the Python binding.

```shell
cd CMakeBuild/python
pip install .
```

### Conda Environment (Optional)

If you want to install the Python binding in a Conda environment, you should additionally specify the Python executable path of the Conda environment.

First, create a Conda environment with Python >=3.11
```shell
conda create -n uipc_env python=3.11
```

!!!NOTE
    **Don't** activate the Conda environment when compiling Libuipc.
    On Windows it's hard to compile with MSVC/NVCC in conda environment, so we build the C++ part in the system environment and install the Python binding in the conda environment.

```shell
cmake -S ../libuipc -DUIPC_BUILD_PYBIND=1 -DUIPC_PYTHON_EXECUTABLE_PATH=<PYTHON_EXE_IN_CONDA_ENV>
cmake --build . --config <Release/RelWithDebInfo> -j8
```
For example, the `<PYTHON_EXE_IN_CONDA_ENV>` may be  `C:\Users\<UserName>\anaconda3\envs\uipc_env\python.exe`

## Check Installation

You can run the `uipc_info.py` to check if the `Pyuipc` is installed correctly.

```shell
cd libuipc/python
python uipc_info.py
```

More samples are at [Pyuipc Samples](https://github.com/spiriMirror/libuipc-samples).




