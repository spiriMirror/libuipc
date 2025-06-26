# Build on Linux

## Prerequisites

The following dependencies are required to build the project.

| Name                                                | Version      | Usage           | Import         |
| --------------------------------------------------- | ------------ | --------------- | -------------- |
| [CMake](https://cmake.org/download/)                | >=3.26       | build system    | system install |
| [Python](https://www.python.org/downloads/)         | >=3.11       | build system    | system install |
| [Cuda](https://developer.nvidia.com/cuda-downloads) | >=12.4       | GPU programming | system install |
| [Vcpkg](https://github.com/microsoft/vcpkg)         | >=2024.11.16 | package manager | git clone      |

## Install Vcpkg

If you haven't installed Vcpkg, you can clone the repository with the following command:

```shell
mkdir ~/Toolchain
cd ~/Toolchain
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
```

The simplest way to let CMake detect Vcpkg is to set the **System Environment Variable** `CMAKE_TOOLCHAIN_FILE` to `~/Toolchain/vcpkg/scripts/buildsystems/vcpkg.cmake`

```shell
# Write in ~/.bashrc
export CMAKE_TOOLCHAIN_FILE="$HOME/Toolchain/vcpkg/scripts/buildsystems/vcpkg.cmake"
```

## Clone Libuipc

Clone the repository with the following command:

```shell
git clone https://github.com/spiriMirror/libuipc.git
```

## Conda Environment

We **recommend** using conda environments to build the project on Linux.

```shell
conda env create -f conda/env.yaml
```

Cuda-12.4.0 requires driver version >= 550.54.14 (https://docs.nvidia.com/deploy/cuda-compatibility/index.html#use-the-right-compat-package), check your nvidia driver version with the following command.

```shell
nvidia-smi
```

## Other Environment

If you don't want to use conda, you can manually install `CMake 3.26`, `GCC 11.4`, `Cuda 12.4` and `Python >=3.11` with your favorite package manager.

## Build Libuipc

Build the project with the following commands.

```shell
conda activate uipc_env
cd libuipc; cd ..; mkdir CMakeBuild; cd CMakeBuild;
cmake -S ../libuipc -DUIPC_BUILD_PYBIND=1 -DCMAKE_BUILD_TYPE=<Release/RelWithDebInfo> 
cmake --build . -j8
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

## Check Installation

You can run the `uipc_info.py` to check if the `Pyuipc` is installed correctly.

```shell
cd libuipc/python
python uipc_info.py
```

More samples are at [Pyuipc Samples](https://github.com/spiriMirror/libuipc-samples).