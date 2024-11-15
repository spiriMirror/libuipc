# Libuipc

A Cross-Platform Modern C++20 **Lib**rary of **U**nified **I**ncremental **P**otential **C**ontact.

This page will guide you through building the project.

We use Vcpkg and PyPI to manage the libraries and use CMake to build the project. 

Vcpkg supports both Windows and Linux, we use it to manage the dependencies and keep the consistency of the development environment.


## Dependencies

The following dependencies are required to build the project.

| Name                                        | Version      | Usage           | Import         |
| ------------------------------------------- | ------------ | --------------- | -------------- |
| [CMake](https://cmake.org/download/)        | >=3.26       | build system    | system install |
| [Python](https://www.python.org/downloads/) | >=3.10       | build system    | system install |
| [Vcpkg](https://github.com/microsoft/vcpkg) | >=2024.04.26 | package manager | git clone      |

The following are **libuipc**'s 3rd-party dependencies. Don't worry, most of them will be automatically installed by Vcpkg.

| Name                                   | Version | Usage                                               | Import         |
| -------------------------------------- | ------- | --------------------------------------------------- | -------------- |
| [muda](https://github.com/MuGdxy/muda) | -       | improve safety and readability of CUDA programming. | submodule      |
| cuda                                   | >=12.0  | GPU programming                                     | system install |
| eigen3                                 | 3.4.0   | matrix calculation                                  | vcpkg          |
| catch2                                 | 3.5.3   | unit tests                                          | vcpkg          |
| libigl                                 | 2.5.0   | mesh processing                                     | vcpkg          |
| rapidcsv                               | 8.80    | csv file IO                                         | vcpkg          |
| spdlog                                 | 1.12.0  | logging                                             | vcpkg          |
| fmt                                    | 10.1.1  | fast string formatting                              | vcpkg          |
| cppitertools                           | 2.1#3   | python-like iteration tools                         | vcpkg          |
| bgfx                                   | 1.127#1 | cross-platform RHI                                  | vcpkg          |
| dylib                                  | 2.2.1   | cross-platform dynamic library loader               | vcpkg          |
| benchmark                              | 1.8.3#3 | microbenchmark support library                      | vcpkg          |
| nlohmann_json                          | 3.11.2  | json file IO                                        | vcpkg          |
| imgui                                  | 1.90.7  | GUI                                                 | vcpkg          |
| glfw3                                  | 3.3.8#2 | window management                                   | vcpkg          |
| magic_enum                             | 0.9.3   | enum to string                                      | vcpkg          |


### Automatic Dependency Installation

The Dependencies and submodules will be automatically installed in the CMake Configuration step.

If the automatic installation fails, please raise an issue with the CMake error message.

## Windows

### Install Vcpkg

If you haven't installed Vcpkg, you can clone the repository with the following command:

```shell
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.bat
```
The simplest way to let CMake detect Vcpkg is to set the system environment variable `CMAKE_TOOLCHAIN_FILE` to `(YOUR_VCPKG_PARENT_FOLDER)/vcpkg/scripts/buildsystems/vcpkg.cmake`

### Build Libuipc

On Windows, you can use the `CMake-GUI` to configure the project and generate the Visual Studio solution file with only a few clicks.

Or, you can use the following commands to build the project.

```shell
cd libuipc; cd ..; mkdir CMakeBuild; cd CMakeBuild
cmake -S ../libuipc -DCMAKE_BUILD_TYPE=<Debug/Release/RelWithDebInfo>
cmake --build .
```

### Run Project

Just run the executable files in `CMakeBuild/<Debug/Release/RelWithDebInfo>/bin` folder.

### Build Pyuipc

Pyuipc is a Python binding of libuipc. It is built with the `pybind11` library.

Download python3 https://www.python.org/downloads/.

Install packages with the following command:

```shell
pip install pybind11 mypy numpy
```

We use `mypy.stubgen` to generate the stub files for the Python binding.  The stub files will be generated in the `libuipc/python/typings` folder automatically after building `pyuipc`.

Add `-DUIPC_BUILD_PYBIND=1` to the CMake command to build the Python binding.

NOTE: `pyuipc` should be built in the **Release** or **RelWithDebInfo** mode.

(Optional) If you need torch support, you need to install the torch package first.

```shell
pip install torch
```

Add `-DUIPC_BUILD_TORCH_EXTENSION=1` to the CMake command to enable the pytorch extension.

### Install Pyuipc

After building the project, install the Python binding with the following command:

```shell
cd libuipc/python; pip install .
```

Then you can use the `pyuipc` in your Python environment with:

```python
from pyuipc_loader import pyuipc
```

## Linux

### Install Vcpkg

If you haven't installed Vcpkg, you can clone the repository with the following command:

```shell
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
```

We recommend using conda environments to build the project on Linux. See the [Conda](#Conda) section for more details. If you don't want to use conda, then go on with the following steps.

### Build Libuipc

You can use the following commands to build the project.

```shell
cd libuipc; cd ..; mkdir CMakeBuild; cd CMakeBuild
cmake -S ../libuipc -DUIPC_BUILD_GUI=0 -DCMAKE_BUILD_TYPE=<Debug/Release/RelWithDebInfo>
cmake --build .
```

To enable GUI support, set `-DUIPC_BUILD_GUI=1`, but you may need to install some additional dependencies manually (system install). See [Linux GUI Support](#Linux-gui-support).

### Run Project

The excutable files are in the `CMakeBuild/<Debug/Release/RelWithDebInfo>/bin` folder. 

To run the programs, you may need to set the environment variable `LD_LIBRARY_PATH` to include the shared libraries in the `CMakeBuild/<Debug/Release/RelWithDebInfo>/bin` folder, otherwise the shared **libuipc** library and the dependent backend modules may not be found.

```shell
cd CMakeBuild/<Debug/Release/RelWithDebInfo>/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.
./hello_uipc
./uipc_test_world
[...]
```

### Build Pyuipc

Pyuipc is a Python binding of libuipc. It is built with the `pybind11` library.

```shell
sudo apt-get install python3-dev
```

Install packages with the following command:

```shell
pip install pybind11 mypy numpy
```

We use `mypy.stubgen` to generate the stub files for the Python binding.  The stub files will be generated in the `libuipc/python/typings` folder automatically after building `pyuipc`.

Add `-DUIPC_BUILD_PYBIND=1` to the CMake command to build the Python binding.

NOTE: `pyuipc` should be built in the **Release** or **RelWithDebInfo** mode.

(Optional) If you need pytorch support, you need to install the pytorch package first.

```shell
pip install torch
```

Then set `-DUIPC_BUILD_TORCH_EXTENSION=1` to the CMake command to enable the pytorch extension.

### Install Pyuipc

After building the project, install the Python binding with the following command:

```shell
cd libuipc/python; pip install .
```

Then you can use the `pyuipc` in your Python environment with:

```python
from pyuipc_loader import pyuipc
```

## Conda

If you are using conda, you can create a new environment with the following command:

```shell
conda create -n uipc_env python=3.10
```

Then activate the environment with:

```shell
conda activate uipc_env
```

### CMake
Install CMake-3.26 with the following command:

```shell
conda install cmake=3.26
```

### GCC

Install the tested version of gcc with the following command:

```shell
conda install gcc_linux-64=11.2.0
```

### Cuda

Install the tested version of cuda with the following command:

```shell
conda install nvidia/label/cuda-12.4.0::cuda-toolkit
```

Cuda-12.4.0 requires driver version >= 550.54.14 (https://docs.nvidia.com/deploy/cuda-compatibility/index.html#use-the-right-compat-package),check your nvidia driver version with the following command

```shell
nvidia-smi
```

### Then

Go back to the [Linux](#linux) or [Windows](#windows) build steps to continue building the project.

## Run Examples

Then you can run the examples in the `libuipc/python` folder.

- `uipc_info.py`: print the basic information of the `uipc` library. 
- `tutorial/`: contains the basic usage of `Pyuipc`.

You can run the `uipc_info.py` to check if the `Pyuipc` is installed correctly.

We use [polyscope](https://polyscope.run/)(v2.3.0) to visualize the scene in Python. If you want to run the GUI examples, you need to install the `polyscope` library with the following command:

```shell
pip install polyscope==2.3.0
```

## Build Document

Download and install doxygen https://www.doxygen.nl/download.html.

Install [mkdocs](https://www.mkdocs.org/) and its plugins:
```shell
pip install mkdocs mkdocs-material mkdocs-literate-nav mkdoxy mkdocs-video
```

Turn on the local server:
```shell
mkdocs serve
```

Open the browser and visit the [localhost:8000](http://127.0.0.1:8000/)

## Troubleshooting

### Linux GUI Support

If your system hasn't installed the GUI application dependencies before, you may need to install the following packages to enable GUI support.

```shell
sudo apt-get install libxi-dev libgl1-mesa-dev libglu1-mesa-dev mesa-common-dev libxrandr-dev libxxf86vm-dev libxinerama-dev libxcursor-dev
```

### Linux Python3 System Requirement

Python3 currently requires the following programs from the system package:

- autoconf
- automake
- autoconf-archive

```shell
sudo apt-get install autoconf automake autoconf-archive
```

### Submodule Manual Initialization & Update

If the submodule is not initialized, you can use the following command to initialize the submodule:

```shell
git submodule update --init
```