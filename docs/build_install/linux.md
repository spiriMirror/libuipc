# Build on Linux

## Install Vcpkg

If you haven't installed Vcpkg, you can clone the repository with the following command:

```shell
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
```
The simplest way to let CMake detect Vcpkg is to set the system environment variable `CMAKE_TOOLCHAIN_FILE` to `(YOUR_VCPKG_PARENT_FOLDER)/vcpkg/scripts/buildsystems/vcpkg.cmake`

We recommend using conda environments to build the project on Linux. 

## Conda Environment

```shell
conda create -n uipc_env python=3.10
conda activate uipc_env
conda install cmake=3.26
conda install nvidia/label/cuda-12.4.0::cuda-toolkit
```

Cuda-12.4.0 requires driver version >= 550.54.14 (https://docs.nvidia.com/deploy/cuda-compatibility/index.html#use-the-right-compat-package), check your nvidia driver version with the following command

```shell
nvidia-smi
```

## Other Environment

If you don't want to use conda, you can manually install `CMake 3.26`, `GCC 11.4`, `Cuda 12.4` and `Python >=3.10` with your favorite package manager.

## Build Libuipc

You can use the following commands to build the project.

```shell
cd libuipc; cd ..; mkdir CMakeBuild; cd CMakeBuild
cmake -S ../libuipc -DUIPC_BUILD_PYBIND=1 -DCMAKE_BUILD_TYPE=<Debug/Release/RelWithDebInfo> 
cmake --build . -j8
```

!!!NOTE
    Use multi-thread to speed up the build process as possible, becasue the NVCC compiler will take a lot of time.

## Run Project

The excutable files are in the `CMakeBuild/<Debug/Release/RelWithDebInfo>/bin` folder. 

```shell
cd CMakeBuild/<Debug/Release/RelWithDebInfo>/bin
./hello_uipc
./uipc_sim_case
[...]
```

## Install Pyuipc

After building the project, install the Python binding with the following command:

```shell
cd libuipc/python
pip install .
```

## Check Installation

You can run the `uipc_info.py` to check if the `Pyuipc` is installed correctly.

```shell
cd libuipc/python
python uipc_info.py
```

More samples are at [Pyuipc Samples](https://github.com/spiriMirror/libuipc-samples).