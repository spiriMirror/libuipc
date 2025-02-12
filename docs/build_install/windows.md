# Build on Windows

## Install Vcpkg

If you haven't installed Vcpkg, you can clone the repository with the following command:

```shell
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.bat
```

The simplest way to let CMake detect Vcpkg is to set the system environment variable `CMAKE_TOOLCHAIN_FILE` to `(YOUR_VCPKG_PARENT_FOLDER)/vcpkg/scripts/buildsystems/vcpkg.cmake`

## Build Libuipc

### CMake-GUI

On Windows, you can use the `CMake-GUI` to configure the project and generate the Visual Studio solution file with only a few clicks.

- Toggling the `UIPC_BUILD_PYBIND` option to `ON` to enable the Python binding.

### CMake-CLI

Or, you can use the following commands to build the project.

```shell
cd libuipc; cd ..; mkdir CMakeBuild; cd CMakeBuild
cmake -S ../libuipc -DUIPC_BUILD_PYBIND=1 
cmake --build . --config <Debug/Release/RelWithDebInfo> -j8
```

!!!NOTE
    Use multi-thread to speed up the build process as possible, becasue the NVCC compiler will take a lot of time.


## Run Project

Just run the executable files in `CMakeBuild/<Debug/Release/RelWithDebInfo>/bin` folder.

## Install Pyuipc in System Environment

After building the project, install the Python binding with the following command:

```shell
cd libuipc/python
pip install .
```

## Install Pyuipc in Conda Environment (Optional)

If you want to install the Python binding in a Conda environment, you should additionally specify the Python executable path of the Conda environment.

```shell
conda create -n uipc_env python=3.10
```

!!!NOTE
    **Don't** activate the Conda environment between the compilation completion.
    On Windows it's hard to compile with MSVC in conda environment, so we build the C++ part in the system environment and install the Python binding in the conda environment.

```shell
cmake -S ../libuipc -DUIPC_BUILD_PYBIND=1 -DUIPC_PYTHON_EXECUTABLE_PATH=<PYTHON_EXE_IN_CONDA_ENV>
cmake --build . --config <Release/RelWithDebInfo> -j8
```
For example, the <PYTHON_EXE_IN_CONDA_ENV> may be  `C:\Users\<UserName>\anaconda3\envs\uipc_env\python.exe`

Then install the Python binding in the Conda environment:

```shell
conda activate uipc_env
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




