# libuipc
A Modern C++20 Library of Unified Incremental Potential Contact.

## Dependencies

The following dependencies are required to build the project.

| Name                                          | Version      | Usage           | Import         |
| --------------------------------------------- | ------------ | --------------- | -------------- |
| [CMake](https://cmake.org/download/)          | >=3.27       | build system    | system install |
| [Python](https://www.python.org/downloads/)   | >=3.10       | build system    | system install |
| [Vcpkg](https://github.com/microsoft/vcpkg)   | >=2024.04.26 | package manager | git clone      |

The following are **libuipc**'s 3rd-party dependencies. Don't worry, most of them will be automatically installed by Vcpkg.

| Name                                   | Version | Usage                                               | Import         |
| -------------------------------------- | ------- | --------------------------------------------------- | -------------- |
| [muda](https://github.com/MuGdxy/muda) | -       | improve safety and readability of CUDA programming. | submodule      |
| cuda                                   | >=12.0  | GPU programming                                     | system install |
| eigen3                                 | 3.4.0   | matrix calculation                                  | package        |
| catch2                                 | 3.5.3   | unit tests                                          | package        |
| libigl                                 | 2.5.0   | mesh processing                                     | package        |
| rapidcsv                               | 8.80    | csv file IO                                         | package        |
| spdlog                                 | 1.12.0  | logging                                             | package        |
| fmt                                    | 10.1.1  | fast string formatting                              | package        |
| cppitertools                           | 2.1#3   | python-like iteration tools                         | package        |
| bgfx                                   | 1.127#1 | cross-platform RHI                                  | package        |
| dylib                                  | 2.2.1   | cross-platform dynamic library loader               | package        |
| benchmark                              | 1.8.3#3 | microbenchmark support library                      | package        |
| nlohmann_json                          | 3.11.2  | json file IO                                        | package        |
| imgui                                  | 1.90.7  | GUI                                                 | package        |
| glfw3                                  | 3.3.8#2 | window management                                   | package        |
| magic_enum                             | 0.9.3   | enum to string                                      | package        |

## Build
We use Vcpkg to manage the libraries and use CMake to build the project. 

The simplest way to let CMake detect Vcpkg is to set the system environment variable `CMAKE_TOOLCHAIN_FILE` to `(YOUR_VCPKG_PARENT_FOLDER)/vcpkg/scripts/buildsystems/vcpkg.cmake`

Vcpkg supports both Windows and Linux; we use it to manage the dependencies and keep the consistency of the development environment.

### Submodules

Some dependencies are managed by git submodules. You need to clone the submodules with the following command:

```shell
git submodule update --init
```

### Automatic Dependency Installation

The rest dependencies are all managed by Vcpkg; they will be automatically installed in the CMake Configuration step.

If the automatic installation fails, please raise an issue with the CMake error message.

### Build Project

#### Windows

On Windows, you can use the `CMake-GUI` to configure the project and generate the Visual Studio solution file with only a few clicks.

You can also use the same commands as Linux to build the project.

#### Linux

On Linux, you can use the following commands to build the project.

```shell
cd libuipc; cd ..; mkdir CMakeBuild; cd CMakeBuild
cmake -S ../libuipc -DUIPC_BUILD_GUI=0
cmake --build .
```

To enable GUI support, set `-DUIPC_BUILD_GUI=1`, but you may need to install some additional dependencies manually (system install). See [Linux GUI Support](#Linux-gui-support).

### Run Project

#### Windows

Just run the executable files in `CMakeBuild/<Debug/Release/RelWithDebInfo>/bin` folder.

#### Linux

Install the project.

```shell
cmake --install . --prefix . --config <Debug/Release/RelWithDebInfo>
```

To run the programs, you need to set the environment variable `LD_LIBRARY_PATH` to include the shared libraries in the `CMakeBuild/<Debug/Release/RelWithDebInfo>/bin` folder, otherwise the shared **libuipc** library and the dependent backend modules will not be found.

```shell
cd CMakeBuild/<Debug/Release/RelWithDebInfo>/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.
./hello_uipc
./uipc_test_world
[...]
```

### Build Pyuipc

Pyuipc is a Python binding of libuipc. It is built with the `pybind11` library.

Add `-DUIPC_BUILD_PYBIND=1` to the CMake command to build the Python binding.

NOTE: Pyuipc should be built in the Release or RelWithDebInfo mode.

#### Run Examples

We use `mypy.stubgen` to generate the stub files for the Python binding. So you need to install `mypy` first.

```shell
pip install mypy
```

Then you can run the examples in the `libuipc/python` folder.

Intellisence is supported via the stub files in `libuipc/python/typings/`.

If you use VSCode, you can open the project at `libuipc/python` and enjoy the Intellisence with Pylance Extension.
```shell
cd python & code .
```

### Build Document

Install [mkdocs](https://www.mkdocs.org/) and its plugins:
```shell
pip install mkdocs mkdocs-material mkdocs-literate-nav mkdoxy
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
