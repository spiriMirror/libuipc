# XMake

## Prerequisites

The following dependencies are required to build the project.

| Name                                                | Version      | Usage           | Import         |
| --------------------------------------------------- | ------------ | --------------- | -------------- |
| [Python](https://www.python.org/downloads/)         | >=3.11       | build system    | system install |
| [Cuda](https://developer.nvidia.com/cuda-downloads) | >=12.4       | GPU programming | system install |

## Build Project

Clone the repository with the following command:

```shell
git clone https://github.com/spiriMirror/libuipc.git
```

Then, you can use the following commands to build the project.

```shell
cd libuipc
xmake f -c
```

## Troubleshooting

### out of memory

Xmake uses a lot of process for parallel compilation in order to accelerate the compilation task. However, nvcc will consume a lot of memory, thus will eventually cause an OOM

✅ Set the multi-process manurally, e.g. `xmake -j8` to set 8 parallel compilation jobs


## dev in uv

Use [uv](https://docs.astral.sh/uv/) to manage a virtual environment for Python development. The following steps configure xmake with Python binding support, create a virtual environment, build the project, and package the Python wheel.

```shell
uv venv --python 3.11
source venv/bin/activate
xmake f --pybind=true --python_system=true --python_version=3.12.x -c
xmake build -j8
xmake pack -v
```
