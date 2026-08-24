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

The default build includes the CUDA backend, examples, and tests. Optional
modules are explicit and disabled by default:

```shell
# Enable either or both optional libraries.
xmake f --usd=true --vdb=true -c

# CPU/interface-only configuration (the none backend still builds).
xmake f --backend_cuda=false -c
```

`--usd=true` resolves OpenUSD 25.08 or newer, and `--vdb=true` resolves a shared
OpenVDB package. XMake's compiler cache is disabled by project policy; use normal
incremental builds rather than enabling ccache externally.

## Troubleshooting

### out of memory

Xmake uses a lot of process for parallel compilation in order to accelerate the compilation task. However, nvcc will consume a lot of memory, thus will eventually cause an OOM

Set the multi-process count manually, for example `xmake -j8` for eight parallel
compilation jobs.


## dev in uv

Use [uv](https://docs.astral.sh/uv/) to manage a virtual environment for Python development. The following steps configure xmake with Python binding support, create a virtual environment, build the project, and package the Python wheel.

```shell
uv venv --python 3.11
source venv/bin/activate
xmake f --pybind=true --python_system=true --python_version=3.12.x -c
xmake build -j8
xmake pack -v
```
