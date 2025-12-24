# Build with Docker

## Prerequisites

Install Docker and Docker Compose:

- [Linux Docker](https://docs.docker.com/engine/install/ubuntu/)
- [Windows Docker Desktop](https://docs.docker.com/desktop/install/windows-install/)

## Quick Start

Clone the repository with the following command:

```shell
git clone https://github.com/spiriMirror/libuipc.git
```

Navigate to the `docker` directory:

```shell
cd libuipc/artifacts/docker
```

Build and start the container:

```shell
docker compose up -d dev-cmake 
docker compose exec dev-cmake bash
```
Build the project inside the container:

**Note:** The container runs as a non-root user (`developer`) to avoid privilege warnings. The user has UID 1001 and GID 1001.

```shell
# /home/developer/libuipc
mkdir build; cd build
cmake -S .. -DUIPC_BUILD_PYBIND=1 -DCMAKE_BUILD_TYPE=<Release/RelWithDebInfo> 
cmake --build . -j$(nproc)
```

Then you can follow the other instructions in [Linux Installation Guide](linux.md) to explore more features.

## Development

If you want to develop the project inside the container, you can attach your IDE to the container.

- [VSCode Dev Container](https://code.visualstudio.com/docs/devcontainers/containers)
- [CLion Dev Container](https://www.jetbrains.com/help/clion/connect-to-devcontainer.html)


## Available Services

| Service           | Description                             |
| ----------------- | --------------------------------------- |
| `dev-empty-conda` | Empty development environment           |
| `dev-cmake`       | CMake development environment           |
| `dev-cmake-cu128` | CMake CUDA 12.8 development environment |
| `dev-cmake-cu130` | CMake CUDA 13.0 development environment |

## Useful Commands

```shell
# Build and start
docker compose up -d dev-cmake
# Create a new shell from the container
docker compose exec dev-cmake bash
# Stop
docker compose stop dev-cmake
# Remove
docker compose down
```

## Troubleshooting

**Build cache issues:**

If the build cache is not working, you can try to remove the cache with the following command:

```shell
docker builder prune -af
```
