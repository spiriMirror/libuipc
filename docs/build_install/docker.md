# Build with Docker

## Prerequisites

Install Docker and Docker Compose:

- [Linux Docker](https://docs.docker.com/engine/install/ubuntu/)
- [Windows Docker Desktop](https://docs.docker.com/desktop/install/windows-install/)

## Available Services

| Service           | Description                             |
| ----------------- | --------------------------------------- |
| `dev-empty-conda` | Empty development environment           |
| `dev-cmake`       | CMake development environment           |
| `dev-cmake-cu128` | CMake CUDA 12.8 development environment |
| `dev-cmake-cu130` | CMake CUDA 13.0 development environment |
| `dev-xmake-cu128` | XMake CUDA 12.8 development environment |
| `dev-xmake-cu130` | XMake CUDA 13.0 development environment |

## CMake

```shell
cd libuipc/artifacts/docker
docker compose up -d dev-cmake-cu128
docker compose exec dev-cmake-cu128 bash
```

Build:

```shell
mkdir build; cd build
cmake -S .. -DUIPC_BUILD_PYBIND=1 -DCMAKE_BUILD_TYPE=Release
cmake --build . -j$(nproc)
```

## XMake

```shell
cd libuipc/artifacts/docker
docker compose up -d dev-xmake-cu128
docker compose exec dev-xmake-cu128 bash
```

Build:

```shell
xmake config --pybind=y
xmake -j$(nproc)
```

## Useful Commands

```shell
# Start container
docker compose up -d <service-name>

# Enter container
docker compose exec <service-name> bash

# Stop container
docker compose stop <service-name>

# Remove containers
docker compose down
```
