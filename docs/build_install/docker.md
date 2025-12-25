# Build with Docker

## Prerequisites

Install Docker and Docker Compose:

- [Linux Docker](https://docs.docker.com/engine/install/ubuntu/)
- [Windows Docker Desktop](https://docs.docker.com/desktop/install/windows-install/)

**Note for Windows users:** For better performance, especially when mounting volumes, it's recommended to use WSL2 (Windows Subsystem for Linux 2). Docker Desktop on Windows can use WSL2 as the backend, which provides significantly faster filesystem performance compared to the Windows filesystem. See [Docker Desktop WSL 2 backend](https://docs.docker.com/desktop/wsl/) for setup instructions.

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

**Note:** The container runs as a non-root user (`developer`) to avoid privilege warnings. The user has UID 1000 and GID 1000.

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

If the build cache is not working properly, you can try to remove the cache with the following command:

```shell
docker builder prune -af
```

**Slow filesystem performance on Windows:**

If you experience slow build times or file operations when using Docker on Windows, this is likely due to filesystem performance issues when mounting Windows directories. To resolve this:

1. **Use WSL2:** Ensure Docker Desktop is configured to use WSL2 backend (Settings → General → Use the WSL 2 based engine)
2. **Clone repository in WSL2:** Clone the repository inside your WSL2 filesystem (e.g., `/home/username/Projects/libuipc`) rather than on the Windows filesystem (e.g., `C:\Users\...`)
3. **Mount from WSL2:** When running docker commands, use the WSL2 path (e.g., `/home/username/Projects/libuipc`) instead of Windows paths (e.g., `/mnt/c/Users/...`)

This will provide native Linux filesystem performance and significantly improve build times.
