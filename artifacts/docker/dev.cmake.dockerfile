# Parameterized Dockerfile for libuipc development environment

ARG CUDA_VERSION=12.8.1
ARG UBUNTU_VERSION=24.04
ARG CMAKE_CUDA_ARCHITECTURES=89
ARG CMAKE_BUILD_TYPE=Release

FROM nvidia/cuda:${CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION}

# Set noninteractive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# =============================================================================
# Install Prerequisites (as per docs/build_install/linux.md)
# =============================================================================
# CMake >= 3.26, Python >= 3.11, CUDA >= 12.4, Vcpkg >= 2025.7.25

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    wget \
    curl \
    build-essential \
    gcc-13 g++-13 \
    pkg-config \
    zip \
    unzip \
    ca-certificates \
    bzip2 \
    && rm -rf /var/lib/apt/lists/*

# Set gcc-13 as default (Ubuntu 24.04 default)
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100

# =============================================================================
# Create non-root user
# =============================================================================
# Create user with UID 1001
# This avoids privilege warnings when mounting volumes from host
RUN groupadd -r developer -g 1001 && \
    useradd -r -u 1001 -g developer -m -s /bin/bash developer && \
    mkdir -p /home/developer && \
    chown -R developer:developer /home/developer

# =============================================================================
# Install Miniconda as developer user
# =============================================================================
# Cache the Miniconda installer download using BuildKit cache mount
# Install to /home/developer/conda instead of /opt/conda
RUN --mount=type=cache,target=/tmp/.cache/wget \
    INSTALLER="/tmp/.cache/wget/Miniconda3-latest-Linux-x86_64.sh" && \
    if [ ! -f "$INSTALLER" ]; then \
        wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O "$INSTALLER"; \
    fi && \
    chmod +x "$INSTALLER" && \
    su developer -c "bash \"$INSTALLER\" -b -p /home/developer/conda" && \
    su developer -c "/home/developer/conda/bin/conda clean -afy"

ENV PATH="/home/developer/conda/bin:${PATH}"

# =============================================================================
# Install Vcpkg as developer user (as per docs: Vcpkg >= 2025.7.25)
# =============================================================================
# Following the installation method from docs/build_install/linux.md
# Install to /home/developer/Toolchain/vcpkg
# Use BuildKit cache mount to cache vcpkg built packages
RUN --mount=type=cache,target=/home/developer/.cache/vcpkg \
    mkdir -p /home/developer/Toolchain && \
    chown developer:developer /home/developer/Toolchain && \
    su developer -c "cd /home/developer/Toolchain && \
        git clone https://github.com/microsoft/vcpkg.git && \
        cd vcpkg && \
        bash bootstrap-vcpkg.sh"

# Verify vcpkg installation
RUN su developer -c "/home/developer/Toolchain/vcpkg/vcpkg version"

# Set up vcpkg cache directories (will be mounted as cache in build step)
# Downloads, buildtrees, and packages are cached to speed up builds
RUN su developer -c "mkdir -p /home/developer/Toolchain/vcpkg/downloads /home/developer/Toolchain/vcpkg/buildtrees /home/developer/Toolchain/vcpkg/packages"

# =============================================================================
# Set up project directory
# =============================================================================
RUN mkdir -p /home/developer/libuipc && \
    chown -R developer:developer /home/developer/libuipc

WORKDIR /home/developer/libuipc

# =============================================================================
# Create Conda Environment (as per docs/build_install/linux.md)
# =============================================================================
# Install conda packages directly instead of using env.yaml files
# This matches the dependencies from conda/env.yaml and conda/env-cu124.yaml
# All operations run as developer user

# Accept conda Terms of Service (required for non-interactive builds)
RUN su developer -c "source /home/developer/conda/etc/profile.d/conda.sh && \
    conda config --set always_yes yes && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r"

# Create conda environment and install packages
# Select cuda-toolkit version based on CUDA_VERSION
# Using BuildKit cache mount to cache conda packages (requires DOCKER_BUILDKIT=1)
RUN --mount=type=cache,target=/home/developer/conda/pkgs \
    CUDA_TOOLKIT_VERSION=${CUDA_VERSION} && \
    echo "Creating conda environment with CUDA toolkit ${CUDA_TOOLKIT_VERSION}..." && \
    su developer -c "source /home/developer/conda/etc/profile.d/conda.sh && \
        conda create -n uipc_env -y \
            python=3.11 \
            cmake=3.26 \
            cuda-toolkit=${CUDA_TOOLKIT_VERSION} && \
        conda install -n uipc_env -y -c conda-forge pkgconfig && \
        conda clean -y --tarballs"

# Set CMAKE_TOOLCHAIN_FILE in conda environment dont need to set it in bashrc
ARG CMAKE_TOOLCHAIN_FILE="/home/developer/Toolchain/vcpkg/scripts/buildsystems/vcpkg.cmake"
# This is important so CMake can find the Vcpkg toolchain file
RUN su developer -c "source /home/developer/conda/etc/profile.d/conda.sh && \
    conda activate uipc_env && \
    conda env config vars set CMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}"

# =============================================================================
# Build libuipc (following docs/build_install/linux.md)
# =============================================================================
ARG BUILD_PROJECT=true
# Use cache mounts for vcpkg downloads, buildtrees, and packages
# Build as developer user
RUN --mount=type=cache,target=/home/developer/Toolchain/vcpkg/downloads \
    --mount=type=cache,target=/home/developer/Toolchain/vcpkg/buildtrees \
    --mount=type=cache,target=/home/developer/Toolchain/vcpkg/packages \
    if [ "$BUILD_PROJECT" = "true" ]; then \
    echo "Building libuipc with CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}..." && \
    su developer -c "source /home/developer/conda/etc/profile.d/conda.sh && \
        conda activate uipc_env && \
        export CMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE} && \
        cd /home/developer/libuipc && \
        mkdir -p build && cd build && \
        cmake -S .. \
            -DUIPC_BUILD_PYBIND=1 \
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
            -DCMAKE_CUDA_ARCHITECTURES=${CMAKE_CUDA_ARCHITECTURES} && \
        cmake --build . -j\$(nproc) && \
        cd /home/developer/libuipc && \
        echo 'Build completed successfully!'"; \
    fi

# Set library path for runtime (adjust based on build type)
# Note: CMAKE_BUILD_TYPE is an ARG, so we'll set this in the entrypoint or use a default
ENV LD_LIBRARY_PATH="/home/developer/libuipc/build/Release/bin:/home/developer/libuipc/build/RelWithDebInfo/bin:/home/developer/libuipc/build/Debug/bin:${LD_LIBRARY_PATH}"

# =============================================================================
# Configure Git for Windows volume mounts (build time)
# =============================================================================
# Set Git config globally to handle Windows/Linux differences
# This applies to any Git repository mounted at runtime
RUN su developer -c "git config --global core.fileMode false && \
    git config --global core.autocrlf input && \
    git config --global core.eol lf && \
    git config --global init.defaultBranch main && \
    git config --global --add safe.directory /home/developer/libuipc || true"

# Create .bashrc to ensure conda is activated in interactive shells
# Ensure home directory and .bashrc have correct permissions
RUN chmod 755 /home/developer && \
    su developer -c "echo 'source /home/developer/conda/etc/profile.d/conda.sh && \
    conda activate uipc_env' > /home/developer/.bashrc" && \
    chown developer:developer /home/developer/.bashrc && \
    chmod 644 /home/developer/.bashrc

WORKDIR /home/developer/libuipc

# Entrypoint script - make it robust to handle permission issues
RUN echo '#!/bin/bash\n\
if [ -f /home/developer/.bashrc ] && [ -r /home/developer/.bashrc ]; then\n\
    source /home/developer/.bashrc\n\
fi\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh && \
    chown developer:developer /entrypoint.sh

# Switch to non-root user
USER developer

ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]
