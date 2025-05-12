FROM nvidia/cuda:12.4.0-devel-ubuntu22.04

# Set noninteractive mode for apt
ENV DEBIAN_FRONTEND=noninteractive
ARG CMAKE_CUDA_ARCHITECTURES=89

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    wget \
    curl \
    build-essential \
    gcc-11 g++-11 \
    python3.11 python3.11-venv python3.11-dev \
    pip \
    pkg-config \
    zip \
    unzip \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Install pip and upgrade
RUN python3.11 -m pip install --upgrade pip && \
    ln -s /usr/bin/python3.11 /usr/bin/python


COPY . /workspace

# Install cmake 3.26
RUN bash /workspace/artifacts/docker/installers/install_cmake.sh
ENV PATH="/opt/cmake/bin:${PATH}"

# Install vcpkg
RUN bash /workspace/artifacts/docker/installers/install_vcpkg.sh

# Set environment variable for CMake to find vcpkg
ENV VCPKG_ROOT=/opt/vcpkg
ENV CMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake

WORKDIR /workspace

RUN mkdir build && cd build && \
    cmake -S .. -DUIPC_BUILD_PYBIND=1 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=${CMAKE_CUDA_ARCHITECTURES}&& \
    cmake --build . -j$(nproc) && \
    cd python && \
    pip install .

ENV LD_LIBRARY_PATH="/workspace/build/Release/bin:${LD_LIBRARY_PATH}"

# Set workdir
WORKDIR /workspace

# Default command
CMD ["bash"]
