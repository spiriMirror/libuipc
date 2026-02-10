# Parameterized Dockerfile for libuipc development environment with XMake

ARG CUDA_VERSION=12.8.1
ARG UBUNTU_VERSION=24.04
ARG BUILD_PROJECT=true

FROM nvidia/cuda:${CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION}

# Set noninteractive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# =============================================================================
# Install Prerequisites (as per docs/build_install/xmake.md)
# =============================================================================
# Python >= 3.11, CUDA >= 12.4, XMake

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
    python3 \
    python3-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set gcc-13 as default (Ubuntu 24.04 default)
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100

# =============================================================================
# Create non-root user
# =============================================================================
# Create user with UID 1000
# This avoids privilege warnings when mounting volumes from host
# Using UID 1000 to match common default user IDs on Linux systems
# Remove existing user/group with UID/GID 1000 if they exist, then create developer user
RUN if getent passwd 1000 > /dev/null 2>&1; then \
        EXISTING_USER=$(getent passwd 1000 | cut -d: -f1); \
        userdel -r $EXISTING_USER 2>/dev/null || true; \
    fi && \
    if getent group 1000 > /dev/null 2>&1; then \
        EXISTING_GROUP=$(getent group 1000 | cut -d: -f1); \
        groupdel $EXISTING_GROUP 2>/dev/null || true; \
    fi && \
    groupadd -r developer -g 1000 && \
    useradd -r -u 1000 -g developer -m -s /bin/bash developer && \
    mkdir -p /home/developer && \
    chown -R developer:developer /home/developer

# =============================================================================
# Install XMake (includes xrepo) as developer user
# =============================================================================
# Install xmake using the official installation script
# XMake includes xrepo, so we don't need to install it separately
# Install as developer user so it's in their home directory
RUN --mount=type=cache,target=/tmp/.cache \
    runuser -u developer -- bash -c "curl -fsSL https://xmake.io/shget.text | bash" && \
    runuser -u developer -- bash -c "export PATH=\"\$HOME/.local/bin:\$PATH\" && xmake --version"

# Add xmake to PATH for developer user
# XMake installs to ~/.local/bin by default
RUN runuser -u developer -- bash -c "echo 'export PATH=\"\$HOME/.local/bin:\$PATH\"' >> ~/.bashrc" && \
    chown developer:developer /home/developer/.bashrc && \
    chmod 644 /home/developer/.bashrc

# Also add to system PATH for non-interactive shells
ENV PATH="/home/developer/.local/bin:${PATH}"

# Verify xmake and xrepo are available
RUN runuser -u developer -- bash -c "xmake --version && xrepo --version"

# =============================================================================
# Install uv (fast Python package manager) as developer user
# =============================================================================
# Install uv using the official installation script
RUN --mount=type=cache,target=/tmp/.cache \
    runuser -u developer -- bash -c "curl -LsSf https://astral.sh/uv/install.sh | sh" && \
    runuser -u developer -- bash -c "export PATH=\"\$HOME/.cargo/bin:\$PATH\" && uv --version"

# Add uv to PATH for developer user
RUN runuser -u developer -- bash -c "echo 'export PATH=\"\$HOME/.cargo/bin:\$PATH\"' >> ~/.bashrc" && \
    chown developer:developer /home/developer/.bashrc && \
    chmod 644 /home/developer/.bashrc

# Also add to system PATH for non-interactive shells
ENV PATH="/home/developer/.cargo/bin:${PATH}"

# Verify uv is available
RUN runuser -u developer -- bash -c "uv --version"

# =============================================================================
# Set up project directory
# =============================================================================
RUN mkdir -p /home/developer/libuipc && \
    chown -R developer:developer /home/developer/libuipc

WORKDIR /home/developer/libuipc

# =============================================================================
# Create pyuipc-env virtual environment and initialize with uv
# =============================================================================
# Create virtual environment named pyuipc-env in the project directory
RUN runuser -u developer -- bash -c "cd /home/developer/libuipc && \
    uv venv pyuipc-env && \
    echo 'Virtual environment pyuipc-env created successfully'"

# Initialize uv project in the python directory (if it exists and doesn't have pyproject.toml)
# This sets up the project structure for uv package management
RUN runuser -u developer -- bash -c "cd /home/developer/libuipc && \
    if [ -d 'python' ] && [ ! -f 'python/pyproject.toml' ]; then \
        cd python && \
        source ../pyuipc-env/bin/activate && \
        uv init --no-readme --no-package . || true; \
    fi"

# =============================================================================
# Build libuipc (optional, controlled by BUILD_PROJECT)
# =============================================================================
# Build as developer user
# XMake will handle dependency management via xrepo
RUN if [ "$BUILD_PROJECT" = "true" ]; then \
    echo "Building libuipc with xmake..." && \
    runuser -u developer -- bash -c "cd /home/developer/libuipc && \
        xmake config --yes && \
        xmake build --all -j\$(nproc) && \
        echo 'Build completed successfully!'"; \
    fi

# Set library path for runtime
ENV LD_LIBRARY_PATH="/home/developer/libuipc/build/linux/x86_64/release:${LD_LIBRARY_PATH}"

WORKDIR /home/developer/libuipc

# =============================================================================
# Entrypoint script - simple wrapper for login shell
# =============================================================================
# Simple entrypoint that just ensures we use a login shell
# Copy entrypoint script (must be done before switching to developer user)
COPY artifacts/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Run as developer user (non-root approach)
USER developer

ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]
