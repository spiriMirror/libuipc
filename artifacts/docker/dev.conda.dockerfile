# Dockerfile for testing conda behavior with libuipc
# Build with: docker build -t libuipc-dev:conda -f artifacts/docker/dev.conda.dockerfile .

FROM ubuntu:24.04

# Set noninteractive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install minimal dependencies for conda
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    ca-certificates \
    bzip2 \
    && rm -rf /var/lib/apt/lists/*

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

# Accept conda Terms of Service and create environment as developer user
RUN su developer -c "source /home/developer/conda/etc/profile.d/conda.sh && \
    conda config --set always_yes yes && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r && \
    conda create -n uipc_env"

# =============================================================================
# Initialize conda and auto-activate uipc_env
# =============================================================================
RUN chmod 755 /home/developer && \
    su developer -c "source /home/developer/conda/etc/profile.d/conda.sh && conda init bash" && \
    su developer -c "echo 'conda activate uipc_env' >> /home/developer/.bashrc" && \
    chown developer:developer /home/developer/.bashrc && \
    chmod 644 /home/developer/.bashrc

# Switch to non-root user
USER developer

WORKDIR /home/developer

CMD ["bash"]

