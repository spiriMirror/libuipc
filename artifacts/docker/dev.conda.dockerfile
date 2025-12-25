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
    runuser -u developer -- bash "$INSTALLER" -b -p /home/developer/conda && \
    runuser -u developer -- /home/developer/conda/bin/conda clean -afy

ENV PATH="/home/developer/conda/bin:${PATH}"

# Accept conda Terms of Service and create environment as developer user
RUN runuser -u developer -- bash -c "source /home/developer/conda/etc/profile.d/conda.sh && \
    conda config --set always_yes yes && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r && \
    conda create -n uipc_env"

# =============================================================================
# Initialize conda and auto-activate uipc_env
# =============================================================================
RUN chmod 755 /home/developer && \
    runuser -u developer -- bash -c "source /home/developer/conda/etc/profile.d/conda.sh && conda init bash" && \
    runuser -u developer -- bash -c "echo 'conda activate uipc_env' >> /home/developer/.bashrc" && \
    chown developer:developer /home/developer/.bashrc && \
    chmod 644 /home/developer/.bashrc

# =============================================================================
# Set up project directory
# =============================================================================
RUN mkdir -p /home/developer/libuipc && \
    chown -R developer:developer /home/developer/libuipc

# =============================================================================
# Entrypoint script - simple wrapper for login shell
# =============================================================================
# Simple entrypoint that just ensures we use a login shell for conda initialization
# Copy entrypoint script (must be done before switching to developer user)
COPY artifacts/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Run as developer user (non-root approach)
USER developer

WORKDIR /home/developer/libuipc

ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]