# Base Dockerfile for colcon-deb build environment
# Supports ROS loong on Ubuntu 22.04 (Jammy)
ARG BASE_IMAGE=ubuntu:jammy
FROM ${BASE_IMAGE}

# Use Agiros ROS Distro Index
ENV ROSDISTRO_INDEX_URL=http://1.94.193.239/yumrepo/agiros/agirosdep/index-v4.yaml

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV RUST_BACKTRACE=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    git \
    build-essential \
    cmake \
    pkg-config \
    debhelper \
    dpkg-dev \
    devscripts \
    lintian \
    fakeroot \
    dh-make \
    python3-pip \
    python3-apt \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    gnupg \
    lsb-release \
    sudo \
    vim \
    tzdata \
    liblttng-ust-dev \
    && ln -fs /usr/share/zoneinfo/Asia/Shanghai /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

RUN pip install -i https://pypi.tuna.tsinghua.edu.cn/simple colcon-common-extensions

RUN pip install http://1.94.193.239/yumrepo/agiros/agirosdep/bloom-0.13.0-py3-none-any.whl
RUN pip install http://1.94.193.239/yumrepo/agiros/agirosdep/agirosdep-0.25.1-py3-none-any.whl

# Install Rust (latest stable)
ENV RUSTUP_DIST_SERVER=https://mirrors.tuna.tsinghua.edu.cn/rustup \
    RUSTUP_UPDATE_ROOT=https://mirrors.tuna.tsinghua.edu.cn/rustup/rustup
RUN curl --proto '=https' --tlsv1.2 -sSf https://rsproxy.cn/rustup-init.sh | sh -s -- -y --default-toolchain stable
ENV PATH="/root/.cargo/bin:${PATH}"

# Configure Cargo to use rsproxy sparse registry
RUN mkdir -p /root/.cargo && \
    echo '[source.crates-io]' > /root/.cargo/config.toml && \
    echo 'replace-with = "rsproxy-sparse"' >> /root/.cargo/config.toml && \
    echo '[source.rsproxy]' >> /root/.cargo/config.toml && \
    echo 'registry = "https://rsproxy.cn/crates.io-index"' >> /root/.cargo/config.toml && \
    echo '[source.rsproxy-sparse]' >> /root/.cargo/config.toml && \
    echo 'registry = "sparse+https://rsproxy.cn/index/"' >> /root/.cargo/config.toml && \
    echo '[registries.rsproxy]' >> /root/.cargo/config.toml && \
    echo 'index = "https://rsproxy.cn/crates.io-index"' >> /root/.cargo/config.toml && \
    echo '[net]' >> /root/.cargo/config.toml && \
    echo 'git-fetch-with-cli = true' >> /root/.cargo/config.toml

# Install rust-script
RUN cargo install rust-script --version 0.35.0

# Install additional cargo tools
RUN cargo install cargo-edit cargo-watch

# Create a non-root user for builds (optional, can be overridden)
ARG USERNAME=builder
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && mkdir -p /home/$USERNAME/.cargo \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME

# Set up agirosdep
RUN agirosdep init || true

# Create workspace directory
RUN mkdir -p /workspace && chown $USERNAME:$USERNAME /workspace

# Copy helper scripts
COPY --chown=1000:1000 scripts/main.sh /scripts/
COPY --chown=1000:1000 scripts/helpers /helpers/
# Set up entrypoint
COPY scripts/entrypoint.sh /usr/local/bin/
# Ensure root for linking and permissions
USER root
RUN chmod +x /usr/local/bin/entrypoint.sh
RUN ln -s /usr/local/bin/entrypoint.sh /entrypoint.sh

# Default to non-root user
USER $USERNAME
WORKDIR /workspace

# Update agirosdep as user
RUN agirosdep update

# Switch back to root for entrypoint
USER root

# Labels
LABEL maintainer="colcon-deb"
LABEL description="Build environment for colcon-deb with ROS loong"
LABEL version="1.0"

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]