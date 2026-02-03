#!/bin/bash
# Container entry point script - runs as root to set up user and permissions
# Then drops privileges and runs main.sh

set -e

# Get host UID/GID from environment or use defaults
HOST_UID=${HOST_UID:-1000}
HOST_GID=${HOST_GID:-1000}
USERNAME="builder"

# Progress reporting
log() {
    echo "::log::level=$1,msg=$2" >&2
}

log "info" "Container entrypoint starting"

# Create group and user matching host UID/GID
if ! getent group $HOST_GID > /dev/null 2>&1; then
    log "info" "Creating group with GID $HOST_GID"
    groupadd -g $HOST_GID $USERNAME
else
    # Group exists, get its name
    USERNAME=$(getent group $HOST_GID | cut -d: -f1)
    log "info" "Using existing group: $USERNAME"
fi

if ! getent passwd $HOST_UID > /dev/null 2>&1; then
    log "info" "Creating user $USERNAME with UID $HOST_UID"
    useradd -u $HOST_UID -g $HOST_GID -m -s /bin/bash $USERNAME
else
    log "info" "User with UID $HOST_UID already exists"
fi

# Set up passwordless sudo for required commands
log "info" "Configuring sudo permissions"
cat > /etc/sudoers.d/$USERNAME << EOF
$USERNAME ALL=(ALL) NOPASSWD: /usr/bin/apt, /usr/bin/apt-get, /usr/bin/agirosdep, /usr/bin/apt-cache
EOF
chmod 0440 /etc/sudoers.d/$USERNAME

# Fix workspace permissions
log "info" "Fixing workspace permissions"
chown -R $HOST_UID:$HOST_GID /workspace || true

# Create directories needed for the build
mkdir -p /workspace/build /workspace/install /workspace/log /workspace/output
chown -R $HOST_UID:$HOST_GID /workspace/build /workspace/install /workspace/log /workspace/output

# Ensure helper scripts are executable
if [ -d /helpers ]; then
    log "info" "Making helper scripts executable"
    chmod +x /helpers/*.rs 2>/dev/null || true
fi

# Set up environment for non-root user
cat > /home/$USERNAME/.bashrc << 'EOF'
# Set default ROS distro
export ROS_DISTRO=${ROS_DISTRO:-loong}

# Source ROS environment if available
if [ -f /opt/agiros/$ROS_DISTRO/setup.bash ]; then
    source /opt/agiros/$ROS_DISTRO/setup.bash
fi

# Source workspace if built
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# Add helpers to PATH
export PATH="/helpers:$PATH"

# Set build environment variables
export PARALLEL_JOBS=${PARALLEL_JOBS:-4}
export BUILD_TYPE=${BUILD_TYPE:-Release}
export ROS_DISTRO=${ROS_DISTRO:-loong}

# Colcon build settings
export COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification

# Set locale
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
EOF

chown $HOST_UID:$HOST_GID /home/$USERNAME/.bashrc

# Check if rust-script is available
if command -v rust-script >/dev/null 2>&1; then
    log "info" "rust-script is available"
else
    log "warning" "rust-script not found - Rust helpers may not work"
fi

# Drop privileges and run main script or provided command
if [ "$#" -gt 0 ]; then
    log "info" "Switching to user $USERNAME and executing command: $*"
    exec su - $USERNAME -c "$*"
else
    log "info" "Switching to user $USERNAME and running main script"
    exec su - $USERNAME -c "/scripts/main.sh"
fi