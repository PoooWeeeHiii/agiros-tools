#!/bin/bash
# Setup build environment for colcon-deb

set -e

echo "Setting up colcon-deb build environment..."

# Ensure cargo/rust binaries are in PATH
export PATH="$HOME/.cargo/bin:/root/.cargo/bin:$PATH"

# Set up colcon defaults
export COLCON_EXTENSION_BLOCKLIST=colcon_core.event_handler.desktop_notification

# Configure dpkg for non-interactive mode
export DEBIAN_FRONTEND=noninteractive

# Set architecture if provided
if [ -n "$TARGET_ARCHITECTURE" ]; then
    echo "Target architecture: $TARGET_ARCHITECTURE"
    export DEB_BUILD_ARCH="$TARGET_ARCHITECTURE"
    
    # Configure cross-compilation if needed
    if [ "$TARGET_ARCHITECTURE" != "$(dpkg --print-architecture)" ]; then
        echo "Setting up cross-compilation for $TARGET_ARCHITECTURE"
        dpkg --add-architecture "$TARGET_ARCHITECTURE"
        apt-get update -qq
    fi
fi

# Create necessary directories
mkdir -p /workspace/build
mkdir -p /workspace/install  
mkdir -p /workspace/log
mkdir -p /workspace/debian_output

# Set permissions
if [ -n "$LOCAL_USER_ID" ]; then
    chown -R builder:builder /workspace
fi

# Install rust-script if needed and requested
if [ "$INSTALL_RUST_SCRIPT" = "true" ]; then
    if ! command -v rust-script >/dev/null 2>&1; then
        echo "Installing rust-script..."
        cargo install rust-script --version 0.35.0 --quiet
    fi
fi

# Verify tools are available
echo "Checking tool availability..."
if ! command -v bloom-generate >/dev/null 2>&1; then
    echo "Warning: bloom-generate not found - some features may not work"
fi
if ! command -v dpkg-buildpackage >/dev/null 2>&1; then
    echo "Error: dpkg-buildpackage not found!"
    exit 1
fi

echo "Environment setup complete!"