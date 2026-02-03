#!/bin/bash
# Install rust-script if not already installed

set -e

if ! command -v rust-script >/dev/null 2>&1; then
    echo "Installing rust-script..."
    cargo install rust-script --version 0.35.0
    echo "rust-script installed successfully!"
else
    echo "rust-script is already installed"
fi