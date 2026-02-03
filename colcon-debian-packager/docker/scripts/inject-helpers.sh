#!/bin/bash
# Script to inject colcon-deb helper scripts into the container

set -e

echo "Injecting colcon-deb helper scripts..."

# Define helper scripts location
HELPERS_SOURCE="${HELPERS_SOURCE:-/opt/colcon-deb/helpers}"
HELPERS_DEST="/usr/local/bin"

# Check if helpers directory is mounted or exists
if [ ! -d "$HELPERS_SOURCE" ]; then
    echo "Warning: Helper scripts directory not found at $HELPERS_SOURCE"
    echo "Please mount the scripts directory with: -v /path/to/scripts:$HELPERS_SOURCE:ro"
    exit 0
fi

# List of helper scripts to inject
HELPER_SCRIPTS=(
    "scanner.rs"
    "debian-preparer.rs"
)

# Copy and setup each helper script
for script in "${HELPER_SCRIPTS[@]}"; do
    script_path="$HELPERS_SOURCE/$script"
    script_name="${script%.rs}"
    
    if [ -f "$script_path" ]; then
        echo "Installing helper: $script"
        
        # Copy to destination
        cp "$script_path" "$HELPERS_DEST/"
        chmod +x "$HELPERS_DEST/$script"
        
        # Create a wrapper script for easier execution
        cat > "$HELPERS_DEST/$script_name" <<EOF
#!/bin/bash
exec rust-script "$HELPERS_DEST/$script" "\$@"
EOF
        chmod +x "$HELPERS_DEST/$script_name"
        
        echo "  Installed as: $script_name"
    else
        echo "Warning: Helper script not found: $script_path"
    fi
done

# Verify rust-script can run the helpers
echo "Verifying helper scripts..."
for script in "${HELPER_SCRIPTS[@]}"; do
    script_name="${script%.rs}"
    if command -v "$script_name" >/dev/null 2>&1; then
        echo "  ✓ $script_name is available"
    else
        echo "  ✗ $script_name failed to install"
    fi
done

echo "Helper injection complete!"