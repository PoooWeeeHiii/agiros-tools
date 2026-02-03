#!/bin/bash
# Main build script - runs as non-root user
# Executes the complete build pipeline

set -e

# Progress reporting functions
log() {
    echo "::log::level=$1,msg=$2" >&2
}

report_stage() {
    echo "::progress::type=stage,value=$1" >&2
    log "info" "Stage: $1"
}

report_error() {
    log "error" "$1"
    exit 1
}

# Ensure we're in the workspace
cd /workspace || report_error "Failed to change to workspace directory"

# Source ROS environment
log "info" "Setting up ROS environment"
if [ -f /opt/agiros/$ROS_DISTRO/setup.bash ]; then
    source /opt/agiros/$ROS_DISTRO/setup.bash
else
    report_error "AGIROS environment not found for distro: $ROS_DISTRO"
fi

# Initialize agirosdep if needed
report_stage "dependencies"
log "info" "Initializing agirosdep"
sudo agirosdep init 2>/dev/null || true
agirosdep update

# Install dependencies
log "info" "Installing ROS dependencies"
sudo agirosdep install --from-paths src --ignore-src -y --rosdistro $ROS_DISTRO || {
    log "warning" "Some dependencies could not be installed, continuing anyway"
}

# Check if rust-script helpers are available
if [ -f /helpers/check-requirements.rs ] && command -v rust-script >/dev/null 2>&1; then
    log "info" "Using rust-script helpers for build orchestration"
    
    # Run the build orchestrator which handles:
    # 1. Package scanning
    # 2. Colcon build
    # 3. Debian directory preparation
    # 4. Parallel .deb creation
    # 5. Repository generation
    /helpers/build-orchestrator.rs \
        --workspace /workspace \
        --debian-dirs /workspace/debian_dirs \
        --output-dir /workspace/output
    
elif [ -f /helpers/build-orchestrator.rs ]; then
    # rust-script not available, fall back to shell implementation
    log "warning" "rust-script not available, using shell fallback"
    
    # Stage: Scanning
    report_stage "scanning"
    log "info" "Scanning for ROS packages"
    
    # Simple package discovery using find
    PACKAGES=""
    for package_xml in $(find /workspace/src -name "package.xml" -type f 2>/dev/null); do
        PKG_DIR=$(dirname "$package_xml")
        # Skip if COLCON_IGNORE exists
        if [ ! -f "$PKG_DIR/COLCON_IGNORE" ]; then
            PKG_NAME=$(grep -oP '(?<=<name>)[^<]+' "$package_xml" 2>/dev/null || echo "unknown")
            PACKAGES="$PACKAGES $PKG_NAME:$PKG_DIR"
            log "info" "Found package: $PKG_NAME"
        fi
    done
    
    if [ -z "$PACKAGES" ]; then
        report_error "No ROS packages found in workspace"
    fi
    
    # Stage: Building with colcon
    report_stage "colcon_build"
    log "info" "Building all packages with colcon"
    
    colcon build \
        --merge-install \
        --parallel-workers ${PARALLEL_JOBS:-4} \
        --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE:-Release} || \
        report_error "Colcon build failed"
    
    # Source the install space
    log "info" "Sourcing install/setup.bash"
    source /workspace/install/setup.bash
    
    # Stage: Preparing debian directories
    report_stage "preparing"
    
    for pkg_info in $PACKAGES; do
        PKG_NAME=$(echo $pkg_info | cut -d: -f1)
        PKG_PATH=$(echo $pkg_info | cut -d: -f2)
        
        log "info" "Preparing debian directory for $PKG_NAME"
        
        DEBIAN_DIR="/workspace/debian_dirs/$PKG_NAME/debian"
        TARGET_DEBIAN="$PKG_PATH/debian"
        
        if [ -d "$DEBIAN_DIR" ]; then
            log "info" "Using existing debian directory for $PKG_NAME"
            cp -r "$DEBIAN_DIR" "$TARGET_DEBIAN"
        else
            log "info" "Generating debian directory for $PKG_NAME with bloom-generate"
            
            cd "$PKG_PATH"
            bloom-generate rosdebian \
                --ros-distro $ROS_DISTRO \
                --debian-inc 0 \
                --os-name ubuntu \
                --os-version jammy \
                . || {
                    log "error" "bloom-generate failed for $PKG_NAME"
                    continue
                }
            
            # Save generated debian directory
            if [ -d "$TARGET_DEBIAN" ]; then
                # Post-process debian/control to enforce agiros naming
                if [ -f "$TARGET_DEBIAN/control" ]; then
                    sed -i "s/Package: ros-$ROS_DISTRO-/Package: agiros-$ROS_DISTRO-/g" "$TARGET_DEBIAN/control"
                    sed -i "s/Source: ros-$ROS_DISTRO-/Source: agiros-$ROS_DISTRO-/g" "$TARGET_DEBIAN/control"
                    log "info" "Updated debian/control to use agiros-$ROS_DISTRO- prefix"
                fi

                mkdir -p "/workspace/debian_dirs/$PKG_NAME"
                cp -r "$TARGET_DEBIAN" "/workspace/debian_dirs/$PKG_NAME/"
                log "info" "Saved generated debian directory for $PKG_NAME"
            fi
        fi
    done
    
    # Stage: Creating .deb packages
    report_stage "packaging"
    
    # Create output directory
    mkdir -p /workspace/output
    
    # Build .deb packages (can be parallelized but keeping simple for fallback)
    for pkg_info in $PACKAGES; do
        PKG_NAME=$(echo $pkg_info | cut -d: -f1)
        PKG_PATH=$(echo $pkg_info | cut -d: -f2)
        
        if [ ! -d "$PKG_PATH/debian" ]; then
            log "warning" "Skipping $PKG_NAME - no debian directory"
            continue
        fi
        
        log "info" "Building .deb package for $PKG_NAME"
        echo "::progress::type=package_start,name=$PKG_NAME" >&2
        
        cd "$PKG_PATH"
        if dpkg-buildpackage -b -uc -us; then
            # Move generated .deb files to output
            find .. -maxdepth 1 -name "*.deb" -type f -exec mv {} /workspace/output/ \;
            echo "::progress::type=package_complete,name=$PKG_NAME,success=true" >&2
            log "info" "Successfully built .deb for $PKG_NAME"
        else
            echo "::progress::type=package_complete,name=$PKG_NAME,success=false" >&2
            log "error" "Failed to build .deb for $PKG_NAME"
        fi
    done
    
    # Stage: Repository generation
    report_stage "repository"
    cd /workspace/output
    /scripts/create-repo.sh
    
else
    report_error "No build orchestrator available"
fi

# Stage: Complete
report_stage "complete"
log "info" "Build completed successfully"

# List generated .deb files
log "info" "Generated .deb packages:"
ls -la /workspace/output/*.deb 2>/dev/null || log "warning" "No .deb files found in output"