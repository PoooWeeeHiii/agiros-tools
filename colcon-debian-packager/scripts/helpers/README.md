# Debian Preparer Helper Script

This directory contains `debian-preparer.rs`, a rust-script helper for preparing Debian package directories for ROS packages.

## Overview

The `debian-preparer.rs` script automates the process of preparing `debian/` directories for ROS packages by:

1. Checking for existing custom debian directories
2. Using `bloom-generate` to create missing directories
3. Validating debian directory structure
4. Saving generated directories for future reuse

## Usage

### Basic Usage

```bash
./debian-preparer.rs \
  --package-name "my_ros_package" \
  --package-path "/path/to/package" \
  --package-version "1.0.0" \
  --debian-dirs "/path/to/debian/collection"
```

### Full Example

```bash
./debian-preparer.rs \
  --package-name "geometry_msgs" \
  --package-path "/ws/src/geometry_msgs" \
  --package-version "4.2.3" \
  --debian-dirs "/ws/debian_dirs" \
  --ros-distro "loong" \
  --maintainer "AGIROS Team <ros@example.com>" \
  --use-bloom true \
  --validate true \
  --json-output true
```

## Command Line Arguments

| Argument             | Short | Required | Default  | Description                                 |
|----------------------|-------|----------|----------|---------------------------------------------|
| `--package-name`     | `-p`  | ✓        | -        | Name of the ROS package                     |
| `--package-path`     | `-s`  | ✓        | -        | Path to the package source directory        |
| `--package-version`  | `-v`  | ✓        | -        | Version of the package                      |
| `--debian-dirs`      | `-d`  | ✓        | -        | Path to debian directories collection       |
| `--ros-distro`       | `-r`  | ✗        | `loong` | ROS distribution name                       |
| `--maintainer`       | `-m`  | ✗        | -        | Maintainer information                      |
| `--use-bloom`        |       | ✗        | `true`   | Use bloom-generate for missing directories  |
| `--force-regenerate` |       | ✗        | `false`  | Force regeneration even if custom exists    |
| `--validate`         |       | ✗        | `true`   | Validate debian directory after preparation |
| `--json-output`      |       | ✗        | `false`  | Output results in JSON format               |

## How It Works

### 1. Custom Directory Check

The script first checks if a custom debian directory exists at:
```
{debian_dirs}/{package_name}/debian/
```

If found and `--force-regenerate` is false, it uses the custom directory.

### 2. Bloom Generation

If no custom directory exists and `--use-bloom` is true, the script runs:
```bash
bloom-generate debian \
  --package-name {package_name} \
  --package-version {debian_version} \
  --ros-distro {ros_distro} \
  --os-name ubuntu \
  --os-version {ubuntu_version}
```

### 3. Directory Saving

Generated directories are automatically saved to the collection for future reuse:
```
{debian_dirs}/{package_name}/debian/
```

### 4. Validation

The script validates that all required files exist:
- `control` - Package metadata
- `changelog` - Version history
- `copyright` - Licensing information
- `rules` - Build instructions

Additional validation checks:
- Control file required fields (Package, Architecture, Maintainer, Description)
- Rules file executable permissions
- Rules file shebang and required targets

## Version Conversion

ROS versions are automatically converted to Debian format:

| ROS Version    | Debian Version   |
|----------------|------------------|
| `1.0.0`        | `1.0.0-1`        |
| `2.1.3-alpha1` | `2.1.3~alpha1-1` |
| `1.5.0-beta2`  | `1.5.0~beta2-1`  |
| `3.0.0-rc1`    | `3.0.0~rc1-1`    |

## Output Formats

### Text Output (Default)

```
Debian preparation result for geometry_msgs
  Used custom directory: false
  Used bloom-generate: true
  Validation passed: true
  Debian path: /ws/src/geometry_msgs/debian
  Warnings:
    - Rules file missing clean target
```

### JSON Output

```json
{
  "package_name": "geometry_msgs",
  "used_custom": false,
  "used_bloom": true,
  "validation_passed": true,
  "debian_path": "/ws/src/geometry_msgs/debian",
  "warnings": [
    "Rules file missing clean target"
  ],
  "errors": []
}
```

## Dependencies

The script requires the following tools:

- `bloom-generate` (from python3-bloom package)
- Standard Unix tools for file operations

Rust dependencies are automatically handled by rust-script:
- `clap` - Command line parsing
- `tokio` - Async runtime
- `serde_json` - JSON serialization
- `walkdir` - Directory traversal
- `regex` - Pattern matching
- `chrono` - Date/time handling

## Exit Codes

- `0` - Success
- `1` - Validation failed (when `--validate` is true)
- Other - Script error or bloom-generate failure

## Integration with colcon-deb

This script is designed to be called by the main colcon-deb Rust application as part of the debian directory preparation pipeline. It provides a bridge between the Rust application and the Python-based bloom tooling.

## Ubuntu Version Mapping

The script automatically maps ROS distributions to Ubuntu versions:

| ROS Distro       | Ubuntu Version    |
|------------------|-------------------|
| `loong`, `iron` | `jammy` (22.04)   |
| `pixiu`          | `noble` (24.04)   |
| `rolling`        | `noble` (24.04)   |
| Others           | `jammy` (default) |
