#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WITH_DOCKER=0
WITH_RUST=0
WITH_COLCON=0

usage() {
  cat <<'EOF'
Usage: install_deps.sh [options]

Options:
  --with-docker   Install Docker (optional, for parallel build with container)
  --with-rust     Install Rust toolchain (optional, for rust-script/colcon-deb)
  --with-colcon   Install colcon (optional, for parallel build)
  -h, --help      Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-docker) WITH_DOCKER=1 ;;
    --with-rust) WITH_RUST=1 ;;
    --with-colcon) WITH_COLCON=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
  shift
done

if [[ ! -f "${ROOT_DIR}/requirements.txt" ]]; then
  echo "requirements.txt not found in ${ROOT_DIR}" >&2
  exit 1
fi

OS_ID=""
if [[ -f /etc/os-release ]]; then
  OS_ID="$(. /etc/os-release && echo "${ID:-}")"
fi

install_python_deps() {
  python3 -m pip install --upgrade pip
  python3 -m pip install -r "${ROOT_DIR}/requirements.txt"
}

if [[ "${OS_ID}" == "ubuntu" || "${OS_ID}" == "debian" ]]; then
  sudo apt-get update
  sudo apt-get install -y \
    python3 python3-pip python3-venv python3-dev \
    git curl ca-certificates \
    build-essential cmake pkg-config \
    devscripts dpkg-dev debhelper dh-python git-buildpackage \
    rsync

  if [[ "${WITH_DOCKER}" == "1" ]]; then
    sudo apt-get install -y docker.io
  fi

  if [[ "${WITH_COLCON}" == "1" ]]; then
    sudo apt-get install -y python3-colcon-common-extensions || true
    python3 -m pip install colcon-common-extensions colcon-ros
  fi

  install_python_deps
elif [[ "${OS_ID}" == "openeuler" ]]; then
  sudo dnf makecache
  sudo dnf install -y \
    python3 python3-pip python3-devel \
    git curl ca-certificates \
    gcc gcc-c++ make cmake pkgconfig \
    rpm-build

  if [[ "${WITH_DOCKER}" == "1" ]]; then
    sudo dnf install -y docker
  fi

  if [[ "${WITH_COLCON}" == "1" ]]; then
    python3 -m pip install colcon-common-extensions colcon-ros
  fi

  install_python_deps
else
  echo "Unsupported OS (ID=${OS_ID}). Please install dependencies manually." >&2
  exit 1
fi

if [[ "${WITH_RUST}" == "1" ]]; then
  if ! command -v rustup >/dev/null 2>&1; then
    curl https://sh.rustup.rs -sSf | sh -s -- -y
    # shellcheck disable=SC1090
    source "${HOME}/.cargo/env"
  fi
  rustup toolchain install stable
  rustup default stable
  cargo install rust-script --version 0.35.0
fi

echo "Dependency installation completed."
