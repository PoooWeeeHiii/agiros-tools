# AGIROS 工具菜单使用说明

本说明基于 `agiros_tools_menu.py` 的当前实现，用于介绍菜单 1–8 的功能层级、操作参数含义，以及运行所需依赖与一键安装脚本。

## 快速入口

```bash
python3 agiros_tools_menu.py
```

如果安装了 `setup.py` 的 entry points，也可以：

```bash
agiros-tools-menu
```

## 菜单功能详解（1–8）

### 1. 从 distribution.yaml 下载 release 仓库（tracks）或源码（code）

**用途**  
根据 rosdistro `distribution.yaml` 自动拉取源码仓库或 release/tracks 仓库，生成下载日志。

**交互参数**  
- `distribution.yaml URL`：rosdistro 的入口地址。  
- `直接下载code? (No=tracks)`：
  - `Yes`：下载源码仓库（code）。
  - `No`：下载 release/tracks 仓库。  
- `源码(code)仓库存放目录 / Release(tracks)仓库存放目录`：目标落盘路径。

**流程**  
1. 校验 URL 与目标目录。  
2. 调用 `yaml_git_downloader_release.download_repos_from_distribution_yaml(...)`。  
3. 生成 `download_log.txt`。  
4. 更新 `distribution_url`、`release_dir` 环境变量。

---

### 2. 处理 tracks.yaml / 下载源码

**用途**  
根据 release 仓库中的 tracks 信息生成源码仓库目录。

**交互参数**  
- `Release 仓库目录`：tracks.yaml 所在目录。  
- `源码目录`：源码输出目录。  
- `Tracks ROS2 发行版名称`：如 `jazzy`。  
- `resume`：启用断点续传。  
- `limit`：限制下载包数量（整数，留空为全部）。

**流程**  
1. 组装参数调用 `oob_tracks_to_sources.main(args)`。  
2. 成功后更新 `release_dir`、`code_dir`、`ros2_distro` 环境变量。

---

### 3. 源码替换 ros->agiros

**用途**  
将源码中的 ROS/发行版标识替换为 AGIROS 对应名称。

**交互参数**  
**操作范围**：  
1. `单包`：进入包搜索/选择。  
2. `批量: {code_dir}`：对整个源码目录批量替换。  
3. `返回`：退出。

**流程**  
1. 单包模式下可“关键字搜索/手动输入”。  
2. 调用 `change_ros2agiros_tag(path, from_str=ros2_distro, to_str=agiros_distro)`。

---

### 4. Bloom 打包

**用途**  
生成 Debian/RPM/gbp 相关打包文件。

**交互参数**  
- 功能类型：`生成 Debian 目录` / `生成 spec 文件` / `生成 debian+spec` / `生成 gbp.conf`  
- 操作范围：`单包` / `批量`
- 单包模式下支持可选 `生成 gbp.conf`。  

**流程（单包）**  
1. 选择包路径。  
2. 运行 `bloom-generate agirosdebian` 或 `bloom-generate agirosrpm`。  
3. 可按需写入 `gbp.conf`。

**流程（批量）**  
1. 调用 `oob_builder_procedural.py`。  
2. 支持 `--limit`、`--dry-run`、`--generate-gbp`。

---

### 5. 构建 (Build)

**用途**  
管理构建队列并执行 Debian/RPM 构建。

**子功能**  
1. **查看队列**：展示构建包列表与完成标记 `#`。  
2. **添加任务**：手动选择或自动扫描生成构建列表。  
3. **执行队列**：按 Debian/RPM 运行构建。  
4. **清空队列**：清空队列文件与元数据。  
5. **编辑构建参数**：调整 gbp/debuild/rpmbuild 参数。

**Debian 构建方式**  
1. **单线程**：  
   - `debuild -us -uc -b`  
2. **并行编译**：  
   - Docker 可用时优先 `colcon-deb`  
   - 否则 fallback 到 `build-orchestrator.rs`

**常用参数/变量**  
- `DEB_OUT`：Debian 输出目录。  
- `PARALLEL`：并行线程数。  
- `DISTRO`：Debian 发行版标识。  
- `DEFAULT_REL_INC`：默认 release_inc。  
- `COLCON_SRC_DIR`：colcon 源码相对目录。  
- `AGIROS_APT_SOURCE`：自定义 APT 源行（并行构建时用于依赖修复）。  
- `AGIROS_AUTO_FIX_DEPS`：缺依赖时自动修复。

---

### 6. 清理生成目录

**用途**  
清理构建中间产物与生成文件。

**流程**  
调用 `clean_generated.py`，通过 `CODE_DIR` 作为清理根目录。

---

### 7. 配置与状态

**用途**  
查看并修改全局配置参数。

**可配置项**  
Release/源码/安装前缀、发行版设置、构建输出路径、构建并行数、APT 源、Git 用户等。

---

### 8. 查看日志

**用途**  
列出当前存在的日志文件并查看内容。

**日志来源**  
在 `release_dir`、`code_dir`、仓库根目录中扫描：  
- `download_log.txt`  
- `failed_repos.txt`  
- `fail.log`  
- 以及 `*.log` / `*log*.txt`

**流程**  
选择日志 → 查看内容（默认显示末尾 4000 字符）→ 返回或继续查看。

---

## 8 个功能的快捷启用方式（命令行）

> 说明：以下命令均可独立运行，适合“只做某一功能”的场景。  
> 建议先导出常用环境变量（可按需替换路径与发行版）：

```bash
export AGIROS_RELEASE_DIR=/path/to/release
export AGIROS_CODE_DIR=/path/to/code
export AGIROS_DISTRO=pixiu
export ROS2_DISTRO=jazzy
export AGIROS_UBUNTU_DEFAULT=jammy
export AGIROS_OE_DEFAULT=24
```

### 1) 从 distribution.yaml 下载 release 仓库（tracks）或源码（code）

**下载 release/tracks（直接读取 distribution.yaml 的 release.url）**

```bash
AGIROS_RELEASE_TARGET_DIR=/path/to/release \
python3 - <<'PY'
import yaml_git_downloader_release as d
d.download_repos_from_yaml("https://github.com/ros/rosdistro/blob/jazzy/2025-10-14/jazzy/distribution.yaml", d.TARGET_DIR)
PY
```

**下载 code（推荐两步）**

```bash
# 1) 先下载 release/tracks
AGIROS_RELEASE_TARGET_DIR=/path/to/release \
python3 - <<'PY'
import yaml_git_downloader_release as d
d.download_repos_from_yaml("https://github.com/ros/rosdistro/blob/jazzy/2025-10-14/jazzy/distribution.yaml", d.TARGET_DIR)
PY

# 2) 再从 tracks.yaml 拉源码
python3 oob_tracks_to_sources.py \
  --release-dir /path/to/release \
  --code-dir /path/to/code \
  --distro "$ROS2_DISTRO" \
  --resume
```

> 如你的 `yaml_git_downloader_release` 版本已包含 `download_repos_from_distribution_yaml(url, target, mode)`，  
> 也可直接指定 `mode=code` 或 `mode=tracks` 进行单步下载。

### 2) 处理 tracks.yaml / 下载源码

```bash
python3 oob_tracks_to_sources.py \
  --release-dir /path/to/release \
  --code-dir /path/to/code \
  --distro "$ROS2_DISTRO" \
  --resume \
  --limit 200
```

### 3) 源码替换 ros->agiros

**单包替换**

```bash
python3 - <<'PY'
from change_ros2agiros import change_ros2agiros_tag
change_ros2agiros_tag("/path/to/code/pkg_name", from_str="jazzy", to_str="pixiu")
PY
```

**批量替换**

```bash
python3 - <<'PY'
from change_ros2agiros import change_ros2agiros_tag
change_ros2agiros_tag("/path/to/code", from_str="jazzy", to_str="pixiu")
PY
```

### 4) Bloom 打包

**单包生成 Debian 目录**

```bash
bloom-generate agirosdebian \
  --ros-distro "$AGIROS_DISTRO" \
  --os-name ubuntu \
  --os-version "$AGIROS_UBUNTU_DEFAULT"
```

**单包生成 RPM spec**

```bash
bloom-generate agirosrpm \
  --ros-distro "$AGIROS_DISTRO" \
  --os-name openeuler \
  --os-version "$AGIROS_OE_DEFAULT"
```

**单包生成 gbp.conf**

```bash
OOB_TRACKS_DIR="$AGIROS_RELEASE_DIR" \
bloom-generate agirosdebian \
  --ros-distro "$AGIROS_DISTRO" \
  --os-name ubuntu \
  --os-version "$AGIROS_UBUNTU_DEFAULT" \
  --generate-gbp \
  --tracks-distro "$ROS2_DISTRO" \
  --distro "$AGIROS_DISTRO" \
  --pkg "<pkg_name>"
```

**批量生成（debian/spec/both/gbp）**

```bash
python3 agiros_oob_builder_procedural.py \
  --release-dir "$AGIROS_RELEASE_DIR" \
  --code-dir "$AGIROS_CODE_DIR" \
  --ros-distro "$AGIROS_DISTRO" \
  --ubuntu-default "$AGIROS_UBUNTU_DEFAULT" \
  --openeuler-default "$AGIROS_OE_DEFAULT" \
  --openeuler-fallback 22 23 \
  --mode both \
  --generate-gbp \
  --limit 0 \
  --dry-run
```

### 5) 构建 (Build)

**Debian 单包构建（gbp）**

```bash
export CODE_DIR="$AGIROS_CODE_DIR"
export DEB_OUT=/path/to/deb_out
export DISTRO="$AGIROS_DISTRO"
export DEFAULT_REL_INC=1
export PARALLEL=8
export GIT_USER_NAME="YourName"
export GIT_USER_EMAIL="you@example.com"
python3 git_build_any.py --path /path/to/code/pkg_name
```

**Debian 批量构建（自动扫描 CODE_DIR）**

```bash
export CODE_DIR="$AGIROS_CODE_DIR"
export DEB_OUT=/path/to/deb_out
export DISTRO="$AGIROS_DISTRO"
export DEFAULT_REL_INC=1
export PARALLEL=8
python3 git_build_any.py --auto
```

**Debian 并行构建（colcon-deb）**

```bash
colcon-deb -c colcon-deb.build-tools.yaml \
  build --output /path/to/deb_out -j 8 --agiros-distro "$AGIROS_DISTRO"
```

**RPM 单包构建**

```bash
export CODE_DIR="$AGIROS_CODE_DIR"
export WORK_DIR=/path/to/code/pkg_name
python3 rpmbuild_any.py --path /path/to/code/pkg_name
```

**RPM 批量构建（自动扫描 CODE_DIR）**

```bash
export CODE_DIR="$AGIROS_CODE_DIR"
python3 rpmbuild_any.py --auto
```

### 6) 清理生成目录

```bash
CODE_DIR="$AGIROS_CODE_DIR" python3 clean_generated.py
```

### 7) 配置与状态（环境变量快速配置）

**临时配置（当前终端生效）**

```bash
export AGIROS_RELEASE_DIR=/path/to/release
export AGIROS_CODE_DIR=/path/to/code
export AGIROS_INSTALL_PREFIX=/opt/agiros/pixiu
export AGIROS_DISTRO=pixiu
export ROS2_DISTRO=jazzy
export AGIROS_UBUNTU_DEFAULT=jammy
export AGIROS_OE_DEFAULT=24
```

**持久配置（写入 .env）**

```bash
cat > .env <<'EOF'
AGIROS_RELEASE_DIR=/path/to/release
AGIROS_CODE_DIR=/path/to/code
AGIROS_INSTALL_PREFIX=/opt/agiros/pixiu
AGIROS_DISTRO=pixiu
ROS2_DISTRO=jazzy
AGIROS_UBUNTU_DEFAULT=jammy
AGIROS_OE_DEFAULT=24
EOF
```

### 8) 查看日志

**列出日志文件**

```bash
find "$AGIROS_RELEASE_DIR" "$AGIROS_CODE_DIR" . -maxdepth 3 -type f \
  \( -name "*.log" -o -name "*log*.txt" \)
```

**快速查看日志末尾**

```bash
tail -n 200 "$AGIROS_RELEASE_DIR/download_log.txt"
```

---

## RISC-V QEMU 环境（Docker 版）使用教程

> 适合 macOS / Windows / Linux，所有构建与运行都在容器内完成。

**首次构建 + 运行（Debian，推荐）**

```bash
USE_PREBUILT_FW=1 ROOTFS_KIND=debian bash run_qemu_riscv_docker.sh
```

**仅运行（已有产物时）**

```bash
RUN_ONLY=1 ROOTFS_KIND=debian bash run_qemu_riscv_docker.sh
```

**指定 QEMU 版本**

```bash
QEMU_VERSION=8.2.5 ROOTFS_KIND=debian bash run_qemu_riscv_docker.sh
```

**使用系统 QEMU（容器内已带 qemu-system-misc）**

```bash
QEMU_VERSION=system ROOTFS_KIND=debian bash run_qemu_riscv_docker.sh
```

**Ubuntu rootfs（注意 root 分区）**

```bash
ROOTFS_KIND=ubuntu ROOTFS_DEV_OVERRIDE=/dev/vda2 bash run_qemu_riscv_docker.sh
```

**Buildroot rootfs（最小系统）**

```bash
ROOTFS_KIND=buildroot bash run_qemu_riscv_docker.sh
```

**不使用 rootfs，仅进入 U-Boot**

```bash
ROOTFS_KIND=none USE_UBOOT=1 bash run_qemu_riscv_docker.sh
```

**离线模式（仅使用缓存）**

```bash
OFFLINE=1 ROOTFS_KIND=debian bash run_qemu_riscv_docker.sh
```

**强制重建**

```bash
FORCE_REBUILD=1 ROOTFS_KIND=debian bash run_qemu_riscv_docker.sh
```

**产物与缓存目录**

- 产物：`./qemu-riscv-env/output`
- 缓存：`./qemu-riscv-env/cache`

**退出 QEMU**

按 `Ctrl + a` 后再按 `x`

**登录提示**

- Debian 通常可用 `root` 登录（部分镜像无密码或默认密码）
- Ubuntu 镜像需要按启动提示完成初始化

---

## RISC-V QEMU 环境（CLI 版）使用教程

> 使用 `agiros-riscv-env` 直接在宿主机编排（不推荐 macOS/Windows，建议 Docker 版）。

**基础运行**

```bash
agiros-riscv-env --prefix ./qemu-riscv-env
```

**指定 QEMU 与 rootfs**

```bash
agiros-riscv-env --qemu-version 8.2.5 --rootfs debian
```

**可选 QEMU 版本**

```bash
agiros-riscv-env --qemu-version system   # 使用系统包管理器的 QEMU
agiros-riscv-env --qemu-version 9.0.2
agiros-riscv-env --qemu-version 8.2.5
agiros-riscv-env --qemu-version 7.2.0
```

**可选 rootfs**

```bash
agiros-riscv-env --rootfs debian
agiros-riscv-env --rootfs ubuntu
agiros-riscv-env --rootfs buildroot
```

**非交互模式**

```bash
agiros-riscv-env --no-prompt --qemu-version 8.2.5 --rootfs debian
```

**离线模式**

```bash
agiros-riscv-env --offline --rootfs debian
```

**跳过依赖安装（你已准备好环境时）**

```bash
agiros-riscv-env --skip-install --rootfs debian
```

**自定义缓存/输出目录**

```bash
agiros-riscv-env --prefix /path/to/qemu-riscv-env --cache-dir /path/to/cache
```

**使用 LLVM 编译内核（macOS 备用方案）**

```bash
agiros-riscv-env --use-llvm --rootfs debian
```

**环境变量方式（等价于参数）**

```bash
export QEMU_VERSION=8.2.5
export ROOTFS_KIND=debian
export OPENSBI_URL=https://github.com/riscv-software-src/opensbi/archive/refs/tags/v1.4.tar.gz
export UBOOT_URL=https://github.com/u-boot/u-boot/archive/refs/tags/v2024.01.tar.gz
export LINUX_URL=https://cdn.kernel.org/pub/linux/kernel/v6.x/linux-6.6.30.tar.xz
export UBUNTU_ROOTFS_URL=https://cdimage.ubuntu.com/releases/24.04/release/ubuntu-24.04-preinstalled-server-riscv64+qemu.img.xz
export DEBIAN_ROOTFS_URL=https://cdimage.debian.org/cdimage/ports/latest/riscv64/cloud/debian-12-genericcloud-riscv64.qcow2
agiros-riscv-env --no-prompt
```

## 全局配置参数说明（环境变量）

| 变量 | 含义 |
|:---|:---|
| `AGIROS_RELEASE_DIR` | Release 仓库目录 |
| `AGIROS_CODE_DIR` | 源码目录 |
| `AGIROS_INSTALL_PREFIX` | 安装前缀目录 |
| `AGIROS_DISTRO` | AGIROS 发行版名称 |
| `ROS2_DISTRO` | ROS2 发行版名称 |
| `AGIROS_UBUNTU_DEFAULT` | Ubuntu 版本代号 |
| `AGIROS_OE_DEFAULT` | openEuler 默认版本 |
| `AGIROS_OE_FALLBACK` | openEuler 回退版本列表 |
| `AGIROS_BLOOM_BIN` | bloom 命令路径或别名 |
| `AGIROS_GENERATE_GBP` | 批量流程是否自动生成 gbp.conf |
| `DEB_OUT` | Debian 输出目录 |
| `DISTRO` | Debian 发行版标识 |
| `DEFAULT_REL_INC` | 默认 release_inc |
| `PARALLEL` | 并行线程数 |
| `COLCON_SRC_DIR` | colcon 源码相对目录 |
| `AGIROS_APT_SOURCE` | APT 源行 |
| `AGIROS_APT_SOURCE_FILE` | APT 源文件路径 |
| `AGIROS_AUTO_FIX_DEPS` | 缺依赖时自动修复 |
| `GIT_USER_NAME` | Git 提交用户名 |
| `GIT_USER_EMAIL` | Git 提交邮箱 |
| `AGIROS_QUEUE_FILE` | 构建队列文件 |
| `AGIROS_QUEUE_META` | 构建队列元数据文件 |

---

## 依赖清单

### Python 依赖（requirements.txt）

```
rich
questionary
requests
PyYAML
python-dotenv
catkin_pkg
setuptools
empy
packaging
python-dateutil
rosdep
rosdistro
vcstools
```

安装方式：

```bash
python3 -m pip install -r requirements.txt
```

---

### 系统依赖（Ubuntu / Debian）

```
python3 python3-pip python3-venv python3-dev
git curl ca-certificates
build-essential cmake pkg-config
devscripts dpkg-dev debhelper dh-python git-buildpackage
rsync
```

可选（并行构建/容器）：

```
docker.io
python3-colcon-common-extensions
```

---

### 系统依赖（openEuler）

```
python3 python3-pip python3-devel
git curl ca-certificates
gcc gcc-c++ make cmake pkgconfig
rpm-build
```

可选（并行构建/容器）：

```
docker
```

---

### 可选工具依赖

- **Rust 工具链**：用于 `rust-script`/`colcon-deb` fallback 并行构建。  
- **colcon**：并行构建入口（若未使用 Docker 也可 pip 安装）。  
- **bloom**：打包时需要的 bloom 相关工具。  
- **gbp / debuild / rpmbuild**：构建 Debian/RPM 时需要。

---

## 一键安装脚本

仓库根目录提供 `install_deps.sh`，用于在启用前统一安装依赖。

**用法示例**  

```bash
bash install_deps.sh
```

带可选组件：

```bash
bash install_deps.sh --with-colcon --with-rust --with-docker
```

---

## 常见入口脚本

- `agiros_tools_menu.py`：主菜单  
- `oob_tracks_to_sources.py`：tracks 下载  
- `yaml_git_downloader_release.py`：distribution 下载  
- `git_build_any.py`：Debian 构建  
- `rpmbuild_any.py`：RPM 构建  
- `clean_generated.py`：清理生成目录  

需要更细化的参数说明或示例流程，可在此文档基础上继续扩展。  
