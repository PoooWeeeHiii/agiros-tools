#!/usr/bin/env python3
import json
import os
import platform
import shlex
import shutil
import sys
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple, Union
from change_ros2agiros import change_ros2agiros_tag 
from debian_dep_sort import compute_series_toposort, discover_debian_package_dirs
from debuild_runner import auto_mark_prebuilt_packages
from os_base import get_sys_info

try:
    from rich import box
    from rich.console import Console
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
except ImportError as exc:  # pragma: no cover - rich is required for this CLI
    raise RuntimeError("rich is required to run agiros_tools_menu.py") from exc

try:
    import questionary
except Exception:
    questionary = None


console = Console()
REPO_ROOT = Path(__file__).resolve().parent

try:
    from dotenv import load_dotenv
except Exception:  # pragma: no cover - optional dependency
    load_dotenv = None

if load_dotenv:
    load_dotenv(dotenv_path=REPO_ROOT / ".env", override=False)


def _fallback_select(message: str, choices: Sequence[str], multiselect: bool = False):
    if not choices:
        return [] if multiselect else None
    console.print(f"[bold cyan]{message}[/]")
    for idx, item in enumerate(choices, start=1):
        console.print(f"  {idx}. {item}")
    prompt = "选择多个请用逗号分隔: " if multiselect else "请输入编号: "
    raw = input(prompt).strip()
    if not raw:
        return [] if multiselect else None
    if multiselect:
        indexes = set()
        for token in raw.split(","):
            token = token.strip()
            if not token.isdigit():
                continue
            idx = int(token)
            if 1 <= idx <= len(choices):
                indexes.add(idx - 1)
        return [choices[i] for i in sorted(indexes)]
    if not raw.isdigit():
        return None
    idx = int(raw)
    if 1 <= idx <= len(choices):
        return choices[idx - 1]
    return None


def ask_select(message: str, choices: Sequence[str]) -> Optional[str]:
    if questionary:
        return questionary.select(message, choices=list(choices)).unsafe_ask()
    return _fallback_select(message, choices)


def ask_checkbox(message: str, choices: Sequence[str]) -> List[str]:
    if questionary:
        return questionary.checkbox(message, choices=list(choices)).unsafe_ask()
    return _fallback_select(message, choices, multiselect=True)


def ask_text(message: str, default: Optional[str] = None) -> Optional[str]:
    if questionary:
        return questionary.text(message, default=default or "").unsafe_ask()
    prompt = f"{message}"
    if default:
        prompt += f" [{default}]"
    prompt += ": "
    raw = input(prompt).strip()
    return raw or default


def ask_confirm(message: str, default: bool = True) -> bool:
    if questionary:
        return questionary.confirm(message, default=default).unsafe_ask()
    suffix = "Y/n" if default else "y/N"
    raw = input(f"{message} ({suffix}): ").strip().lower()
    if not raw:
        return default
    return raw in {"y", "yes"}


def run_stream(cmd: Sequence[str], cwd: Optional[Path] = None, env: Optional[Dict[str, str]] = None) -> int:
    display = " ".join(shlex.quote(str(x)) for x in cmd)
    working_dir = str(cwd or Path.cwd())
    console.print(f"[bold blue]$[/] {display}\n   [dim]cwd={working_dir}[/]")
    proc = subprocess.Popen(
        list(map(str, cmd)),
        cwd=str(cwd) if cwd else None,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    assert proc.stdout is not None
    for line in proc.stdout:
        console.print(line.rstrip(), markup=False)
    proc.wait()
    if proc.returncode != 0:
        console.print(f"[bold red]命令退出码: {proc.returncode}[/]")
    return proc.returncode


def shlex_split(value: str) -> List[str]:
    return shlex.split(value) if value.strip() else []


@dataclass
class BuildTask:
    display_name: str
    path: Path
    kind: str  # debian | rpm
    extra_args: List[str] = field(default_factory=list)




def to_display_name(state: "MenuState", pkg_path: Path) -> str:
    try:
        return str(pkg_path.relative_to(state.code_dir))
    except ValueError:
        return str(pkg_path)


@dataclass
class MenuState:
    #_agiros_release_Tags: str = "loong/2025-12"
    #_ros2_release_Tags: str = "humble/2025-10-20" # 

    _agiros_release_Tags: str = "pixiu/2025-12"
    _ros2_release_Tags: str = "jazzy/2025-10-14" # https://github.com/ros/rosdistro/blob/humble/2025-10-20/humble/distribution.yaml

    _agiros_distro: str = _agiros_release_Tags.split("/")[0] 
    _ros2_distro: str = _ros2_release_Tags.split("/")[0]  # 取_release_Tags第一个斜杠前面的部分
    _prefix = _ros2_release_Tags.replace("/", "_")
    ros2_distro: str = os.environ.get("ROS2_DISTRO", _ros2_distro)
    #distribution_url: str = "http://1.94.193.239/yumrepo/agiros/agirosdep/loong/distribution.yaml"
    distribution_url: str = f"https://github.com/ros/rosdistro/blob/{_ros2_release_Tags}/{ros2_distro}/distribution.yaml" ##https://github.com/ros/rosdistro/blob/jazzy/2025-10-14/jazzy/distribution.yaml
    
    _ros2_release_dir = _prefix + "_ros2_release" #ros2_release_dir取值为release_Tags的/换成横杠，并加上_ros2_release
    release_dir: Path = Path(os.environ.get("AGIROS_RELEASE_DIR", _ros2_release_dir))
    _ros2_code_dir = _prefix + "_ros2_code" #ros2_code_dir取值为release_Tags的/换成横杠，并加上_ros2_code
    code_dir: Path = Path(os.environ.get("AGIROS_CODE_DIR", _ros2_code_dir))
    code_label: str = os.environ.get("AGIROS_CODE_LABEL", "code_dir")
    third_party_dir: Path = Path(os.environ.get("THIRD_PARTY_DIR", "/mnt/3-party"))
    install_prefix: Path = Path(os.environ.get("AGIROS_INSTALL_PREFIX", f"/opt/agiros/{_agiros_distro}"))
    
    agiros_distro: str = os.environ.get("AGIROS_DISTRO", _agiros_distro)
    ubuntu_version: str = os.environ.get("AGIROS_UBUNTU_DEFAULT", "jammy") # "noble"
    openeuler_default: str = os.environ.get("AGIROS_OE_DEFAULT", "24")
    openeuler_fallback: List[str] = field(default_factory=lambda: [item.strip() for item in os.environ.get("AGIROS_OE_FALLBACK", "22,23").split(",") if item.strip()])
    bloom_bin: str = os.environ.get("AGIROS_BLOOM_BIN", "bloom-generate")
    auto_generate_gbp: bool = bool(int(os.environ.get("AGIROS_GENERATE_GBP", "0")))
    debian_build_args: List[str] = field(default_factory=lambda: ["--git-ignore-branch", "--git-ignore-new", "-us", "-uc"])
    rpm_build_base: str = os.environ.get("AGIROS_RPMBUILD_BIN", "rpmbuild")
    rpm_build_args: List[str] = field(default_factory=lambda: ["-ba"])
    deb_out_dir: Path = Path(os.environ.get("DEB_OUT", str(Path.home() / (_prefix + "_deb_out"))))
    deb_distro: str = os.environ.get("DISTRO", _agiros_distro)
    deb_release_inc: str = os.environ.get("DEFAULT_REL_INC", "1")
    deb_parallel: str = os.environ.get("PARALLEL", str(os.cpu_count() or 4))
    colcon_src_dir: str = os.environ.get("COLCON_SRC_DIR", "")
    agiros_apt_source: str = os.environ.get("AGIROS_APT_SOURCE", "")
    agiros_apt_source_file: Path = Path(os.environ.get("AGIROS_APT_SOURCE_FILE", "/etc/apt/sources.list.d/agiros.list"))
    auto_fix_deps: bool = bool(int(os.environ.get("AGIROS_AUTO_FIX_DEPS", "1")))
    git_user_name: str = os.environ.get("GIT_USER_NAME", "PoooWeeeHiii")
    git_user_email: str = os.environ.get("GIT_USER_EMAIL", "powehi041210@gmail.com")
    queue_file: Path = Path(os.environ.get("AGIROS_QUEUE_FILE", str(REPO_ROOT / (_prefix + "_build_queue.txt"))))
    queue_meta_file: Path = Path(os.environ.get("AGIROS_QUEUE_META", ""))
    build_queue: List[BuildTask] = field(default_factory=list)
    queue_packages: List[str] = field(default_factory=list)
    package_status: Dict[str, bool] = field(default_factory=dict)
    os_name, os_version, os_codename, os_arch, python_version = get_sys_info()

    def __post_init__(self) -> None:
        self.queue_file = self._normalize_path(self.queue_file)
        queue_meta_env = os.environ.get("AGIROS_QUEUE_META")
        if queue_meta_env:
            self.queue_meta_file = self._normalize_path(queue_meta_env)
        else:
            self.queue_meta_file = self._meta_path_for_queue(self.queue_file)
        self.ensure_queue_file()
        self.load_queue_from_file()

    def _normalize_path(self, value: Union[str, Path]) -> Path:
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = (REPO_ROOT / path).resolve()
        return path

    def _meta_path_for_queue(self, queue_path: Path) -> Path:
        base = str(queue_path)
        return Path(f"{base}.meta.json")

    def _prepend_env_path(self, env: Dict[str, str], key: str, new_value: Optional[str]) -> None:
        if not new_value:
            return
        existing = env.get(key, "")
        parts = [item for item in existing.split(":") if item]
        parts = [item for item in parts if item != new_value]
        env[key] = ":".join([new_value] + parts) if parts else new_value

    def apply_install_prefix_env(self, env: Dict[str, str]) -> None:
        prefix = str(self.install_prefix)
        if not prefix:
            return
        env["AGIROS_INSTALL_PREFIX"] = prefix
        for key in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH"):
            self._prepend_env_path(env, key, prefix)
        self._prepend_env_path(env, "PKG_CONFIG_PATH", str(self.install_prefix / "lib/pkgconfig"))
        self._prepend_env_path(env, "LD_LIBRARY_PATH", str(self.install_prefix / "lib"))
        self._prepend_env_path(env, "PATH", str(self.install_prefix / "bin"))
        self._prepend_env_path(env, "PYTHONPATH", str(self.install_prefix / "lib/python3/dist-packages"))

    def build_env(self) -> Dict[str, str]:
        env = os.environ.copy()
        self.apply_install_prefix_env(env)
        return env

    def update_env(self) -> None:
        mappings = {
            "AGIROS_RELEASE_DIR": str(self.release_dir),
            "AGIROS_RELEASE_TARGET_DIR": str(self.release_dir),
            "AGIROS_CODE_DIR": str(self.code_dir),
            "AGIROS_CODE_LABEL": self.code_label,
            "ROS2_DISTRO": self.ros2_distro,
            "AGIROS_DISTRO": self.agiros_distro,
            "AGIROS_INSTALL_PREFIX": str(self.install_prefix),
            "AGIROS_UBUNTU_DEFAULT": self.ubuntu_version,
            "OS_NAMECODE": self.os_codename,
            "AGIROS_OE_DEFAULT": self.openeuler_default,
            "AGIROS_OE_FALLBACK": ",".join(self.openeuler_fallback),
            "AGIROS_BLOOM_BIN": self.bloom_bin,
            "AGIROS_GENERATE_GBP": "1" if self.auto_generate_gbp else "0",
            "AGIROS_RPMBUILD_BIN": self.rpm_build_base,
            "DEB_OUT": str(self.deb_out_dir),
            "DISTRO": self.deb_distro,
            "DEFAULT_REL_INC": self.deb_release_inc,
            "PARALLEL": self.deb_parallel,
            "AGIROS_APT_SOURCE": self.agiros_apt_source,
            "AGIROS_APT_SOURCE_FILE": str(self.agiros_apt_source_file),
            "AGIROS_AUTO_FIX_DEPS": "1" if self.auto_fix_deps else "0",
            "GIT_USER_NAME": self.git_user_name,
            "GIT_USER_EMAIL": self.git_user_email,
            "AGIROS_QUEUE_FILE": str(self.queue_file),
            "AGIROS_QUEUE_META": str(self.queue_meta_file),
        }
        for key, value in mappings.items():
            os.environ[key] = value
        if self.colcon_src_dir:
            colcon_src = self.colcon_src_dir.strip()
        else:
            colcon_src = "src" if (self.code_dir / "src").exists() else "."
        os.environ["COLCON_SRC_DIR"] = colcon_src
        self.apply_install_prefix_env(os.environ)

    def refresh_from_env(self) -> None:
        """Sync state fields from process-wide environment variables."""
        env = os.environ

        def _set_path(env_key: str, attr: str) -> None:
            value = env.get(env_key)
            if not value:
                return
            if attr == "queue_file":
                path = self._normalize_path(value)
                self.queue_meta_file = self._meta_path_for_queue(path)
            else:
                path = Path(value).expanduser()
                try:
                    path = path.resolve()
                except Exception:
                    pass
            setattr(self, attr, path)

        def _set_str(env_key: str, attr: str) -> None:
            value = env.get(env_key)
            if value:
                setattr(self, attr, value)

        def _set_bool(env_key: str, attr: str) -> None:
            if env_key not in env:
                return
            value = env.get(env_key, "")
            setattr(self, attr, value.lower() not in {"0", "", "false"})

        def _set_list(env_key: str, attr: str) -> None:
            raw = env.get(env_key)
            if raw is None:
                return
            items = [item.strip() for item in raw.split(",") if item.strip()]
            setattr(self, attr, items)

        _set_path("AGIROS_RELEASE_DIR", "release_dir")
        _set_path("AGIROS_CODE_DIR", "code_dir")
        _set_path("AGIROS_INSTALL_PREFIX", "install_prefix")
        _set_path("DEB_OUT", "deb_out_dir")
        _set_str("AGIROS_CODE_LABEL", "code_label")
        _set_str("ROS2_DISTRO", "ros2_distro")
        _set_str("AGIROS_DISTRO", "agiros_distro")
        _set_str("AGIROS_UBUNTU_DEFAULT", "ubuntu_version")
        _set_str("AGIROS_OE_DEFAULT", "openeuler_default")
        _set_list("AGIROS_OE_FALLBACK", "openeuler_fallback")
        _set_str("AGIROS_BLOOM_BIN", "bloom_bin")
        _set_bool("AGIROS_GENERATE_GBP", "auto_generate_gbp")
        _set_str("AGIROS_RPMBUILD_BIN", "rpm_build_base")
        _set_str("DISTRO", "deb_distro")
        _set_str("DEFAULT_REL_INC", "deb_release_inc")
        _set_str("PARALLEL", "deb_parallel")
        _set_str("COLCON_SRC_DIR", "colcon_src_dir")
        _set_str("AGIROS_APT_SOURCE", "agiros_apt_source")
        _set_path("AGIROS_APT_SOURCE_FILE", "agiros_apt_source_file")
        _set_bool("AGIROS_AUTO_FIX_DEPS", "auto_fix_deps")
        _set_str("GIT_USER_NAME", "git_user_name")
        _set_str("GIT_USER_EMAIL", "git_user_email")
        _set_path("AGIROS_QUEUE_FILE", "queue_file")
        if env.get("AGIROS_QUEUE_META"):
            _set_path("AGIROS_QUEUE_META", "queue_meta_file")
        else:
            self.queue_meta_file = self._meta_path_for_queue(self.queue_file)
        self.ensure_queue_file()
        self.load_queue_from_file()

    def ensure_queue_file(self) -> None:
        path = self.queue_file
        parent = path.parent
        if not parent.exists():
            parent.mkdir(parents=True, exist_ok=True)
        if not path.exists():
            path.touch()
        meta_parent = self.queue_meta_file.parent
        if not meta_parent.exists():
            meta_parent.mkdir(parents=True, exist_ok=True)
        if not self.queue_meta_file.exists():
            self.queue_meta_file.write_text("{}", encoding="utf-8")

    def load_queue_from_file(self) -> List[BuildTask]:
        path = self.queue_file
        if not path.exists():
            self.build_queue = []
            self.queue_packages = []
            self.package_status = {}
            return []
        packages: List[str] = []
        status: Dict[str, bool] = {}
        legacy_meta: Dict[str, Dict[str, Any]] = {}
        with path.open("r", encoding="utf-8") as handle:
            for raw_line in handle:
                line = raw_line.strip()
                if not line:
                    continue
                parsed: Optional[Any] = None
                if line.startswith("{") and line.endswith("}"):
                    try:
                        parsed = json.loads(line)
                    except json.JSONDecodeError:
                        parsed = None
                completed = False
                name = ""
                if isinstance(parsed, dict) and parsed.get("name"):
                    name = str(parsed.get("name") or "").strip()
                    completed = bool(parsed.get("completed", False))
                    kind = str(parsed.get("kind", "debian"))
                    path_str = str(parsed.get("path") or "")
                    extra_raw = parsed.get("extra_args")
                    extra_list: List[str] = []
                    if isinstance(extra_raw, list):
                        extra_list = [str(item) for item in extra_raw]
                    elif extra_raw:
                        extra_list = [str(extra_raw)]
                    entry = legacy_meta.setdefault(name, {"path": path_str, "kinds": {}})
                    if path_str:
                        entry["path"] = path_str
                    kinds_dict = entry.setdefault("kinds", {})
                    if isinstance(kinds_dict, dict):
                        kinds_dict[kind] = {"extra_args": extra_list}
                else:
                    if line.endswith("#"):
                        completed = True
                        line = line[:-1].strip()
                    name = line.strip()
                if name:
                    name = Path(name).name
                if not name:
                    continue
                if name not in packages:
                    packages.append(name)
                status[name] = status.get(name, False) or completed

        meta: Dict[str, Dict[str, object]]
        try:
            meta_raw = self.queue_meta_file.read_text(encoding="utf-8")
            loaded = json.loads(meta_raw) if meta_raw.strip() else {}
            meta = loaded if isinstance(loaded, dict) else {}
        except Exception:
            meta = {}

        if meta:
            normalized_meta: Dict[str, Dict[str, Any]] = {}
            for key, value in meta.items():
                new_key = Path(str(key)).name
                if not new_key:
                    continue
                if not isinstance(value, dict):
                    continue
                if new_key in normalized_meta:
                    # merge kinds/path if duplicates
                    existing = normalized_meta[new_key]
                    existing_path = str(existing.get("path") or "")
                    incoming_path = str(value.get("path") or "")
                    if incoming_path and not existing_path:
                        existing["path"] = incoming_path
                    kinds_existing = existing.setdefault("kinds", {})
                    kinds_incoming = value.get("kinds")
                    if isinstance(kinds_existing, dict) and isinstance(kinds_incoming, dict):
                        kinds_existing.update(kinds_incoming)
                else:
                    normalized_meta[new_key] = value
            meta = normalized_meta

        if legacy_meta:
            for pkg, info in legacy_meta.items():
                existing = meta.get(pkg) if isinstance(meta.get(pkg), dict) else {}
                merged_path = ""
                if isinstance(existing, dict):
                    merged_path = str(existing.get("path") or "")
                info_path = str(info.get("path") or "")
                path_to_use = info_path or merged_path
                merged_kinds: Dict[str, Any] = {}
                if isinstance(existing, dict) and isinstance(existing.get("kinds"), dict):
                    merged_kinds.update(existing["kinds"])  # type: ignore[arg-type]
                if isinstance(info.get("kinds"), dict):
                    merged_kinds.update(info["kinds"])  # type: ignore[arg-type]
                meta[pkg] = {"path": path_to_use, "kinds": merged_kinds}

        tasks: List[BuildTask] = []
        for pkg in packages:
            info = meta.get(pkg, {})
            base_path_str = ""
            if isinstance(info, dict):
                base_path_str = str(info.get("path") or "")
                kinds_info = info.get("kinds") if isinstance(info.get("kinds"), dict) else {}
            else:
                kinds_info = {}
            if base_path_str:
                base_path = Path(base_path_str).expanduser()
            else:
                base_path = (self.code_dir / pkg).expanduser()
            try:
                base_path = base_path.resolve()
            except Exception:
                pass
            if not kinds_info:
                tasks.append(BuildTask(display_name=pkg, path=base_path, kind="debian", extra_args=[]))
                continue
            for kind, payload in kinds_info.items():
                extra: List[str] = []
                if isinstance(payload, dict):
                    raw_extra = payload.get("extra_args")
                    if isinstance(raw_extra, list):
                        extra = [str(item) for item in raw_extra]
                    elif raw_extra:
                        extra = [str(raw_extra)]
                tasks.append(BuildTask(display_name=pkg, path=base_path, kind=str(kind), extra_args=extra))

        self.queue_packages = packages
        self.package_status = status
        self.build_queue = tasks
        return tasks

    def save_queue(self, tasks: Optional[List[BuildTask]] = None) -> None:
        tasks = list(tasks if tasks is not None else self.build_queue)
        unique: List[BuildTask] = []
        seen = set()
        for task in tasks:
            package_name = task.path.name
            task.display_name = package_name
            key = (package_name, task.kind)
            if key in seen:
                continue
            seen.add(key)
            unique.append(task)
        tasks = unique
        package_order = list(self.queue_packages)
        for task in tasks:
            package_name = task.path.name
            if package_name not in package_order:
                package_order.append(package_name)
        # Remove packages without tasks
        package_order = [pkg for pkg in package_order if any(t.path.name == pkg for t in tasks)]
        status = {pkg: self.package_status.get(pkg, False) for pkg in package_order}
        self.queue_packages = package_order
        self.package_status = status
        self.build_queue = []
        for task in tasks:
            task.display_name = task.path.name
            self.build_queue.append(task)
        self._write_queue_file()
        self._write_meta_from_tasks(tasks)

    def append_task_to_queue(self, task: BuildTask) -> None:
        self.add_tasks([task])

    def clear_queue(self) -> None:
        self.ensure_queue_file()
        self.queue_file.write_text("", encoding="utf-8")
        self.queue_meta_file.write_text("{}", encoding="utf-8")
        self.queue_packages = []
        self.package_status = {}
        self.build_queue = []

    def add_tasks(self, tasks: Sequence[BuildTask], *, reset_completed: bool = True) -> Tuple[int, int]:
        if not tasks:
            return (0, 0)
        added = 0
        total = 0
        for task in tasks:
            total += 1
            package_name = task.path.name
            task.display_name = package_name
            replaced = False
            for existing in self.build_queue:
                if existing.path.name == package_name and existing.kind == task.kind:
                    existing.path = task.path
                    existing.extra_args = list(task.extra_args)
                    existing.display_name = package_name
                    replaced = True
                    break
            if not replaced:
                self.build_queue.append(task)
                added += 1
            if package_name not in self.queue_packages:
                self.queue_packages.append(package_name)
            if not replaced:
                self.package_status[package_name] = False
            else:
                if reset_completed or package_name not in self.package_status:
                    self.package_status[package_name] = False
            self.package_status.setdefault(package_name, False)
        self.save_queue()
        return added, total

    def _write_queue_file(self) -> None:
        self.ensure_queue_file()
        with self.queue_file.open("w", encoding="utf-8") as handle:
            for pkg in self.queue_packages:
                suffix = "#" if self.package_status.get(pkg) else ""
                handle.write(f"{pkg}{suffix}\n")

    def _write_meta_from_tasks(self, tasks: List[BuildTask]) -> None:
        meta: Dict[str, Dict[str, object]] = {}
        for task in tasks:
            entry = meta.setdefault(task.display_name, {"path": str(task.path), "kinds": {}})
            entry["path"] = str(task.path)
            kinds = entry.setdefault("kinds", {})
            if isinstance(kinds, dict):
                kinds[task.kind] = {"extra_args": list(task.extra_args)}
        self.queue_meta_file.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")

    def summary_rows(self) -> List[Tuple[str, str]]:
        
        return [
            ("操作系统", f"名称:{self.os_name} 版本:{self.os_version} 版本代号:{self.os_codename} 架构:{self.os_arch}"),
            ("Python 版本", self.python_version),
            ("distribution.yaml URL", self.distribution_url),
            ("distribution.yaml 本地目录", f"预先存放在源码code目录下,将不从URL下载"),
            ("Release 仓库目录", str(self.release_dir)),
            ("安装前缀目录", str(self.install_prefix)),
            ("源码code目录", str(self.code_dir)),
            ("ROS2 发行版", self.ros2_distro),
            ("AGIROS 发行版", self.agiros_distro),
            ("Ubuntu 代号", f"目标:{self.ubuntu_version}  当前版本代号:{self.os_codename}" if self.os_name == 'Ubuntu' else f"{self.ubuntu_version}"), #如果当前系统是ubuntu，则显示ubuntu版本
            ("openEuler 默认", f"目标:{self.openeuler_default}  当前版本代号:{self.os_codename}" if self.os_name == 'openEuler' else f"{self.openeuler_default}"), #如果当前系统是openEuler，则显示openEuler默认版本
            ("openEuler 回退", ", ".join(self.openeuler_fallback) or "-"),
            ("bloom 命令", self.bloom_bin),
            ("批量生成 gbp.conf", "启用" if self.auto_generate_gbp else "关闭"),
            ("Debian 构建参数", " ".join(self.debian_build_args)),
            ("RPM 构建命令", f"{self.rpm_build_base} {' '.join(self.rpm_build_args)}".strip()),
            ("Debian 输出目录", str(self.deb_out_dir)),
            ("Debian 发行版", self.deb_distro),
            ("Debian release_inc", self.deb_release_inc),
            ("并行构建线程", self.deb_parallel),
            ("COLCON_SRC_DIR", self.colcon_src_dir or "(auto)"),
            ("AGIROS APT 源", self.agiros_apt_source or "(未设置)"),
            ("AGIROS APT 源文件", str(self.agiros_apt_source_file)),
            ("自动修复依赖", "启用" if self.auto_fix_deps else "关闭"),
            ("Git User", f"{self.git_user_name} <{self.git_user_email}>"),
            ("队列文件", str(self.queue_file)),
            ("队列元数据", str(self.queue_meta_file)),
            ("构建包数量", f"{len(self.queue_packages)} 项"),
        ]


def render_state_panel(state: MenuState) -> None:
    state.refresh_from_env()
    table = Table.grid(expand=False)
    table.add_column(justify="right", style="cyan", no_wrap=True)
    table.add_column(style="white", overflow="fold")
    for key, value in state.summary_rows():
        table.add_row(key, value)
    console.print(Panel(table, title="AGIROS 工具菜单", box=box.ROUNDED))


def ensure_directory(path: Path) -> None:
    if not path.exists():
        path.mkdir(parents=True, exist_ok=True)


def handle_download_release(state: MenuState) -> None:
    url = ask_text("distribution.yaml URL", state.distribution_url)
    if not url:
        console.print("[yellow]已取消：缺少 URL[/]")
        return
    
    
    _code_or_tracks = ask_confirm("直接下载code? (No=tracks)", default=True)
    if _code_or_tracks:
        code_or_tracks = "code"
        dir = str(state.code_dir)
        target = ask_text("源码(code)仓库存放目录", dir)
    else:
        code_or_tracks = "tracks"
        dir = str(state.release_dir)
        target = ask_text("Release(tracks)仓库存放目录", dir)
        
    if not target:
        console.print("[yellow]已取消：缺少目录[/]")
        return
    target_path = Path(target).expanduser().resolve()
    ensure_directory(target_path)

    console.print(Panel(Text("开始下载 Release 仓库", style="bold"), subtitle=str(target_path), box=box.ROUNDED))
    import yaml_git_downloader_release as downloader

    downloader.TARGET_DIR = str(target_path)
    downloader.LOG_FILE = os.path.join(downloader.TARGET_DIR, "download_log.txt")
    try:
        downloader.download_repos_from_distribution_yaml(url, str(target_path), code_or_tracks)
    except Exception as exc:
        console.print(f"[bold red]下载失败: {exc}[/]")
        return

    state.distribution_url = url
    state.release_dir = target_path
    state.update_env()


def handle_tracks_download(state: MenuState) -> None:
    release_dir = ask_text("Release 仓库目录", str(state.release_dir))
    code_dir = ask_text("源码目录", str(state.code_dir))
    distro = ask_text("Tracks ROS2 发行版名称", state.ros2_distro)
    resume = ask_confirm("启用断点续传 (resume)?", default=True)
    limit_raw = ask_text("限制下载包数量 (留空则全部下载)", "")
    args = [
        f"--release-dir={Path(release_dir).expanduser().resolve()}",
        f"--code-dir={Path(code_dir).expanduser().resolve()}",
        f"--distro={distro}",
    ]
    if resume:
        args.append("--resume")
    if limit_raw:
        try:
            int(limit_raw)
        except ValueError:
            console.print("[yellow]limit 必须是数字，忽略该参数[/]")
        else:
            args.extend(["--limit", limit_raw])

    console.print(Panel(Text("处理 tracks.yaml，下载源码", style="bold magenta"), subtitle="oob_tracks_to_sources.py", box=box.ROUNDED))
    import oob_tracks_to_sources as tracks_downloader

    exit_code = tracks_downloader.main(args)
    if exit_code != 0:
        console.print(f"[bold red]处理失败，退出码 {exit_code}[/]")
        return
    state.release_dir = Path(release_dir).expanduser().resolve()
    state.code_dir = Path(code_dir).expanduser().resolve()
    state.ros2_distro = distro
    state.update_env()


def list_code_packages(code_dir: Path) -> List[Path]:
    if not code_dir.exists():
        return []

    depth_limited_packages: List[Path] = []
    for root, dirs, files in os.walk(code_dir):
        rel_parts = Path(root).relative_to(code_dir).parts
        if len(rel_parts) > 2:
            dirs[:] = []
            continue
        if "package.xml" in files:
            depth_limited_packages.append(Path(root))
            dirs[:] = []

    if depth_limited_packages:
        return sorted(set(depth_limited_packages), key=lambda p: str(p))

    return [p for p in sorted(code_dir.iterdir()) if p.is_dir()]


def detect_linux_distribution() -> Optional[str]:
    os_id = ""
    try:
        info = platform.freedesktop_os_release()  # type: ignore[attr-defined]
    except Exception:
        info = {}
    if isinstance(info, dict):
        os_id = str(info.get("ID", "")).lower()
    if not os_id:
        os_release = Path("/etc/os-release")
        if os_release.exists():
            try:
                for line in os_release.read_text(encoding="utf-8", errors="ignore").splitlines():
                    if line.startswith("ID="):
                        os_id = line.partition("=")[2].strip().strip('"').lower()
                        break
            except Exception:
                os_id = ""
    if "ubuntu" in os_id:
        return "ubuntu"
    if "openeuler" in os_id:
        return "openeuler"
    return None


def prompt_package_path(state: MenuState) -> Optional[Path]:
    packages = list_code_packages(state.code_dir)
    if not packages:
        console.print("[yellow]未在源码目录中发现包，建议手动输入路径。[/]")

    def _resolve_package(choice_name: str) -> Optional[Path]:
        for pkg in packages:
            if to_display_name(state, pkg) == choice_name:
                return pkg
        return None

    while True:
        choice = ask_select("选择源码包目录", ["关键字查询", "手动输入", "返回"])
        if choice in (None, "返回"):
            return None
        if choice == "手动输入":
            custom = ask_text("请输入源码包路径", "")
            if not custom:
                continue
            return Path(custom).expanduser().resolve()
        if choice == "关键字查询":
            keyword = ask_text("请输入匹配关键字", "")
            if not keyword:
                console.print("[yellow]未输入关键字。[/]")
                continue
            keyword_lower = keyword.lower()
            matches: List[Tuple[str, Path]] = []
            for pkg in packages:
                display = to_display_name(state, pkg)
                if keyword_lower in display.lower():
                    matches.append((display, pkg))
            if not matches:
                console.print(f"[yellow]未找到匹配 \"{keyword}\" 的源码包。[/]")
                continue
            display_choices = [name for name, _ in matches] + ["重新搜索", "返回"]
            selection = ask_select("匹配的源码包", display_choices)
            if selection in (None, "返回"):
                continue
            if selection == "重新搜索":
                continue
            pkg_path = _resolve_package(selection)
            if pkg_path:
                return pkg_path
            console.print("[red]选择的包无法解析，请重试。[/]")


def _packages_for_kind(state: MenuState, target_kind: str) -> List[str]:
    return [
        pkg
        for pkg in state.queue_packages
        if any(t.display_name == pkg and t.kind == target_kind for t in state.build_queue)
    ]


def review_completed_packages(state: MenuState, target_kind: str) -> None:
    packages = _packages_for_kind(state, target_kind)
    if not packages:
        return
    completed = [pkg for pkg in packages if state.package_status.get(pkg)]
    pending = [pkg for pkg in packages if not state.package_status.get(pkg)]
    if completed:
        console.print("[cyan]已标记完成的包：[/] " + ", ".join(completed))
    if pending and ask_confirm("是否需要在开始前标记更多已完成的包 (#)?", default=False):
        auto_marked = auto_mark_prebuilt_packages(state.queue_file, state.code_dir)
        if auto_marked:
            state.load_queue_from_file()
            console.print(f"[green]已自动标记完成: {', '.join(auto_marked)}[/]")
        else:
            console.print("[yellow]未发现已构建完成但未标记的包。[/]")
    if completed and ask_confirm("是否需要移除某些包的完成标记 (#)?", default=False):
        unselect = ask_checkbox("选择需要取消 # 的包", completed)
        if unselect:
            for pkg in unselect:
                state.package_status[pkg] = False
            state.save_queue()
            console.print(f"[yellow]已取消 # 标记: {', '.join(unselect)}[/]")


def optimize_debian_build_queue(state: MenuState) -> bool:
    """Reorder Debian tasks using dependency-aware topological sort."""
    state.load_queue_from_file()
    # 先让用户同步已完成的包（打 #），避免重复构建
    review_completed_packages(state, "debian")
    state.load_queue_from_file()

    debian_packages = [
        task.display_name
        for task in state.build_queue
        if task.kind == "debian"
    ]
    if not debian_packages:
        console.print("[yellow]当前队列中没有 Debian 构建任务，无法优化排序。[/]")
        return False

    pending_packages = [
        pkg for pkg in state.queue_packages
        if not state.package_status.get(pkg)
        and any(t.display_name == pkg and t.kind == "debian" for t in state.build_queue)
    ]
    if not pending_packages:
        console.print("[cyan]没有待构建的 Debian 包，已全部标记为完成。[/]")
        return True

    existing_entries = [
        (task.display_name, task.path)
        for task in state.build_queue
        if task.kind == "debian"
    ]
    package_dirs = discover_debian_package_dirs(state.code_dir, existing_entries)
    if not package_dirs:
        console.print("[red]未找到任何包含 debian/control 的源码包，无法构建依赖图。[/]")
        return False
    order_hint = {pkg: idx for idx, pkg in enumerate(state.queue_packages)}
    try:
        series, unresolved = compute_series_toposort(
            package_dirs, pending_packages, order_hint=order_hint
        )
    except KeyError as exc:
        console.print(f"[red]依赖分析失败: {exc}[/]")
        return False
    except ValueError as exc:
        console.print(f"[red]检测到依赖环，无法优化排序: {exc}[/]")
        return False
    if unresolved:
        console.print("[yellow]发现未在源码目录中找到的依赖（未加入队列）：[/] " + ", ".join(sorted(unresolved)))
        console.print("[yellow]请确认这些依赖是否存在于本地源码目录，并确保 debian/control 可被扫描。[/]")

    # 拆成多个弱连通系列，组件间可并行；组件内保持拓扑序
    optimized_order: List[str] = []
    for idx, comp in enumerate(series, start=1):
        console.print(f"[cyan]系列 {idx}（{len(comp)} 个包，可并行于其他系列）：[/] " + ", ".join(comp))
        for pkg in comp:
            if pkg not in optimized_order:
                optimized_order.append(pkg)

    # 已完成的包保持在前（保持原顺序），未完成的按拓扑排序，其余尾部保持
    completed_prefix = [pkg for pkg in state.queue_packages if state.package_status.get(pkg)]
    seen: Set[str] = set(optimized_order)
    tail = [pkg for pkg in state.queue_packages if pkg not in seen and pkg not in completed_prefix]
    state.queue_packages = completed_prefix + optimized_order + tail

    existing_task_keys = {(task.display_name, task.kind) for task in state.build_queue}
    added_packages: List[str] = []
    for pkg in optimized_order:
        key = (pkg, "debian")
        if key in existing_task_keys:
            continue
        pkg_path = package_dirs.get(pkg)
        if not pkg_path:
            console.print(f"[yellow]跳过缺少路径的依赖包 {pkg}[/]")
            continue
        state.build_queue.append(BuildTask(pkg, pkg_path, "debian", []))
        existing_task_keys.add(key)
        added_packages.append(pkg)

    for pkg in optimized_order:
        state.package_status.setdefault(pkg, False)
    order_map = {pkg: idx for idx, pkg in enumerate(state.queue_packages)}
    state.build_queue.sort(
        key=lambda task: (
            order_map.get(task.display_name, len(order_map)),
            0 if task.kind == "debian" else 1,
        )
    )
    state.save_queue()
    if added_packages:
        console.print(f"[green]已将依赖包加入队列: {', '.join(added_packages)}[/]")
    console.print(f"[green]Debian 构建队列已按依赖拓扑排序并写入: {state.queue_file}[/]")
    return True

def ros2agiros_menu(state: MenuState) -> None:
    while True:
        scope = ask_select("请选择操作范围", ["单包", f"批量:{state.code_dir}", "返回"])
        if scope in (None, "返回"):
            return
        if scope == "单包":
            pkg_path = prompt_package_path(state)
            if not pkg_path:
                continue
            change_ros2agiros_tag(pkg_path, from_str=state.ros2_distro, to_str=state.agiros_distro)
        else:
            change_ros2agiros_tag(state.code_dir, from_str=state.ros2_distro, to_str=state.agiros_distro)
        

def build_bloom_command(state: MenuState, kind: str) -> List[str]:
    base = shlex_split(state.bloom_bin)
    if not base:
        base = ["bloom-generate"]
    text = " ".join(base)
    if "generate_cmd" in text or text.endswith("agirosdebian") or text.endswith("agirosrpm"):
        return base
    if kind == "debian":
        return base + ["agirosdebian"]
    return base + ["agirosrpm"]


def run_single_bloom(state: MenuState, kind: str, package_path: Path, generate_gbp: bool = False) -> None:
    ensure_directory(package_path)
    cmd = build_bloom_command(state, "debian" if kind in {"debian", "gbp"} else "rpm")
    if "agirosdebian" not in cmd and kind in {"debian", "gbp"} and "generate_cmd" not in " ".join(cmd):
        cmd.append("agirosdebian")
    if kind == "rpm" and "agirosrpm" not in cmd and "generate_cmd" not in " ".join(cmd):
        cmd.append("agirosrpm")
    if kind in {"debian", "gbp"}:
        cmd += ["--ros-distro", state.agiros_distro, "--os-name", "ubuntu", "--os-version", state.ubuntu_version]
        if generate_gbp:
            cmd.append("--generate-gbp")
            cmd += ["--tracks-distro", state.ros2_distro]
            cmd += ["--distro", state.agiros_distro]
            cmd += ["--pkg", package_path.name]
    else:
        cmd += ["--ros-distro", state.agiros_distro, "--os-name", "openeuler", "--os-version", state.openeuler_default]
    env = state.build_env()
    if generate_gbp:
        env["OOB_TRACKS_DIR"] = str(state.release_dir)
        env["OOB_TRACKS_DISTRO"] = state.ros2_distro
        env["AGIROS_DISTRO"] = state.agiros_distro
    rc = run_stream(cmd, cwd=package_path, env=env)
    if rc == 0:
        console.print("[green]完成[/]")


def run_batch_bloom(state: MenuState, mode: str) -> None:
    script = REPO_ROOT / "oob_builder_procedural.py"
    if not script.exists():
        console.print(f"[bold red]未找到 {script}[/]")
        return
    limit_raw = ask_text("限制处理包数量 (留空=全部)", "")
    dry_run = ask_confirm("启用 dry-run?", default=False)
    cmd: List[str] = [
        sys.executable,
        str(script),
        "--release-dir",
        str(state.release_dir),
        "--code-dir",
        str(state.code_dir),
        "--ros-distro",
        state.agiros_distro,
        "--ubuntu-default",
        state.ubuntu_version,
        "--openeuler-default",
        state.openeuler_default,
        "--mode",
        mode,
    ]
    if state.openeuler_fallback:
        cmd.append("--openeuler-fallback")
        cmd.extend(state.openeuler_fallback)
    cmd.extend(["--bloom-bin", state.bloom_bin])
    if limit_raw:
        try:
            int(limit_raw)
        except ValueError:
            console.print("[yellow]limit 必须是数字，忽略该参数[/]")
        else:
            cmd.extend(["--limit", limit_raw])
    if dry_run:
        cmd.append("--dry-run")
    if mode != "gbp" and state.auto_generate_gbp:
        cmd.append("--generate-gbp")
    run_stream(cmd, cwd=REPO_ROOT, env=state.build_env())


def bloom_menu(state: MenuState) -> None:
    while True:
        choice = ask_select("Bloom 打包", ["生成 Debian 目录", "生成 spec 文件", "生成 debian+spec", "生成 gbp.conf", "返回"])
        if choice in (None, "返回"):
            return
        scope = ask_select("请选择操作范围", ["单包", "批量", "返回"])
        if scope in (None, "返回"):
            continue
        generate_gbp = state.auto_generate_gbp or (choice in {"生成 Debian 目录", "生成 debian+spec", "生成 gbp.conf"} and ask_confirm("生成 gbp.conf?", default=choice != "生成 spec 文件"))
        if scope == "单包":
            pkg_path = prompt_package_path(state)
            if not pkg_path:
                continue
            if choice == "生成 Debian 目录":
                run_single_bloom(state, "debian", pkg_path, generate_gbp)
                if ask_confirm("将 Debian 构建加入队列?", default=False):
                    state.append_task_to_queue(BuildTask(pkg_path.name, pkg_path, "debian"))
            elif choice == "生成 spec 文件":
                run_single_bloom(state, "rpm", pkg_path)
                if ask_confirm("将 RPM 构建加入队列?", default=False):
                    state.append_task_to_queue(BuildTask(pkg_path.name, pkg_path, "rpm"))
            elif choice == "生成 debian+spec":
                run_single_bloom(state, "debian", pkg_path, generate_gbp)
                run_single_bloom(state, "rpm", pkg_path)
                if ask_confirm("将 Debian 构建加入队列?", default=False):
                    state.append_task_to_queue(BuildTask(pkg_path.name, pkg_path, "debian"))
                if ask_confirm("将 RPM 构建加入队列?", default=False):
                    state.append_task_to_queue(BuildTask(pkg_path.name, pkg_path, "rpm"))
            else:
                run_single_bloom(state, "gbp", pkg_path, True)
                if ask_confirm("将 Debian 构建加入队列?", default=False):
                    state.append_task_to_queue(BuildTask(pkg_path.name, pkg_path, "debian"))
        else:
            mode = {
                "生成 Debian 目录": "debian",
                "生成 spec 文件": "spec",
                "生成 debian+spec": "both",
                "生成 gbp.conf": "gbp",
            }[choice]
            run_batch_bloom(state, mode)


def describe_build_task(task: BuildTask, state: MenuState) -> str:
    alias = state.code_label
    pretty_path = f"{alias}/{task.display_name}"
    return f"{task.display_name} ({task.kind}) - {pretty_path}"


def run_debuild_install(state: MenuState, path: Path) -> int:
    script = REPO_ROOT / "deb_install_any.py"
    if not script.exists():
        console.print(f"[bold red]未找到 {script}[/]")
        return 1
    cmd = [sys.executable, str(script), "--work-dir", str(path)]
    return run_stream(cmd, cwd=path, env=state.build_env())


def apply_deb_build_options(env: Dict[str, str], parallel_hint: Optional[str], run_tests: bool) -> None:
    """Ensure DEB_BUILD_OPTIONS keeps parallel hint and adds nocheck when skipping tests."""
    options = [item for item in env.get("DEB_BUILD_OPTIONS", "").split() if item]
    if parallel_hint:
        parallel_token = f"parallel={parallel_hint}"
        if not any(opt.startswith("parallel=") for opt in options):
            options.append(parallel_token)
    if not run_tests and "nocheck" not in options:
        options.append("nocheck")
    if options:
        env["DEB_BUILD_OPTIONS"] = " ".join(options)


def yaml_quote(value: str) -> str:
    return json.dumps(value, ensure_ascii=True)


def resolve_colcon_src_dir(state: "MenuState") -> str:
    if state.colcon_src_dir:
        candidate = state.code_dir / state.colcon_src_dir
        if candidate.exists():
            return state.colcon_src_dir
    return "src" if (state.code_dir / "src").exists() else "."


def docker_socket_available() -> bool:
    if os.environ.get("DOCKER_HOST"):
        return True
    sock = Path("/var/run/docker.sock")
    try:
        return sock.exists() and sock.is_socket()
    except Exception:
        return sock.exists()


def ensure_cargo_bin_on_path(env: Dict[str, str]) -> None:
    cargo_bin = str(Path.home() / ".cargo" / "bin")
    path_value = env.get("PATH", "")
    if cargo_bin and cargo_bin not in path_value.split(":"):
        env["PATH"] = f"{cargo_bin}:{path_value}" if path_value else cargo_bin


def write_colcon_deb_config(
    state: MenuState,
    config_path: Path,
    docker_image: str,
    parallel_jobs: int,
) -> None:
    debian_dirs = state.code_dir / "debian_dirs"
    lines = [
        f"colcon_repo: {yaml_quote(str(state.code_dir))}",
        f"debian_dirs: {yaml_quote(str(debian_dirs))}",
        "docker:",
        f"  image: {yaml_quote(docker_image)}",
        f"ros_distro: {yaml_quote(state.deb_distro)}",
        f"output_dir: {yaml_quote(str(state.deb_out_dir))}",
        f"parallel_jobs: {parallel_jobs}",
        "",
    ]
    config_path.write_text("\n".join(lines), encoding="utf-8")


def resolve_colcon_deb_runner(tool_root: Path) -> Optional[List[str]]:
    env_bin = os.environ.get("COLCON_DEB_BIN")
    if env_bin:
        return [env_bin]
    for candidate in (
        tool_root / "target" / "release" / "colcon-deb",
        tool_root / "target" / "debug" / "colcon-deb",
    ):
        if candidate.exists():
            return [str(candidate)]
    if shutil.which("colcon-deb"):
        return ["colcon-deb"]
    if shutil.which("cargo"):
        manifest = tool_root / "Cargo.toml"
        if manifest.exists():
            return ["cargo", "run", "--bin", "colcon-deb", "--manifest-path", str(manifest), "--"]
    return None


def auto_build_colcon_deb(state: MenuState, tool_root: Path) -> bool:
    if not shutil.which("cargo"):
        console.print("[red]未找到 cargo，无法自动构建 colcon-deb。请安装 Rust 或设置 COLCON_DEB_BIN。[/]")
        return False
    manifest = tool_root / "Cargo.toml"
    if not manifest.exists():
        console.print(f"[red]未找到 {manifest}，无法自动构建。[/]")
        return False
    console.print("[cyan]正在自动构建 colcon-deb...[/]")
    env = state.build_env()
    rc = run_stream(
        ["cargo", "build", "--release", "--bin", "colcon-deb", "--manifest-path", str(manifest)],
        cwd=tool_root,
        env=env,
    )
    return rc == 0


def run_colcon_deb_fallback_build(state: MenuState) -> int:
    tool_root = REPO_ROOT / "colcon-debian-packager"
    orchestrator = tool_root / "scripts" / "helpers" / "build-orchestrator.rs"
    if not orchestrator.exists():
        console.print(f"[red]未找到 build-orchestrator: {orchestrator}[/]")
        return 1
    env = state.build_env()
    ensure_cargo_bin_on_path(env)
    if not shutil.which("rust-script"):
        if shutil.which("cargo") and ask_confirm("未找到 rust-script，是否使用 cargo 安装?", default=True):
            rc = run_stream(
                ["cargo", "install", "rust-script", "--version", "0.35.0"],
                cwd=tool_root,
                env=env,
            )
            if rc != 0:
                return rc
        else:
            console.print("[red]未找到 rust-script，无法执行 build-orchestrator。[/]")
            return 1
    env.setdefault("ROS_DISTRO", state.deb_distro)
    env.setdefault("PARALLEL_JOBS", str(state.deb_parallel))
    src_dir = resolve_colcon_src_dir(state)
    env["COLCON_SRC_DIR"] = src_dir
    env.setdefault("COLCON_HELPERS_DIR", str(tool_root / "scripts" / "helpers"))
    env.setdefault("COLCON_SCRIPTS_DIR", str(tool_root / "scripts"))
    cmd = [
        "rust-script",
        str(orchestrator),
        "--workspace",
        str(state.code_dir),
        "--src-dir",
        src_dir,
        "--queue-file",
        str(state.queue_file),
        "--debian-dirs",
        str(state.code_dir / "debian_dirs"),
        "--output-dir",
        str(state.deb_out_dir),
    ]
    console.print("[yellow]未检测到 Docker daemon，使用本地 build-orchestrator 执行并行构建。[/]")
    return run_stream(cmd, cwd=tool_root, env=env)


def run_colcon_deb_parallel_build(state: MenuState) -> int:
    tool_root = REPO_ROOT / "colcon-debian-packager"
    if not tool_root.exists():
        console.print(f"[red]未找到 colcon-debian-packager: {tool_root}[/]")
        return 1
    if not state.code_dir.exists():
        console.print(f"[red]源码目录不存在: {state.code_dir}[/]")
        return 1
    if not docker_socket_available() or os.environ.get("COLCON_DEB_FORCE_FALLBACK") == "1":
        return run_colcon_deb_fallback_build(state)
    config_path = REPO_ROOT / "colcon-deb.build-tools.yaml"
    docker_image = os.environ.get("COLCON_DEB_IMAGE", f"colcon-deb:{state.deb_distro}")
    try:
        parallel_jobs = int(state.deb_parallel)
    except ValueError:
        parallel_jobs = os.cpu_count() or 4
    write_colcon_deb_config(state, config_path, docker_image, parallel_jobs)
    runner = resolve_colcon_deb_runner(tool_root)
    if not runner:
        if not auto_build_colcon_deb(state, tool_root):
            return 1
        runner = resolve_colcon_deb_runner(tool_root)
        if not runner:
            console.print("[red]自动构建完成但仍未找到 colcon-deb，可设置 COLCON_DEB_BIN。[/]")
            return 1
    cmd = runner + ["-c", str(config_path), "build", "--output", str(state.deb_out_dir)]
    if parallel_jobs:
        cmd += ["-j", str(parallel_jobs)]
    cmd += ["--agiros-distro", state.deb_distro]
    env = state.build_env()
    env.setdefault("ROS_DISTRO", state.deb_distro)
    env["COLCON_SRC_DIR"] = resolve_colcon_src_dir(state)
    return run_stream(cmd, cwd=tool_root, env=env)


def run_debian_build(
    state: MenuState,
    path: Path,
    extra_args: Optional[List[str]] = None,
    builder: str = "gbp",
    auto_install: bool = False,
    run_tests: bool = True,
) -> int:
    if builder == "debuild":
        script = REPO_ROOT / "debuild_runner.py"
        if not script.exists():
            console.print(f"[bold red]未找到 {script}[/]")
            return 1
        env = state.build_env()
        env.setdefault("WORK_DIR", str(path))
        env.setdefault("PARALLEL", state.deb_parallel)
        apply_deb_build_options(env, env.get("PARALLEL", state.deb_parallel), run_tests)
        cmd: List[str] = [sys.executable, str(script), "--work-dir", str(path)]
        if extra_args:
            cmd.append("--")
            cmd.extend(extra_args)
        rc = run_stream(cmd, cwd=path, env=env)
        if rc == 0 and auto_install:
            install_rc = run_debuild_install(state, path)
            if install_rc != 0:
                console.print("[red]安装失败[/]")
                return install_rc
        return rc

    script = REPO_ROOT / "git_build_any.py"
    env = state.build_env()
    if script.exists():
        env.setdefault("WORK_DIR", str(path))
        env.setdefault("CODE_DIR", str(state.code_dir))
        env.setdefault("DEB_OUT", str(state.deb_out_dir))
        env.setdefault("DISTRO", state.deb_distro)
        env.setdefault("DEFAULT_REL_INC", state.deb_release_inc)
        env.setdefault("PARALLEL", state.deb_parallel)
        env.setdefault("GIT_USER_NAME", state.git_user_name)
        env.setdefault("GIT_USER_EMAIL", state.git_user_email)
        apply_deb_build_options(env, env.get("PARALLEL", state.deb_parallel), run_tests)
        while True:
            rc = run_stream([sys.executable, str(script)], cwd=path, env=env)
            if rc == 0:
                return 0

            action = ask_select("Debian 构建失败，接下来如何操作？", ["输入命令后重试", "退出构建"])
            if action != "输入命令后重试":
                return rc

            user_cmd = ask_text("请输入需要执行的命令（将在包目录下运行）", "")
            if not user_cmd:
                console.print("[yellow]未输入命令，继续尝试构建。[/]")
                continue
            run_stream(["bash", "-lc", user_cmd], cwd=path, env=env)
        # unreachable

    cmd = ["gbp", "buildpackage"] + state.debian_build_args
    if extra_args:
        cmd += extra_args
    env = state.build_env()
    apply_deb_build_options(env, state.deb_parallel, run_tests)
    return run_stream(cmd, cwd=path, env=env)


def run_rpm_build(
    state: MenuState,
    path: Path,
    extra_args: Optional[List[str]] = None,
    run_tests: bool = True,
) -> int:
    script = REPO_ROOT / "rpmbuild_any.py"
    env = state.build_env()
    if not run_tests:
        env["AGIROS_SKIP_TESTS"] = "1"
    if script.exists():
        env.setdefault("WORK_DIR", str(path))
        env.setdefault("CODE_DIR", str(state.code_dir))
        while True:
            rc = run_stream([sys.executable, str(script)], cwd=path, env=env)
            if rc == 0:
                return 0

            action = ask_select("RPM 构建失败，接下来如何操作？", ["输入命令后重试", "退出构建"])
            if action != "输入命令后重试":
                return rc

            user_cmd = ask_text("请输入需要执行的命令（将会在包目录下运行）", "")
            if not user_cmd:
                console.print("[yellow]未输入命令，继续尝试构建。[/]")
                continue
            run_stream(["bash", "-lc", user_cmd], cwd=path, env=env)
        # unreachable

    rpm_dir = path / "rpm"
    specs = sorted(rpm_dir.glob("*.spec")) if rpm_dir.exists() else []
    if not specs:
        console.print(f"[yellow]{path} 未找到 rpm/*.spec[/]")
        return 1
    rc = 0
    for spec in specs:
        cmd = [state.rpm_build_base] + state.rpm_build_args
        if not run_tests and "--nocheck" not in cmd:
            cmd.append("--nocheck")
        cmd.append(str(spec))
        if extra_args:
            cmd += extra_args
        rc = run_stream(cmd, cwd=path, env=env)
        if rc != 0:
            break
    return rc


def execute_build(
    task: BuildTask,
    state: MenuState,
    debian_builder: str = "gbp",
    auto_install: bool = False,
    run_tests: bool = True,
) -> bool:
    console.print(Panel(f"开始构建: {describe_build_task(task, state)}", box=box.ROUNDED))
    success = True
    if task.kind == "debian":
        if run_debian_build(
            state,
            task.path,
            task.extra_args,
            builder=debian_builder,
            auto_install=auto_install,
            run_tests=run_tests,
        ) != 0:
            success = False
    elif task.kind == "rpm":
        if run_rpm_build(state, task.path, task.extra_args, run_tests=run_tests) != 0:
            success = False
    else:
        console.print(f"[red]未知的构建类型: {task.kind}[/]")
        success = False
    console.print("[green]构建完成[/]" if success else "[red]构建失败[/]")
    return success


def manage_build_queue(state: MenuState) -> None:
    while True:
        state.load_queue_from_file()
        options = [
            "查看队列",
            "添加任务",
            "执行队列",
            "清空队列",
            "编辑构建参数",
            "返回",
        ]
        choice = ask_select("构建菜单", options)
        if choice in (None, "返回"):
            return
        if choice == "查看队列":
            if not state.queue_packages:
                console.print("[cyan]队列为空[/]")
                if ask_confirm("是否扫描并生成构建列表?", default=True):
                    handle_scan_and_generate(state)
                    state.load_queue_from_file()
                    continue
            for idx, pkg in enumerate(state.queue_packages, start=1):
                kinds = [task.kind for task in state.build_queue if task.display_name == pkg]
                kinds_text = ", ".join(sorted(set(kinds))) if kinds else "-"
                mark = " #" if state.package_status.get(pkg) else ""
                console.print(f"{idx}. {pkg}{mark} ({kinds_text})")
            if state.queue_packages and ask_confirm("移除包?", default=False):
                idx_raw = ask_text("输入要移除的编号", "")
                if idx_raw and idx_raw.isdigit():
                    idx = int(idx_raw) - 1
                    if 0 <= idx < len(state.queue_packages):
                        removed_pkg = state.queue_packages.pop(idx)
                        state.package_status.pop(removed_pkg, None)
                        state.build_queue = [task for task in state.build_queue if task.display_name != removed_pkg]
                        state.save_queue()
                        console.print(f"[yellow]已移除 {removed_pkg}[/]")
        elif choice == "添加任务":
            mode = ask_select("请选择添加方式", ["手动选择源码包", "扫描并生成构建列表", "返回"])
            if mode in (None, "返回"):
                continue
            if mode == "扫描并生成构建列表":
                handle_scan_and_generate(state)
                continue
            pkg_path = prompt_package_path(state)
            if not pkg_path:
                continue
            kind = ask_select("构建类型", ["debian", "rpm"])
            if not kind:
                continue
            task = BuildTask(pkg_path.name, pkg_path, kind)
            state.append_task_to_queue(task)
        elif choice == "执行队列":
            if not state.queue_packages:
                console.print("[cyan]队列为空[/]")
                continue
            build_choice = ask_select("选择构建类型", ["构建 Debian 包", "构建 RPM 包"])
            if build_choice is None:
                continue
            target_kind = "debian" if build_choice == "构建 Debian 包" else "rpm"
            run_tests = True
            review_completed_packages(state, target_kind)
            auto_install_deb = False
            if target_kind == "debian":
                mode = ask_select("选择 Debian 编译方式", ["单线程编译", "并行编译", "返回"])
                if mode in (None, "返回"):
                    continue
                pending = [
                    pkg
                    for pkg in state.queue_packages
                    if not state.package_status.get(pkg)
                    and any(t.display_name == pkg and t.kind == "debian" for t in state.build_queue)
                ]
                if mode == "并行编译":
                    if not pending:
                        console.print("[cyan]没有待构建的 Debian 包，可通过扫描/添加任务生成。[/]")
                        continue
                    console.print("[cyan]并行编译将使用 colcon-debian-packager 构建整个工作区。[/]")
                    if not ask_confirm("继续执行并行编译吗?", default=True):
                        continue
                    rc = run_colcon_deb_parallel_build(state)
                    if rc == 0:
                        state.load_queue_from_file()
                        console.print("[green]并行构建完成，队列已按已构建包自动更新。[/]")
                    continue
                auto_install_deb = ask_confirm("构建成功后自动安装生成的 deb 包吗?", default=False)
                while True:
                    pending = [
                        pkg
                        for pkg in state.queue_packages
                        if not state.package_status.get(pkg)
                        and any(t.display_name == pkg and t.kind == "debian" for t in state.build_queue)
                    ]
                    if not pending:
                        console.print("[cyan]没有待构建的 Debian 包，可通过扫描/添加任务生成。[/]")
                        break
                    options = [
                        "使用 git-buildpackage (gbp)",
                        "使用 debuild -us -uc -b",
                        "优化排序",
                    ]
                    selection = ask_select("选择 Debian 构建方式", options)
                    if selection == "优化排序":
                        optimize_debian_build_queue(state)
                        continue
                    if selection == "使用 debuild -us -uc -b":
                        debian_builder = "debuild"
                        run_tests = ask_confirm("构建时需要运行测试吗？选择“否”将仅编译并跳过测试。", default=True)
                    else:
                        debian_builder = "gbp"
                        run_tests = True
                    break
                if not pending:
                    continue
            else:
                pending = [
                    pkg
                    for pkg in state.queue_packages
                    if any(t.display_name == pkg and t.kind == "rpm" for t in state.build_queue)
                ]
                if not pending:
                    console.print("[cyan]队列中没有可构建的 RPM 包。[/]")
                    continue
                debian_builder = "gbp"  # unused for RPM tasks

            failed_packages: List[str] = []
            aborted = False
            queue_snapshot = list(state.queue_packages)
            try:
                for pkg in queue_snapshot:
                    if pkg not in pending:
                        continue
                    tasks_for_pkg = [
                        task
                        for task in state.build_queue
                        if task.display_name == pkg and task.kind == target_kind
                    ]
                    if not tasks_for_pkg:
                        continue
                    if target_kind == "debian" and state.package_status.get(pkg):
                        console.print(f"[cyan]{pkg} 已标记完成，跳过")
                        continue
                    package_failed = False
                    for task in tasks_for_pkg:
                        if not execute_build(
                            task,
                            state,
                            debian_builder=debian_builder,
                            auto_install=auto_install_deb,
                            run_tests=run_tests,
                        ):
                            package_failed = True
                            break
                    if package_failed:
                        failed_packages.append(pkg)
                        if target_kind == "debian":
                            state.package_status[pkg] = False
                        if not ask_confirm("继续执行剩余包?", default=True):
                            aborted = True
                            break
                    else:
                        if target_kind == "debian":
                            state.package_status[pkg] = True
                    state.save_queue()
            except KeyboardInterrupt:
                aborted = True
                console.print("[yellow]已接收到暂停请求 (Ctrl+C)，当前进度已保存，可稍后继续。[/]")

            if failed_packages:
                console.print("[yellow]以下包构建失败：[/]")
                for pkg in failed_packages:
                    console.print(f"- {pkg}")
            if target_kind == "debian" and not failed_packages and not aborted:
                console.print("[green]选定的 Debian 包已成功构建并标记为 #[/]")
        elif choice == "清空队列":
            state.clear_queue()
            console.print("[yellow]构建队列已清空[/]")
        elif choice == "编辑构建参数":
            edit_build_parameters(state)


def edit_build_parameters(state: MenuState) -> None:
    while True:
        choice = ask_select("编辑构建参数", ["Debian 构建参数", "RPM 构建命令", "切换自动生成 gbp.conf", "返回"])
        if choice in (None, "返回"):
            return
        if choice == "Debian 构建参数":
            current = " ".join(state.debian_build_args)
            new_value = ask_text("请输入 gbp buildpackage 附加参数", current)
            state.debian_build_args = shlex_split(new_value or "")
        elif choice == "RPM 构建命令":
            base = ask_text("rpmbuild 命令 (可包含路径)", state.rpm_build_base)
            args = ask_text("rpmbuild 参数", " ".join(state.rpm_build_args))
            if base:
                state.rpm_build_base = base
            state.rpm_build_args = shlex_split(args or "")
        elif choice == "切换自动生成 gbp.conf":
            state.auto_generate_gbp = not state.auto_generate_gbp
            console.print(f"[cyan]批量操作自动生成 gbp.conf {'已启用' if state.auto_generate_gbp else '已关闭'}[/]")
        state.update_env()


def handle_clean(state: MenuState) -> None:
    script = REPO_ROOT / "clean_generated.py"
    if not script.exists():
        console.print(f"[red]未找到 {script}[/]")
        return
    env = state.build_env()
    env["CODE_DIR"] = str(state.code_dir)
    rc = run_stream([sys.executable, str(script)], cwd=REPO_ROOT, env=env)
    if rc == 0:
        console.print("[green]清理完成[/]")


def handle_scan_and_generate(state: MenuState) -> None:
    state.refresh_from_env()
    state.load_queue_from_file()
    if not state.code_dir.exists():
        console.print(f"[red]源码目录不存在: {state.code_dir}[/]")
        return

    detected = detect_linux_distribution()
    target: Optional[str] = None
    if detected == "ubuntu":
        selection = ask_select(
            "检测到当前系统为 Ubuntu，选择要扫描的目标？",
            ["使用 Ubuntu (生成 debian 构建)", "改为 openEuler (生成 rpm 构建)", "返回"],
        )
        if selection == "使用 Ubuntu (生成 debian 构建)":
            target = "ubuntu"
        elif selection == "改为 openEuler (生成 rpm 构建)":
            target = "openeuler"
    elif detected == "openeuler":
        selection = ask_select(
            "检测到当前系统为 openEuler，选择要扫描的目标？",
            ["使用 openEuler (生成 rpm 构建)", "改为 Ubuntu (生成 debian 构建)", "返回"],
        )
        if selection == "使用 openEuler (生成 rpm 构建)":
            target = "openeuler"
        elif selection == "改为 Ubuntu (生成 debian 构建)":
            target = "ubuntu"
    else:
        selection = ask_select(
            "请选择要扫描的目标系统",
            ["Ubuntu (生成 debian 构建)", "openEuler (生成 rpm 构建)", "返回"],
        )
        if selection == "Ubuntu (生成 debian 构建)":
            target = "ubuntu"
        elif selection == "openEuler (生成 rpm 构建)":
            target = "openeuler"

    if target not in {"ubuntu", "openeuler"}:
        console.print("[yellow]已取消扫描。[/]")
        return

    packages = list_code_packages(state.code_dir)
    if not packages:
        console.print("[yellow]未在源码目录找到任何 package.xml，无法生成构建列表。[/]")
        return

    if target == "ubuntu":
        matched = [pkg for pkg in packages if (pkg / "debian").is_dir()]
        build_kind = "debian"
        os_label = "Ubuntu"
    else:
        matched = []
        for pkg in packages:
            rpm_dir = pkg / "rpm"
            if rpm_dir.is_dir() and any(rpm_dir.glob("*.spec")):
                matched.append(pkg)
        build_kind = "rpm"
        os_label = "openEuler"

    if not matched:
        console.print(f"[yellow]{os_label} 环境未找到已生成的 {build_kind} 构建目录。[/]")
        return

    tasks = [BuildTask(pkg.name, pkg, build_kind, []) for pkg in matched]
    added, total = state.add_tasks(tasks, reset_completed=False)
    new_count = added
    existing_count = total - added

    console.print(f"[green]扫描完成：共识别 {total} 个{os_label} 构建包。[/]")
    if new_count:
        console.print(f"[cyan]新增 {new_count} 个包至构建列表。[/]")
    if existing_count:
        console.print(f"[cyan]{existing_count} 个包已存在于构建列表，保持原有状态。[/]")
    console.print(f"[dim]构建列表文件: {state.queue_file}[/]")


def handle_configuration(state: MenuState) -> None:
    while True:
        render_state_panel(state)
        choice = ask_select(
            "配置与状态",
            [
                "修改 Release 目录",
                "修改 源码目录",
                 "修改 安装前缀",
                "修改 distribution.yaml URL",
                "修改 ROS/Tracks 配置",
                "修改 openEuler 参数",
                "修改 Bloom 命令",
                "修改 构建队列文件路径",
                "修改 Debian 构建配置",
                "返回",
            ],
        )
        if choice in (None, "返回"):
            return
        if choice == "修改 Release 目录":
            value = ask_text("新的 Release 仓库目录", str(state.release_dir))
            if value:
                state.release_dir = Path(value).expanduser().resolve()
        elif choice == "修改 源码目录":
            value = ask_text("新的源码目录", str(state.code_dir))
            if value:
                state.code_dir = Path(value).expanduser().resolve()
        elif choice == "修改 安装前缀":
            value = ask_text("新的安装前缀目录", str(state.install_prefix))
            if value:
                state.install_prefix = Path(value).expanduser().resolve()
        elif choice == "修改 distribution.yaml URL":
            value = ask_text("新的 URL", state.distribution_url)
            if value:
                state.distribution_url = value
        elif choice == "修改 ROS/Tracks 配置":
            ros = ask_text("AGIROS 发行版", state.agiros_distro)
            tracks = ask_text("ROS2 发行版", state.ros2_distro)
            ubuntu = ask_text("Ubuntu 版本", state.ubuntu_version)
            state.agiros_distro = ros or state.agiros_distro
            state.ros2_distro = tracks or state.ros2_distro
            state.ubuntu_version = ubuntu or state.ubuntu_version
        elif choice == "修改 openEuler 参数":
            default = ask_text("openEuler 默认版本", state.openeuler_default)
            fallback = ask_text("openEuler 回退列表 (逗号分隔)", ", ".join(state.openeuler_fallback))
            if default:
                state.openeuler_default = default
            if fallback is not None:
                state.openeuler_fallback = [item.strip() for item in fallback.split(",") if item.strip()]
        elif choice == "修改 Bloom 命令":
            bloom = ask_text("bloom 可执行命令", state.bloom_bin)
            if bloom:
                state.bloom_bin = bloom
        elif choice == "修改 构建队列文件路径":
            value = ask_text("构建队列文件路径", str(state.queue_file))
            meta_value = ask_text("构建队列元数据文件路径 (留空则使用默认 .meta.json)", str(state.queue_meta_file))
            if value:
                state.queue_file = state._normalize_path(value)
            if meta_value:
                state.queue_meta_file = state._normalize_path(meta_value)
            else:
                state.queue_meta_file = state._meta_path_for_queue(state.queue_file)
            state.ensure_queue_file()
            state.load_queue_from_file()
        elif choice == "修改 Debian 构建配置":
            code_label = ask_text("主界面源码前缀标签", state.code_label)
            deb_out = ask_text("Debian 输出目录", str(state.deb_out_dir))
            distro = ask_text("Debian DISTRO (gbp release_tag 用)", state.deb_distro)
            release_inc = ask_text("默认 release_inc", state.deb_release_inc)
            parallel = ask_text("并行线程数", state.deb_parallel)
            colcon_src = ask_text("COLCON_SRC_DIR (相对源码目录, 留空=自动)", state.colcon_src_dir)
            apt_source = ask_text("AGIROS APT 源行 (留空=不设置)", state.agiros_apt_source)
            apt_source_file = ask_text(
                "AGIROS APT 源文件路径",
                str(state.agiros_apt_source_file),
            )
            auto_fix = ask_confirm(
                "缺失依赖时自动修复 (apt-get -f install)?",
                default=state.auto_fix_deps,
            )
            git_name = ask_text("Git 提交用户名", state.git_user_name)
            git_email = ask_text("Git 提交邮箱", state.git_user_email)
            if code_label:
                state.code_label = code_label
            if deb_out:
                state.deb_out_dir = Path(deb_out).expanduser().resolve()
            if distro:
                state.deb_distro = distro
            if release_inc:
                state.deb_release_inc = release_inc
            if parallel:
                state.deb_parallel = parallel
            if colcon_src is not None:
                state.colcon_src_dir = colcon_src.strip()
            if apt_source is not None:
                state.agiros_apt_source = apt_source
            if apt_source_file:
                state.agiros_apt_source_file = Path(apt_source_file).expanduser().resolve()
            state.auto_fix_deps = auto_fix
            if git_name:
                state.git_user_name = git_name
            if git_email:
                state.git_user_email = git_email
        state.update_env()


def gather_log_candidates(state: MenuState) -> List[Path]:
    candidates: List[Path] = []
    seen: Set[str] = set()

    def _add(path: Path) -> None:
        try:
            resolved = path.resolve()
        except Exception:
            resolved = path
        key = str(resolved)
        if key in seen:
            return
        if resolved.exists() and resolved.is_file():
            candidates.append(resolved)
            seen.add(key)

    base_dirs = [state.release_dir, state.code_dir, REPO_ROOT]
    names = ["download_log.txt", "failed_repos.txt", "fail.log"]
    patterns = ["*.log", "*log*.txt"]

    for base in base_dirs:
        if not base or not base.exists():
            continue
        for name in names:
            _add(base / name)
        for pattern in patterns:
            for path in base.glob(pattern):
                _add(path)

    return sorted(candidates, key=lambda p: str(p))


def handle_logs(state: MenuState) -> None:
    while True:
        logs = gather_log_candidates(state)
        options = [str(p) for p in logs] + ["自定义路径", "返回"]
        choice = ask_select("查看日志", options)
        if choice in (None, "返回"):
            return
        if choice == "自定义路径":
            path_str = ask_text("输入日志文件路径", "")
        else:
            path_str = choice
        if not path_str:
            continue
        path = Path(path_str).expanduser()
        if not path.exists():
            console.print(f"[red]未找到 {path}[/]")
            continue
        content = path.read_text(encoding="utf-8", errors="ignore")
        console.print(Panel(Text(content if len(content) < 4000 else content[-4000:], style="white"), title=str(path), box=box.ROUNDED))
        action = ask_select("日志查看", ["返回", "继续查看"])
        if action in (None, "返回"):
            return


def main() -> None:
    state = MenuState()
    state.update_env()
    while True:
        render_state_panel(state)
        choice = ask_select(
            "请选择操作",
            [
                "从distribution.yaml下载release仓库（tracks）或源码（code）",
                "处理 tracks.yaml / 下载源码",
                "源码替换ros->agiros",
                "Bloom 打包",
                "构建 (Build)",
                "清理生成目录",
                "配置与状态",
                "查看日志",
                "退出",
            ],
        )
        if choice == "从distribution.yaml下载release仓库（tracks）或源码（code）":
            handle_download_release(state)
        elif choice == "处理 tracks.yaml / 下载源码":
            handle_tracks_download(state)
        elif choice == "源码替换ros->agiros":
            ros2agiros_menu(state)
        elif choice == "Bloom 打包":
            bloom_menu(state)
        elif choice == "构建 (Build)":
            manage_build_queue(state)
        elif choice == "清理生成目录":
            handle_clean(state)
        elif choice == "配置与状态":
            handle_configuration(state)
        elif choice == "查看日志":
            handle_logs(state)
        elif choice == "退出" or choice is None:
            console.print("[cyan]Bye[/]")
            break


if __name__ == "__main__":
    main()
