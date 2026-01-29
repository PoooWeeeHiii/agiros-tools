#!/usr/bin/env python3
import json
import os
import platform
import shlex
import sys
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

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
        console.print(line.rstrip())
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
    distribution_url: str = "http://1.94.193.239/yumrepo/agiros/agirosdep/loong/distribution.yaml"
    release_dir: Path = Path(os.environ.get("AGIROS_RELEASE_DIR", "ros2_release_dir"))
    code_dir: Path = Path(os.environ.get("AGIROS_CODE_DIR", "ros2_code_dir"))
    code_label: str = os.environ.get("AGIROS_CODE_LABEL", "code_dir")
    tracks_distro: str = os.environ.get("AGIROS_TRACKS_DISTRO", "jazzy")
    ros_distro: str = os.environ.get("AGIROS_ROS_DISTRO", "loong")
    ubuntu_version: str = os.environ.get("AGIROS_UBUNTU_DEFAULT", "jammy")
    openeuler_default: str = os.environ.get("AGIROS_OE_DEFAULT", "24")
    openeuler_fallback: List[str] = field(default_factory=lambda: [item.strip() for item in os.environ.get("AGIROS_OE_FALLBACK", "22,23").split(",") if item.strip()])
    bloom_bin: str = os.environ.get("AGIROS_BLOOM_BIN", "bloom-generate")
    auto_generate_gbp: bool = bool(int(os.environ.get("AGIROS_GENERATE_GBP", "0")))
    debian_build_args: List[str] = field(default_factory=lambda: ["--git-ignore-branch", "--git-ignore-new", "-us", "-uc"])
    rpm_build_base: str = os.environ.get("AGIROS_RPMBUILD_BIN", "rpmbuild")
    rpm_build_args: List[str] = field(default_factory=lambda: ["-ba"])
    deb_out_dir: Path = Path(os.environ.get("DEB_OUT", str(Path.home() / "deb_out")))
    deb_distro: str = os.environ.get("DISTRO", "loong")
    deb_release_inc: str = os.environ.get("DEFAULT_REL_INC", "1")
    deb_parallel: str = os.environ.get("PARALLEL", str(os.cpu_count() or 4))
    git_user_name: str = os.environ.get("GIT_USER_NAME", "PoooWeeeHiii")
    git_user_email: str = os.environ.get("GIT_USER_EMAIL", "powehi041210@gmail.com")
    queue_file: Path = Path(os.environ.get("AGIROS_QUEUE_FILE", str(REPO_ROOT / "build_queue.txt")))
    queue_meta_file: Path = Path(os.environ.get("AGIROS_QUEUE_META", ""))
    build_queue: List[BuildTask] = field(default_factory=list)
    queue_packages: List[str] = field(default_factory=list)
    package_status: Dict[str, bool] = field(default_factory=dict)

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

    def update_env(self) -> None:
        mappings = {
            "AGIROS_RELEASE_DIR": str(self.release_dir),
            "AGIROS_RELEASE_TARGET_DIR": str(self.release_dir),
            "AGIROS_CODE_DIR": str(self.code_dir),
            "AGIROS_CODE_LABEL": self.code_label,
            "AGIROS_TRACKS_DISTRO": self.tracks_distro,
            "AGIROS_ROS_DISTRO": self.ros_distro,
            "AGIROS_DISTRO": self.ros_distro,
            "AGIROS_UBUNTU_DEFAULT": self.ubuntu_version,
            "AGIROS_OE_DEFAULT": self.openeuler_default,
            "AGIROS_OE_FALLBACK": ",".join(self.openeuler_fallback),
            "AGIROS_BLOOM_BIN": self.bloom_bin,
            "AGIROS_GENERATE_GBP": "1" if self.auto_generate_gbp else "0",
            "AGIROS_RPMBUILD_BIN": self.rpm_build_base,
            "DEB_OUT": str(self.deb_out_dir),
            "DISTRO": self.deb_distro,
            "DEFAULT_REL_INC": self.deb_release_inc,
            "PARALLEL": self.deb_parallel,
            "GIT_USER_NAME": self.git_user_name,
            "GIT_USER_EMAIL": self.git_user_email,
            "AGIROS_QUEUE_FILE": str(self.queue_file),
            "AGIROS_QUEUE_META": str(self.queue_meta_file),
        }
        for key, value in mappings.items():
            os.environ[key] = value

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
        _set_path("DEB_OUT", "deb_out_dir")
        _set_str("AGIROS_CODE_LABEL", "code_label")
        _set_str("AGIROS_TRACKS_DISTRO", "tracks_distro")
        _set_str("AGIROS_ROS_DISTRO", "ros_distro")
        _set_str("AGIROS_UBUNTU_DEFAULT", "ubuntu_version")
        _set_str("AGIROS_OE_DEFAULT", "openeuler_default")
        _set_list("AGIROS_OE_FALLBACK", "openeuler_fallback")
        _set_str("AGIROS_BLOOM_BIN", "bloom_bin")
        _set_bool("AGIROS_GENERATE_GBP", "auto_generate_gbp")
        _set_str("AGIROS_RPMBUILD_BIN", "rpm_build_base")
        _set_str("DISTRO", "deb_distro")
        _set_str("DEFAULT_REL_INC", "deb_release_inc")
        _set_str("PARALLEL", "deb_parallel")
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
            ("Release 仓库", str(self.release_dir)),
            ("源码目录", str(self.code_dir)),
            ("distribution.yaml URL", self.distribution_url),
            ("Tracks 发行版", self.tracks_distro),
            ("ROS 发行版", self.ros_distro),
            ("Ubuntu 版本", self.ubuntu_version),
            ("openEuler 默认", self.openeuler_default),
            ("openEuler 回退", ", ".join(self.openeuler_fallback) or "-"),
            ("bloom 命令", self.bloom_bin),
            ("批量生成 gbp.conf", "启用" if self.auto_generate_gbp else "关闭"),
            ("Debian 构建参数", " ".join(self.debian_build_args)),
            ("RPM 构建命令", f"{self.rpm_build_base} {' '.join(self.rpm_build_args)}".strip()),
            ("Debian 输出目录", str(self.deb_out_dir)),
            ("Debian 发行版", self.deb_distro),
            ("Debian release_inc", self.deb_release_inc),
            ("并行构建线程", self.deb_parallel),
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
    target = ask_text("Release 仓库存放目录", str(state.release_dir))
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
        downloader.download_repos_from_yaml(url, str(target_path))
    except Exception as exc:
        console.print(f"[bold red]下载失败: {exc}[/]")
        return

    state.distribution_url = url
    state.release_dir = target_path
    state.update_env()


def handle_tracks_download(state: MenuState) -> None:
    release_dir = ask_text("Release 仓库目录", str(state.release_dir))
    code_dir = ask_text("源码目录", str(state.code_dir))
    distro = ask_text("Tracks 发行版名称", state.tracks_distro)
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
    state.tracks_distro = distro
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
        cmd += ["--ros-distro", state.ros_distro, "--os-name", "ubuntu", "--os-version", state.ubuntu_version]
        if generate_gbp:
            cmd.append("--generate-gbp")
            cmd += ["--tracks-distro", state.tracks_distro]
            cmd += ["--distro", state.ros_distro]
            cmd += ["--pkg", package_path.name]
    else:
        cmd += ["--ros-distro", state.ros_distro, "--os-name", "openeuler", "--os-version", state.openeuler_default]
    env = os.environ.copy()
    if generate_gbp:
        env["OOB_TRACKS_DIR"] = str(state.release_dir)
        env["OOB_TRACKS_DISTRO"] = state.tracks_distro
        env["AGIROS_DISTRO"] = state.ros_distro
    rc = run_stream(cmd, cwd=package_path, env=env)
    if rc == 0:
        console.print("[green]完成[/]")


def run_batch_bloom(state: MenuState, mode: str) -> None:
    script = REPO_ROOT / "agiros_oob_builder_procedural.py"
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
        state.ros_distro,
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
    run_stream(cmd, cwd=REPO_ROOT, env=os.environ.copy())


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


def run_debian_build(state: MenuState, path: Path, extra_args: Optional[List[str]] = None) -> int:
    script = REPO_ROOT / "git_build_any.py"
    env = os.environ.copy()
    if script.exists():
        env.setdefault("WORK_DIR", str(path))
        env.setdefault("CODE_DIR", str(state.code_dir))
        env.setdefault("DEB_OUT", str(state.deb_out_dir))
        env.setdefault("DISTRO", state.deb_distro)
        env.setdefault("DEFAULT_REL_INC", state.deb_release_inc)
        env.setdefault("PARALLEL", state.deb_parallel)
        env.setdefault("GIT_USER_NAME", state.git_user_name)
        env.setdefault("GIT_USER_EMAIL", state.git_user_email)
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
    env = os.environ.copy()
    return run_stream(cmd, cwd=path, env=env)


def run_rpm_build(state: MenuState, path: Path, extra_args: Optional[List[str]] = None) -> int:
    script = REPO_ROOT / "rpmbuild_any.py"
    env = os.environ.copy()
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
        cmd = [state.rpm_build_base] + state.rpm_build_args + [str(spec)]
        if extra_args:
            cmd += extra_args
        rc = run_stream(cmd, cwd=path, env=os.environ.copy())
        if rc != 0:
            break
    return rc


def execute_build(task: BuildTask, state: MenuState) -> bool:
    console.print(Panel(f"开始构建: {describe_build_task(task, state)}", box=box.ROUNDED))
    success = True
    if task.kind == "debian":
        if run_debian_build(state, task.path, task.extra_args) != 0:
            success = False
    elif task.kind == "rpm":
        if run_rpm_build(state, task.path, task.extra_args) != 0:
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
            pending = [pkg for pkg in state.queue_packages if not state.package_status.get(pkg)]
            if not pending:
                console.print("[cyan]所有包均已标记完成 (#)，如需重新构建请先移除或重新加入。[/]")
                continue
            failed_packages: List[str] = []
            aborted = False
            for pkg in state.queue_packages:
                tasks_for_pkg = [task for task in state.build_queue if task.display_name == pkg]
                if not tasks_for_pkg:
                    continue
                if state.package_status.get(pkg):
                    console.print(f"[cyan]{pkg} 已标记完成，跳过[/]")
                    continue
                package_failed = False
                for task in tasks_for_pkg:
                    if not execute_build(task, state):
                        package_failed = True
                        break
                if package_failed:
                    failed_packages.append(pkg)
                    state.package_status[pkg] = False
                    if not ask_confirm("继续执行剩余包?", default=True):
                        aborted = True
                        break
                else:
                    state.package_status[pkg] = True
            state.save_queue()
            if failed_packages:
                console.print("[yellow]以下包构建失败，已保持未完成状态：[/]")
                for pkg in failed_packages:
                    console.print(f"- {pkg}")
            if not failed_packages and not aborted:
                console.print("[green]队列包均已成功构建并标记为 #[/]")
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
    env = os.environ.copy()
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
        elif choice == "修改 distribution.yaml URL":
            value = ask_text("新的 URL", state.distribution_url)
            if value:
                state.distribution_url = value
        elif choice == "修改 ROS/Tracks 配置":
            ros = ask_text("ROS 发行版", state.ros_distro)
            tracks = ask_text("Tracks 发行版", state.tracks_distro)
            ubuntu = ask_text("Ubuntu 版本", state.ubuntu_version)
            state.ros_distro = ros or state.ros_distro
            state.tracks_distro = tracks or state.tracks_distro
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
            if git_name:
                state.git_user_name = git_name
            if git_email:
                state.git_user_email = git_email
        state.update_env()


def gather_log_candidates(state: MenuState) -> List[Path]:
    candidates = [
        state.release_dir / "download_log.txt",
        state.release_dir / "failed_repos.txt",
        REPO_ROOT / "fail.log",
    ]
    return [path for path in candidates if path.exists()]


def handle_logs(state: MenuState) -> None:
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
        return
    path = Path(path_str).expanduser()
    if not path.exists():
        console.print(f"[red]未找到 {path}[/]")
        return
    content = path.read_text(encoding="utf-8", errors="ignore")
    console.print(Panel(Text(content if len(content) < 4000 else content[-4000:], style="white"), title=str(path), box=box.ROUNDED))


def main() -> None:
    state = MenuState()
    state.update_env()
    while True:
        render_state_panel(state)
        choice = ask_select(
            "请选择操作",
            [
                "下载 release 仓库",
                "处理 tracks.yaml / 下载源码",
                "Bloom 打包",
                "构建 (Build)",
                "清理生成目录",
                "配置与状态",
                "查看日志",
                "退出",
            ],
        )
        if choice == "下载 release 仓库":
            handle_download_release(state)
        elif choice == "处理 tracks.yaml / 下载源码":
            handle_tracks_download(state)
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
