#!/usr/bin/env python3
"""AGIROS Debian package builder focused on RISC-V Ubuntu environments."""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Set

os.environ.setdefault("DEBIAN_FRONTEND", "noninteractive")

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_DEBUILD = ["debuild", "-us", "-uc", "-b"]
DEFAULT_BASE_DIR = Path(os.environ.get("AGIROS_CODE_DIR", "/mnt/lfb_ws/jazzy"))
DEFAULT_QUEUE_FILE = Path(
    os.environ.get("AGIROS_QUEUE_FILE", REPO_ROOT / "build_queue.txt")
).resolve()
DEFAULT_HELPER_SCRIPT = (REPO_ROOT / "build-helpers.sh").resolve()
DEPENDENCY_SOURCE_PREFIXES = [
    prefix.strip()
    for prefix in os.environ.get("AGIROS_DEP_PREFIXES", "agiros-pixiu-").split(",")
    if prefix.strip()
]
BASE_PACKAGES = [
    "apt-utils",
    "ca-certificates",
    "gnupg",
    "build-essential",
    "cmake",
    "devscripts",
    "debhelper",
    "dh-python",
    "fakeroot",
    "lintian",
    "equivs",
    "python3",
    "python3-all",
    "python3-pytest",
    "python3-flake8",
    "python3-setuptools",
    "python3-venv",
    "git",
    "curl",
    "xz-utils",
    "file",
]
THIRD_PARTY_DEFAULT = Path(os.environ.get("THIRD_PARTY_DIR", "/tmp"))
DEFAULT_INSTALL_SCRIPT = REPO_ROOT / "deb_install_any.py"
ALLOW_INSTALL_FAIL_AS_WARNING = (
    os.environ.get("AGIROS_ALLOW_INSTALL_FAIL_AS_WARNING", "1").strip().lower() not in {"0", "false", "no"}
)
THIRD_PARTY_PATTERNS = [
    "python3-catkin-pkg-modules_*.deb",
    "python3-catkin-pkg_*.deb",
    "python3-rospkg-modules_*.deb",
    "python3-rosdistro_*.deb",
    "python3-rosdistro-modules_*.deb",
    "python3-vcstools_*.deb",
    "libignition-cmake2-dev_2.1.1+dfsg-3_riscv64.deb",
]
PREFERRED_PACKAGES = {"python3-catkin-pkg": "python3-catkin-pkg-modules"}
BROKEN_BUILD_DEPS_PACKAGE = "agiros-pixiu-canopen-core-build-deps"
AUTO_KILL_LOCK_HOLDERS = os.environ.get("AGIROS_AUTO_KILL_LOCK_HOLDERS", "1").strip().lower() not in {
    "0",
    "false",
    "no",
}
APT_LOCK_PATH = Path(os.environ.get("AGIROS_APT_LOCKFILE", "/mnt/lfb_ws/.locks/apt.lock"))
APT_LOCK_TIMEOUT = int(os.environ.get("AGIROS_APT_LOCK_TIMEOUT", "3600"))
APT_WAIT_FOR_LOCK = os.environ.get("AGIROS_APT_WAIT_FOR_LOCK", "1").strip().lower() not in {"0", "false", "no"}
APT_LOCK_WAIT_TIMEOUT = int(os.environ.get("AGIROS_APT_LOCK_WAIT_TIMEOUT", "600"))
APT_LOCK_WAIT_INTERVAL = float(os.environ.get("AGIROS_APT_LOCK_WAIT_INTERVAL", "5"))
APT_MAX_RETRIES = int(os.environ.get("AGIROS_APT_MAX_RETRIES", "3"))
LOCK_FILES = (Path("/var/lib/dpkg/lock-frontend"), Path("/var/lib/dpkg/lock"))
GIT_MIRROR = os.environ.get("AGIROS_GIT_MIRROR", "").strip().rstrip("/")
GIT_MIRROR_MAP_FILE = Path(os.environ.get("AGIROS_GIT_MIRROR_MAP_FILE", REPO_ROOT / "git_mirror_map.txt"))
GIT_MIRROR_SOURCES = (
    "https://github.com/",
    "git@github.com:",
)


@dataclass
class QueueEntry:
    name: str
    path: Path
    completed: bool
    kind: str = "debian"


@dataclass
class MissingDependency:
    display: str
    candidates: List[str]

    def valid_candidates(self) -> List[str]:
        return [candidate for candidate in self.candidates if candidate]


def allow_invalid_work_dir() -> bool:
    return os.environ.get("AGIROS_ALLOW_INVALID_WORKDIR", "1").strip().lower() not in {"0", "false", "no"}


def print_info(message: str) -> None:
    print(f"[INFO] {message}")


def print_warn(message: str) -> None:
    print(f"[WARN] {message}", file=sys.stderr)


def run_cmd(cmd: Sequence[str], *, check: bool = True, cwd: Path | None = None) -> subprocess.CompletedProcess:
    print_info(f"$ {' '.join(cmd)}")
    return subprocess.run(cmd, cwd=str(cwd) if cwd else None, check=check)


_APT_GUARD_MISSING_WARNED = False
_GIT_MIRROR_CONFIGURED = False


def get_helper_script() -> Path:
    override = os.environ.get("AGIROS_BUILD_HELPERS")
    if override:
        return Path(override).expanduser().resolve()
    return DEFAULT_HELPER_SCRIPT


def run_apt_guard() -> None:
    global _APT_GUARD_MISSING_WARNED
    helper = get_helper_script()
    if not helper.exists():
        if not _APT_GUARD_MISSING_WARNED:
            print_warn(f"Apt guard script not found at {helper}, skipping dpkg state cleanup.")
            _APT_GUARD_MISSING_WARNED = True
        return
    run_with_lock_retries(
        ["bash", "-c", f"source '{helper}' && fix_and_clean_broken_agiros"],
        check=False,
        use_apt_lock=True,
    )


def _configure_git_mirror() -> None:
    """Configure git mirror/insteadOf to avoid GitHub TLS failures."""
    global _GIT_MIRROR_CONFIGURED
    if _GIT_MIRROR_CONFIGURED:
        return
    try:
        subprocess.run(["git", "--version"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        print_warn("git not found; skip git mirror configuration.")
        _GIT_MIRROR_CONFIGURED = True
        return

    applied: List[str] = []
    if GIT_MIRROR:
        mirror_prefix = GIT_MIRROR if GIT_MIRROR.endswith("/") else f"{GIT_MIRROR}/"
        for source in GIT_MIRROR_SOURCES:
            subprocess.run(
                ["git", "config", "--global", f"url.{mirror_prefix}.insteadOf", source],
                check=False,
            )
        applied.append(f"default mirror -> {mirror_prefix}")

    if GIT_MIRROR_MAP_FILE.exists():
        try:
            with GIT_MIRROR_MAP_FILE.open("r", encoding="utf-8") as handle:
                for raw in handle:
                    line = raw.strip()
                    if not line or line.startswith("#"):
                        continue
                    parts = line.split()
                    if len(parts) != 2:
                        continue
                    source, mirror = parts
                    mirror = mirror if mirror.endswith("/") else f"{mirror}/"
                    subprocess.run(
                        ["git", "config", "--global", f"url.{mirror}.insteadOf", source],
                        check=False,
                    )
                    applied.append(f"{source} -> {mirror}")
        except OSError as exc:
            print_warn(f"Failed to read git mirror map {GIT_MIRROR_MAP_FILE}: {exc}")

    if applied:
        print_info("Configured git mirrors: " + "; ".join(applied))
    _GIT_MIRROR_CONFIGURED = True


def wrap_with_apt_lock(cmd: Sequence[str], *, enable: bool = True) -> List[str]:
    if not enable:
        return list(cmd)
    if not APT_LOCK_PATH:
        return list(cmd)
    try:
        APT_LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        print_warn(f"Failed to ensure apt lock directory: {exc}")
        return list(cmd)
    if cmd and cmd[0] == "flock":
        return list(cmd)
    return ["flock", "-w", str(APT_LOCK_TIMEOUT), str(APT_LOCK_PATH), *cmd]


def _print_process_output(proc: subprocess.CompletedProcess) -> None:
    if proc.stdout:
        sys.stdout.write(proc.stdout)
    if proc.stderr:
        sys.stderr.write(proc.stderr)
    sys.stdout.flush()
    sys.stderr.flush()


def _extract_lock_pids(output: str) -> Set[int]:
    pids: Set[int] = set()
    for match in re.findall(r"process(?:\\D+)?(\\d+)", output, flags=re.IGNORECASE):
        try:
            pids.add(int(match))
        except ValueError:
            continue
    return pids


def _maybe_print_process_details(pids: Set[int]) -> None:
    if not pids:
        return
    try:
        proc = subprocess.run(
            ["ps", "-p", ",".join(str(pid) for pid in sorted(pids)), "-o", "pid,ppid,etimes,cmd"],
            text=True,
            capture_output=True,
            check=False,
        )
        if proc.stdout:
            print_info("Lock holders:")
            sys.stdout.write(proc.stdout)
    except Exception as exc:  # noqa: BLE001
        print_warn(f"Unable to describe lock holders: {exc}")


def _print_related_package_processes() -> None:
    patterns = ("apt-get", " apt ", "dpkg", "mk-build-deps", "dpkg-buildpackage", "debuild_runner")
    try:
        proc = subprocess.run(
            ["ps", "axo", "pid,ppid,etimes,cmd"],
            text=True,
            capture_output=True,
            check=False,
        )
    except Exception as exc:  # noqa: BLE001
        print_warn(f"Unable to list related processes: {exc}")
        return
    if not proc.stdout:
        return
    lines = proc.stdout.splitlines()
    header, *rows = lines
    matches = [line for line in rows if any(pattern in line for pattern in patterns)]
    if matches:
        print_info("Related package processes (ps axo pid,ppid,etimes,cmd):")
        print(header)
        for line in matches:
            print(line)


def _is_process_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def _terminate_processes(pids: Iterable[int]) -> None:
    victims = {pid for pid in pids if pid not in {os.getpid(), os.getppid()}}
    if not victims:
        return
    print_warn(f"Attempting to terminate lock holders: {', '.join(str(pid) for pid in sorted(victims))}")
    for sig, desc in ((signal.SIGTERM, "TERM"), (signal.SIGKILL, "KILL")):
        try:
            for pid in list(victims):
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    continue
                except PermissionError:
                    print_warn(f"No permission to send {desc} to PID {pid}")
            if sig == signal.SIGTERM:
                time.sleep(5)
        finally:
            victims = {pid for pid in victims if _is_process_alive(pid)}
            if not victims:
                break


def _wait_for_lock_release() -> bool:
    if not APT_WAIT_FOR_LOCK or APT_LOCK_WAIT_TIMEOUT <= 0:
        return False
    print_info(f"Waiting up to {APT_LOCK_WAIT_TIMEOUT}s for dpkg/apt lock to clear...")
    start = time.time()
    while time.time() - start < APT_LOCK_WAIT_TIMEOUT:
        busy = False
        for lock in LOCK_FILES:
            if not lock.exists():
                continue
            try:
                proc = subprocess.run(
                    ["fuser", str(lock)],
                    text=True,
                    capture_output=True,
                    check=False,
                )
                if proc.returncode == 0:
                    busy = True
                    break
            except FileNotFoundError:
                if not lock.exists():
                    continue
            except Exception as exc:  # noqa: BLE001
                print_warn(f"Unable to probe lock {lock}: {exc}")
        if not busy:
            return True
        time.sleep(APT_LOCK_WAIT_INTERVAL)
    print_warn(f"Lock still held after waiting {APT_LOCK_WAIT_TIMEOUT}s.")
    return False


def _fix_dpkg_state() -> None:
    for cmd in (["dpkg", "--configure", "-a"], ["apt-get", "-f", "install", "-y"]):
        effective_cmd = wrap_with_apt_lock(cmd, enable=True)
        proc = subprocess.run(effective_cmd, text=True, capture_output=True, check=False)
        _print_process_output(proc)
        if proc.returncode != 0:
            print_warn(f"{' '.join(cmd)} returned {proc.returncode}")


def _purge_broken_build_deps() -> None:
    if not BROKEN_BUILD_DEPS_PACKAGE:
        return
    cmd = ["apt-get", "remove", "--purge", "-y", BROKEN_BUILD_DEPS_PACKAGE]
    effective_cmd = wrap_with_apt_lock(cmd, enable=True)
    proc = subprocess.run(effective_cmd, text=True, capture_output=True, check=False)
    _print_process_output(proc)
    if proc.returncode != 0:
        print_warn(f"Failed to purge {BROKEN_BUILD_DEPS_PACKAGE}, continuing.")


def _handle_lock_and_recover(cmd: Sequence[str], combined_output: str) -> tuple[bool, bool]:
    lock_keywords = (
        "could not get lock",
        "unable to acquire the dpkg frontend lock",
        "is another process using it?",
        "is locked by another process",
        "dpkg was interrupted",
    )
    lower_output = combined_output.lower()
    pids = _extract_lock_pids(combined_output)
    has_lock = bool(pids) or any(keyword in lower_output for keyword in lock_keywords)
    if not has_lock:
        return (False, False)

    print_warn(f"Detected apt/dpkg lock while running: {' '.join(cmd)}")
    if pids:
        _maybe_print_process_details(pids)
    _print_related_package_processes()

    waited = False
    if _wait_for_lock_release():
        print_info("Lock released after waiting, will retry command.")
        return (True, False)
    if APT_WAIT_FOR_LOCK:
        waited = True

    if not AUTO_KILL_LOCK_HOLDERS:
        if waited:
            print_warn("Auto-kill disabled (AGIROS_AUTO_KILL_LOCK_HOLDERS=0); waited but lock persists, skipping recovery.")
        else:
            print_warn("Auto-kill disabled (AGIROS_AUTO_KILL_LOCK_HOLDERS=0), skipping recovery.")
        return (False, False)

    _terminate_processes(pids or [])
    _fix_dpkg_state()
    _purge_broken_build_deps()
    _fix_dpkg_state()
    return (True, True)


def run_with_lock_retries(
    cmd: Sequence[str],
    *,
    cwd: Path | None = None,
    check: bool = True,
    max_retries: int | None = None,
    use_apt_lock: bool = False,
) -> subprocess.CompletedProcess:
    attempts = 0
    effective_max = APT_MAX_RETRIES if max_retries is None else max_retries
    last_proc: Optional[subprocess.CompletedProcess] = None
    while attempts <= effective_max:
        effective_cmd = wrap_with_apt_lock(cmd, enable=use_apt_lock)
        print_info(f"$ {' '.join(effective_cmd)}")
        proc = subprocess.run(
            effective_cmd,
            cwd=str(cwd) if cwd else None,
            text=True,
            capture_output=True,
            check=False,
        )
        _print_process_output(proc)
        if proc.returncode == 0:
            return proc

        combined = (proc.stdout or "") + "\\n" + (proc.stderr or "")
        should_retry, consume_attempt = _handle_lock_and_recover(cmd, combined)
        if attempts < effective_max and should_retry:
            if consume_attempt:
                attempts += 1
            print_info("Retrying command after handling apt/dpkg lock...")
            continue

        last_proc = proc
        break

    if check and last_proc is not None:
        raise subprocess.CalledProcessError(
            last_proc.returncode,
            cmd,
            output=last_proc.stdout,
            stderr=last_proc.stderr,
        )
    return last_proc if last_proc is not None else proc


def parse_queue_line(line: str, base_dir: Path) -> Optional[QueueEntry]:
    stripped = line.strip()
    if not stripped:
        return None
    parsed: Optional[dict] = None
    if stripped.startswith("{") and stripped.endswith("}"):
        try:
            parsed = json.loads(stripped)
        except json.JSONDecodeError:
            parsed = None

    pkg_name = ""
    pkg_path: Optional[Path] = None
    completed = False
    kind = "debian"

    if parsed and isinstance(parsed, dict):
        pkg_name = str(parsed.get("name") or parsed.get("display_name") or "").strip()
        if not pkg_name:
            return None
        kind = str(parsed.get("kind", "debian")).lower()
        if kind not in {"debian", "gbp"}:
            return None
        completed = bool(parsed.get("completed", False))
        path_hint = parsed.get("path")
        if isinstance(path_hint, str) and path_hint.strip():
            pkg_path = Path(path_hint.strip())
    else:
        completed = stripped.endswith("#")
        if completed:
            stripped = stripped[:-1].strip()
        if not stripped:
            return None
        pkg_name = Path(stripped).name

    if not pkg_name:
        return None
    if pkg_path is None:
        pkg_path = base_dir / pkg_name
    if not pkg_path.is_absolute():
        pkg_path = base_dir / pkg_path
    return QueueEntry(name=pkg_name, path=pkg_path, completed=completed, kind=kind)


def load_queue(queue_file: Path, base_dir: Path, include_completed: bool) -> List[QueueEntry]:
    if not queue_file.exists():
        raise SystemExit(f"[ERR ] Queue file not found: {queue_file}")
    entries: List[QueueEntry] = []
    with queue_file.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            entry = parse_queue_line(raw_line, base_dir)
            if not entry:
                continue
            if entry.kind != "debian":
                continue
            if entry.completed and not include_completed:
                continue
            entries.append(entry)
    return entries


def _sanitize_dep_token(token: str) -> str:
    cleaned = token.strip()
    if not cleaned:
        return ""
    cleaned = cleaned.lstrip("-•").strip()
    cleaned = cleaned.split("(", 1)[0].strip()
    cleaned = cleaned.split("[", 1)[0].strip()
    cleaned = cleaned.split(":", 1)[0].strip()
    cleaned = cleaned.split(";", 1)[0].strip()
    cleaned = cleaned.rstrip(".")
    return cleaned


def detect_missing_build_dependencies(pkg_dir: Path) -> List[MissingDependency]:
    control = pkg_dir / "debian" / "control"
    if not control.exists():
        return []
    try:
        proc = subprocess.run(
            ["dpkg-checkbuilddeps"],
            cwd=str(pkg_dir),
            capture_output=True,
            text=True,
            check=False,
        )
    except FileNotFoundError:
        print_warn("dpkg-checkbuilddeps not found, unable to inspect missing build dependencies.")
        return []
    if proc.returncode == 0:
        return []
    combined = "\n".join(part for part in (proc.stderr, proc.stdout) if part).strip()
    if not combined:
        return []
    match = re.search(r"Unmet build dependencies:(.*)", combined, re.IGNORECASE | re.DOTALL)
    if not match:
        return []
    block = match.group(1)
    for sentinel in ("dpkg-checkbuilddeps:", "\n\n"):
        idx = block.find(sentinel)
        if idx != -1:
            block = block[:idx]
            break
    tokens = re.split(r",|\n", block)
    missing: List[MissingDependency] = []
    seen: Set[str] = set()
    for raw in tokens:
        display = raw.strip()
        if not display:
            continue
        candidates: List[str] = []
        segments = display.split("|") if "|" in display else display.split()
        for option in segments:
            candidate = _sanitize_dep_token(option)
            if candidate and candidate not in candidates:
                candidates.append(candidate)
        if not candidates:
            continue
        canonical = candidates[0]
        if canonical in seen:
            continue
        seen.add(canonical)
        missing.append(MissingDependency(display=display, candidates=candidates))
    return missing


def show_missing_dependencies(missing: List[MissingDependency]) -> None:
    if not missing:
        return
    width = len(str(len(missing)))
    border = "=" * 60
    print()
    print(border)
    print("缺失构建依赖 / Unmet build dependencies")
    print(border)
    for idx, dep in enumerate(missing, start=1):
        print(f"{idx:>{width}}. {dep.display}")
    print(border)
    sys.stdout.flush()  # 确保依赖列表先显示


def _strip_known_prefix(candidate: str) -> List[str]:
    variants = [candidate]
    for prefix in DEPENDENCY_SOURCE_PREFIXES:
        if candidate.startswith(prefix):
            stripped = candidate[len(prefix) :]
            if stripped and stripped not in variants:
                variants.append(stripped)
    return variants


def resolve_dependency_source(candidate: str, base_dir: Path) -> Optional[tuple[str, Path]]:
    for name_variant in _strip_known_prefix(candidate):
        path = (base_dir / name_variant).resolve()
        if path.exists() and (path / "debian").is_dir():
            return (name_variant, path)
    return None


def insert_missing_dependencies(
    entries: List[QueueEntry],
    selected: List[MissingDependency],
    base_dir: Path,
    current_index: int,
    completed: Set[str],
) -> List[str]:
    if not selected:
        return []
    inserted: List[str] = []
    insertion_offset = 0
    for dep in selected:
        target_entry: Optional[QueueEntry] = None
        for candidate in dep.valid_candidates():
            name_variants = _strip_known_prefix(candidate)
            if any(name in completed for name in name_variants):
                continue
            existing_index = None
            for idx in range(current_index + 1, len(entries)):
                if entries[idx].name in name_variants:
                    existing_index = idx
                    break
            if existing_index is not None:
                target_entry = entries.pop(existing_index)
                break
            resolved = resolve_dependency_source(candidate, base_dir)
            if resolved:
                resolved_name, resolved_path = resolved
                target_entry = QueueEntry(name=resolved_name, path=resolved_path, completed=False)
                break
        if not target_entry:
            print_warn(f"未能在工作区中找到依赖 {dep.display} 对应的包，已跳过。")
            continue
        if target_entry.name in inserted:
            continue
        entries.insert(current_index + insertion_offset, target_entry)
        inserted.append(target_entry.name)
        insertion_offset += 1
    return inserted


def has_deb_in_directory(directory: Path) -> bool:
    return any(directory.glob("*.deb"))


def install_directory(install_script: Path, directory: Path) -> None:
    if not has_deb_in_directory(directory):
        print_info(f"No .deb artifacts found in {directory}, skipping installation.")
        return
    cmd = build_install_cmd(install_script) + [str(directory)]
    proc = run_cmd(cmd, check=False)
    if proc.returncode != 0:
        print_warn(f"Failed to install artifacts from {directory}")


def install_from_workdir(install_script: Path, pkg_dir: Path) -> bool:
    cmd = build_install_cmd(install_script) + ["--work-dir", str(pkg_dir)]
    return run_cmd(cmd, check=False).returncode == 0


def mark_completed(queue_file: Path, completed: List[str], base_dir: Path) -> None:
    if not completed:
        return
    completed_set = set(completed)
    lines: List[str] = []
    with queue_file.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            stripped = raw_line.rstrip("\n")
            entry = parse_queue_line(stripped, base_dir)
            if not entry:
                lines.append(stripped)
                continue
            if entry.name not in completed_set:
                lines.append(stripped)
                continue
            if stripped.strip().startswith("{"):
                try:
                    data = json.loads(stripped.strip())
                except json.JSONDecodeError:
                    lines.append(stripped)
                    continue
                data["completed"] = True
                lines.append(json.dumps(data, ensure_ascii=False))
            else:
                payload = stripped.rstrip("#").strip() + "#"
                lines.append(payload)
    with queue_file.open("w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


def get_source_package_name(work_dir: Path, fallback: str) -> str:
    changelog = work_dir / "debian" / "changelog"
    if not work_dir.exists() or not changelog.exists():
        return fallback
    try:
        proc = subprocess.run(
            ["dpkg-parsechangelog", "-S", "Source"],
            cwd=str(work_dir),
            text=True,
            capture_output=True,
            check=False,
        )
        if proc.returncode == 0:
            candidate = proc.stdout.strip()
            if candidate:
                return candidate
    except FileNotFoundError:
        pass
    try:
        with changelog.open("r", encoding="utf-8") as handle:
            first_line = handle.readline()
    except OSError:
        return fallback
    match = re.match(r"([A-Za-z0-9.+-]+)\s*\(", first_line)
    if match:
        return match.group(1)
    return fallback


def has_prebuilt_artifact(entry: QueueEntry) -> bool:
    source_name = get_source_package_name(entry.path, entry.name)
    parent = entry.path.parent
    pattern = f"{source_name}_*.deb"
    for candidate in parent.glob(pattern):
        if not candidate.is_file():
            continue
        if "-dbgsym_" in candidate.name:
            continue
        return True
    return False


def auto_mark_prebuilt_packages(queue_file: Path, base_dir: Path) -> List[str]:
    candidates: List[str] = []
    with queue_file.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            entry = parse_queue_line(raw_line, base_dir)
            if not entry or entry.completed:
                continue
            if has_prebuilt_artifact(entry):
                candidates.append(entry.name)
    unique = list(dict.fromkeys(candidates))
    if not unique:
        return []
    print_info("Detected prebuilt packages, marking as completed: " + ", ".join(unique))
    mark_completed(queue_file, unique, base_dir)
    return unique


def apt_update() -> None:
    proc = run_with_lock_retries(["apt-get", "update"], check=False, use_apt_lock=True)
    if proc.returncode != 0:
        print_warn("apt-get update returned non-zero, continuing anyway.")


def apt_install(packages: Sequence[str]) -> None:
    if not packages:
        return
    run_apt_guard()
    run_with_lock_retries(
        ["apt-get", "install", "-y", "--no-install-recommends", *packages],
        use_apt_lock=True,
    )


def apt_fix_dependencies() -> None:
    run_with_lock_retries(
        ["apt-get", "-o", "Debug::pkgProblemResolver=yes", "-f", "install", "-y"],
        check=False,
        use_apt_lock=True,
    )


def build_install_cmd(install_script: Path) -> List[str]:
    cmd = [sys.executable, str(install_script)]
    if ALLOW_INSTALL_FAIL_AS_WARNING:
        cmd.append("--allow-install-fail-as-warning")
    return cmd


def install_base_packages(skip: bool) -> None:
    if skip:
        print_info("Skipping base dependency installation (requested).")
        return
    apt_update()
    apt_install(BASE_PACKAGES)


def get_deb_package_name(deb_path: Path) -> str:
    proc = subprocess.run(
        ["dpkg-deb", "-f", str(deb_path), "Package"],
        text=True,
        capture_output=True,
        check=False,
    )
    if proc.returncode != 0:
        return ""
    return proc.stdout.strip()


def is_package_installed(name: str) -> bool:
    proc = subprocess.run(
        ["dpkg-query", "-W", "-f", "${Status}", name],
        text=True,
        capture_output=True,
        check=False,
    )
    return proc.returncode == 0 and "install ok installed" in proc.stdout


def should_skip_package(pkg_name: str) -> bool:
    preferred = PREFERRED_PACKAGES.get(pkg_name)
    if not preferred:
        return False
    if is_package_installed(preferred):
        print_info(f"Skipping {pkg_name} because {preferred} is already installed.")
        return True
    return False


def install_single_deb(deb_path: Path) -> None:
    print_info(f"dpkg -i {deb_path}")
    proc = run_with_lock_retries(["dpkg", "-i", str(deb_path)], check=False, use_apt_lock=True)
    if proc.returncode == 0:
        return
    print_warn("dpkg reported missing dependencies, attempting apt-get -f install.")
    apt_fix_dependencies()
    proc = run_with_lock_retries(["dpkg", "-i", str(deb_path)], check=False, use_apt_lock=True)
    if proc.returncode != 0:
        print_warn(f"Failed to install {deb_path}, continuing without it.")


def install_third_party(third_dir: Path, skip: bool) -> None:
    if skip:
        print_info("Skipping third-party package installation (requested).")
        return
    if not third_dir.is_dir():
        print_warn(f"Third-party directory not found: {third_dir}")
        return
    print_info(f"Attempting to pre-install common ROS Python wheels from {third_dir}")
    for pattern in THIRD_PARTY_PATTERNS:
        matches = sorted(third_dir.glob(pattern))
        for deb_path in matches:
            pkg_name = get_deb_package_name(deb_path)
            if pkg_name and should_skip_package(pkg_name):
                continue
            install_single_deb(deb_path)


def install_build_deps(work_dir: Path, skip: bool) -> bool:
    if skip:
        print_info("Skipping mk-build-deps (requested).")
        return True
    control = work_dir / "debian" / "control"
    if not control.exists():
        print_warn(f"{control} not found, unable to resolve build dependencies automatically.")
        return True
    run_apt_guard()
    cmd = [
        "mk-build-deps",
        "-i",
        "-r",
        "-t",
        "apt-get -o Debug::pkgProblemResolver=yes --no-install-recommends -y",
        str(control),
    ]
    proc = run_with_lock_retries(cmd, cwd=work_dir, check=False, use_apt_lock=True)
    if proc.returncode != 0:
        print_warn("mk-build-deps failed, 请检查缺失依赖或稍后重试。")
        return False
    return True


def build_cmd(extra: Sequence[str]) -> List[str]:
    return list(DEFAULT_DEBUILD) + list(extra or [])


def run_debuild(work_dir: Path, extra: Sequence[str]) -> int:
    env = os.environ.copy()
    env.setdefault("DEBIAN_FRONTEND", "noninteractive")
    parallel = env.get("PARALLEL")
    if parallel:
        env.setdefault("DEB_BUILD_OPTIONS", f"parallel={parallel}")
    cmd = build_cmd(extra)
    print_info(f"cwd={work_dir}")
    print_info(f"Starting debuild for {work_dir.name}")
    proc = subprocess.run(cmd, cwd=str(work_dir), env=env)
    return proc.returncode


def run_build_pipeline(
    work_dir: Path,
    *,
    extra: Sequence[str],
    third_dir: Path,
    skip_base_deps: bool,
    skip_third_party: bool,
    skip_build_deps: bool,
) -> int:
    install_base_packages(skip_base_deps)
    install_third_party(third_dir, skip_third_party)
    if not install_build_deps(work_dir, skip_build_deps):
        return 1
    return run_debuild(work_dir, extra)


def collect_artifacts(work_dir: Path) -> List[Path]:
    parent = work_dir.parent
    candidates = sorted(
        path for path in parent.glob("*.deb") if path.is_file() and "-dbgsym_" not in path.name
    )
    return candidates


def extract_extra_args(args: argparse.Namespace) -> List[str]:
    extra = list(args.remainder or [])
    if extra and extra[0] == "--":
        extra = extra[1:]
    return extra


def create_missing_entries(selected: List[MissingDependency], base_dir: Path) -> List[QueueEntry]:
    entries: List[QueueEntry] = []
    seen: Set[str] = set()
    for dep in selected:
        target_entry: Optional[QueueEntry] = None
        for candidate in dep.valid_candidates():
            if candidate in seen:
                continue
            resolved = resolve_dependency_source(candidate, base_dir)
            if resolved:
                resolved_name, resolved_path = resolved
                target_entry = QueueEntry(name=resolved_name, path=resolved_path, completed=False)
                break
        if target_entry:
            seen.add(target_entry.name)
            entries.append(target_entry)
        else:
            print_warn(f"未能在 {base_dir} 下找到依赖 {dep.display} 对应的源码目录，已跳过。")
    return entries


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build Debian packages with automated dependency handling.")
    parser.add_argument(
        "--queue-file",
        type=Path,
        nargs="?",
        const=DEFAULT_QUEUE_FILE,
        help="批量构建: 指定队列文件（未提供路径时默认使用 build_queue.txt）。",
    )
    parser.add_argument(
        "--base-dir",
        type=Path,
        default=DEFAULT_BASE_DIR,
        help="批量构建: 工作区源码目录 (默认读取 AGIROS_CODE_DIR 或 /mnt/lfb_ws/jazzy)。",
    )
    parser.add_argument(
        "--install-script",
        type=Path,
        default=DEFAULT_INSTALL_SCRIPT,
        help="批量构建: 用于安装构建产物的脚本 (默认 deb_install_any.py)。",
    )
    parser.add_argument("--install-existing", action="store_true", help="批量构建: 先安装 base_dir 下已有的 .deb 文件。")
    parser.add_argument("--include-completed", action="store_true", help="批量构建: 包含已标记完成 (#) 的条目。")
    parser.add_argument(
        "--no-install-artifacts",
        dest="install_artifacts",
        action="store_false",
        help="批量构建: 跳过安装构建产物。",
    )
    parser.add_argument(
        "pkg_dir",
        nargs="?",
        type=Path,
        help="单包构建: 包目录 (默认 --work-dir, WORK_DIR 环境变量或当前目录)。",
    )
    parser.add_argument(
        "--work-dir",
        type=Path,
        default=None,
        help="单包构建: 指定包目录 (默认: WORK_DIR 环境变量或当前目录)。",
    )
    parser.add_argument(
        "--third-party-dir",
        type=Path,
        default=None,
        help="第三方 .deb 目录 (默认 /tmp，可用 THIRD_PARTY_DIR 环境变量覆盖)。",
    )
    parser.add_argument("--skip-base-deps", action="store_true", help="跳过基础依赖安装。")
    parser.add_argument("--skip-third-party", action="store_true", help="跳过第三方 .deb 预安装。")
    parser.add_argument("--skip-build-deps", action="store_true", help="跳过 mk-build-deps 步骤。")
    parser.add_argument(
        "remainder",
        nargs=argparse.REMAINDER,
        help="传递给 debuild 的额外参数 (使用 -- 分隔)。",
    )
    parser.set_defaults(install_artifacts=True)
    return parser.parse_args()


def resolve_work_dir(args: argparse.Namespace) -> Path:
    if args.pkg_dir and args.work_dir:
        raise SystemExit("Specify either positional pkg_dir or --work-dir, not both.")
    base = args.pkg_dir or args.work_dir
    if base is None:
        base = Path(os.environ.get("WORK_DIR", Path.cwd()))
    base = base.resolve()
    allow_invalid = allow_invalid_work_dir()
    missing_reason: Optional[str] = None
    if not base.exists():
        missing_reason = f"[ERR ] work_dir does not exist: {base}"
    elif not (base / "debian").is_dir():
        missing_reason = f"[ERR ] {base} is not a Debian package (missing debian/)."
    if missing_reason:
        if allow_invalid:
            print_warn(f"{missing_reason} -> 已忽略 (AGIROS_ALLOW_INVALID_WORKDIR=1)")
        else:
            raise SystemExit(missing_reason)
    return base


def process_queue_entries(
    entries: List[QueueEntry],
    *,
    args: argparse.Namespace,
    base_dir: Path,
    install_script: Path,
    third_dir: Path,
    extra_args: Sequence[str],
    mark_success: bool = False,
    queue_file: Optional[Path] = None,
) -> List[str]:
    if not entries:
        print_info("Queue is empty. Nothing to build.")
        return []

    built_successfully: List[str] = []
    completed_names: Set[str] = set()
    base_deps_installed = False
    idx = 0
    while idx < len(entries):
        entry = entries[idx]
        if not entry.path.exists():
            print_warn(f"Skipping {entry.name}: directory not found ({entry.path}).")
            idx += 1
            continue
        debian_dir = entry.path / "debian"
        if not debian_dir.is_dir():
            print_warn(f"Skipping {entry.name}: missing debian/ directory.")
            idx += 1
            continue
        print()
        print("==========================================")
        print(f"Building: {entry.name}")
        print("==========================================")
        skip_base = args.skip_base_deps or base_deps_installed
        rc = run_build_pipeline(
            entry.path,
            extra=extra_args,
            third_dir=third_dir,
            skip_base_deps=skip_base,
            skip_third_party=args.skip_third_party,
            skip_build_deps=args.skip_build_deps,
        )
        if rc != 0:
            missing = detect_missing_build_dependencies(entry.path)
            selected: List[MissingDependency] = []
            if missing:
                show_missing_dependencies(missing)
                print_info("自动尝试将缺失依赖加入队列并优先构建。")
                selected = missing
            inserted = insert_missing_dependencies(entries, selected, base_dir, idx, completed_names)
            if inserted:
                print_info("已将以下依赖插入到当前包之前准备构建: " + ", ".join(inserted))
                print_info("将从新的依赖开始继续构建，完成后会重新尝试当前包。")
                continue
            print_warn(f"{entry.name} failed to build.")
            idx += 1
            continue
        if not base_deps_installed and not args.skip_base_deps:
            base_deps_installed = True
        if args.install_artifacts:
            if not install_from_workdir(install_script, entry.path):
                print_warn(f"{entry.name} built but installation failed.")
                idx += 1
                continue
        artifacts = collect_artifacts(entry.path)
        if artifacts:
            print_info("Build artifacts:")
            for artifact in artifacts:
                print(f"  - {artifact}")
        else:
            print_warn("Build finished but no .deb artifacts were found next to the source directory.")
        built_successfully.append(entry.name)
        completed_names.add(entry.name)
        print_info(f"{entry.name} completed.")
        idx += 1

    if mark_success and queue_file:
        mark_completed(queue_file, built_successfully, base_dir)

    print()
    print_info(f"Build finished. Success={len(built_successfully)} / {len(entries)}")
    return built_successfully


def run_queue_mode(args: argparse.Namespace) -> None:
    queue_file = args.queue_file.resolve()
    base_dir = args.base_dir.resolve()
    install_script = args.install_script.resolve()
    third_source = args.third_party_dir if args.third_party_dir else THIRD_PARTY_DEFAULT
    third_dir = third_source.resolve()
    auto_mark_prebuilt_packages(queue_file, base_dir)
    entries = load_queue(queue_file, base_dir, include_completed=args.include_completed)
    if args.install_existing:
        install_directory(install_script, base_dir)
    extra_args = extract_extra_args(args)
    process_queue_entries(
        entries,
        args=args,
        base_dir=base_dir,
        install_script=install_script,
        third_dir=third_dir,
        extra_args=extra_args,
        mark_success=True,
        queue_file=queue_file,
    )


def run_single_mode(args: argparse.Namespace) -> None:
    work_dir = resolve_work_dir(args)
    third_source = args.third_party_dir if args.third_party_dir else THIRD_PARTY_DEFAULT
    third_dir = third_source.resolve()
    extra_args = extract_extra_args(args)
    if not work_dir.exists() or not (work_dir / "debian").is_dir():
        print_warn(
            f"{work_dir} 缺少 Debian 源目录 (missing debian/)，已跳过。"
            "若需强制校验，请设置 AGIROS_ALLOW_INVALID_WORKDIR=0。"
        )
        return
    rc = run_build_pipeline(
        work_dir,
        extra=extra_args,
        third_dir=third_dir,
        skip_base_deps=args.skip_base_deps,
        skip_third_party=args.skip_third_party,
        skip_build_deps=args.skip_build_deps,
    )
    if rc != 0:
        missing = detect_missing_build_dependencies(work_dir)
        if missing:
            print()
            print_warn("构建失败，检测到缺失的构建依赖:")
            show_missing_dependencies(missing)
            print_info("自动尝试构建缺失依赖后再继续当前包。")
            base_dir = args.base_dir.resolve()
            install_script = args.install_script.resolve()
            dep_entries = create_missing_entries(missing, base_dir)
            if dep_entries:
                dep_entries.append(QueueEntry(name=work_dir.name, path=work_dir, completed=False))
                built = process_queue_entries(
                    dep_entries,
                    args=args,
                    base_dir=base_dir,
                    install_script=install_script,
                    third_dir=third_dir,
                    extra_args=extra_args,
                    mark_success=False,
                    queue_file=None,
                )
                if built and built[-1] == work_dir.name:
                    return
            else:
                print_warn("未能定位缺失依赖的源码目录，请手动处理后重试。")
        raise SystemExit(rc)

    artifacts = collect_artifacts(work_dir)
    if not artifacts:
        print_warn("Build finished but no .deb artifacts were found next to the source directory.")
    else:
        print_info("Build artifacts:")
        for artifact in artifacts:
            print(f"  - {artifact}")


def main() -> None:
    args = parse_args()
    if args.queue_file is not None:
        run_queue_mode(args)
        return
    run_single_mode(args)


if __name__ == "__main__":
    main()
