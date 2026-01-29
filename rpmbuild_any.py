#!/usr/bin/env python3
"""Python port of rpmbuild_any.sh."""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable, List


HOME = Path.home()
CODE_DIR = Path(os.environ.get("CODE_DIR", "/opt/code_dir"))
WORK_DIR = Path(os.environ.get("WORK_DIR", os.getcwd()))
RPMS_OUT = Path(os.environ.get("RPMS_OUT", HOME / "rpms_out"))
OS_NAME = os.environ.get("OS_NAME", "openeuler")
OS_VERSION = os.environ.get("OS_VERSION", "24")
ARCH = os.environ.get("ARCH", os.uname().machine)
AGIROSDEP_INDEX = os.environ.get("AGIROSDEP_INDEX", "http://1.94.193.239/yumrepo/agiros/agirosdep/index-v4.yaml")
AGIROSDEP_WHL = os.environ.get("AGIROSDEP_WHL", "http://1.94.193.239/yumrepo/agiros/agirosdep/agirosdep-0.25.1-py3-none-any.whl")
YUM_BASEURL = os.environ.get("YUM_BASEURL", f"http://1.94.193.239/yumrepo/agiros/openeuler2403lts/2506/{ARCH}")
YUM_SECTION = os.environ.get("YUM_SECTION", "release")
BLOOM_BIN = os.environ.get("BLOOM_BIN", "bloom-generate")
SKIP_DEBUG = int(os.environ.get("SKIP_DEBUG", "1"))
VERBOSE = int(os.environ.get("VERBOSE", "0"))

REQUIRED_CMDS = ("rpmbuild", "rpmdev-setuptree", "git", "python3", "dnf")


def log(message: str) -> None:
    print(f"[INFO] {message}")


def warn(message: str) -> None:
    print(f"[WARN] {message}", file=sys.stderr)


def err(message: str) -> None:
    print(f"[ERR ] {message}", file=sys.stderr)


def ensure_command(name: str) -> None:
    if shutil.which(name) is None:
        err(f"missing command: {name}")
        raise SystemExit(127)


def run(cmd: List[str], *, cwd: Path | None = None, check: bool = True) -> subprocess.CompletedProcess:
    if VERBOSE:
        log(f"$ {' '.join(cmd)}")
    return subprocess.run(cmd, cwd=str(cwd) if cwd else None, check=check, text=True)


def try_run(cmd: List[str], *, cwd: Path | None = None) -> None:
    try:
        run(cmd, cwd=cwd, check=False)
    except Exception as exc:  # pragma: no cover - best-effort cleanup
        warn(f"command failed (ignored): {' '.join(cmd)} -> {exc}")


def maybe_install_agirosdep() -> None:
    script = f"""
import importlib, subprocess, sys
try:
    importlib.import_module('agirosdep')
except Exception:
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', '--upgrade', 'pip'])
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', '{AGIROSDEP_WHL}'])
"""
    subprocess.run([sys.executable, "-c", script], check=False)


def ensure_repo() -> None:
    repo = Path("/etc/yum.repos.d/agiros.repo")
    content = (
        "[agiros]\n"
        "name=AGIROS repo\n"
        f"baseurl={YUM_BASEURL}/{YUM_SECTION}/\n"
        "enabled=1\n"
        "gpgcheck=0\n"
    )
    repo.write_text(content, encoding="utf-8")


def has_spec(pkg_dir: Path) -> Path | None:
    specs = sorted(pkg_dir.glob("rpm/*.spec"))
    return specs[0] if specs else None


def ensure_sources_dir(pkg_dir: Path) -> Path:
    sources = pkg_dir / "rpm" / "SOURCES"
    sources.mkdir(parents=True, exist_ok=True)
    return sources


def pkg_version(pkg_dir: Path) -> str:
    pkgxml = pkg_dir / "package.xml"
    if not pkgxml.exists():
        return ""
    text = pkgxml.read_text(encoding="utf-8", errors="ignore")
    match = re_search(r"<\s*version\s*>\s*([^<]+)\s*<\s*/\s*version\s*>", text)
    return match if match else ""


def re_search(pattern: str, text: str) -> str | None:
    import re

    match = re.search(pattern, text)
    return match.group(1).strip() if match else None


def make_source_tar(pkg_dir: Path, spec_path: Path) -> Path:
    ensure_sources_dir(pkg_dir)
    pkg = pkg_dir.name
    version = pkg_version(pkg_dir) or "0.0.0"
    with spec_path.open("r", encoding="utf-8") as fh:
        lines = fh.readlines()
    name_line = next((line for line in lines if line.startswith("Name:")), None)
    spec_name = name_line.split()[1] if name_line else pkg

    # normalise Source0 and %setup
    content = "".join(lines)
    if "Source0:" in content:
        content = content.replace(next(line for line in lines if line.startswith("Source0:")), "Source0: %{name}-%{version}.tar.gz\n", 1)
    else:
        content += "\nSource0: %{name}-%{version}.tar.gz\n"
    if "%setup" in content:
        content = re_sub(r"(%setup[^\n]*-n)[ \t]+[^\n]+", r"\\1 %{name}-%{version}", content)
    spec_path.write_text(content, encoding="utf-8")

    tar_name = f"{spec_name}-{version}.tar.gz"
    tar_path = pkg_dir / "rpm" / "SOURCES" / tar_name
    if tar_path.exists():
        tar_path.unlink()

    parent = pkg_dir.parent
    base = pkg_dir.name
    exclude = ["--exclude", f"{base}/debian", "--exclude", f"{base}/rpm", "--exclude-vcs"]
    transform = ["--transform", f"s@^{base}@{spec_name}-{version}@"]
    run([
        "tar",
        "-C",
        str(parent),
        *exclude,
        *transform,
        "-czf",
        str(tar_path),
        base,
    ])
    log(f"Tar created successfully: {tar_path.relative_to(pkg_dir)}")
    return tar_path


def re_sub(pattern: str, repl: str, text: str) -> str:
    import re

    return re.sub(pattern, repl, text, count=1)


def build_one_pkg(pkg_dir: Path) -> bool:
    log(f"==== Build: {pkg_dir} ====")
    spec = has_spec(pkg_dir)
    if spec is None:
        log("No spec found, try bloom-generate")
        env = os.environ.copy()
        env["ROS_OS_OVERRIDE"] = f"{OS_NAME}:{OS_VERSION}"
        env.setdefault("AGIROS_ROS_DISTRO", "loong")
        try:
            run([BLOOM_BIN, "agirosrpm", "--ros-distro", "loong", "--os-name", OS_NAME, "--os-version", OS_VERSION], cwd=pkg_dir)
        except subprocess.CalledProcessError:
            err(f"bloom generate failed in {pkg_dir}")
            return False
        spec = has_spec(pkg_dir)
        if spec is None:
            err(f"spec still missing after bloom for {pkg_dir}")
            return False

    make_source_tar(pkg_dir, spec)
    try_run(["dnf", "builddep", "-y", str(spec)])

    rpmb_args = [
        "rpmbuild",
        "-ba",
        str(spec),
        "--define",
        f"_topdir   {HOME}/rpmbuild",
        "--define",
        f"_sourcedir {pkg_dir}/rpm/SOURCES",
        "--define",
        f"_specdir  {pkg_dir}/rpm",
        "--define",
        f"_builddir {HOME}/rpmbuild/BUILD",
        "--define",
        f"_srcrpmdir {HOME}/rpmbuild/SRPMS",
        "--define",
        f"_rpmdir   {HOME}/rpmbuild/RPMS",
    ]
    if SKIP_DEBUG == 1:
        rpmb_args.extend(
            [
                "--define",
                "debug_package %{nil}",
                "--define",
                "_enable_debug_packages 0",
                "--define",
                "_debuginfo_packages 0",
                "--define",
                "_debugsource_packages 0",
            ]
        )

    try:
        run(rpmb_args, cwd=pkg_dir)
    except subprocess.CalledProcessError:
        err(f"rpmbuild failed in {pkg_dir}")
        return False

    target = RPMS_OUT / ARCH / pkg_dir.name
    target.mkdir(parents=True, exist_ok=True)
    copy_patterns(Path(HOME) / "rpmbuild" / "RPMS" / ARCH, target, "*.rpm")
    copy_patterns(Path(HOME) / "rpmbuild" / "SRPMS", target, "*.src.rpm")
    log(f"Artifacts => {target}")
    return True


def copy_patterns(source_dir: Path, target_dir: Path, pattern: str) -> None:
    for path in source_dir.glob(pattern):
        if path.exists():
            shutil.copy(path, target_dir / path.name)


def find_packages(base: Path) -> Iterable[Path]:
    for pkgxml in base.rglob("package.xml"):
        yield pkgxml.parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build RPM packages")
    parser.add_argument("--auto", action="store_true", help="auto-discover packages under CODE_DIR")
    parser.add_argument("--path", type=Path, help="explicit package path (default: WORK_DIR)")
    return parser.parse_args()


def main() -> None:
    for cmd in REQUIRED_CMDS:
        ensure_command(cmd)

    maybe_install_agirosdep()
    run(["rpmdev-setuptree"], check=False)
    ensure_repo()
    run(["dnf", "makecache", "-y"], check=False)

    args = parse_args()
    RPMS_OUT.mkdir(parents=True, exist_ok=True)

    if args.auto:
        ok = fail = 0
        failed_log = Path.cwd() / "fail.log"
        failed_log.write_text("", encoding="utf-8")
        for pkg_dir in sorted({path.resolve() for path in find_packages(CODE_DIR)}):
            if build_one_pkg(pkg_dir):
                ok += 1
            else:
                fail += 1
                with failed_log.open("a", encoding="utf-8") as fh:
                    fh.write(str(pkg_dir) + "\n")
        log(f"Done. success={ok}, failed={fail}")
        if fail:
            warn(f"Failed list written to {failed_log}")
            raise SystemExit(1)
        return

    target = (args.path or WORK_DIR).resolve()
    if not build_one_pkg(target):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
