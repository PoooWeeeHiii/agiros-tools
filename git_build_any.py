#!/usr/bin/env python3
"""
Python port of the original git_build_any.sh helper.

It keeps behaviour fully compatible with the shell script:
  * honours the same environment defaults (CODE_DIR, DEB_OUT, DISTRO, etc.)
  * supports ``--auto`` to build every package under CODE_DIR
  * performs the same cleaning, git-tagging, dependency install and artefact copy
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable, List, Optional


# ------------------------- Environment defaults -----------------------------
HOME = Path.home()
CODE_DIR = Path(os.environ.get("CODE_DIR", HOME / "agiros_tools" / "code_dir"))
DEB_OUT = Path(os.environ.get("DEB_OUT", HOME / "deb_out"))
DISTRO = os.environ.get("DISTRO", "loong")
DEFAULT_REL_INC = int(os.environ.get("DEFAULT_REL_INC", "1"))
PARALLEL = int(os.environ.get("PARALLEL", os.cpu_count() or 1))
GIT_USER_NAME = os.environ.get("GIT_USER_NAME", "PoooWeeeHiii")
GIT_USER_EMAIL = os.environ.get("GIT_USER_EMAIL", "powehi041210@gmail.com")

REQUIRED_CMDS = ("gbp", "git", "mk-build-deps", "dpkg-buildpackage")


# ------------------------------ Helpers ------------------------------------
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


def run(
    cmd: List[str],
    *,
    check: bool = True,
    cwd: Optional[Path] = None,
    capture: bool = False,
    env: Optional[dict[str, str]] = None,
) -> subprocess.CompletedProcess:
    log(f"$ {' '.join(cmd)}")
    return subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        check=check,
        text=True,
        capture_output=capture,
        env=env,
    )


def first_line(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").splitlines()[0]
    except Exception:
        return ""


def pkg_name_from_changelog(pkg_dir: Path) -> str:
    changelog = pkg_dir / "debian" / "changelog"
    line = first_line(changelog)
    if line:
        return line.split()[0]
    return pkg_dir.name


def version_from_changelog(pkg_dir: Path) -> str:
    changelog = pkg_dir / "debian" / "changelog"
    line = first_line(changelog)
    if line:
        match = re.search(r"\(([^)]+)\)", line)
        if match:
            return match.group(1)
    return ""


def version_from_package_xml(pkg_dir: Path) -> str:
    pkgxml = pkg_dir / "package.xml"
    if not pkgxml.exists():
        return ""
    text = pkgxml.read_text(encoding="utf-8", errors="ignore")
    match = re.search(r"<\s*version\s*>\s*([^<]+)\s*<\s*/\s*version\s*>", text)
    return match.group(1).strip() if match else ""


def ensure_gbp_conf(pkg_dir: Path) -> None:
    debian_dir = pkg_dir / "debian"
    debian_dir.mkdir(parents=True, exist_ok=True)
    gbp_conf = debian_dir / "gbp.conf"
    if gbp_conf.exists():
        return
    gbp_conf.write_text(
        "[git-buildpackage]\n"
        "upstream-tag=release/{distro}/{package}/{version}-{release_inc}\n"
        "upstream-tree=tag\n\n"
        "[buildpackage]\n"
        "upstream-tag=release/{distro}/{package}/{version}-{release_inc}\n"
        "upstream-tree=tag\n",
        encoding="utf-8",
    )
    log("created debian/gbp.conf")


def render_tag(pkg_dir: Path, pkg: str, version: str, release_inc: int) -> str:
    gbp_conf = pkg_dir / "debian" / "gbp.conf"
    for line in gbp_conf.read_text(encoding="utf-8").splitlines():
        if line.startswith("upstream-tag="):
            pattern = line.split("=", 1)[1]
            break
    else:
        return f"release/{DISTRO}/{pkg}/{version}-{release_inc}"
    mapping = {
        "distro": DISTRO,
        "package": pkg,
        "pkg": pkg,
        "version": version,
        "release_inc": release_inc,
    }
    # allow both {var} and :{var}
    pattern = re.sub(r":\{(\w+)\}", r"{\1}", pattern)
    try:
        return pattern.format(**mapping)
    except Exception:
        return pattern


def ensure_deb_source_options(pkg_dir: Path) -> None:
    source_dir = pkg_dir / "debian" / "source"
    source_dir.mkdir(parents=True, exist_ok=True)
    format_file = source_dir / "format"
    if not format_file.exists():
        format_file.write_text("3.0 (quilt)\n", encoding="utf-8")
    options_file = source_dir / "options"
    extend_line = (
        'extend-diff-ignore = "^(rpm/|rpm/SOURCES/|build/|CMakeFiles/|.*\\.tar(\\.gz|\\.xz|\\.bz2)?|.*\\.(bin|out)'
        "|^\\.obj-.*|CMakeCache\\.txt)$\"\n"
    )
    if options_file.exists():
        content = options_file.read_text(encoding="utf-8")
        if "extend-diff-ignore" in content:
            return
    with options_file.open("a", encoding="utf-8") as fh:
        fh.write(extend_line)


def ensure_gitignore_rpm(pkg_dir: Path) -> None:
    gitignore = pkg_dir / ".gitignore"
    entries = [
        "rpm/",
        "build/",
        ".obj-*",
        "CMakeFiles/",
        "CMakeCache.txt",
        "*.tar",
        "*.tar.gz",
        "*.tar.xz",
        "*.tar.bz2",
        "*.bin",
        "*.out",
    ]
    existing = set()
    if gitignore.exists():
        existing.update(line.strip() for line in gitignore.read_text(encoding="utf-8").splitlines())
    with gitignore.open("a", encoding="utf-8") as fh:
        for entry in entries:
            if entry not in existing:
                fh.write(entry + "\n")


def delete_path(path: Path) -> None:
    if path.is_dir():
        shutil.rmtree(path, ignore_errors=True)
    elif path.exists():
        path.unlink(missing_ok=True)  # type: ignore[arg-type]


def nuke_local_artifacts(pkg_dir: Path) -> None:
    delete_path(pkg_dir / "rpm")
    for tar in pkg_dir.glob("*.tar*"):
        if not tar.match("debian/*"):
            delete_path(tar)
    for pattern in ("build", ".obj-*", "CMakeFiles", "CMakeCache.txt", "cmake_install.cmake"):
        for target in pkg_dir.glob(pattern):
            delete_path(target)


def clean_rpm_artifacts(pkg_dir: Path) -> None:
    rpm_dir = pkg_dir / "rpm"
    if not rpm_dir.exists():
        return
    for path in ["BUILD", "BUILDROOT", "RPMS", "SRPMS", "tmp"]:
        delete_path(rpm_dir / path)
    sources = rpm_dir / "SOURCES"
    if sources.exists():
        for tar in sources.glob("*.tar*"):
            delete_path(tar)


def ensure_git_repo(pkg_dir: Path) -> None:
    git_dir = pkg_dir / ".git"
    if git_dir.exists():
        run(["git", "config", "user.name", GIT_USER_NAME], check=False, cwd=pkg_dir)
        run(["git", "config", "user.email", GIT_USER_EMAIL], check=False, cwd=pkg_dir)
        return
    run(["git", "init"], cwd=pkg_dir)
    run(["git", "config", "user.name", GIT_USER_NAME], cwd=pkg_dir)
    run(["git", "config", "user.email", GIT_USER_EMAIL], cwd=pkg_dir)
    run(["git", "add", "-A"], cwd=pkg_dir)
    run(["git", "commit", "-m", "Initial import"], cwd=pkg_dir)
    log(f"initialized nested git repo in {pkg_dir}")


def ensure_tag(pkg_dir: Path, tag: str) -> None:
    result = run(["git", "rev-parse", f"{tag}^{{tree}}"], cwd=pkg_dir, check=False, capture=True)
    if result.returncode == 0:
        log(f"tag exists: {tag}")
        return
    run(["git", "add", "-A"], cwd=pkg_dir, check=False)
    run(["git", "commit", "-m", f"Prepare for {tag}"], cwd=pkg_dir, check=False)
    run(["git", "tag", "-a", tag, "-m", tag], cwd=pkg_dir)
    log(f"created tag: {tag}")


def install_build_deps(pkg_dir: Path) -> None:
    run(["sudo", "apt-get", "update"], check=False, cwd=pkg_dir)
    run(["sudo", "apt-get", "install", "-y", "devscripts", "equivs"], check=False, cwd=pkg_dir)
    control = pkg_dir / "debian" / "control"
    if control.exists():
        run(["sudo", "mk-build-deps", "-i", "-r", "-t", "apt-get -y", str(control)], check=False, cwd=pkg_dir)


def is_python_pybuild(pkg_dir: Path) -> bool:
    rules = pkg_dir / "debian" / "rules"
    if rules.exists() and "pybuild" in rules.read_text(encoding="utf-8", errors="ignore"):
        return True
    for filename in ("pyproject.toml", "setup.py", "setup.cfg"):
        if (pkg_dir / filename).exists():
            return True
    control = pkg_dir / "debian" / "control"
    if control.exists():
        text = control.read_text(encoding="utf-8", errors="ignore")
        if re.search(r"dh-python|python3-all|python3-.*-dev", text):
            return True
    return False


def python_pre_clean(pkg_dir: Path) -> None:
    run(["fakeroot", "debian/rules", "clean"], check=False, cwd=pkg_dir)
    for path in [
        ".pc",
        ".pybuild",
        ".pytest_cache",
        ".eggs",
        "build",
        "dist",
        "debian/.debhelper",
        "debian/debhelper-build-stamp",
        "debian/files",
    ]:
        delete_path(pkg_dir / path)
    for pattern in ("*.debhelper.log", "*.substvars"):
        for target in (pkg_dir / "debian").glob(pattern):
            delete_path(target)
    parent = pkg_dir.parent
    for pattern in ("*.deb", "*.dsc", "*.changes", "*.build", "*.buildinfo", "*.orig.tar.*"):
        for target in parent.glob(pattern):
            delete_path(target)
    for pattern in (".obj-*", "__pycache__", "CMakeFiles", "CMakeCache.txt", "cmake_install.cmake"):
        for target in pkg_dir.glob(pattern):
            delete_path(target)


def python_install_build_deps(pkg_dir: Path) -> None:
    run(
        [
            "sudo",
            "apt-get",
            "install",
            "-y",
            "devscripts",
            "debhelper",
            "dh-python",
            "python3-all",
            "python3-setuptools",
            "python3-wheel",
            "python3-pip",
            "python3-pytest",
            "python3-flake8",
            "fakeroot",
            "git-buildpackage",
        ],
        check=False,
        cwd=pkg_dir,
    )
    control = pkg_dir / "debian" / "control"
    if control.exists():
        run(["sudo", "mk-build-deps", "-i", "-r", "-t", "apt-get -y", str(control)], check=False, cwd=pkg_dir)


def python_build_with_gbp(pkg_dir: Path) -> None:
    env = os.environ.copy()
    env["DEB_BUILD_OPTIONS"] = f"parallel={PARALLEL}"
    run(
        [
            "gbp",
            "buildpackage",
            "--git-ignore-branch",
            "--git-ignore-new",
            "--git-no-pristine-tar",
            "--git-upstream-tree=HEAD",
            "--git-builder=debuild -us -uc",
        ],
        cwd=pkg_dir,
        env=env,
    )


def copy_artifacts(pkg_dir: Path, pkg_name: str) -> None:
    dest = DEB_OUT / DISTRO / pkg_name
    dest.mkdir(parents=True, exist_ok=True)
    parent = pkg_dir.parent
    patterns = [
        f"{pkg_name}_*",
        f"{pkg_name}-dbgsym_*",
        "*.build",
        "*.changes",
    ]
    for pattern in patterns:
        for src in parent.glob(pattern):
            if src.exists():
                shutil.copy(src, dest / src.name)
    log(f"Artifacts => {dest}")


def build_one_pkg(pkg_dir: Path) -> bool:
    log(f"==== Build: {pkg_dir} ====")
    if not (pkg_dir / "debian").is_dir():
        err(f"missing debian/ in {pkg_dir}")
        return False

    nuke_local_artifacts(pkg_dir)
    ensure_gbp_conf(pkg_dir)
    ensure_deb_source_options(pkg_dir)
    ensure_gitignore_rpm(pkg_dir)
    clean_rpm_artifacts(pkg_dir)

    pkg_name = pkg_name_from_changelog(pkg_dir)
    version = version_from_changelog(pkg_dir) or version_from_package_xml(pkg_dir) or "0.0.0"

    ensure_git_repo(pkg_dir)
    tag = render_tag(pkg_dir, pkg_name, version, DEFAULT_REL_INC)
    ensure_tag(pkg_dir, tag)

    try:
        if is_python_pybuild(pkg_dir):
            log("Detected Python/pybuild packaging → use Python flow")
            python_pre_clean(pkg_dir)
            python_install_build_deps(pkg_dir)
            python_build_with_gbp(pkg_dir)
        else:
            install_build_deps(pkg_dir)
            env = os.environ.copy()
            env["DEB_BUILD_OPTIONS"] = f"parallel={PARALLEL}"
            current_branch = subprocess.run(
                ["git", "rev-parse", "--abbrev-ref", "HEAD"],
                cwd=pkg_dir,
                text=True,
                capture_output=True,
                check=True,
            ).stdout.strip()
            run(
                [
                    "gbp",
                    "buildpackage",
                    "--git-ignore-branch",
                    "--git-ignore-new",
                    "--git-no-pristine-tar",
                    f"--git-debian-branch={current_branch}",
                    "-us",
                    "-uc",
                ],
                cwd=pkg_dir,
                env=env,
            )
    except subprocess.CalledProcessError:
        err(f"gbp build failed in {pkg_dir}")
        return False

    copy_artifacts(pkg_dir, pkg_name)
    return True


def find_packages(base: Path) -> Iterable[Path]:
    for debian_dir in base.rglob("debian"):
        if debian_dir.is_dir():
            yield debian_dir.parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build Debian packages using git-buildpackage.")
    parser.add_argument("--auto", action="store_true", help="build every package under CODE_DIR")
    parser.add_argument("--path", type=Path, help="explicit package path for single build (default: current directory)")
    return parser.parse_args()


def main() -> None:
    for cmd in REQUIRED_CMDS:
        ensure_command(cmd)

    args = parse_args()
    DEB_OUT.mkdir(parents=True, exist_ok=True)

    if args.auto:
        ok = 0
        fail = 0
        fail_list = DEB_OUT / "fail.list"
        fail_list.write_text("", encoding="utf-8")
        unique_dirs = sorted({path.resolve() for path in find_packages(CODE_DIR)})
        for pkg_dir in unique_dirs:
            if build_one_pkg(pkg_dir):
                ok += 1
            else:
                fail += 1
                with fail_list.open("a", encoding="utf-8") as fh:
                    fh.write(str(pkg_dir) + "\n")
        log(f"Done. success={ok} failed={fail}")
        if fail > 0:
            warn(f"Failed list => {fail_list}")
            raise SystemExit(1)
        return

    target_dir = args.path or Path.cwd()
    if not build_one_pkg(target_dir.resolve()):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
