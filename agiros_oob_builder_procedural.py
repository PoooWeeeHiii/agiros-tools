#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
改进版 AGIROS OOB 构建器（过程式，支持 resume 与错误重跑，多包支持，自动回退 openEuler 版本）
- 解析 tracks.yaml 的 jazzy 段
- 根据 actions 判断需要生成 debian/spec
- 已生成但错误的（如包含 !nocheck）会重跑
- 正确生成的会跳过
- 将失败记录保存到 fail.log
- 支持单个源码目录下多个 package.xml 的子包，逐个处理
- 生成 spec 时优先尝试 openEuler:24（示例），如果 agirosdep 缺失则回退到其他 openEuler 版本
- 所有缺失的 rosdep rules 会记录到 fail.log，交互默认 "n"

本次微调：
- 将 argparse 默认值提取到 全局 Defaults 区，保持其余代码不变；
- 通过 Defaults 提供统一默认值，必要时可被环境变量覆盖。
- 新增 `--generate-gbp` 开关：在批量生成 debian/ 时，
  通过给 `bloom-generate agirosdebian` 追加 `--generate-gbp`，并注入
  `OOB_TRACKS_DIR` / `OOB_TRACKS_DISTRO` 环境变量，批量为各包生成 `debian/gbp.conf`。
"""
import shlex
import argparse
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, Optional
import yaml
import shlex  # 新增：支持把 BLOOM_BIN 解析为多段命令


# ----------------------------- Global Defaults -----------------------------
class Defaults:
    """集中管理 argparse 默认值，可通过环境变量覆盖。"""
    AGIROS_DISTRO: str = os.environ.get("AGIROS_DISTRO", "loong")
    ROS2_DISTRO: str = os.environ.get("ROS2_DISTRO", "jazzy")
    OS_NAMECODE: str = os.environ.get("OS_NAMECODE", "Unknow")
    UBUNTU_DEFAULT: str = os.environ.get("AGIROS_UBUNTU_DEFAULT", "jammy")
    OPENEULER_DEFAULT: str = os.environ.get("AGIROS_OE_DEFAULT", "24")
    # 多值用逗号分隔覆盖：AGIROS_OE_FALLBACK="22,23"
    OPENEULER_FALLBACK_RAW: str = os.environ.get("AGIROS_OE_FALLBACK", "22,23")
    OPENEULER_FALLBACK = [v.strip() for v in OPENEULER_FALLBACK_RAW.split(',') if v.strip()]
    # 修复：在类体内直接定义字段，且默认值为单命令名；允许环境变量传入多段命令（稍后用 shlex.split 解析）
    BLOOM_BIN: str = os.environ.get(
        "AGIROS_BLOOM_BIN",
        "bloom-generate"
    )
    LIMIT: int = int(os.environ.get("AGIROS_LIMIT", "0"))
    DRY_RUN_DEFAULT: bool = bool(int(os.environ.get("AGIROS_DRYRUN", "0")))
    GENERATE_GBP_DEFAULT: bool = bool(int(os.environ.get("AGIROS_GENERATE_GBP", "0")))


def log(msg: str):
    print(msg, flush=True)


def run(cmd, cwd=None, dry_run=False, env: Optional[Dict[str, str]] = None):
    shown = " ".join(map(str, cmd))
    prefix = "[DRY]" if dry_run else "[RUN]"
    env_hint = f" env[OOB_TRACKS_DIR]={env.get('OOB_TRACKS_DIR')}" if env and 'OOB_TRACKS_DIR' in env else ""
    log(f"{prefix} {shown}{env_hint} (cwd={cwd or os.getcwd()})")
    if dry_run:
        return 0, None
    proc = subprocess.Popen(
        cmd,
        cwd=str(cwd) if cwd else None,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env if env is not None else os.environ.copy(),
    )
    out_lines = []
    for line in proc.stdout:
        sys.stdout.write(line)
        sys.stdout.flush()
        out_lines.append(line)
        if "Continue [Y/n]?" in line:
            proc.stdin.write("n\n")
            proc.stdin.flush()
    proc.wait()
    rc = proc.returncode
    if rc != 0:
        return rc, "\n".join(out_lines)
    return rc, None

def build_cmd_for(kind: str, common_bin: str) -> list:
    """
    - kind == 'debian'：优先用环境变量 AGIROS_BLOOM_BIN_DEBIAN
    - 其他：用 common_bin（即 --bloom-bin）
    """
    if kind == 'debian':
        deb_bin = os.environ.get("AGIROS_BLOOM_BIN_DEBIAN", "").strip()
        if deb_bin:
            return shlex.split(deb_bin)
    return shlex.split(common_bin)

def is_direct_module_cmd(cmd_tokens: list) -> bool:
    """是否是模块入口（如 '... generate_cmd'），若是则不要再拼子命令名。"""
    return "generate_cmd" in " ".join(cmd_tokens)


# ----------------------------- Resume 检查 -----------------------------

def is_valid_debian(pkg_dir: Path) -> bool:
    control = pkg_dir / "debian" / "control"
    if not control.exists():
        return False
    text = control.read_text(encoding="utf-8", errors="ignore")
    if "!nocheck" in text:
        return False
    if "Depends:" not in text:
        return False
    return True


def is_valid_spec(pkg_dir: Path) -> bool:
    rpm_dir = pkg_dir / "rpm"
    if not rpm_dir.is_dir():
        return False
    specs = list(rpm_dir.glob("*.spec"))
    if not specs:
        return False
    for s in specs:
        text = s.read_text(encoding="utf-8", errors="ignore")
        if "!nocheck" in text:
            return False
    return True


# ----------------------------- Tracks Parser -----------------------------

class TracksParser:
    def __init__(self, distro: str = "jazzy"):
        self.distro = distro

    def _find_distro_case_insensitive(self, tracks: Dict[str, Any]) -> Optional[str]:
        target = self.distro.lower()
        for k in tracks.keys():
            if isinstance(k, str) and k.lower() == target:
                return k
        for k in tracks.keys():
            if isinstance(k, str) and target in k.lower():
                return k
        return None

    def parse_file(self, tracks_yaml_path: Path) -> Optional[Dict[str, Any]]:
        try:
            data = yaml.safe_load(tracks_yaml_path.read_text(encoding="utf-8")) or {}
        except Exception as e:
            log(f"[WARN] 解析 YAML 失败: {tracks_yaml_path} -> {e}")
            return None

        tracks = None
        if isinstance(data, dict) and "tracks" in data:
            tracks = data["tracks"]
        elif isinstance(data, dict):
            tracks = data
        else:
            return None

        key = self._find_distro_case_insensitive(tracks)
        if key is None:
            return None
        section = tracks.get(key)
        return section if isinstance(section, dict) else None


# ----------------------------- Main Flow -----------------------------

def find_subpackages(pkg_dir: Path):
    results = []
    for root, dirs, files in os.walk(pkg_dir):
        if "package.xml" in files:
            results.append(Path(root))
    return results if results else [pkg_dir]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--release-dir", required=True)
    ap.add_argument("--code-dir", required=True)
    ap.add_argument("--ros-distro", default=Defaults.AGIROS_DISTRO)
    ap.add_argument("--ros2-distro", default=Defaults.ROS2_DISTRO)
    ap.add_argument("--ubuntu-default", default=Defaults.UBUNTU_DEFAULT)

    # 将原来的 rhel-* 参数替换为 openeuler-* 参数
    ap.add_argument("--openeuler-default", default=Defaults.OPENEULER_DEFAULT,
                    help="openEuler 首选版本，如 22/23/24")
    ap.add_argument("--openeuler-fallback", nargs="*", default=Defaults.OPENEULER_FALLBACK,
                    help="openEuler 版本回退列表，按顺序尝试")

    ap.add_argument("--bloom-bin", default=Defaults.BLOOM_BIN)
    ap.add_argument("--limit", type=int, default=Defaults.LIMIT)
    ap.add_argument("--dry-run", action="store_true", default=Defaults.DRY_RUN_DEFAULT)

    # 控制是否在生成 debian 时同步生成 gbp.conf
    ap.add_argument("--generate-gbp", action="store_true", default=Defaults.GENERATE_GBP_DEFAULT,
                    help="批量生成 debian/ 时，同时生成 debian/gbp.conf（需 agirosdebian 支持）")
    ap.add_argument("--mode", choices=["debian", "spec", "both", "gbp"], default="both",
                    help="控制批量构建模式：仅生成 debian、仅 spec、全部或仅生成 gbp.conf")

    args = ap.parse_args()

    _distro = args.ros2_distro
    release_dir = Path(args.release_dir)
    code_dir = Path(args.code_dir)
    fail_log = Path("fail.log")

    log(f"[INFO] 初始化 OOB Builder: code_dir={code_dir}, release_dir={release_dir}")

    if args.mode == "gbp":
        args.generate_gbp = True

    parser = TracksParser(_distro)
    pkgs = []
    for child in sorted(release_dir.iterdir()):
        if not child.is_dir():
            continue
        for name in ("tracks.yaml", "track.yaml"):
            p = child / name
            if p.exists():
                pkgs.append((child.name, p))
                break

    log(f"[INFO] 开始扫描 {release_dir}... 共发现 {len(pkgs)} 个含 tracks.yaml 的包目录")

    total = 0
    with fail_log.open("w", encoding="utf-8") as flog:
        for pkg_name, yaml_path in pkgs[: args.limit or len(pkgs)]:
            section = parser.parse_file(yaml_path)
            if not section:
                log(f"[SKIP] {pkg_name}/{yaml_path.name}: 无 {_distro} 段")
                continue

            actions = section.get("actions") or []
            if not actions:
                log(f"[SKIP] {pkg_name}/{yaml_path.name}: {_distro}.actions 为空")
                continue

            # 检测需求
            need_ubuntu = any("--os-name ubuntu" in a for a in actions)
            need_oe = any("--os-name openeuler" in a for a in actions) or any("--os-name rhel" in a for a in actions)
            if not (need_ubuntu or need_oe):
                log(f"[SKIP] {pkg_name}: 无 ubuntu/openeuler 相关 actions")
                continue

            pkg_dir = code_dir / pkg_name
            if not pkg_dir.is_dir():
                log(f"[SKIP] {code_dir.name} 中不存在: {pkg_dir.name}")
                continue

            subpackages = find_subpackages(pkg_dir)

            for subpkg in subpackages:
                log(f"[INFO] 处理包: {pkg_name}/{subpkg.relative_to(pkg_dir)}")

                sub_need_ubuntu = need_ubuntu and args.mode in ("debian", "both", "gbp")
                sub_need_oe = need_oe and args.mode in ("spec", "both")

                if args.mode == "gbp":
                    deb_dir = subpkg / "debian"
                    if not deb_dir.is_dir():
                        log(f"[SKIP] {pkg_name}: 未找到 debian/ 目录，无法生成 gbp.conf。")
                        continue
                if args.mode != "gbp" and sub_need_ubuntu and is_valid_debian(subpkg):
                    log(f"[RESUME] {pkg_name}: 已有有效 debian，跳过 ubuntu 生成。")
                    sub_need_ubuntu = False
                if sub_need_oe and is_valid_spec(subpkg):
                    log(f"[RESUME] {pkg_name}: 已有有效 spec，跳过 openEuler 生成。")
                    sub_need_oe = False
                if not (sub_need_ubuntu or sub_need_oe):
                    if args.mode == "gbp":
                        gbp_conf = subpkg / "debian" / "gbp.conf"
                        if gbp_conf.exists():
                            log(f"[RESUME] {pkg_name}: 已存在 gbp.conf，跳过。")
                        else:
                            sub_need_ubuntu = True
                    else:
                        continue
                if args.mode == "gbp" and not sub_need_ubuntu:
                    continue

                try:
                    if sub_need_ubuntu:
                        base = build_cmd_for('debian', args.bloom_bin)
                        deb_cmd = base + (["agirosdebian"] if not is_direct_module_cmd(base) else []) + [
                            "--ros-distro", args.agiros_distro,
                            "--os-name", "ubuntu",
                            "--os-version", args.ubuntu_default,
                        ]

                        deb_env = None
                        if args.generate_gbp:
                            # 通过环境变量把 tracks 目录与目标 distro 注入给 agirosdebian
                            deb_env = os.environ.copy()
                            deb_env["OOB_TRACKS_DIR"] = str(release_dir)
                            tracks_key = deb_env.get("ROS2_DISTRO") or deb_env.get("OOB_TRACKS_DISTRO") or "jazzy"
                            deb_env["OOB_TRACKS_DISTRO"] = tracks_key
                            deb_env["AGIROS_DISTRO"] = args.agiros_distro
                            deb_cmd.append("--generate-gbp")
                            deb_cmd.extend(["--tracks-distro", tracks_key])
                            deb_cmd.extend(["--distro", args.agiros_distro])
                            deb_cmd.extend(["--pkg", subpkg.name])
                        rc, out = run(deb_cmd, cwd=subpkg, dry_run=args.dry_run, env=deb_env)
                        if rc == 0:
                            total += 1
                            log(f"[OK] {pkg_name}: 已生成 debian/ {'(含 gbp.conf)' if args.generate_gbp else ''}")
                        else:
                            with fail_log.open("a", encoding="utf-8") as flog2:
                                flog2.write(f"{pkg_name} ubuntu 生成debian失败 rc={rc}\n")
                                if out:
                                    for l in out.splitlines():
                                        if "no agirosdep rule for" in l:
                                            flog2.write(f"agirosdep 缺失 rule: {l}\n")

                    if sub_need_oe:
                        versions = [args.openeuler_default] + [v for v in args.openeuler_fallback if v != args.openeuler_default]
                        success = False
                        for ver in versions:
                            base = build_cmd_for('rpm', args.bloom_bin)
                            rpm_cmd = base + (["agirosrpm"] if not is_direct_module_cmd(base) else []) + [
                                "--ros-distro", args.agiros_distro,
                                "--os-name", "openeuler",
                                "--os-version", ver,
                            ]

                            rc, out = run(rpm_cmd, cwd=subpkg, dry_run=args.dry_run)
                            if rc == 0:
                                total += 1
                                log(f"[OK] {pkg_name}: 已生成 rpm/ (openeuler:{ver})")
                                success = True
                                break
                            else:
                                with fail_log.open("a", encoding="utf-8") as flog2:
                                    flog2.write(f"{pkg_name} openeuler:{ver} 失败 rc={rc}\n")
                                    if out:
                                        for l in out.splitlines():
                                            if "No agirosdep rule for" in l:
                                                flog2.write(f"缺失 rule: {l}\n")
                        if not success:
                            log(f"[ERR] {pkg_name}: 所有 openEuler 版本均失败")

                except Exception as e:
                    with fail_log.open("a", encoding="utf-8") as flog2:
                        flog2.write(f"{pkg_name} 异常: {e}\n")
                    log(f"[ERR] {pkg_name}: 发生异常 {e}")

    log(f"[INFO] 所有包处理完成。成功生成数：{total}")
    log(f"[INFO] 失败记录已保存到 {fail_log}")


if __name__ == "__main__":
    main()
