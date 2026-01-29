#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
改进版：从 release 仓库的 tracks.yaml 解析 Jazzy 上游源码并下载到 code_dir
改动点：
- checkout 失败时，会自动尝试 fetch 并切远程分支
- 仍失败时，检测远程默认分支或最新 tag 并切换
- 支持 --resume, --limit, 彩色日志

本次微调：
- 新增“全局默认参数”集中管理区 Defaults，把命令行默认值（尤其是 distro）统一放在这里；
- 其余代码逻辑未变，仅将默认值引用改为使用 Defaults。
"""

import argparse
import os
import sys
import subprocess
import traceback
from typing import Any, Dict, List, Optional, Tuple

try:
    import yaml
except Exception:
    print("[ERROR] 请先安装 PyYAML: pip install pyyaml", file=sys.stderr)
    raise


# ----------------------------- Global Defaults -----------------------------
class Defaults:
    """集中管理命令行参数默认值，可通过环境变量覆盖。"""
    # 允许通过环境变量覆盖默认发行版（可选），便于 CI/不同机器灵活切换
    DISTRO: str = os.environ.get("AGIROS_DISTRO_DEFAULT", "loong")
    # argparse 的 store_true 默认为 False，这里也集中声明，便于未来统一改动
    RESUME: bool = False
    # --limit 默认 None（不限数量）
    LIMIT: Optional[int] = None


# ----------------------------- Logging -----------------------------
class Ansi:
    RED = "\u001b[31m"
    GREEN = "\u001b[32m"
    YELLOW = "\u001b[33m"
    BLUE = "\u001b[34m"
    RESET = "\u001b[0m"


class Logger:
    def __init__(self, use_color: Optional[bool] = None):
        if use_color is None:
            use_color = sys.stdout.isatty()
        self.use_color = use_color

    def _wrap(self, msg: str, color: str) -> str:
        if self.use_color:
            return f"{color}{msg}{Ansi.RESET}"
        return msg

    def error(self, msg: str):
        print(self._wrap(f"[ERROR] {msg}", Ansi.RED), file=sys.stderr)

    def warn(self, msg: str):
        print(self._wrap(f"[WARNING] {msg}", Ansi.YELLOW))

    def info(self, msg: str):
        print(self._wrap(f"[INFO] {msg}", Ansi.BLUE))

    def success(self, msg: str):
        print(self._wrap(f"[SUCCESS] {msg}", Ansi.GREEN))


# ----------------------------- Utilities -----------------------------
class Shell:
    def __init__(self, logger: Logger):
        self.logger = logger

    def run(self, cmd: List[str], cwd: Optional[str] = None) -> Tuple[int, str, str]:
        try:
            proc = subprocess.Popen(
                cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
            out, err = proc.communicate()
            return proc.returncode, out, err
        except FileNotFoundError:
            self.logger.error(f"命令未找到：{' '.join(cmd)}")
            return 127, "", f"Command not found: {cmd[0]}"
        except Exception as e:
            self.logger.error(f"执行命令异常：{' '.join(cmd)}\n{e}")
            return 1, "", str(e)


# ----------------------------- Git Helper -----------------------------
class GitHelper:
    def __init__(self, logger: Logger, sh: Shell):
        self.logger = logger
        self.sh = sh

    def safe_checkout(self, repo_dir: str, branch_or_tag: str) -> bool:
        """尝试切换分支/标签；失败时自动 fetch 远程分支或 fallback 到默认分支/tag"""
        # 直接尝试 checkout
        rc, out, err = self.sh.run(["git", "checkout", branch_or_tag], cwd=repo_dir)
        if rc == 0:
            return True

        self.logger.warn(f"checkout '{branch_or_tag}' 失败: {err.strip()}")

        # 尝试 fetch 远程分支
        rc, _, err = self.sh.run(["git", "fetch", "origin", branch_or_tag], cwd=repo_dir)
        if rc == 0:
            rc2, _, err2 = self.sh.run(
                ["git", "checkout", "-b", branch_or_tag, f"origin/{branch_or_tag}"], cwd=repo_dir
            )
            if rc2 == 0:
                return True
            else:
                self.logger.warn(f"切换远程分支失败: {err2.strip()}")

        # 检测远程默认分支
        rc, out, err = self.sh.run(["git", "remote", "show", "origin"], cwd=repo_dir)
        if rc == 0:
            for line in out.splitlines():
                if "HEAD branch:" in line:
                    default_branch = line.split(":")[-1].strip()
                    self.logger.info(f"检测到远程默认分支：{default_branch}")
                    rc2, _, err2 = self.sh.run(["git", "checkout", default_branch], cwd=repo_dir)
                    if rc2 == 0:
                        return True
                    self.logger.warn(f"切换默认分支失败: {err2.strip()}")

        # 检测最新 tag
        rc, out, err = self.sh.run(["git", "tag"], cwd=repo_dir)
        tags = [t.strip() for t in out.splitlines() if t.strip()]
        if tags:
            latest = sorted(tags)[-1]
            self.logger.info(f"尝试切换到最新 tag：{latest}")
            rc2, _, err2 = self.sh.run(["git", "checkout", "--detach", latest], cwd=repo_dir)
            if rc2 == 0:
                return True
            self.logger.warn(f"切换最新 tag 失败: {err2.strip()}")

        self.logger.error(f"最终无法 checkout {branch_or_tag}")
        return False


# ----------------------------- Tracks Parser -----------------------------
class TracksParser:
    def __init__(self, logger: Logger, distro: str = Defaults.DISTRO):
        self.logger = logger
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

    def parse_file(self, tracks_yaml_path: str) -> Optional[Dict[str, Any]]:
        if not os.path.isfile(tracks_yaml_path):
            self.logger.warn(f"未找到 tracks.yaml: {tracks_yaml_path}")
            return None
        try:
            with open(tracks_yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.logger.error(f"解析 YAML 失败: {tracks_yaml_path} -> {e}")
            return None

        tracks = None
        if isinstance(data, dict) and "tracks" in data:
            tracks = data["tracks"]
        elif isinstance(data, dict):
            tracks = data
        else:
            self.logger.warn(f"tracks.yaml 结构异常: {tracks_yaml_path}")
            return None

        key = self._find_distro_case_insensitive(tracks)
        if key is None:
            self.logger.warn(f"{os.path.dirname(tracks_yaml_path)} 不包含 '{self.distro}' 的 track")
            return None

        section = tracks.get(key)
        return section if isinstance(section, dict) else None


# ----------------------------- Model & Decision -----------------------------
class RepoPlan:
    def __init__(self, name: str, upstream_url: str, branch_or_tag: str):
        self.name = name
        self.upstream_url = upstream_url
        self.branch_or_tag = branch_or_tag


class Planner:
    def __init__(self, logger: Logger):
        self.logger = logger

    def make_plan(self, repo_name: str, track: Dict[str, Any]) -> Optional[RepoPlan]:
        upstream_url = track.get("vcs_uri")
        if not upstream_url:
            self.logger.warn(f"[{repo_name}] 缺少 vcs_uri")
            return None

        branch_or_tag = track.get("devel_branch") or track.get("version") or "main"
        return RepoPlan(repo_name, upstream_url, branch_or_tag)


# ----------------------------- Downloader -----------------------------
class GitDownloader:
    def __init__(self, logger: Logger):
        self.logger = logger
        self.sh = Shell(logger)
        self.helper = GitHelper(logger, self.sh)

    def clone(self, url: str, dest: str, branch_or_tag: Optional[str]) -> bool:
        os.makedirs(os.path.dirname(dest), exist_ok=True)

        if os.path.isdir(os.path.join(dest, ".git")):
            self.logger.info(f"[更新] {dest}")
            self.sh.run(["git", "fetch", "--all"], cwd=dest)
        else:
            self.logger.info(f"[克隆] {url} → {dest}")
            rc, _, err = self.sh.run(["git", "clone", url, dest])
            if rc != 0:
                self.logger.error(f"git clone 失败: {err.strip()}")
                return False

        return self.helper.safe_checkout(dest, branch_or_tag)


# ----------------------------- Coordinator -----------------------------
class RepoProcessor:
    def __init__(self, logger: Logger, distro: str, code_dir: str, resume: bool = False):
        self.logger = logger
        self.distro = distro
        self.code_dir = os.path.abspath(code_dir)
        self.resume = resume
        self.parser = TracksParser(logger, distro)
        self.planner = Planner(logger)
        self.downloader = GitDownloader(logger)

    def process_repo_dir(self, repo_dir: str) -> bool:
        repo_name = os.path.basename(repo_dir.rstrip(os.sep))
        section = self.parser.parse_file(os.path.join(repo_dir, "tracks.yaml"))
        if not section:
            return False

        plan = self.planner.make_plan(repo_name, section)
        if not plan:
            return False

        target_dir = os.path.join(self.code_dir, repo_name)
        if self.resume and os.path.isdir(target_dir):
            self.logger.info(f"[Resume] 跳过已存在: {target_dir}")
            return True

        return self.downloader.clone(plan.upstream_url, target_dir, plan.branch_or_tag)


# ----------------------------- Runner -----------------------------
class Runner:
    def __init__(self, logger: Logger, release_dir: str, code_dir: str, distro: str, resume: bool, limit: Optional[int]):
        self.logger = logger
        self.release_dir = os.path.abspath(release_dir)
        self.code_dir = os.path.abspath(code_dir)
        self.distro = distro
        self.resume = resume
        self.limit = limit
        self.proc = RepoProcessor(logger, distro, code_dir, resume)

    def discover_repos(self) -> List[str]:
        if not os.path.isdir(self.release_dir):
            self.logger.error(f"release_dir 不存在: {self.release_dir}")
            return []
        return [os.path.join(self.release_dir, d) for d in sorted(os.listdir(self.release_dir)) if os.path.isdir(os.path.join(self.release_dir, d))]

    def run(self) -> Tuple[int, int]:
        os.makedirs(self.code_dir, exist_ok=True)
        repos = self.discover_repos()
        if self.limit:
            repos = repos[: self.limit]

        total = len(repos)
        ok = 0
        for idx, repo_dir in enumerate(repos, 1):
            self.logger.info(f"({idx}/{total}) 处理: {os.path.basename(repo_dir)}")
            try:
                if self.proc.process_repo_dir(repo_dir):
                    ok += 1
                else:
                    self.logger.warn(f"[{os.path.basename(repo_dir)}] 未完成")
            except Exception:
                self.logger.error(f"[{os.path.basename(repo_dir)}] 异常:\n{traceback.format_exc()}")
        self.logger.success(f"完成: 成功 {ok} / 总计 {total}")
        return ok, total


# ----------------------------- main -----------------------------
def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--release-dir", required=True)
    ap.add_argument("--code-dir", required=True)
    ap.add_argument("--distro", default=Defaults.DISTRO)
    ap.add_argument("--resume", action="store_true", default=Defaults.RESUME)
    ap.add_argument("--limit", type=int, default=Defaults.LIMIT)
    return ap.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    logger = Logger()
    runner = Runner(logger, args.release_dir, args.code_dir, args.distro, args.resume, args.limit)
    ok, _ = runner.run()
    return 0 if ok > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
