import os
import subprocess
import requests
import yaml
from tqdm import tqdm
import datetime

# ---------------- 全局配置 ----------------
TARGET_DIR = os.environ.get("AGIROS_RELEASE_TARGET_DIR", "ros2_release_dir")
LOG_FILE = os.path.join(TARGET_DIR, "download_log.txt")
GIT_CLONE_TIMEOUT = int(os.environ.get("AGIROS_GIT_CLONE_TIMEOUT", "600"))

# ANSI 颜色定义
class Color:
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    RESET = "\033[0m"


def log_message(message: str, color: str = Color.RESET):
    os.makedirs(TARGET_DIR, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    formatted = f"[{timestamp}] {message}"
    print(f"{color}{formatted}{Color.RESET}")  # 终端彩色输出
    with open(LOG_FILE, "a") as f:
        f.write(formatted + "\n")


def safe_git_clone_or_resume(repo_url, repo_path):
    """
    克隆仓库，如果已存在则尝试 git fetch 断点续传。
    返回 True 表示成功，False 表示失败。
    并记录日志：
    - 仓库路径冲突或不是 Git 仓库
    - 仓库下载失败
    - 已经下载过（跳过）
    """
    if os.path.exists(repo_path):
        if os.path.isdir(os.path.join(repo_path, ".git")):
            log_message(f"[Skip] {repo_path} 已存在，跳过下载。", Color.YELLOW)
            try:
                subprocess.run(["git", "-C", repo_path, "fetch", "--all"],
                               check=True,
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
                subprocess.run(["git", "-C", repo_path, "reset", "--hard", "origin/HEAD"],
                               check=True,
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
                return True
            except subprocess.CalledProcessError:
                log_message(f"[Error] {repo_path} fetch/reset 失败。", Color.RED)
                return False
        else:
            log_message(
                f"[Info] {repo_path} 已存在且缺少 .git 目录，判定为手动准备的包，跳过 git clone。",
                Color.BLUE,
            )
            return True
    else:
        try:
            subprocess.run(
                ["git", "clone", repo_url, repo_path],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=GIT_CLONE_TIMEOUT,
            )
            log_message(f"[OK] 成功克隆 {repo_url} → {repo_path}", Color.GREEN)
            return True
        except subprocess.TimeoutExpired:
            log_message(
                f"[Warning] 克隆超时：{repo_url} → {repo_path}，已运行超过 {GIT_CLONE_TIMEOUT}s。",
                Color.YELLOW,
            )
            return False
        except subprocess.CalledProcessError:
            log_message(f"[Error] 克隆失败：{repo_url} → {repo_path}", Color.RED)
            return False


def download_repos_from_yaml(yaml_url: str, target_dir: str = TARGET_DIR):
    os.makedirs(target_dir, exist_ok=True)
    log_message(f"[Start] 从 {yaml_url} 下载 distribution.yaml", Color.BLUE)

    try:
        response = requests.get(yaml_url)
        response.raise_for_status()
        yaml_content = response.text
    except Exception as e:
        log_message(f"[Error] 下载 YAML 失败: {e}", Color.RED)
        raise

    data = yaml.safe_load(yaml_content)

    repos = []
    if "repositories" in data:
        for repo_name, repo_info in data["repositories"].items():
            if "release" in repo_info and "url" in repo_info["release"]:
                repos.append((repo_name, repo_info["release"]["url"]))

    total = len(repos)
    log_message(f"[Info] Found {total} repositories to download.", Color.BLUE)
    print(f"[Info] Found {total} release repositories to download.\n")

    failed_repos = []

    with tqdm(total=total, desc="Downloading repos", unit="repo") as pbar:
        for idx, (repo_name, repo_url) in enumerate(repos, start=1):
            repo_path = os.path.join(target_dir, repo_name)

            ok = safe_git_clone_or_resume(repo_url, repo_path)

            if ok:
                tqdm.write(f"[{idx}/{total}] {Color.GREEN}[OK]{Color.RESET} {repo_name}")
            else:
                tqdm.write(f"[{idx}/{total}] {Color.RED}[Error]{Color.RESET} {repo_name} from {repo_url}")
                failed_repos.append((repo_name, repo_url))

            pbar.update(1)

    if failed_repos:
        failed_file = os.path.join(target_dir, "failed_repos.txt")
        with open(failed_file, "w") as f:
            for name, url in failed_repos:
                f.write(f"{name} {url}\n")
        log_message(f"[Warning] {len(failed_repos)} failed repos written to {failed_file}", Color.YELLOW)
        print(f"\n{Color.YELLOW}[Warning]{Color.RESET} {len(failed_repos)} repositories failed. See {failed_file}")

    log_message(f"[Done] Finished downloading {total} repositories.", Color.GREEN)
    print(f"\n{Color.GREEN}[Done]{Color.RESET} Finished downloading {total} repositories. "
          f"Success: {total - len(failed_repos)}, Failed: {len(failed_repos)}")


if __name__ == "__main__":
    yaml_url = "http://1.94.193.239/yumrepo/agiros/agirosdep/loong/distribution.yaml"
    download_repos_from_yaml(yaml_url, TARGET_DIR)
