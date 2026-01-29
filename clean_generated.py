#!/usr/bin/env python3
"""Remove generated debian/ and rpm/ directories under CODE_DIR."""
from __future__ import annotations

import os
import sys
from pathlib import Path
import shutil


CODE_DIR = Path(os.environ.get("CODE_DIR", "/opt/code_dir")).expanduser()


def log(message: str) -> None:
    print(f"[INFO] {message}")


def main() -> int:
    if not CODE_DIR.exists():
        print(f"[WARN] CODE_DIR 不存在: {CODE_DIR}")
        return 0

    log(f"清理 {CODE_DIR} 下的 debian/ 和 rpm/ 目录...")
    removed = 0
    for directory in CODE_DIR.rglob("*"):
        if directory.is_dir() and directory.name.lower() in {"debian", "rpm"}:
            shutil.rmtree(directory, ignore_errors=True)
            removed += 1
            log(f"删除目录: {directory}")

    log("清理完成。下次运行 agiros_oob_builder_procedural.py 会重新生成。")
    log(f"共移除 {removed} 个目录。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
