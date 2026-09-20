#!/usr/bin/env python3
"""从 catalog / menagerie 列出并检查夹爪资产。"""

from __future__ import annotations

import argparse
from pathlib import Path

from teleop_birth.scene.plugins.gripper import GripperPlugin


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, default=Path.cwd())
    args = ap.parse_args()
    root = args.root.resolve()
    print("可用夹爪（catalog）:")
    for g in GripperPlugin().list_available(root):
        mark = "OK" if g["exists"] or g.get("builtin") else "MISSING"
        print(f"  [{mark}] {g['id']}: {g['title']}")
        if not g["exists"] and not g.get("builtin"):
            print("         → bash scripts/bootstrap_third_party.sh")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
