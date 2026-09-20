#!/usr/bin/env python3
"""自变量 ArtiXon 物流自动采集。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_SRC = Path(__file__).resolve().parents[2] / "src"
if str(REPO_SRC) not in sys.path:
    sys.path.insert(0, str(REPO_SRC))

from teleop_birth.robot.arm.artixon.collection.collector import LogisticsCollector
from teleop_birth.robot.arm.artixon.paths import configs_dir


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=None)
    ap.add_argument("--output-dir", type=str, default=None)
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--config", type=Path, default=None)
    args = ap.parse_args()
    cfg_path = Path(args.config or (configs_dir() / "artixon_collect.yaml"))
    col = LogisticsCollector.from_yaml(cfg_path)
    if args.episodes is not None:
        col.cfg["episodes"] = int(args.episodes)
    if args.output_dir:
        col.cfg["output_dir"] = str(args.output_dir)
    if args.no_render:
        col.cfg["render"] = False
    results = col.run()
    ok = sum(1 for r in results if r.success)
    print(f"[artixon] done: {ok}/{len(results)} success")
    return 0 if ok == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
