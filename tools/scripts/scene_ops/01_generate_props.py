#!/usr/bin/env python3
"""生成场景物品。默认读 configs/scene/cli.yaml；可用参数覆盖。

  uv run python tools/scripts/scene_ops/01_generate_props.py
  uv run python tools/scripts/scene_ops/01_generate_props.py --render
  # 或交互: ./start.sh →「场景：生成物品」
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from teleop_birth.cli.interactive_scene import GenerateConfig, load_scene_cli
from teleop_birth.cli.scene_runner import SceneOpsRunner


def main() -> None:
    base = GenerateConfig.from_dict(dict(load_scene_cli(ROOT).get("generate") or {}))
    ap = argparse.ArgumentParser(description="生成场景物品（配置: configs/scene/cli.yaml）")
    ap.add_argument("--props", default=",".join(base.props), help="逗号分隔物品名")
    ap.add_argument("--settle-steps", type=int, default=base.settle_steps)
    ap.add_argument("--render", action="store_true", default=base.render)
    ap.add_argument("--no-render", action="store_true")
    args = ap.parse_args()
    render = False if args.no_render else bool(args.render)
    cfg = GenerateConfig(
        props=[x.strip() for x in args.props.split(",") if x.strip()],
        settle_steps=args.settle_steps,
        render=render,
    )
    SceneOpsRunner(ROOT).run_generate(cfg)


if __name__ == "__main__":
    main()
