#!/usr/bin/env python3
"""摆放场景物品。默认读 configs/scene/cli.yaml。

  uv run python tools/scripts/scene_ops/02_place_props.py
  # 或交互: ./start.sh →「场景：摆放物品」
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from teleop_birth.cli.interactive_scene import PlaceConfig, load_scene_cli
from teleop_birth.cli.scene_runner import SceneOpsRunner


def main() -> None:
    base = PlaceConfig.from_dict(dict(load_scene_cli(ROOT).get("place") or {}))
    ap = argparse.ArgumentParser(description="摆放场景物品（配置: configs/scene/cli.yaml）")
    ap.add_argument("--props", default=",".join(base.props))
    ap.add_argument("--seed", type=int, default=base.seed)
    ap.add_argument("--repeats", type=int, default=base.repeats)
    ap.add_argument("--render", action="store_true", default=base.render)
    ap.add_argument("--no-render", action="store_true")
    args = ap.parse_args()
    render = False if args.no_render else bool(args.render)
    cfg = PlaceConfig(
        props=[x.strip() for x in args.props.split(",") if x.strip()],
        seed=args.seed,
        repeats=args.repeats,
        render=render,
    )
    SceneOpsRunner(ROOT).run_place(cfg)


if __name__ == "__main__":
    main()
