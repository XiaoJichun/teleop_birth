#!/usr/bin/env python3
"""语义采集 → MCAP。默认读 configs/scene/cli.yaml（默认开窗口可盯着看）。

  uv sync --extra collect
  uv run python tools/scripts/scene_ops/03_collect_pick_place.py
  uv run python tools/scripts/scene_ops/03_collect_pick_place.py --no-render  # 无头
  # 或交互: ./start.sh →「一键：自动化采数据」
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from teleop_birth.cli.interactive_scene import CollectConfig, load_scene_cli
from teleop_birth.cli.scene_runner import SceneOpsRunner


def main() -> None:
    base = CollectConfig.from_dict(dict(load_scene_cli(ROOT).get("collect") or {}))
    ap = argparse.ArgumentParser(description="语义采集 MCAP（配置: configs/scene/cli.yaml）")
    ap.add_argument("--task", default=base.task)
    ap.add_argument("--episodes", type=int, default=base.episodes)
    ap.add_argument("--seed", type=int, default=base.seed)
    ap.add_argument("--arm", default=base.arm, choices=["left", "right"])
    ap.add_argument("--output-dir", default=base.output_dir)
    ap.add_argument("--steps-per-segment", type=int, default=base.steps_per_segment)
    ap.add_argument("--hold-steps", type=int, default=base.hold_steps)
    ap.add_argument("--pace-s", type=float, default=base.pace_s)
    ap.add_argument("--render", action="store_true", default=base.render)
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--no-sticky-grasp", action="store_true")
    args = ap.parse_args()
    render = False if args.no_render else bool(args.render)
    pace = 0.0 if args.no_render else float(args.pace_s)
    cfg = CollectConfig(
        task=args.task,
        episodes=args.episodes,
        seed=args.seed,
        arm=args.arm,
        render=render,
        pace_s=pace,
        output_dir=args.output_dir,
        steps_per_segment=args.steps_per_segment,
        hold_steps=args.hold_steps,
        sticky_grasp=False if args.no_sticky_grasp else base.sticky_grasp,
        camera_enabled=base.camera_enabled,
        camera_width=base.camera_width,
        camera_height=base.camera_height,
        camera_every_n_steps=base.camera_every_n_steps,
        recording=base.recording,
        backend=base.backend,
        ik=base.ik,
        end_effector=base.end_effector,
        paths=base.paths,
    )
    if cfg.render:
        print(f"[collect] MuJoCo 窗口开启，pace_s={cfg.pace_s} hold_steps={cfg.hold_steps}")
    SceneOpsRunner(ROOT).run_collect(cfg)


if __name__ == "__main__":
    main()
