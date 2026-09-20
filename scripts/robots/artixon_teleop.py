#!/usr/bin/env python3
"""自变量 ArtiXon Arm-6A Pico 遥操入口。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
REPO_SRC = REPO_ROOT / "src"
if str(REPO_SRC) not in sys.path:
    sys.path.insert(0, str(REPO_SRC))

import numpy as np
from omegaconf import OmegaConf

from teleop_birth.teleop.config.factory import build_mapper, build_source, shared_loop_hz
from teleop_birth.teleop.pipeline import TeleopLoop
from teleop_birth.teleop.plot import plotter_from_cfg
from teleop_birth.robot.arm.artixon.paths import configs_dir
from teleop_birth.robot.arm.artixon.teleop_backend import LogisticsTeleopBackend
from teleop_birth.teleop.schema import EXTREMITY_LEFT_EE, EXTREMITY_RIGHT_EE, identity_pose


def _load_cfg(path: Path) -> dict:
    raw = OmegaConf.to_container(OmegaConf.load(path), resolve=True)
    assert isinstance(raw, dict)
    return raw


def main() -> int:
    ap = argparse.ArgumentParser(description="ArtiXon Arm-6A Pico 遥操")
    ap.add_argument("--config", type=Path, default=None)
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--hz", type=float, default=None)
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()

    cfg_path = Path(args.config or (configs_dir() / "artixon_teleop.yaml"))
    cfg = _load_cfg(cfg_path)
    scene_file = str(cfg.get("scene_file") or "artixon_logistics_scene.yaml")
    scene_path = configs_dir() / scene_file
    scene = _load_cfg(scene_path) if scene_path.is_file() else {}

    render = bool(cfg.get("render", True)) and not args.no_render
    hz = float(args.hz if args.hz is not None else shared_loop_hz(REPO_ROOT))
    duration = float(args.duration if args.duration is not None else cfg.get("duration_s", 0.0))
    print_every = int(cfg.get("print_every", 50))
    scale = float(cfg.get("scale", 1.0))
    sticky = dict(cfg.get("sticky") or {})
    ik_cfg = dict(cfg.get("ik") or {})
    mapper_cfg = {"type": "dual_arm", "scale": scale, **dict(cfg.get("mapper") or {})}
    source_cfg = dict(cfg.get("source") or {})
    source_cfg.setdefault("type", "xrobo")
    source_cfg.setdefault("include_body", False)
    source_cfg.setdefault("setup_config", "configs/setup/xrobo.yaml")

    print("\n" + "=" * 72)
    print("  物流分拣 Pico 遥操（ArtiXon Arm-6A ×2）")
    print("=" * 72)
    print(f"  配置: {cfg_path}")
    print(f"  场景: {scene_path}")
    print("=" * 72 + "\n")

    backend = LogisticsTeleopBackend(
        scene_cfg=scene,
        ik_cfg=ik_cfg,
        render=render,
        sim_substeps=int(cfg.get("sim_substeps", 4)),
        sticky_attach_radius_m=float(sticky.get("attach_radius_m", 0.09)),
        sticky_gripper_min=float(sticky.get("gripper_min", 0.25)),
        position_ik_only=bool(ik_cfg.get("position_only", False)),
        tacto_cfg=dict(cfg.get("tacto") or {}),
        repo_root=REPO_ROOT,
    )
    source = build_source(source_cfg, project_root=REPO_ROOT)
    mapper = build_mapper(mapper_cfg, scale_override=scale)
    plotter = plotter_from_cfg(cfg.get("plot"), project_root=REPO_ROOT, loop_hz=hz)
    backend.trace_plotter = plotter
    step_i = {"i": 0, "prev_xl": None, "stale": False, "last_ms": 0.0}

    from teleop_birth.teleop.plan_rounds import TeleopRoundManager, load_plan_from_env

    plan = load_plan_from_env()
    round_mgr = TeleopRoundManager(plan, REPO_ROOT) if plan else None

    def on_step(obs, target, state) -> None:
        if round_mgr is not None:
            if not step_i.get("plan_attached"):
                round_mgr.attach(backend)
                step_i["plan_attached"] = True
            round_mgr.on_obs(obs)
        i = step_i["i"]
        if i % print_every == 0:
            le = state.extremities.get(EXTREMITY_LEFT_EE, identity_pose())
            re = state.extremities.get(EXTREMITY_RIGHT_EE, identity_pose())
            xl = obs.left_controller
            sticky_on = float((state.extra or {}).get("sticky", 0.0)) > 0.5
            print(
                f"[{i:04d}] L=({le[0]:+.3f},{le[1]:+.3f},{le[2]:+.3f}) "
                f"R=({re[0]:+.3f},{re[1]:+.3f},{re[2]:+.3f}) "
                f"sticky={sticky_on} stream={'OK' if getattr(obs,'stream_ok',True) else 'DOWN'}"
            )
            step_i["prev_xl"] = xl.copy()
        step_i["i"] = i + 1

    loop = TeleopLoop(
        source=source,
        mapper=mapper,
        backend=backend,
        hz=hz,
        latency_window=200,
        on_step=on_step,
        trace=plotter,
    )
    try:
        stats = loop.run(duration_s=None if duration <= 0 else duration)
    except KeyboardInterrupt:
        print("\n用户中断。")
        return 0
    except ImportError as exc:
        print(f"\n依赖缺失:\n{exc}")
        return 1
    print("--- done ---", stats.steps)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
