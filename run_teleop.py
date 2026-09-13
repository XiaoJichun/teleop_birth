#!/usr/bin/env python3
"""Hydra 入口：从 YAML 组装并运行遥操循环。"""

from __future__ import annotations

import sys
from pathlib import Path

import hydra
from omegaconf import DictConfig, OmegaConf

from teleop_birth.cli import tui
from teleop_birth.config.factory import build_teleop_loop


def _project_root() -> Path:
    return Path(__file__).resolve().parent


@hydra.main(version_base=None, config_path="configs", config_name="config")
def main(cfg: DictConfig) -> None:
    root = _project_root()
    print("\n" + "=" * 72)
    print("  运行配置")
    print("=" * 72)
    print(OmegaConf.to_yaml(cfg))

    if str(cfg.source.type).lower() == "xrobo":
        from teleop_birth.xr.env import apply_xrobo_library_path
        from teleop_birth.xr.sources import check_xrobo_sdk_available

        apply_xrobo_library_path(root)
        ok, msg = check_xrobo_sdk_available(root)
        if not ok:
            print("\n" + "!" * 72)
            print("  XRobo / Pico 前置检查失败")
            print("!" * 72)
            print(msg)
            print("\n推荐直接跑一键脚本:")
            print("  ./start_pico_sim.sh")
            sys.exit(1)

    if bool(cfg.runtime.get("interactive_confirm", False)):
        ok = tui.confirm("确认以上配置并开始遥操？", default=True)
        if not ok:
            print("已取消。")
            return

    try:
        loop, meta = build_teleop_loop(cfg, root)
    except ImportError as exc:
        print(f"\n依赖缺失:\n{exc}")
        print("详见: doc/Pico连接指南.md")
        sys.exit(1)

    duration = float(meta["duration_s"])
    duration_arg = None if duration <= 0 else duration
    print(
        f"启动遥操: hz={cfg.runtime.hz} duration={duration_arg} "
        f"source={cfg.source.type} backend={cfg.backend.type} "
        f"mapper={cfg.mapper.type} ee={cfg.end_effector.type}"
    )
    try:
        stats = loop.run(duration_s=duration_arg)
    except KeyboardInterrupt:
        print("\n用户中断。")
        return
    except ImportError as exc:
        print(f"\n依赖缺失:\n{exc}")
        print("详见: doc/Pico连接指南.md")
        sys.exit(1)

    print("--- done ---")
    print(f"steps={stats.steps}")
    for key, value in stats.latency.items():
        print(f"{key}={value:.2f}")
    if meta.get("log_path"):
        print(f"log: {meta['log_path']}")


if __name__ == "__main__":
    if len(sys.argv) == 1 and sys.stdin.isatty():
        print(
            "提示: 无参数启动请使用 ./start.sh（交互菜单）。\n"
            "或显式传入 Hydra 覆盖，例如:\n"
            "  uv run python run_teleop.py source=mock backend=mujoco_dual_ur runtime.duration_s=3"
        )
        sys.exit(2)
    main()
