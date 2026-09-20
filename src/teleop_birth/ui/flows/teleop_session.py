"""Pico 仿真遥操：选场景 → 随机化物体/区域 → 轮次 → 格式 → 预采样布局 → 启动。"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

from teleop_birth.ui import tui
from teleop_birth.ui.flows.session_common import build_session_plan
from teleop_birth.ui.session.round_layouts import pregenerate_episode_layouts, save_plan_with_layouts


def action_pico_teleop_session(root: Path) -> None:
    tui.section("Pico仿真遥操")
    plan = build_session_plan(root, need_format=True)
    if plan is None:
        print("已取消。")
        return

    session_dir = root / plan.output_dir / "teleop_plans"
    session_dir.mkdir(parents=True, exist_ok=True)
    plan_path = session_dir / f"{plan.scene_path.stem}_plan.json"

    layouts: list = []
    if plan.region is not None and plan.prop_names:
        try:
            inline = list(plan.scene_cfg.get("prop_specs") or [])
            layouts = pregenerate_episode_layouts(
                root,
                prop_names=plan.prop_names,
                region=plan.region,
                episodes=plan.episodes,
                seed=plan.seed,
                prop_specs_inline=inline,
            )
            print(f"  已预采样 {len(layouts)} 轮无碰撞布局")
        except Exception as exc:  # noqa: BLE001
            print(f"  预采样布局失败（仍可遥操）: {exc}")

    payload = {
        "scene": str(plan.scene_path),
        "props": plan.prop_names,
        "region": plan.region.to_dict() if plan.region else None,
        "episodes": plan.episodes,
        "output_dir": plan.output_dir,
        "dataset_format": plan.dataset_format,
        "seed": plan.seed,
        "layouts": layouts,
        "note": (
            "episodes=随机化轮次；layouts[i] 为第 i 轮物品位姿。"
            "环境变量 TELEOP_PLAN_JSON 指向本文件；"
            "遥操中按 左手X 或 右手A 切换下一轮物品位置。"
        ),
    }
    save_plan_with_layouts(plan_path, payload)
    # 方便 start_pico_sim / 后续环读取
    latest = session_dir / "latest_plan.json"
    latest.write_text(plan_path.read_text(encoding="utf-8"), encoding="utf-8")
    print(f"  会话计划已写: {plan_path}")
    print(f"  快捷链接: {latest}")

    adapter = str(
        (plan.scene_cfg.get("backend") or {}).get("type") or plan.scene_cfg.get("type") or ""
    )
    tui.section("确认启动")
    print(f"  场景: {plan.scene_path.name}")
    print(f"  物体: {plan.prop_names}")
    print(f"  轮次: {plan.episodes}")
    print(f"  格式: {plan.dataset_format}")
    print(f"  输出: {plan.output_dir}")
    if layouts:
        print(f"  第 0 轮示例: {json.dumps(layouts[0], ensure_ascii=False)[:120]}…")
    if not tui.confirm("Pico仿真遥操开始？", default=True):
        return

    if "logistics" in adapter or "artixon" in adapter:
        script = root / "start_logistics_teleop.sh"
    else:
        script = root / "start_pico_sim.sh"
    if not script.is_file():
        print(f"未找到启动脚本: {script}")
        return

    env = dict(**__import__("os").environ)
    env["TELEOP_PLAN_JSON"] = str(plan_path)
    env["TELEOP_DATASET_FORMAT"] = str(plan.dataset_format)
    print(f"\n>>> bash {script}")
    print(f"  TELEOP_PLAN_JSON={plan_path}\n")
    code = subprocess.call(["bash", str(script)], cwd=root, env=env)
    if code != 0:
        print(f"\n退出码: {code}")
