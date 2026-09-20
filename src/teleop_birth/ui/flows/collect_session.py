"""自动数据采集：选场景 → 随机化 → 观摩选项 → 标定确认 → 开始。"""

from __future__ import annotations

import json
from pathlib import Path

from teleop_birth.scene.config import CollectConfig
from teleop_birth.ui import tui
from teleop_birth.ui.actions import _ensure_collect_deps
from teleop_birth.ui.flows.session_common import build_session_plan
from teleop_birth.ui.interactive_scene import load_scene_cli
from teleop_birth.ui.scene_runner import SceneOpsRunner


def action_auto_collect_session(root: Path) -> None:
    tui.section("自动数据采集")
    if not _ensure_collect_deps(root):
        return

    # 含：选场景 / 选物体 / 随机区域 / 区域 MuJoCo 预览 / 轮次与输出
    plan = build_session_plan(root, need_format=False)
    if plan is None:
        print("已取消。")
        return

    # 保存随机化计划
    session_meta = root / plan.output_dir / "_pending_session"
    session_meta.mkdir(parents=True, exist_ok=True)
    meta_path = session_meta / "randomize_plan.json"
    meta_path.write_text(
        json.dumps(
            {
                "scene": str(plan.scene_path),
                "props": plan.prop_names,
                "region": plan.region.to_dict() if plan.region else None,
                "episodes": plan.episodes,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    cli = dict(load_scene_cli(root).get("collect") or {})
    try:
        scene_rel = str(plan.scene_path.relative_to(root / "configs"))
    except ValueError:
        scene_rel = str(plan.scene_path)
    cli["scene"] = scene_rel
    cli["episodes"] = plan.episodes
    cli["output_dir"] = plan.output_dir
    cli["calibrate_before_collect"] = True
    if plan.region is not None:
        cli["randomize_region"] = plan.region.to_dict()
    cli["randomize_props"] = list(plan.prop_names)
    cli["dataset_format"] = plan.dataset_format

    stype = str(
        (plan.scene_cfg.get("backend") or {}).get("type") or plan.scene_cfg.get("type") or ""
    )

    # 物流场景走专用采集
    if "logistics" in stype or "artixon" in stype:
        tui.section("确认采集（物流）")
        print(f"  场景: {plan.scene_path.name}")
        print(f"  物体: {plan.prop_names}")
        print(f"  轮次: {plan.episodes}")
        print(f"  随机化计划: {meta_path}")
        if not tui.confirm("启动物流自动采集？", default=True):
            return
        from teleop_birth.scene.plugins.collect import CollectPlugin

        CollectPlugin().run_logistics(root)
        return

    # —— 双 UR 等桌面场景：补回观摩 / 粘性抓取等步骤 ——
    tui.section("采集观摩与抓取选项")
    print("  （默认来自 configs/scene/cli.yaml → collect，可现场改）")
    render = tui.confirm(
        "打开 MuJoCo 可视化窗口（方便盯着采集）？",
        default=bool(cli.get("render", True)),
    )
    if render is None:
        print("已取消。")
        return
    pace_default = str(cli.get("pace_s", 0.02))
    pace_raw = tui.text("观摩节拍 pace_s（秒/步，0=尽快）", default=pace_default)
    if pace_raw is None:
        print("已取消。")
        return
    try:
        pace_s = float(pace_raw.strip())
    except ValueError:
        pace_s = float(cli.get("pace_s", 0.02))
    sticky = tui.confirm(
        "启用接触锁定？（推荐开：双指垫碰到后再锁定，停顿已缩短；关=纯摩擦，仿真里很难抬起）",
        default=bool(cli.get("sticky_grasp", True)),
    )
    if sticky is None:
        print("已取消。")
        return
    steps_default = str(cli.get("steps_per_segment", 40))
    steps_raw = tui.text("每段插值步数 steps_per_segment（≥2）", default=steps_default)
    if steps_raw is None:
        print("已取消。")
        return
    try:
        steps = max(2, int(steps_raw.strip()))
    except ValueError:
        steps = int(cli.get("steps_per_segment", 40))

    cli["render"] = bool(render)
    cli["pace_s"] = float(pace_s)
    cli["sticky_grasp"] = bool(sticky)
    cli["steps_per_segment"] = int(steps)
    # 场景任务：优先用场景 yaml 的 collect.task，否则 cli 默认
    scene_task = (plan.scene_cfg.get("collect") or {}).get("task")
    if scene_task:
        cli["task"] = str(scene_task)

    cfg = CollectConfig.from_dict(cli)

    tui.section("确认采集")
    print(f"  场景: {plan.scene_path.name}")
    print(f"  任务: {cfg.task}")
    print(f"  物体: {plan.prop_names}")
    print(f"  轮次(episodes): {cfg.episodes}")
    print(f"  输出: {cfg.output_dir}")
    print(f"  格式: {cfg.dataset_format}")
    print(f"  render={cfg.render}  pace_s={cfg.pace_s}  sticky={cfg.sticky_grasp}")
    print(f"  steps_per_segment={cfg.steps_per_segment}")
    if plan.region is not None:
        print(f"  随机化区域: {plan.region.world_bounds()}")
    print(f"  随机化计划: {meta_path}")
    print("  → 将先相机标定，再自动采集写入 MCAP")
    if cfg.render:
        print("  ★ 采集中 MuJoCo 会显示随机化区域绿盒（可用 PyQt 面板开关显隐）")
    if not tui.confirm("确认开始采集（先标定再采）？", default=True):
        return

    # 把区域挂到全局 overlay，供 MuJoCo viewer 绘制
    if plan.region is not None and cfg.render:
        try:
            from teleop_birth.ui.session.region_panel import ensure_region_ui
            from teleop_birth.ui.session.region_viz import get_region_overlay

            get_region_overlay().set_region(plan.region)
            get_region_overlay().set_visible(True)
            ensure_region_ui(plan.region)
        except Exception as exc:  # noqa: BLE001
            print(f"  [region] 面板跳过: {exc}")

    print(f"\n>>> 开始采集 task={cfg.task} episodes={cfg.episodes} → {cfg.output_dir}\n")
    try:
        SceneOpsRunner(root).run_collect(cfg)
    except ImportError as exc:
        print(f"\n执行失败: {exc}")
        print("请执行: uv sync --extra collect  后重试。")
