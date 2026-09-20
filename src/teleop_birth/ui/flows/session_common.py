"""会话向导共用：选场景 / 选物体 / 随机化区域 / 轮次。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from teleop_birth.scene.plugins.scene import load_scene_yaml
from teleop_birth.scene.props.catalog import PropCatalog
from teleop_birth.ui import tui
from teleop_birth.ui.scene_builder.draft import list_built_scenes
from teleop_birth.ui.session.randomize import (
    RandomizeRegion,
    parse_region_text,
    region_from_arrays,
)
from teleop_birth.ui.util import load_yaml


@dataclass
class SessionPlan:
    scene_path: Path
    scene_cfg: dict[str, Any]
    prop_names: list[str] = field(default_factory=list)
    region: RandomizeRegion | None = None
    episodes: int = 3
    output_dir: str = "logs/mcap"
    dataset_format: str = "mcap"
    seed: int = 0


def pick_built_scene(root: Path) -> Path | None:
    scenes = list_built_scenes(root)
    if not scenes:
        print("没有已搭建场景。请先「搭建场景」。")
        return None
    labels = []
    for p in scenes:
        try:
            title = load_scene_yaml(p).get("title") or p.stem
        except Exception:
            title = p.stem
        rel = p.relative_to(root) if p.is_relative_to(root) else p
        labels.append((str(p), f"{title}  [{rel}]"))
    choice = tui.select("选择已搭建好的场景", labels, allow_cancel=True)
    return Path(choice) if choice else None


def pick_props(root: Path, scene_cfg: dict[str, Any]) -> list[str] | None:
    cat = PropCatalog(root / "configs" / "scene" / "props")
    scene_props = list(scene_cfg.get("props") or [])
    names = scene_props or cat.list_names()
    if not names:
        print("场景与目录均无物品。")
        return None
    # checkbox 若不可用则多选循环
    if hasattr(tui, "checkbox"):
        opts = [(n, n) for n in names]
        defaults = scene_props[:2] if scene_props else names[:2]
        picked = tui.checkbox("选择需要随机化的物体（≥2 时自动防碰撞）", opts, defaults=defaults)
        if picked is None:
            return None
        if not picked:
            print("未选择物体。")
            return None
        return list(picked)
    # fallback
    print("可选物体:", ", ".join(names))
    raw = tui.text("输入物体名（逗号分隔）", default=",".join(names[:2]))
    if raw is None:
        return None
    return [x.strip() for x in raw.replace("，", ",").split(",") if x.strip()]


def pick_region(scene_cfg: dict[str, Any]) -> RandomizeRegion | None:
    db = dict(scene_cfg.get("desktop_base") or {})
    base_xy = list(db.get("xy") or [0.0, 0.0])
    base_z = float(db.get("table_top_z") or scene_cfg.get("table_top_z") or 0.42)
    print(
        "随机化区域相对机器人 base：\n"
        "  平面: [[ox,oy],[sx,sy]]  — 中心相对 base 的偏移 + 区域边长（米；≥10 视为毫米）\n"
        "  立体: [[ox,oy,oz],[sx,sy,sz]]\n"
        "示例: [[0.15,0.40],[0.25,0.20]] 或 [[150,400],[250,200]]（毫米）"
    )
    raw = tui.text(
        "输入随机化区域数组",
        default="[[0.15, 0.40], [0.25, 0.20]]",
    )
    if raw is None:
        return None
    try:
        off, size = parse_region_text(raw)
    except ValueError as exc:
        print(f"  解析失败: {exc}")
        return None
    sep_raw = tui.text("物体最小间距 (m)", default="0.08")
    if sep_raw is None:
        return None
    try:
        sep = float(sep_raw.strip())
    except ValueError:
        sep = 0.08
    return region_from_arrays(
        off,
        size,
        base_xy=base_xy,
        base_z=base_z,
        min_separation_m=sep,
    )


def preview_region_in_mujoco(root: Path, scene_cfg: dict[str, Any], region) -> None:
    """输入区域后：询问是否在 MuJoCo 中预览绿盒（PyQt 可开关显隐）。"""
    tui.section("随机化区域预览")
    print(f"  世界包围盒: {region.world_bounds()}")
    if not tui.confirm("在 MuJoCo 中预览该区域？（绿盒；可用 PyQt 开关显隐）", default=True):
        print("  已跳过区域预览。")
        return
    try:
        from teleop_birth.ui.session.region_panel import ensure_region_ui
        from teleop_birth.ui.session.region_viz import get_region_overlay
        from teleop_birth.ui.flows.browse_scenes import _build_scene_model

        ov = get_region_overlay()
        ov.set_region(region)
        ov.set_visible(True)
        ensure_region_ui(region)

        scene_path = scene_cfg.get("_path")
        if not scene_path:
            st = str((scene_cfg.get("backend") or {}).get("type") or scene_cfg.get("type") or "")
            if "logistics" in st or "artixon" in st:
                scene_path = root / "configs/scenes/logistics_x2arm.yaml"
            else:
                scene_path = root / "configs/scenes/dual_ur_pick_place.yaml"
        model = _build_scene_model(root, Path(scene_path))
        import mujoco
        import mujoco.viewer

        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        print("  MuJoCo 预览已开：绿色半透明盒=随机化区域；关闭窗口后继续。")
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                ov.apply(viewer)
                viewer.sync()
                import time

                time.sleep(0.02)
        print("  区域预览结束，继续后续步骤。")
    except Exception as exc:  # noqa: BLE001
        print(f"  区域预览失败（可继续采集）: {exc}")


def pick_episodes_and_output(default_ep: int = 3, default_out: str = "logs/mcap") -> tuple[int, str] | None:
    from teleop_birth.ui.interactive_scene import prompt_collect_episodes_and_output

    return prompt_collect_episodes_and_output(
        default_episodes=default_ep,
        default_output_dir=default_out,
        title="采集轮次与输出目录",
    )


def pick_dataset_format(root: Path) -> str | None:
    path = root / "configs" / "ui" / "dataset_formats.yaml"
    data = load_yaml(path) if path.is_file() else {}
    formats = dict(data.get("formats") or {})
    if not formats:
        formats = {"mcap": {"title": "MCAP", "status": "ready"}}
    opts = [
        (k, f"{v.get('title') or k}  [{v.get('status') or '?'}]")
        for k, v in formats.items()
    ]
    return tui.select("选择采集格式", opts, allow_cancel=True)


def build_session_plan(root: Path, *, need_format: bool) -> SessionPlan | None:
    scene_path = pick_built_scene(root)
    if not scene_path:
        return None
    scene_cfg = load_scene_yaml(scene_path)
    props = pick_props(root, scene_cfg)
    if props is None:
        return None
    region = pick_region(scene_cfg)
    if region is None:
        return None
    print(f"  区域世界包围盒: {region.world_bounds()}")
    # 把路径塞进 cfg，供预览选模型
    scene_cfg = dict(scene_cfg)
    scene_cfg["_path"] = str(scene_path)
    try:
        preview_region_in_mujoco(root, scene_cfg, region)
    except Exception as exc:  # noqa: BLE001
        print(f"  区域预览跳过: {exc}")
    io = pick_episodes_and_output()
    if io is None:
        return None
    episodes, output_dir = io
    fmt = "mcap"
    if need_format:
        picked = pick_dataset_format(root)
        if picked is None:
            return None
        fmt = picked
        if fmt != "mcap":
            print(f"  将先写 MCAP，结束后自动导出为 {fmt}")
    return SessionPlan(
        scene_path=scene_path,
        scene_cfg=scene_cfg,
        prop_names=props,
        region=region,
        episodes=episodes,
        output_dir=output_dir,
        dataset_format=fmt,
    )
