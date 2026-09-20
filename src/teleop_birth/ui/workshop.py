"""场景工坊 TUI（UI 层）：配置场景 / 夹爪 / 相机 / 传感器 → 遥操或采集。"""

from __future__ import annotations

import subprocess
from pathlib import Path

from teleop_birth.ui import tui
from teleop_birth.ui.interactive_scene import apply_collect_io_prompts, load_scene_cli, prompt_collect_episodes_and_output
from teleop_birth.scene.plugins.camera import CameraPlugin
from teleop_birth.scene.plugins.collect import CollectPlugin
from teleop_birth.scene.plugins.gripper import GripperPlugin
from teleop_birth.scene.plugins.physics import PhysicsPlugin
from teleop_birth.scene.plugins.scene import compose_scene, load_scene_yaml
from teleop_birth.scene.plugins.scene.adapters import resolve_adapter
from teleop_birth.scene.plugins.sensor import SensorPlugin
from teleop_birth.scene.config import CollectConfig
from teleop_birth.ui.util import load_yaml


def _ensure_plugins_imported() -> None:
    # 触发 @register
    from teleop_birth.scene.plugins.frame import (  # noqa: F401
        ArmFrameMapPlugin,
        DesktopBasePlugin,
        FrameVizPlugin,
        GripToTcpPlugin,
        WristOrientPlugin,
        XrAlignPlugin,
    )

    _ = (CameraPlugin, CollectPlugin, GripperPlugin, SensorPlugin, PhysicsPlugin)


def action_workshop(root: Path) -> None:
    _ensure_plugins_imported()
    tui.section("场景工坊（插件）")
    print("  场景清单: configs/scenes/")
    print("  资产目录: src/teleop_birth/scene/assets/catalog.yaml")
    choice = tui.select(
        "工坊操作",
        [
            ("list_scenes", "列出场景清单"),
            ("list_robots", "列出已注册机器人（registry）"),
            ("inspect", "检查场景插件依赖"),
            ("list_grippers", "列出可用夹爪（catalog）"),
            ("list_cameras", "列出可用相机（catalog）"),
            ("frame", "坐标系插件（臂映射 / R_a / 桌面 base / 可视化）"),
            ("tune_camera", "相机视野调参（CameraTuner）"),
            ("run_scene", "打开场景：选遥操或自动采集"),
            ("back", "返回"),
        ],
        allow_cancel=True,
    )
    if choice in (None, "back"):
        return
    if choice == "list_scenes":
        _list_scenes(root)
    elif choice == "list_robots":
        _list_robots()
    elif choice == "inspect":
        _inspect_scene(root)
    elif choice == "list_grippers":
        for g in GripperPlugin().list_available(root):
            mark = "OK" if g["exists"] or g.get("builtin") else "MISSING"
            print(f"  [{mark}] {g['id']}: {g['title']} ({g['source']})")
    elif choice == "list_cameras":
        for c in CameraPlugin().list_available(root):
            mark = "OK" if c["exists"] else "MISSING"
            print(f"  [{mark}] {c['id']}: {c['title']}")
    elif choice == "frame":
        from teleop_birth.ui.interactive_frame import action_frame_workshop

        action_frame_workshop(root)
    elif choice == "tune_camera":
        CameraPlugin().tune(root, select=tui.select)
    elif choice == "run_scene":
        _run_scene(root)


def action_frame_viz(root: Path) -> None:
    """兼容旧 handler：进入坐标系插件子菜单。"""
    from teleop_birth.ui.interactive_frame import action_frame_workshop

    action_frame_workshop(root)


def _list_scenes(root: Path) -> None:
    d = root / "configs" / "scenes"
    if not d.is_dir():
        print("  （无 configs/scenes）")
        return
    for p in sorted(d.glob("*.yaml")):
        cfg = load_scene_yaml(p)
        print(f"  - {p.name}: {cfg.get('title') or cfg.get('id') or p.stem}  type={cfg.get('type')}")


def _list_robots() -> None:
    from teleop_birth.robot import list_robots

    for r in list_robots():
        print(f"  [{r.status:12}] {r.category:16} {r.id:20} {r.title}")
        if r.notes:
            print(f"                 {r.notes}")


def _pick_scene(root: Path) -> Path | None:
    d = root / "configs" / "scenes"
    cands = sorted(d.glob("*.yaml")) if d.is_dir() else []
    if not cands:
        print("没有场景清单。")
        return None
    labels = [(str(p), f"{p.name} — {(load_scene_yaml(p).get('title') or p.stem)}") for p in cands]
    choice = tui.select("选择场景", labels, allow_cancel=True)
    return Path(choice) if choice else None


def _inspect_scene(root: Path) -> None:
    path = _pick_scene(root)
    if not path:
        return
    ctx = compose_scene(root, path)
    PhysicsPlugin().attach(ctx)
    GripperPlugin().attach(ctx)
    CameraPlugin().attach(ctx)
    SensorPlugin().attach(ctx)
    CollectPlugin().attach(ctx)
    from teleop_birth.scene.plugins.frame import ArmFrameMapPlugin, DesktopBasePlugin, XrAlignPlugin

    ArmFrameMapPlugin().attach(ctx)
    XrAlignPlugin().attach(ctx)
    DesktopBasePlugin().attach(ctx)
    print(f"  adapter={ctx.name_map.get('adapter')}")
    if ctx.name_map.get("arm_profile"):
        print(f"  arm_profile={ctx.name_map.get('arm_profile')}")
    if ctx.name_map.get("mount_origin_z") is not None:
        print(
            f"  desktop_base={ctx.name_map.get('desktop_base_arm')} "
            f"table_top_z={ctx.name_map.get('table_top_z')} "
            f"mount_origin_z={ctx.name_map.get('mount_origin_z')}"
        )
    for plug in (GripperPlugin(), CameraPlugin(), SensorPlugin(), ArmFrameMapPlugin(), DesktopBasePlugin()):
        errs = plug.validate(ctx)
        if errs:
            print(f"  [{plug.id}] 问题:")
            for e in errs:
                print(f"    - {e}")
        else:
            print(f"  [{plug.id}] OK")


def _run_scene(root: Path) -> None:
    path = _pick_scene(root)
    if not path:
        return
    ctx_info = resolve_adapter(root, path)
    adapter = str(ctx_info["adapter"])
    mode = tui.select(
        "启动方式",
        [
            ("teleop", "Pico 遥操"),
            ("collect", "自动采集 → MCAP"),
            ("cancel", "取消"),
        ],
        allow_cancel=True,
    )
    if mode in (None, "cancel"):
        return
    if mode == "teleop":
        script_name = ctx_info.get("teleop_runner") or "start_pico_sim.sh"
        script = root / str(script_name)
        subprocess.call(["bash", str(script)], cwd=root)
        return

    # collect
    if ctx_info.get("collect_kind") == "logistics":
        cfg_path = root / "configs" / "robots" / "artixon_collect.yaml"
        raw = load_yaml(cfg_path) if cfg_path.is_file() else {}
        got = prompt_collect_episodes_and_output(
            default_episodes=int(raw.get("episodes", 2)),
            default_output_dir=str(raw.get("output_dir") or "logs/mcap/artixon"),
            title="采集前确认",
        )
        if got is None:
            print("已取消。")
            return
        episodes, output_dir = got
        script = root / "scripts" / "robots" / "artixon_collect.py"
        code = subprocess.call(
            [
                "uv",
                "run",
                "python",
                str(script),
                "--episodes",
                str(episodes),
                "--output-dir",
                str(output_dir),
            ],
            cwd=root,
        )
        if code != 0:
            print(f"采集退出码: {code}")
        return

    raw = dict(load_scene_cli(root).get("collect") or {})
    cfg = CollectConfig.from_dict(raw)
    if apply_collect_io_prompts(cfg, title="采集前确认") is None:
        print("已取消。")
        return
    print(f"  episodes={cfg.episodes}  output_dir={cfg.output_dir}")
    CollectPlugin().run_dual_ur(root, cfg)


def action_camera_tuner(root: Path) -> None:
    CameraPlugin().tune(root, select=tui.select)


def action_list_assets(root: Path) -> None:
    tui.section("资产 Catalog")
    print("夹爪:")
    for g in GripperPlugin().list_available(root):
        print(f"  - {g['id']}: {g['title']}")
    print("相机:")
    for c in CameraPlugin().list_available(root):
        print(f"  - {c['id']}: {c['title']}")
