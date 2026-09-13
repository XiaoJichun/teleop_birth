"""主菜单动作：全部由 configs/ui/menu.yaml 的 handler 字段引用。"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from teleop_birth.cli import tui
from teleop_birth.cli.interactive_setup import build_interactive_overrides
from teleop_birth.cli.interactive_scene import (
    CollectConfig,
    GenerateConfig,
    PlaceConfig,
    build_collect_config,
    build_generate_config,
    build_place_config,
    load_scene_cli,
)
from teleop_birth.cli.scene_runner import SceneOpsRunner
from teleop_birth.cli.util import hydra_default_choice, load_yaml


def _print_scene_cfg(title: str, data: dict) -> None:
    tui.section(title)
    print("  （来自 configs/scene/cli.yaml，改文件即改一键行为）")
    for key, value in data.items():
        print(f"  {key}: {value}")


def _ensure_collect_deps(root: Path) -> bool:
    """确保 mcap 可用；缺失则 uv sync --extra collect。"""
    try:
        import mcap  # noqa: F401
        return True
    except ImportError:
        print("\n未安装 mcap，正在执行: uv sync --extra collect …")
        code = subprocess.call(["uv", "sync", "--extra", "collect"], cwd=root)
        if code != 0:
            print(f"依赖安装失败，退出码 {code}")
            return False
        try:
            import mcap  # noqa: F401
            return True
        except ImportError:
            print("仍无法 import mcap，请手动: uv sync --extra collect")
            return False


def action_pico_sim_one_click(root: Path) -> None:
    """调用 start_pico_sim.sh：自动装依赖、起 PC Service、开 MuJoCo。"""
    script = root / "start_pico_sim.sh"
    print(f"\n>>> bash {script}\n")
    code = subprocess.call(["bash", str(script)], cwd=root)
    if code != 0:
        print(f"\n一键启动退出码: {code}")


def action_run_teleop(root: Path) -> None:
    overrides = build_interactive_overrides(root)
    if overrides is None:
        print("\n已取消，未启动遥操。")
        return
    cmd = [sys.executable, str(root / "run_teleop.py"), *overrides]
    print(f"\n>>> {' '.join(cmd)}\n")
    code = subprocess.call(cmd, cwd=root)
    if code != 0:
        print(f"\n遥操退出码: {code}")


def action_scene_generate_one_click(root: Path) -> None:
    """一键生成物品：直接读 configs/scene/cli.yaml → generate。"""
    raw = dict(load_scene_cli(root).get("generate") or {})
    cfg = GenerateConfig.from_dict(raw)
    _print_scene_cfg("一键生成物品", raw)
    SceneOpsRunner(root).run_generate(cfg)


def action_scene_place_one_click(root: Path) -> None:
    """一键随机摆放：直接读 configs/scene/cli.yaml → place。"""
    raw = dict(load_scene_cli(root).get("place") or {})
    cfg = PlaceConfig.from_dict(raw)
    _print_scene_cfg("一键摆放物品", raw)
    SceneOpsRunner(root).run_place(cfg)


def action_scene_collect_one_click(root: Path) -> None:
    """一键自动化采数据：直接读 configs/scene/cli.yaml → collect。"""
    if not _ensure_collect_deps(root):
        return
    raw = dict(load_scene_cli(root).get("collect") or {})
    cfg = CollectConfig.from_dict(raw)
    _print_scene_cfg("一键自动化采数据 → MCAP", raw)
    if cfg.render:
        print("\n  ★ 将打开 MuJoCo 窗口，可人工盯着采集过程")
        print(f"  ★ 观摩节拍 pace_s={cfg.pace_s}（改 configs/scene/cli.yaml）")
    print(f"\n>>> 开始采集 task={cfg.task} episodes={cfg.episodes}\n")
    SceneOpsRunner(root).run_collect(cfg)


def action_scene_generate_props(root: Path) -> None:
    cfg = build_generate_config(root)
    if cfg is None:
        print("\n已取消，未生成物品。")
        return
    SceneOpsRunner(root).run_generate(cfg)


def action_scene_place_props(root: Path) -> None:
    cfg = build_place_config(root)
    if cfg is None:
        print("\n已取消，未摆放物品。")
        return
    SceneOpsRunner(root).run_place(cfg)


def action_scene_collect(root: Path) -> None:
    if not _ensure_collect_deps(root):
        return
    cfg = build_collect_config(root)
    if cfg is None:
        print("\n已取消，未开始采集。")
        return
    SceneOpsRunner(root).run_collect(cfg)


def action_setup_xrobo(root: Path) -> None:
    from teleop_birth.xr.sources import check_xrobo_sdk_available

    ok, msg = check_xrobo_sdk_available(root)
    tui.section("XRobo SDK 状态")
    if ok:
        print("  xrobotoolkit_sdk: 已安装")
    else:
        print("  xrobotoolkit_sdk: 未安装")
        print(msg)

    setup = load_yaml(root / "configs" / "setup" / "xrobo.yaml")
    extract = (setup.get("pc_service") or {}).get("extract_dir")
    print(f"  本地 PC Service 目录: {root / extract}")
    print(f"  连接文档: {root / 'doc' / 'Pico连接指南.md'}")

    do_install = tui.confirm("现在运行 install_xrobo.sh（无需 sudo）？", default=not ok)
    if not do_install:
        return
    script = root / "tools" / "scripts" / "install_xrobo.sh"
    code = subprocess.call(["bash", str(script)], cwd=root)
    if code != 0:
        print(f"\n安装退出码: {code}")
        return
    ok2, msg2 = check_xrobo_sdk_available(root)
    print("\n安装后检查:", "OK" if ok2 else msg2)


def action_show_defaults(root: Path) -> None:
    cfg_root = root / "configs"
    tui.section("Hydra defaults")
    print(f"  [device]  source       = {hydra_default_choice(cfg_root, 'source')}")
    print(f"  [device]  backend      = {hydra_default_choice(cfg_root, 'backend')}")
    print(f"  [device]  end_effector = {hydra_default_choice(cfg_root, 'end_effector')}")
    print(f"  [algo]    ik           = {hydra_default_choice(cfg_root, 'ik')}")
    print(f"  [algo]    mapper       = {hydra_default_choice(cfg_root, 'mapper')}")
    runtime = load_yaml(cfg_root / "algorithm" / "runtime" / "default.yaml")
    paths = load_yaml(cfg_root / "device" / "paths" / "default.yaml")
    print("  algorithm/runtime/default.yaml:")
    for key, value in runtime.items():
        print(f"    {key}: {value}")
    print("  device/paths/default.yaml:")
    for key, value in paths.items():
        print(f"    {key}: {value}")

    scene_cli = load_scene_cli(root)
    tui.section("场景 defaults（configs/scene/cli.yaml）")
    for section in ("generate", "place", "collect"):
        block = scene_cli.get(section) or {}
        print(f"  {section}:")
        for key, value in block.items():
            print(f"    {key}: {value}")


def action_run_tests(root: Path) -> None:
    cmd = [sys.executable, "-m", "pytest", "-q"]
    print(f"\n>>> {' '.join(cmd)}\n")
    code = subprocess.call(cmd, cwd=root)
    if code != 0:
        print(f"\n测试退出码: {code}")
