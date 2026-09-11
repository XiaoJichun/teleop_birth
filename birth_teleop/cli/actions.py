"""主菜单动作：全部由 configs/menu.yaml 的 handler 字段引用。"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from birth_teleop.cli import tui
from birth_teleop.cli.interactive_setup import build_interactive_overrides
from birth_teleop.cli.util import hydra_default_choice, load_yaml


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


def action_setup_xrobo(root: Path) -> None:
    from birth_teleop.xr.sources import check_xrobo_sdk_available

    ok, msg = check_xrobo_sdk_available(root)
    tui.section("XRobo SDK 状态")
    if ok:
        print("  xrobotoolkit_sdk: 已安装")
    else:
        print("  xrobotoolkit_sdk: 未安装")
        print(msg)

    setup = load_yaml(root / "configs" / "xrobo_setup.yaml")
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
    cfg = load_yaml(root / "configs" / "config.yaml")
    tui.section("configs/config.yaml")
    print(f"  defaults.source       = {hydra_default_choice(root / 'configs', 'source')}")
    print(f"  defaults.backend      = {hydra_default_choice(root / 'configs', 'backend')}")
    print(f"  defaults.mapper       = {hydra_default_choice(root / 'configs', 'mapper')}")
    print(f"  defaults.end_effector = {hydra_default_choice(root / 'configs', 'end_effector')}")
    runtime = cfg.get("runtime") or {}
    paths = cfg.get("paths") or {}
    print("  runtime:")
    for key, value in runtime.items():
        print(f"    {key}: {value}")
    print("  paths:")
    for key, value in paths.items():
        print(f"    {key}: {value}")


def action_run_tests(root: Path) -> None:
    cmd = [sys.executable, "-m", "pytest", "-q"]
    print(f"\n>>> {' '.join(cmd)}\n")
    code = subprocess.call(cmd, cwd=root)
    if code != 0:
        print(f"\n测试退出码: {code}")
