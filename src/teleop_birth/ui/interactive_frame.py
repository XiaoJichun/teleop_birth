"""坐标系插件交互式终端：臂映射 / R_a 生成 / 桌面 base / 可视化。"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np

from teleop_birth.ui import tui

_AXIS_OPTS: list[tuple[str, str]] = [
    ("+x", "+X / right"),
    ("-x", "-X / left"),
    ("+y", "+Y / forward"),
    ("-y", "-Y / back"),
    ("+z", "+Z / up"),
    ("-z", "-Z / down"),
]


def _ensure() -> None:
    from teleop_birth.scene.plugins.frame import (  # noqa: F401
        ArmFrameMapPlugin,
        DesktopBasePlugin,
        FrameVizPlugin,
        GripToTcpPlugin,
        WristOrientPlugin,
        XrAlignPlugin,
    )


def action_frame_workshop(root: Path) -> None:
    """主菜单 / 工坊入口：坐标系插件子菜单。"""
    _ensure()
    while True:
        tui.section("坐标系插件（手柄↔臂 / R_a / M / 桌面 base）")
        print("  配置: configs/plugins/frame/")
        print("  核心: teleop_birth.teleop.frame + scene.plugins.frame")
        choice = tui.select(
            "坐标系操作",
            [
                ("list_profiles", "① 列出臂坐标剖面（仿真/真机共用）"),
                ("pick_profile", "① 查看剖面并打印 R_a / YAML"),
                ("build_ra", "① 终端生成 R_a（手选右/上/前映射）"),
                ("orient_walk", "② 腕部姿态分步演示（R_t）"),
                ("list_m", "③ 列出握把→夹爪 M 剖面"),
                ("build_m", "③ 终端生成 M（尖端朝下/朝前）"),
                ("desktop_list", "④ 列出桌面 base 剖面"),
                ("desktop_calc", "④ 计算 mount_z（桌面顶 → URDF 原点）"),
                ("viz_qt", "共用界面 PyQt（R_a + R_t + M，需 --extra viz）"),
                ("sim_real", "仿真↔真机衔接说明"),
                ("back", "返回"),
            ],
            allow_cancel=True,
        )
        if choice in (None, "back"):
            return
        if choice == "list_profiles":
            _list_arm_profiles()
        elif choice == "pick_profile":
            _pick_and_show_profile()
        elif choice == "build_ra":
            _interactive_build_ra()
        elif choice == "orient_walk":
            _interactive_orient_walk()
        elif choice == "list_m":
            _list_grip_tcp()
        elif choice == "build_m":
            _interactive_build_m()
        elif choice == "desktop_list":
            _list_desktop_bases()
        elif choice == "desktop_calc":
            _calc_desktop_mount()
        elif choice == "viz_qt":
            _launch_qt_viz()
        elif choice == "sim_real":
            _print_sim_real_bridge()


def action_frame_viz(root: Path) -> None:
    """兼容旧菜单项：直接进坐标系工坊。"""
    action_frame_workshop(root)


def _list_arm_profiles() -> None:
    from teleop_birth.scene.plugins.frame import ArmFrameMapPlugin

    tui.section("臂坐标剖面")
    for p in ArmFrameMapPlugin().list_available():
        print(f"  - {p['id']}: {p['title']}")
        sims = ",".join(p.get("sim_backends") or []) or "（任意）"
        reals = ",".join(p.get("real_backends") or []) or "（任意）"
        print(f"      sim=[{sims}]  real=[{reals}]")
        if p.get("notes"):
            print(f"      {p['notes']}")


def _pick_and_show_profile() -> None:
    from teleop_birth.scene.plugins.frame import ArmFrameMapPlugin
    from teleop_birth.teleop.frame.profiles import FrameMapPort, get_arm_profile

    plug = ArmFrameMapPlugin()
    opts = [(p["id"], f"{p['id']} — {p['title']}") for p in plug.list_available()]
    pid = tui.select("选择臂坐标剖面", opts, allow_cancel=True)
    if not pid:
        return
    kind = tui.select(
        "后端类型（同一 R_a，只换 RobotEndpoint）",
        [("sim", "仿真 sim"), ("real", "真机 real"), ("mock", "mock")],
        default="sim",
        allow_cancel=True,
    )
    if not kind:
        return
    port = FrameMapPort(profile=get_arm_profile(str(pid)), kind=str(kind))  # type: ignore[arg-type]
    R = port.align_matrix()
    tui.section(f"剖面 {pid}  ({kind})")
    _print_ra(R)
    print("  Mapper YAML 片段:")
    print(f"    arm_profile: {pid}")
    print("    # 或直接写矩阵:")
    _print_yaml_matrix(R, indent=4)
    q = port.controller_quat_xyzw()
    if q:
        print(f"    controller_quat_xyzw: {q}")


def _interactive_build_ra() -> None:
    from teleop_birth.scene.plugins.frame import XrAlignPlugin
    from teleop_birth.teleop.frame.axes import openxr_semantic_basis

    tui.section("生成 R_a（OpenXR → 机械臂房间系）")
    print("  头显：+X 右、+Y 上、−Z 前")
    print("  选「映射后」落在房间系的哪根轴")

    right = tui.select("XR 右 (+X) →", _AXIS_OPTS, default="+x", allow_cancel=True)
    if right is None:
        return
    up = tui.select("XR 上 (+Y) →", _AXIS_OPTS, default="+z", allow_cancel=True)
    if up is None:
        return
    forward = tui.select("XR 前 (−Z) →", _AXIS_OPTS, default="+y", allow_cancel=True)
    if forward is None:
        return

    try:
        R = XrAlignPlugin().build(xr_right_to=right, xr_up_to=up, xr_forward_to=forward)
    except Exception as exc:  # noqa: BLE001
        print(f"  生成失败: {exc}")
        return

    tui.section("结果")
    _print_ra(R)
    print("  自检（头显语义 → 房间）:")
    for name, v in openxr_semantic_basis().items():
        w = R @ v
        print(f"    XR {name:8} {np.array2string(v, precision=0)} → {np.array2string(w, precision=4)}")
    print("  mapper YAML:")
    print("    xr_mapping:")
    print(f"      right: \"{right}\"")
    print(f"      up: \"{up}\"")
    print(f"      forward: \"{forward}\"")
    print("    # 或")
    _print_yaml_matrix(R, indent=4, key="xr_align_matrix")

    # ASCII 简图
    print("\n  轴向简图（房间系）:")
    _ascii_axes(R)


def _list_desktop_bases() -> None:
    from teleop_birth.scene.plugins.frame import DesktopBasePlugin

    tui.section("桌面 base 剖面")
    for s in DesktopBasePlugin().list_available():
        print(f"  - {s['id']}: {s['title']}")
        print(
            f"      origin_above_bottom={s['origin_above_bottom_m']} m  "
            f"pedestal={s['pedestal_height_m']} m  body={s['base_body']}"
        )
        if s.get("notes"):
            print(f"      {s['notes']}")


def _calc_desktop_mount() -> None:
    from teleop_birth.scene.plugins.frame import DesktopBasePlugin
    from teleop_birth.teleop.frame.desktop_base import get_desktop_base, mount_origin_z, mount_pose_xyz

    plug = DesktopBasePlugin()
    opts = [(s["id"], f"{s['id']} — {s['title']}") for s in plug.list_available()]
    arm_id = tui.select("选择机械臂", opts, allow_cancel=True)
    if not arm_id:
        return
    raw_top = tui.text("桌面顶面高度 table_top_z (m)", default="0.44")
    if raw_top is None:
        return
    raw_xy = tui.text("基座 XY（逗号分隔，可选）", default="0.0, 0.0")
    if raw_xy is None:
        return
    try:
        top = float(raw_top.strip())
        parts = [float(x) for x in raw_xy.replace("，", ",").split(",") if x.strip()]
        xy: Sequence[float] = parts if len(parts) >= 2 else (0.0, 0.0)
    except ValueError as exc:
        print(f"  输入无效: {exc}")
        return

    spec = get_desktop_base(str(arm_id))
    z = mount_origin_z(top, spec)
    xyz = mount_pose_xyz(top, xy, spec)
    tui.section(f"桌面挂载 — {arm_id}")
    print(f"  table_top_z           = {top:.4f} m")
    print(f"  pedestal_height_m     = {spec.pedestal_height_m:.4f} m")
    print(f"  origin_above_bottom_m = {spec.origin_above_bottom_m:.4f} m")
    print(f"  → mount_origin_z      = {z:.4f} m")
    print(f"  → mount pos xyz       = [{xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}]")
    Tw = plug.mount_T(top, xy, str(arm_id))
    print("  → T_world_urdf (4×4):")
    for row in Tw:
        print(f"      [{row[0]: .4f}, {row[1]: .4f}, {row[2]: .4f}, {row[3]: .4f}]")
    print("  场景 YAML 片段:")
    print("    desktop_base:")
    print(f"      arm_id: {arm_id}")


def _interactive_orient_walk() -> None:
    """终端分步演示第 2 步姿态（与 WristOrientPlugin / PyQt 页签② 同一套）。"""
    from teleop_birth.scene.plugins.frame import GripToTcpPlugin, WristOrientPlugin

    tui.section("第 2 步：腕部姿态（小白分步）")
    print(
        """
  人话：按住 Grip 之后——
    · 左右转头/转身 → 夹爪绕房间竖直轴转（ψ / R_yaw）
    · 拧腕、侧倾 → 贴在夹爪自己的轴上转（R_tool），不会把「朝下」翻开
  公式：R_t = R_yaw @ R_r0 @ R_tool
""".rstrip()
    )
    m_opts = [(p["id"], f"{p['id']} — {p['title']}") for p in GripToTcpPlugin().list_available()]
    mid = tui.select("M 剖面（home 朝向）", m_opts, default="tip_down", allow_cancel=True)
    if mid is None:
        return
    raw_p = tui.text("绕手柄「上」转多少度？（→ 房间竖直转 ψ）", default="45")
    if raw_p is None:
        return
    raw_r = tui.text("绕手柄「右」转多少度？（→ 侧倾，经 M）", default="0")
    if raw_r is None:
        return
    raw_t = tui.text("绕瞄准轴拧多少度？（→ 拧腕，经 M）", default="30")
    if raw_t is None:
        return
    try:
        pitch = float(raw_p.strip())
        roll = float(raw_r.strip())
        twist = float(raw_t.strip())
    except ValueError as exc:
        print(f"  输入无效: {exc}")
        return

    res = WristOrientPlugin().map_demo(
        pitch_deg=pitch, roll_deg=roll, yaw_deg=twist, grip_to_tcp=str(mid)
    )
    print()
    for line in res.explain_zh():
        print(f"  {line}")
    print(f"\n  ψ = {res.psi:.4f} rad = {np.degrees(res.psi):.1f}°")
    print(f"  w (已清零 wy) = {np.array2string(res.w, precision=3)}")
    print("  R_t =")
    for row in res.R_t:
        print(f"    [{row[0]:8.4f}, {row[1]:8.4f}, {row[2]:8.4f}]")
    print(f"\n  配置：mapper.grip_to_tcp: {mid}")
    print("  图形对照：本菜单 →「共用界面 PyQt」→ 页签②/③")


def _list_grip_tcp() -> None:
    from teleop_birth.scene.plugins.frame import GripToTcpPlugin

    tui.section("握把→夹爪 M 剖面")
    for p in GripToTcpPlugin().list_available():
        print(f"  - {p['id']}: {p['title']}  (home={p.get('home_kind')})")
        if p.get("notes"):
            print(f"      {p['notes']}")


def _interactive_build_m() -> None:
    from teleop_birth.scene.plugins.frame import GripToTcpPlugin

    tui.section("生成 M（握把体轴 → 夹爪体轴）")
    print("  握把：+X 右、+Y 上、−Z 瞄准；夹爪 +Z = 接近轴（尖端）")
    print("  尖端朝下选 tip_down；尖端朝前选 tip_forward；或手选三轴")

    plug = GripToTcpPlugin()
    mode = tui.select(
        "方式",
        [
            ("preset", "用预设剖面"),
            ("custom", "手选 right/up/aim 映射"),
        ],
        default="preset",
        allow_cancel=True,
    )
    if mode is None:
        return
    if mode == "preset":
        opts = [(p["id"], f"{p['id']} — {p['title']}") for p in plug.list_available()]
        pid = tui.select("剖面", opts, default="tip_down", allow_cancel=True)
        if not pid:
            return
        try:
            M = plug.build(profile_id=str(pid))
        except Exception as exc:  # noqa: BLE001
            print(f"  生成失败: {exc}")
            return
        tui.section(f"结果 — {pid}")
        _print_ra(M)
        print("  mapper YAML:")
        print(f"    grip_to_tcp: {pid}")
        return

    right = tui.select("握把右 (+X) →", _AXIS_OPTS, default="+y", allow_cancel=True)
    if right is None:
        return
    up = tui.select("握把上 (+Y) →", _AXIS_OPTS, default="+x", allow_cancel=True)
    if up is None:
        return
    aim = tui.select("握把瞄准 (−Z) →", _AXIS_OPTS, default="+z", allow_cancel=True)
    if aim is None:
        return
    try:
        M = plug.build(grip_right_to=right, grip_up_to=up, grip_aim_to=aim)
    except Exception as exc:  # noqa: BLE001
        print(f"  生成失败: {exc}")
        return
    tui.section("结果")
    _print_ra(M)
    print("  mapper YAML:")
    print("    grip_to_tcp_mapping:")
    print(f"      right: \"{right}\"")
    print(f"      up: \"{up}\"")
    print(f"      aim: \"{aim}\"")
    _print_yaml_matrix(M, indent=4, key="grip_to_tcp_matrix")


def _launch_qt_viz() -> None:
    from teleop_birth.scene.plugins.frame import FrameVizPlugin

    tui.section("共用坐标系界面（PyQt）")
    print("  页签① R_a   页签② R_t   页签③ M")
    print("  依赖: uv sync --extra viz")
    which = tui.select(
        "打开哪个页签？",
        [
            ("axes", "① 轴对齐 R_a"),
            ("orient", "② 腕部姿态 R_t"),
            ("m", "③ 握把→夹爪 M"),
            ("cancel", "取消"),
        ],
        default="axes",
        allow_cancel=True,
    )
    if which in (None, "cancel"):
        return
    try:
        FrameVizPlugin().launch(tab=str(which))
    except ImportError as exc:
        print(f"  {exc}")
        print("  无 PyQt 时请用上方「终端生成 R_a / M」/「腕部姿态分步演示」。")
    except Exception as exc:  # noqa: BLE001
        print(f"  启动失败: {exc}")


def _print_sim_real_bridge() -> None:
    tui.section("仿真 ↔ 真机无缝衔接")
    print(
        """
  坐标映射与 Backend 解耦：
    1. 选同一 arm_profile（或同一 xr_mapping）→ 同一 R_a
    2. 仿真：backend.type = mujoco_dual_ur / logistics_x2arm …
    3. 真机：backend.type = ur_realtime / artixon_realtime …（实现 RobotEndpoint）
    4. Mapper / TeleopLoop / XR 源 不变；只换 RobotEndpoint

  交互式遥操配置里也可选臂剖面（自定义遥操 → 算法参数）。

  代码接口:
    from teleop_birth.scene.plugins.frame import ArmFrameMapPlugin
    port = ArmFrameMapPlugin().for_arm("mujoco_table_zup", kind="real")
    R_a = port.align_matrix()
    overrides = port.mapper_overrides()
""".rstrip()
    )


def _print_ra(R: np.ndarray) -> None:
    print("  R_a =")
    for row in R:
        print(f"    [{row[0]:8.4f}, {row[1]:8.4f}, {row[2]:8.4f}]")


def _print_yaml_matrix(R: np.ndarray, *, indent: int = 2, key: str = "xr_align_matrix") -> None:
    pad = " " * indent
    print(f"{pad}{key}:")
    for row in R:
        print(f"{pad}  - [{row[0]:.6g}, {row[1]:.6g}, {row[2]:.6g}]")


def _ascii_axes(R: np.ndarray) -> None:
    """在终端打印映射后的右/上/前单位向量。"""
    from teleop_birth.teleop.frame.axes import openxr_semantic_basis

    basis = openxr_semantic_basis()
    labels = {"right": "右", "up": "上", "forward": "前"}
    for name, v in basis.items():
        w = R @ v
        axis = ""
        for i, lab in enumerate("XYZ"):
            if abs(w[i]) > 0.5:
                sign = "+" if w[i] > 0 else "-"
                axis = f"{sign}{lab}"
                break
        print(f"    手{labels[name]} → 房间 {axis or np.array2string(w, precision=2)}")
