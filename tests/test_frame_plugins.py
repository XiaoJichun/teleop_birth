"""坐标系插件：R_a 生成 / 臂剖面 / 桌面 base。"""

from __future__ import annotations

import numpy as np
import pytest

from teleop_birth.teleop.control import openxr_yup_to_mujoco_zup, resolve_xr_align
from teleop_birth.teleop.frame import (
    build_ra,
    get_arm_profile,
    get_desktop_base,
    mount_origin_z,
    resolve_align_matrix,
    table_top_z_from_scene,
)


def test_build_ra_matches_legacy_openxr_yup_to_zup():
    R = build_ra(xr_right_to="+x", xr_up_to="+z", xr_forward_to="+y")
    np.testing.assert_allclose(R, openxr_yup_to_mujoco_zup(), atol=1e-12)
    # XR forward (-Z) → +Y
    np.testing.assert_allclose(R @ np.array([0.0, 0.0, -1.0]), [0.0, 1.0, 0.0])
    # XR up (+Y) → +Z
    np.testing.assert_allclose(R @ np.array([0.0, 1.0, 0.0]), [0.0, 0.0, 1.0])


def test_resolve_align_from_profile_and_mapping():
    R_prof = resolve_align_matrix(profile_id="mujoco_table_zup")
    R_map = resolve_align_matrix(mapping={"right": "+x", "up": "+z", "forward": "+y"})
    R_name = resolve_xr_align("openxr_yup_to_zup", None)
    np.testing.assert_allclose(R_prof, R_map)
    np.testing.assert_allclose(R_name, R_map)


def test_arm_profile_sim_real_port():
    from teleop_birth.teleop.frame.profiles import FrameMapPort

    p = get_arm_profile("mujoco_table_zup")
    sim = FrameMapPort(profile=p, kind="sim")
    real = FrameMapPort(profile=p, kind="real")
    np.testing.assert_allclose(sim.align_matrix(), real.align_matrix())
    assert "xr_align_matrix" in sim.mapper_overrides()


def test_desktop_base_ur5e_idempotent_with_legacy_table():
    scene = {
        "base_height": 0.42,
        "table": {"size": [0.55, 0.35, 0.02], "z_offset_from_base": -0.02},
    }
    top = table_top_z_from_scene(scene)
    z = mount_origin_z(top, "ur5e")
    assert top == pytest.approx(0.42)
    assert z == pytest.approx(0.42)


def test_desktop_base_se3_contact_to_urdf():
    """换臂：接触系一致，仅 T_contact_urdf 的 Z 抬升不同。"""
    from teleop_birth.teleop.frame import T_contact_urdf, T_world_contact, T_world_urdf

    top, xy = 0.44, (0.1, 0.2)
    Twc = T_world_contact(top, xy)
    np.testing.assert_allclose(Twc[:3, 3], [0.1, 0.2, 0.44])

    T_ur5e = T_contact_urdf("ur5e")
    T_art = T_contact_urdf("artixon_6a")
    np.testing.assert_allclose(T_ur5e, np.eye(4))
    np.testing.assert_allclose(T_art[2, 3], 0.02)

    Tw_u = T_world_urdf(top, xy, "ur5e")
    Tw_a = T_world_urdf(top, xy, "artixon_6a")
    np.testing.assert_allclose(Tw_u[:3, 3], [0.1, 0.2, 0.44])
    np.testing.assert_allclose(Tw_a[:3, 3], [0.1, 0.2, 0.46])
    # 轴一致（无额外旋转）
    np.testing.assert_allclose(Tw_u[:3, :3], np.eye(3))
    np.testing.assert_allclose(Tw_a[:3, :3], np.eye(3))


def test_desktop_base_artixon_matches_legacy_mount():
    # table_z=0.42, half=0.02 → top=0.44; pedestal 0.02 → mount 0.46 = 旧 table_z+0.04
    top = 0.42 + 0.02
    z = mount_origin_z(top, get_desktop_base("artixon_6a"))
    assert z == pytest.approx(0.46)


def test_desktop_base_plugin_attaches_T():
    from pathlib import Path

    from teleop_birth.scene.plugins.base import PluginContext
    from teleop_birth.scene.plugins.frame import DesktopBasePlugin

    ctx = PluginContext(
        project_root=Path("."),
        scene_cfg={
            "base_height": 0.42,
            "table": {"size": [0.55, 0.35, 0.02], "z_offset_from_base": -0.02},
            "desktop_base": {"arm_id": "ur5e", "xy": [0.0, 0.0]},
        },
    )
    DesktopBasePlugin().attach(ctx)
    assert ctx.extras["mount_origin_z"] == pytest.approx(0.42)
    T = np.asarray(ctx.extras["T_world_base"])
    assert T.shape == (4, 4)
    np.testing.assert_allclose(T[2, 3], 0.42)


def test_xr_align_plugin_build():
    from teleop_birth.scene.plugins.frame import XrAlignPlugin

    R = XrAlignPlugin().build(xr_right_to="+x", xr_up_to="+z", xr_forward_to="+y")
    np.testing.assert_allclose(R, openxr_yup_to_mujoco_zup())


def test_wrist_orient_yaw_only():
    """绕手柄上转 → 房间 ψ；拧腕为 0 时接近轴方向与 home 一致（仍朝下）。"""
    from teleop_birth.scene.plugins.frame import WristOrientPlugin

    res = WristOrientPlugin().map_demo(pitch_deg=90.0, roll_deg=0.0, yaw_deg=0.0)
    assert abs(res.psi) > 1.0  # ~90°
    np.testing.assert_allclose(res.w, 0.0, atol=1e-8)
    # 接近轴（TCP +Z）仍朝世界 −Z
    np.testing.assert_allclose(res.R_t[:, 2], res.R_r0[:, 2], atol=1e-5)


def test_build_M_tip_down_matches_legacy():
    from teleop_birth.teleop.frame import build_M, default_grip_to_tcp_M, resolve_grip_tcp_M

    M = build_M(grip_right_to="+y", grip_up_to="+x", grip_aim_to="+z")
    np.testing.assert_allclose(M, default_grip_to_tcp_M(), atol=1e-12)
    np.testing.assert_allclose(resolve_grip_tcp_M("tip_down"), M)
    # aim (-Z) → approach (+Z)
    np.testing.assert_allclose(M @ np.array([0.0, 0.0, -1.0]), [0.0, 0.0, 1.0])


def test_tip_forward_aim_still_to_approach():
    from teleop_birth.teleop.frame import get_grip_tcp_profile, resolve_grip_tcp_M

    M = resolve_grip_tcp_M(profile_id="tip_forward")
    np.testing.assert_allclose(M @ np.array([0.0, 0.0, -1.0]), [0.0, 0.0, 1.0])
    assert float(np.linalg.det(M)) == pytest.approx(1.0)
    home = get_grip_tcp_profile("tip_forward").home_R()
    # tip_forward home：接近轴朝世界 +Y
    np.testing.assert_allclose(home[:, 2], [0.0, 1.0, 0.0], atol=1e-12)


def test_grip_to_tcp_plugin_and_mapper():
    from teleop_birth.scene.plugins.frame import GripToTcpPlugin
    from teleop_birth.teleop.mapping import DualArmMapper
    from teleop_birth.teleop.schema import ButtonState, XRObservation, identity_pose

    M = GripToTcpPlugin().build(profile_id="tip_forward")
    np.testing.assert_allclose(M, np.diag([1.0, -1.0, -1.0]))

    m = DualArmMapper(
        scale=1.0,
        filter_alpha=1.0,
        map_head=False,
        grip_threshold=0.5,
        clutch_mode="always",
        hold_s=0.0,
        position_only=False,
        grip_to_tcp="tip_forward",
    )
    np.testing.assert_allclose(m.clutch._orient.M, M)
    xr0 = identity_pose()
    robot0 = identity_pose()
    robot0[:3] = [0.0, 0.4, 0.5]
    obs = XRObservation(
        timestamp_ns=0,
        left_controller=xr0,
        right_controller=xr0,
        buttons=ButtonState(left_grip=1.0, right_grip=1.0),
    )
    m.map(obs, {"left_ee": robot0, "right_ee": robot0})


def test_wrist_orient_matches_clutch():
    from teleop_birth.teleop.control import RelativeClutch, openxr_yup_to_mujoco_zup
    from teleop_birth.teleop.frame.orientation import WristOrientationMap
    from teleop_birth.teleop.schema import identity_pose, matrix_to_pose, pose_to_matrix

    Ra = openxr_yup_to_mujoco_zup()
    R_tcp = np.diag([1.0, -1.0, -1.0])
    T = np.eye(4)
    T[:3, :3] = R_tcp
    T[:3, 3] = [0.0, 0.4, 0.5]
    robot0 = matrix_to_pose(T)
    xr0 = identity_pose()
    xr1 = identity_pose()
    # +90° about XR +Y
    xr1[3:] = np.array([0.0, np.sin(np.pi / 4), 0.0, np.cos(np.pi / 4)])

    clutch = RelativeClutch(
        grip_threshold=0.5,
        mode="always",
        position_only=False,
        xr_align=Ra,
    )
    clutch.update("left", 1.0, xr0, robot0, scale=1.0)
    eng, target = clutch.update("left", 1.0, xr1, robot0, scale=1.0)
    assert eng and target is not None

    ori = WristOrientationMap(R_a=Ra).map_from_poses(
        xr_ref_pose=xr0, xr_now_pose=xr1, robot_ref_pose=robot0
    )
    np.testing.assert_allclose(pose_to_matrix(target)[:3, :3], ori.R_t, atol=1e-6)


def test_mapper_accepts_arm_profile():
    from teleop_birth.teleop.mapping import DualArmMapper
    from teleop_birth.teleop.schema import ButtonState, XRObservation, identity_pose

    m = DualArmMapper(
        scale=1.0,
        filter_alpha=1.0,
        map_head=False,
        grip_threshold=0.5,
        clutch_mode="always",
        hold_s=0.0,
        position_only=True,
        arm_profile="mujoco_table_zup",
    )
    xr0 = identity_pose()
    robot0 = identity_pose()
    robot0[:3] = [0.0, 0.4, 0.5]
    obs = XRObservation(
        timestamp_ns=0,
        left_controller=xr0,
        right_controller=xr0,
        buttons=ButtonState(left_grip=1.0, right_grip=1.0),
    )
    m.map(obs, {"left_ee": robot0, "right_ee": robot0})
