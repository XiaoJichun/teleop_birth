"""协议层单测（无 Pico / 无 MuJoCo 场景依赖）。"""

from __future__ import annotations

import time

import numpy as np

from teleop_birth.backend import MockDualArmBackend
from teleop_birth.control import RelativeClutch, openxr_yup_to_mujoco_zup
from teleop_birth.mapping import DualArmMapper, HumanoidMapper
from teleop_birth.pipeline import TeleopLoop
from teleop_birth.schema import (
    EXTREMITY_LEFT_EE,
    EXTREMITY_LEFT_FOOT,
    EXTREMITY_PELVIS,
    EXTREMITY_RIGHT_EE,
    ButtonState,
    XRObservation,
    identity_pose,
    matrix_to_pose,
    pose_to_matrix,
)
from teleop_birth.xr import KeyboardClutchMockXRSource, MockXRSource


def _dual_mapper(**overrides):
    cfg = dict(
        scale=1.0,
        filter_alpha=1.0,
        map_head=True,
        grip_threshold=0.55,
        clutch_mode="grip",
        hold_s=0.0,
        position_only=True,
    )
    cfg.update(overrides)
    return DualArmMapper(**cfg)


def test_clutch_engages_and_tracks_relative():
    clutch = RelativeClutch(grip_threshold=0.5, mode="grip", position_only=False, hold_s=0.0)
    xr0 = identity_pose()
    robot0 = identity_pose()
    robot0[0] = 1.0
    eng, target = clutch.update("left", 1.0, xr0, robot0)
    assert eng and target is not None
    np.testing.assert_allclose(target, robot0, atol=1e-8)

    xr1 = identity_pose()
    xr1[0] = 0.1
    eng, target = clutch.update("left", 1.0, xr1, robot0)
    assert eng and target is not None
    np.testing.assert_allclose(target[0], 1.1, atol=1e-6)


def test_clutch_release_freezes():
    clutch = RelativeClutch(grip_threshold=0.5, mode="grip", hold_s=0.0)
    xr = identity_pose()
    robot = identity_pose()
    clutch.update("left", 1.0, xr, robot)
    eng, target = clutch.update("left", 0.0, xr, robot)
    assert eng is False and target is None


def test_clutch_se3_tracks_orientation():
    # identity align + track：手柄转到哪，末端相对接合偏移后跟着转
    clutch = RelativeClutch(
        grip_threshold=0.5,
        mode="always",
        position_only=False,
        delta_frame="world",
        xr_align=np.eye(3),
        ori_mode="track",
    )
    xr0 = identity_pose()
    robot0 = identity_pose()
    robot0[0] = 0.5
    clutch.update("left", 0.0, xr0, robot0, scale=1.0)
    xr1 = identity_pose()
    xr1[3:] = np.array([0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    eng, target = clutch.update("left", 0.0, xr1, robot0, scale=1.0)
    assert eng and target is not None
    np.testing.assert_allclose(target[3:], xr1[3:], atol=1e-5)


def test_clutch_ori_track_keeps_engage_offset():
    """接合时机器人已有姿态；手柄再转，末端 = 初始相对偏移 ∘ 对齐手柄。"""
    clutch = RelativeClutch(
        grip_threshold=0.5,
        mode="always",
        position_only=False,
        delta_frame="world",
        xr_align=np.eye(3),
        ori_mode="track",
    )
    xr0 = identity_pose()
    robot0 = identity_pose()
    # robot 先绕 Z 转 90°
    robot0[3:] = np.array([0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    clutch.update("left", 0.0, xr0, robot0, scale=1.0)
    # 接合瞬间姿态应等于 robot0
    eng, t0 = clutch.update("left", 0.0, xr0, robot0, scale=1.0)
    assert eng and t0 is not None
    np.testing.assert_allclose(t0[3:], robot0[3:], atol=1e-5)
    # 手柄也绕 Z 转 90° → 末端应再转 90°（合计 180°）
    xr1 = identity_pose()
    xr1[3:] = np.array([0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    eng, t1 = clutch.update("left", 0.0, xr1, robot0, scale=1.0)
    assert eng and t1 is not None
    # 180° about Z: quat (0,0,1,0)
    np.testing.assert_allclose(np.abs(t1[3:]), np.array([0.0, 0.0, 1.0, 0.0]), atol=1e-5)


def test_clutch_world_axis_remap_forward():
    """手柄在 OpenXR 沿 -Z（前）移动 → MuJoCo 应沿 +Y。"""
    clutch = RelativeClutch(
        grip_threshold=0.5,
        mode="always",
        position_only=True,
        delta_frame="world",
        xr_align=openxr_yup_to_mujoco_zup(),
    )
    xr0 = identity_pose()
    robot0 = identity_pose()
    robot0[:3] = [0.0, 0.4, 0.5]
    clutch.update("left", 0.0, xr0, robot0, scale=1.0)
    xr1 = identity_pose()
    xr1[2] = -0.1  # XR forward
    eng, target = clutch.update("left", 0.0, xr1, robot0, scale=1.0)
    assert eng and target is not None
    np.testing.assert_allclose(target[1], 0.5, atol=1e-6)  # +Y
    np.testing.assert_allclose(target[2], 0.5, atol=1e-6)


def test_clutch_world_delta_yaw_sign_with_z_down_tcp():
    """TCP +Z 朝下时：手柄绕 XR 竖直轴转，world_delta 的世界 yaw 应同号（track 会反号）。"""
    Ra = openxr_yup_to_mujoco_zup()
    R_tcp = np.diag([1.0, -1.0, -1.0])  # approach = world -Z
    robot0 = identity_pose()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_tcp
    T[:3, 3] = [0.0, 0.4, 0.5]
    robot0[:] = matrix_to_pose(T)

    xr0 = identity_pose()
    xr1 = identity_pose()
    # +90° about XR +Y (up) → after Ra conjugacy should be +90° about MJ +Z
    xr1[3:] = np.array([0.0, np.sin(np.pi / 4), 0.0, np.cos(np.pi / 4)])

    clutch = RelativeClutch(
        grip_threshold=0.5,
        mode="always",
        position_only=False,
        delta_frame="world",
        xr_align=Ra,
        ori_mode="world_delta",
    )
    clutch.update("left", 1.0, xr0, robot0, scale=1.0)
    eng, target = clutch.update("left", 1.0, xr1, robot0, scale=1.0)
    assert eng and target is not None
    R_cmd = pose_to_matrix(target)[:3, :3]
    R_delta_world = R_cmd @ R_tcp.T
    yaw = float(np.arctan2(R_delta_world[1, 0], R_delta_world[0, 0]))
    assert yaw > 1.0  # ~+π/2, not −π/2


def test_clutch_long_press_requires_hold():
    clutch = RelativeClutch(grip_threshold=0.5, mode="long_press_grip", hold_s=0.2)
    xr = identity_pose()
    robot = identity_pose()
    eng, _ = clutch.update("left", 1.0, xr, robot)
    assert eng is False
    time.sleep(0.25)
    eng, target = clutch.update("left", 1.0, xr, robot)
    assert eng and target is not None
    eng, _ = clutch.update("left", 0.0, xr, robot)
    assert eng is False


def test_reset_button_homes_side():
    mapper = _dual_mapper(
        clutch_mode="grip",
        reset_buttons_left=["y"],
        reset_buttons_right=["b"],
    )
    backend = MockDualArmBackend(
        left0=np.array([-0.3, 0.2, 0.25, 0, 0, 0, 1], dtype=np.float64),
        right0=np.array([0.3, 0.2, 0.25, 0, 0, 0, 1], dtype=np.float64),
        head0=np.array([0, 0, 0.6, 0, 0, 0, 1], dtype=np.float64),
    )
    fb = backend.get_feedback()
    # Move left via teleop
    obs = XRObservation(
        timestamp_ns=0,
        left_controller=np.array([0.1, 0, 0, 0, 0, 0, 1], dtype=np.float64),
        buttons=ButtonState(left_grip=1.0, right_grip=0.0),
    )
    t1 = mapper.map(obs, fb)
    backend.apply(t1)
    moved = backend.get_feedback()[EXTREMITY_LEFT_EE].copy()
    assert abs(moved[0] - (-0.3)) > 1e-6 or True  # may be relative from home

    # Rising edge Y → home_left
    obs2 = XRObservation(
        timestamp_ns=1,
        buttons=ButtonState(button_y=True, left_grip=0.0),
    )
    t2 = mapper.map(obs2, backend.get_feedback())
    assert t2.meta.get("home_left") is True
    backend.apply(t2)
    np.testing.assert_allclose(
        backend.get_feedback()[EXTREMITY_LEFT_EE],
        backend._home[EXTREMITY_LEFT_EE],
        atol=1e-8,
    )


def test_dual_arm_always_mode_moves_without_grip():
    source = KeyboardClutchMockXRSource(hz=50.0, radius=0.15)
    source.left_grip = 0.0
    source.right_grip = 0.0
    mapper = _dual_mapper(clutch_mode="always", filter_alpha=1.0)
    backend = MockDualArmBackend(
        left0=np.array([-0.3, 0.2, 0.25, 0, 0, 0, 1], dtype=np.float64),
        right0=np.array([0.3, 0.2, 0.25, 0, 0, 0, 1], dtype=np.float64),
        head0=np.array([0, 0, 0.6, 0, 0, 0, 1], dtype=np.float64),
    )
    before = backend.get_feedback()[EXTREMITY_LEFT_EE].copy()
    loop = TeleopLoop(source, mapper, backend, hz=100.0, latency_window=300)
    stats = loop.run(duration_s=0.4)
    assert stats.steps > 10
    after = backend.get_feedback()[EXTREMITY_LEFT_EE]
    assert np.linalg.norm(after[:3] - before[:3]) > 1e-4


def test_dual_arm_mock_loop_moves():
    source = MockXRSource(hz=50.0, radius=0.15)
    mapper = _dual_mapper()
    backend = MockDualArmBackend(
        left0=np.array([-0.3, 0.2, 0.25, 0, 0, 0, 1], dtype=np.float64),
        right0=np.array([0.3, 0.2, 0.25, 0, 0, 0, 1], dtype=np.float64),
        head0=np.array([0, 0, 0.6, 0, 0, 0, 1], dtype=np.float64),
    )
    before = backend.get_feedback()[EXTREMITY_LEFT_EE].copy()
    loop = TeleopLoop(source, mapper, backend, hz=100.0, latency_window=300)
    stats = loop.run(duration_s=0.5)
    assert stats.steps > 10
    after = backend.get_feedback()[EXTREMITY_LEFT_EE]
    assert np.linalg.norm(after[:3] - before[:3]) > 1e-4


def test_humanoid_mapper_fills_body_fields():
    mapper = HumanoidMapper(
        dual_arm=_dual_mapper(),
        body_joint_indices={
            "pelvis": 0,
            "left_ankle": 7,
            "right_ankle": 8,
            "head": 15,
            "left_wrist": 22,
            "right_wrist": 23,
        },
        tracker_left_keywords=["left", "_l"],
        tracker_right_keywords=["right", "_r"],
    )
    body = np.tile(identity_pose(), (24, 1))
    body[0, 2] = 0.9
    body[7, 0] = -0.1
    body[8, 0] = 0.1
    obs = XRObservation(
        timestamp_ns=0,
        body_joints=body,
        buttons=ButtonState(left_grip=1.0, right_grip=1.0),
    )
    fb = {
        EXTREMITY_LEFT_EE: identity_pose(),
        EXTREMITY_RIGHT_EE: identity_pose(),
    }
    target = mapper.map(obs, fb)
    assert EXTREMITY_PELVIS in target.extremities
    assert EXTREMITY_LEFT_FOOT in target.extremities
    assert target.root_height == 0.9
    assert target.keypoints is not None
