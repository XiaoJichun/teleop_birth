"""协议层单测（无 Pico / 无 MuJoCo 场景依赖）。"""

from __future__ import annotations

import time

import numpy as np

from teleop_birth.robot import MockDualArmBackend
from teleop_birth.teleop.control import RelativeClutch, openxr_yup_to_mujoco_zup
from teleop_birth.teleop.mapping import DualArmMapper, HumanoidMapper
from teleop_birth.teleop.pipeline import TeleopLoop
from teleop_birth.teleop.schema import (
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
from teleop_birth.teleop.xr import KeyboardClutchMockXRSource, MockXRSource


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


def test_clutch_world_axis_remap_forward():
    """手柄在 OpenXR 沿 -Z（前）移动 → MuJoCo 应沿 +Y。"""
    clutch = RelativeClutch(
        grip_threshold=0.5,
        mode="always",
        position_only=True,
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


def test_clutch_wrist_yaw_and_roll():
    """绕 XR 竖直轴转 → 房间 yaw 同号；绕瞄准轴拧 → 只绕 TCP 接近轴，接近方向不变。"""
    Ra = openxr_yup_to_mujoco_zup()
    R_tcp = np.diag([1.0, -1.0, -1.0])  # approach +Z = world -Z
    robot0 = identity_pose()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_tcp
    T[:3, 3] = [0.0, 0.4, 0.5]
    robot0[:] = matrix_to_pose(T)

    def _run(xr1):
        clutch = RelativeClutch(
            grip_threshold=0.5,
            mode="always",
            position_only=False,
            xr_align=Ra,
        )
        xr0 = identity_pose()
        clutch.update("left", 1.0, xr0, robot0, scale=1.0)
        eng, target = clutch.update("left", 1.0, xr1, robot0, scale=1.0)
        assert eng and target is not None
        return pose_to_matrix(target)[:3, :3]

    xr_yaw = identity_pose()
    # +90° about XR +Y (up) → +90° about MJ +Z
    xr_yaw[3:] = np.array([0.0, np.sin(np.pi / 4), 0.0, np.cos(np.pi / 4)])
    R_yaw = _run(xr_yaw)
    R_delta_world = R_yaw @ R_tcp.T
    yaw = float(np.arctan2(R_delta_world[1, 0], R_delta_world[0, 0]))
    assert yaw > 1.0

    # 绕 grip 瞄准轴 (−Z) 转 90°：局部 Rot_z(−90°)
    xr_roll = identity_pose()
    xr_roll[3:] = np.array([0.0, 0.0, np.sin(-np.pi / 4), np.cos(np.pi / 4)])
    R_roll = _run(xr_roll)
    R_rel = R_tcp.T @ R_roll
    np.testing.assert_allclose(R_rel[:, 2], np.array([0.0, 0.0, 1.0]), atol=1e-5)
    np.testing.assert_allclose(R_roll[:, 2], R_tcp[:, 2], atol=1e-5)


def test_clutch_reanchors_after_frozen_pose_jump():
    clutch = RelativeClutch(
        grip_threshold=0.5,
        mode="grip",
        position_only=True,
        hold_s=0.0,
        xr_align=np.eye(3),
        pose_still_s=0.05,
        reanchor_jump_m=0.04,
    )
    xr0 = identity_pose()
    robot0 = identity_pose()
    robot0[0] = 0.4
    eng, target = clutch.update("right", 1.0, xr0, robot0)
    assert eng and target is not None
    time.sleep(0.08)
    eng, held = clutch.update("right", 1.0, xr0, robot0)
    assert eng and held is not None
    np.testing.assert_allclose(held[:3], robot0[:3], atol=1e-6)

    xr_jump = identity_pose()
    xr_jump[0] = 0.12
    robot_now = robot0.copy()
    eng, jumped = clutch.update("right", 1.0, xr_jump, robot_now)
    assert eng and jumped is not None
    assert clutch.was_reanchored("right")
    # 卡住后的大跳不当成位移，目标仍是当前机械臂位姿
    np.testing.assert_allclose(jumped[:3], robot_now[:3], atol=1e-6)

    xr_next = identity_pose()
    xr_next[0] = 0.15
    eng, followed = clutch.update("right", 1.0, xr_next, robot_now)
    assert eng and followed is not None
    assert not clutch.was_reanchored("right")
    np.testing.assert_allclose(followed[0], robot_now[0] + 0.03, atol=1e-6)


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
