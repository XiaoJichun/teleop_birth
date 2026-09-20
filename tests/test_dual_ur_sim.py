"""MuJoCo dual UR + EE + factory 测试（配置来自 YAML）。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

mujoco = pytest.importorskip("mujoco")

from teleop_birth.robot import G1BackendStub, MujocoDualURBackend
from teleop_birth.robot.ee import DexHandStub, EECommand, ParallelGripper
from teleop_birth.teleop.mapping import DualArmMapper
from teleop_birth.teleop.pipeline import TeleopLoop
from teleop_birth.teleop.schema import EXTREMITY_LEFT_EE, EXTREMITY_RIGHT_EE, SparseTarget, identity_pose
from teleop_birth.robot.arm.ur5e import build_dual_ur5e_2f85
from teleop_birth.teleop.xr import MockXRSource

ROOT = Path(__file__).resolve().parents[1]


def _load(rel: str) -> dict:
    with (ROOT / rel).open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_build_dual_model_from_yaml():
    scene = _load("configs/device/backend/mujoco_dual_ur.yaml")["scene"]
    menagerie = ROOT / "third_party" / "mujoco_menagerie"
    model, name_map = build_dual_ur5e_2f85(menagerie, scene)
    assert model.nu == 14
    assert "left_tcp_site" in name_map


def test_parallel_gripper_maps_openness():
    buf = {"v": 0.0}

    def set_ctrl(v):
        buf["v"] = v

    g = ParallelGripper(
        set_ctrl=set_ctrl,
        get_openness=lambda: buf["v"] / 255.0,
        ctrl_open=0.0,
        ctrl_closed=255.0,
    )
    g.apply(EECommand(gripper_openness=1.0))
    assert buf["v"] == 255.0


def test_dex_stub_accepts_scalar():
    hand = DexHandStub(n_fingers=5)
    hand.apply(EECommand(gripper_openness=0.7))
    assert hand.read().gripper_openness == 0.7
    assert len(hand.read().finger_positions) == 5


def test_mujoco_dual_ur_teleop_loop():
    backend_cfg = _load("configs/device/backend/mujoco_dual_ur.yaml")
    ik_cfg = _load("configs/algorithm/ik/dls.yaml")
    ee_cfg = _load("configs/device/end_effector/gripper.yaml")
    source = MockXRSource(hz=50.0, radius=0.15)
    mapper = DualArmMapper(scale=0.5, filter_alpha=1.0, map_head=True, grip_threshold=0.55)
    backend = MujocoDualURBackend(
        menagerie=ROOT / "third_party" / "mujoco_menagerie",
        scene_cfg=backend_cfg["scene"],
        ik_cfg=ik_cfg,
        ee_cfg=ee_cfg,
        render=False,
    )
    before = {"pose": None}

    def capture(obs, target, state):
        if before["pose"] is None:
            before["pose"] = state.extremities[EXTREMITY_LEFT_EE].copy()

    loop = TeleopLoop(source, mapper, backend, hz=40.0, latency_window=300, on_step=capture)
    stats = loop.run(duration_s=1.0, max_steps=40)
    assert stats.steps >= 20
    after = backend.get_feedback()[EXTREMITY_LEFT_EE]
    assert before["pose"] is not None
    assert np.linalg.norm(after[:3] - before["pose"][:3]) > 1e-3


def test_g1_stub_consumes_sparse_target():
    backend = G1BackendStub(mode="whole_body")
    backend.connect()
    pose = identity_pose()
    pose[0] = 0.2
    target = SparseTarget(
        timestamp_ns=0,
        extremities={EXTREMITY_LEFT_EE: pose, EXTREMITY_RIGHT_EE: identity_pose()},
        active={EXTREMITY_LEFT_EE: True, EXTREMITY_RIGHT_EE: False},
        left_gripper=0.3,
    )
    state = backend.apply(target)
    np.testing.assert_allclose(state.extremities[EXTREMITY_LEFT_EE][0], 0.2)
    backend.disconnect()
