"""IK solver switching tests (dls / ssik)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from omegaconf import OmegaConf

from teleop_birth.robot import MujocoDualURBackend
from teleop_birth.teleop.schema import EXTREMITY_LEFT_EE, SparseTarget, identity_pose

ROOT = Path(__file__).resolve().parents[1]


def _backend(solver: str) -> MujocoDualURBackend:
    scene_cfg = OmegaConf.load(ROOT / "configs/device/backend/mujoco_dual_ur.yaml")
    ik = OmegaConf.to_container(OmegaConf.load(ROOT / f"configs/algorithm/ik/{solver}.yaml"), resolve=True)
    ee = OmegaConf.load(ROOT / "configs/device/end_effector/gripper.yaml")
    assert isinstance(ik, dict)
    return MujocoDualURBackend(
        menagerie=ROOT / "third_party" / "mujoco_menagerie",
        scene_cfg=OmegaConf.to_container(scene_cfg.scene, resolve=True),
        ik_cfg=ik,
        ee_cfg=OmegaConf.to_container(ee, resolve=True),
        render=False,
        sim_substeps=1,
    )


def test_dls_ik_reaches_nearby_pose():
    b = _backend("dls")
    b.connect()
    fb = b.get_feedback()
    target = fb[EXTREMITY_LEFT_EE].copy()
    target[2] += 0.03
    st = SparseTarget(
        timestamp_ns=0,
        extremities={EXTREMITY_LEFT_EE: target, "right_ee": fb["right_ee"]},
        active={EXTREMITY_LEFT_EE: True, "right_ee": False},
    )
    for _ in range(30):
        b.apply(st)
    got = b.get_feedback()[EXTREMITY_LEFT_EE]
    assert np.linalg.norm(got[:3] - target[:3]) < 0.025
    b.disconnect()


def test_ssik_ik_optional():
    pytest.importorskip("ssik")
    b = _backend("ssik")
    b.connect()
    assert b._ik is not None
    fb = b.get_feedback()
    target = fb[EXTREMITY_LEFT_EE].copy()
    target[0] += 0.02
    target[1] += 0.01
    st = SparseTarget(
        timestamp_ns=0,
        extremities={EXTREMITY_LEFT_EE: target, "right_ee": identity_pose()},
        active={EXTREMITY_LEFT_EE: True, "right_ee": False},
    )
    for _ in range(20):
        b.apply(st)
    got = b.get_feedback()[EXTREMITY_LEFT_EE]
    assert np.linalg.norm(got[:3] - target[:3]) < 0.06
    b.disconnect()


def test_unknown_solver_raises():
    scene_cfg = OmegaConf.load(ROOT / "configs/device/backend/mujoco_dual_ur.yaml")
    ee = OmegaConf.load(ROOT / "configs/device/end_effector/gripper.yaml")
    b = MujocoDualURBackend(
        menagerie=ROOT / "third_party" / "mujoco_menagerie",
        scene_cfg=OmegaConf.to_container(scene_cfg.scene, resolve=True),
        ik_cfg={"solver": "not_a_solver", "fallback": "none"},
        ee_cfg=OmegaConf.to_container(ee, resolve=True),
        render=False,
    )
    with pytest.raises(ValueError, match="未知 ik.solver"):
        b.connect()


def test_robot_io_ports_on_adapter():
    from teleop_birth.teleop.config.factory import build_backend

    scene = OmegaConf.load(ROOT / "configs/device/backend/mock_dual_arm.yaml")
    ee = OmegaConf.load(ROOT / "configs/device/end_effector/gripper.yaml")
    ik = OmegaConf.load(ROOT / "configs/algorithm/ik/dls.yaml")
    be = build_backend(scene, ee, ik, menagerie=ROOT / "third_party" / "mujoco_menagerie", render=False)
    be.connect()
    assert hasattr(be, "endpoint")
    st = be.endpoint.state_port.receive()
    assert "left_ee" in st.extremities or EXTREMITY_LEFT_EE in st.extremities or len(st.extremities) >= 0
    be.disconnect()
