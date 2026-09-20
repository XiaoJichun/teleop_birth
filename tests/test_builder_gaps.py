"""夹爪映射 / 人形预览 / MCAP 张量导出。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from teleop_birth.ui.scene_builder.draft import SceneDraft
from teleop_birth.ui.scene_builder.gripper_util import apply_gripper_to_scene_cfg
from teleop_birth.ui.scene_builder.preview import _compile_empty_humanoid


ROOT = Path(__file__).resolve().parents[1]


def test_apply_robotiq_gripper_xml():
    scene: dict = {
        "gripper_xml": "robotiq_2f85/2f85.xml",
        "gripper_attach_site": "attachment_site",
    }
    menagerie = ROOT / "third_party" / "mujoco_menagerie"
    apply_gripper_to_scene_cfg(ROOT, scene, "robotiq_2f85", menagerie=menagerie)
    assert scene.get("_gripper_applied") == "robotiq_2f85"
    assert "robotiq" in str(scene["gripper_xml"])
    scene.pop("_gripper_applied", None)


def test_builtin_gripper_warns():
    scene: dict = {"gripper_xml": "robotiq_2f85/2f85.xml"}
    apply_gripper_to_scene_cfg(ROOT, scene, "artixon_parallel")
    assert "builtin" in str(scene.get("_gripper_warn") or "").lower() or "内置" in str(
        scene.get("_gripper_warn") or ""
    )


def test_empty_humanoid_preview_compiles():
    draft = SceneDraft(
        mode="semantic",
        robot_id="unitree_g1",
        robot_title="G1",
        robot_category="biped",
        mount_kind="empty",
    )
    draft.props = [
        {
            "name": "cube_h",
            "kind": "box",
            "size": [0.02, 0.02, 0.02],
            "rgba": [1, 0, 0, 1],
            "friction": [1, 0.005, 0.0001],
            "pos": [0.2, 0.0, 0.05],
            "quat_wxyz": [1, 0, 0, 0],
            "mass": 0.05,
            "freejoint": True,
        }
    ]
    model = _compile_empty_humanoid(ROOT, draft)
    assert model.nq > 0


def test_parse_and_export_real_mcap(tmp_path: Path):
    pytest.importorskip("mcap_protobuf")
    from teleop_birth.scene.recording.export_formats import export_hdf5, export_lerobot
    from teleop_birth.scene.recording.mcap_episode import parse_mcap_episode
    from teleop_birth.scene.recording.mcap_writer import McapEpisodeWriter

    path = tmp_path / "ep0.mcap"
    with McapEpisodeWriter(path, task_info={"task": "t"}) as w:
        for i in range(3):
            w.log_step(
                step=i,
                stage="pick",
                ee_poses={
                    "left": np.array([0.1, 0.0, 0.5, 0, 0, 0, 1], dtype=np.float64),
                    "right": np.array([0.2, 0.0, 0.5, 0, 0, 0, 1], dtype=np.float64),
                },
                prop_poses={},
                target_pose=np.array([0.2, 0.0, 0.5, 0, 0, 0, 1], dtype=np.float64),
                gripper={"left": 0.0, "right": 0.5},
            )

    ep = parse_mcap_episode(path)
    assert ep.meta.get("source_mcap") == "ep0.mcap"
    assert len(ep.ee_left) + len(ep.ee_right) >= 1

    lr = export_lerobot(path, version="3.0", meta={"task": "t", "steps": 3})
    assert (lr / "meta" / "info.json").is_file()
    assert (lr / "data" / "observation_state.npy").is_file() or (
        lr / "data" / "ee_right.npy"
    ).is_file()

    h5py = pytest.importorskip("h5py")
    h5 = export_hdf5(path, meta={"episode": 0})
    assert h5.is_file()
    with h5py.File(h5, "r") as f:
        assert f.attrs["format"] == "teleop_birth_hdf5_v2"
        assert "mcap_bytes" in f
