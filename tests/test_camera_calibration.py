"""采集前相机标定：内参/外参解析 → 写入数据集。"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from teleop_birth.scene.recording.calibration import (
    CameraCalibration,
    apply_calibration_to_config,
    build_calibration,
    load_calibration,
    save_calibration,
    validate_calibration,
)
from teleop_birth.scene.recording.camera import CameraViewSpec, MultiCameraConfig

ROOT = Path(__file__).resolve().parents[1]


def _sample_views() -> list[CameraViewSpec]:
    return [
        CameraViewSpec.from_dict(
            {
                "name": "third_person",
                "type": "world",
                "role": "third_person",
                "camera_asset": "realsense_d455",
                "pos": [0.05, -0.55, 1.2],
                "lookat": [0.18, 0.4, 0.45],
                "fovy_deg": 58.0,
                "intrinsics": {"fx": 390.0, "fy": 390.0, "cx": 320.0, "cy": 240.0},
                "T_parent_cam": [
                    [1, 0, 0, 0.05],
                    [0, 1, 0, -0.55],
                    [0, 0, 1, 1.2],
                    [0, 0, 0, 1],
                ],
            }
        ),
        CameraViewSpec.from_dict(
            {
                "name": "wrist_right",
                "type": "site",
                "role": "wrist",
                "camera_asset": "realsense_d405",
                "site": "R_g_pinch",
                "offset_xyz": [-0.06, 0.0, 0.05],
                "lookat_xyz": [0.1, 0.0, 0.0],
                "fovy_deg": 58.0,
                "intrinsics": {"fx": 430.0, "fy": 430.0, "cx": 320.0, "cy": 240.0},
                "T_parent_cam": [
                    [1, 0, 0, -0.06],
                    [0, 1, 0, 0.0],
                    [0, 0, 1, 0.05],
                    [0, 0, 0, 1],
                ],
            }
        ),
    ]


def test_build_and_validate_calibration():
    calib = build_calibration(views=_sample_views(), width=640, height=480)
    assert validate_calibration(calib) == []
    assert "third_person" in calib["cameras"]
    assert calib["cameras"]["wrist_right"]["intrinsics"]["fx"] == pytest.approx(430.0)
    assert calib["cameras"]["third_person"]["K"][0][0] == pytest.approx(390.0)


def test_save_load_and_apply(tmp_path: Path):
    cfg = MultiCameraConfig(views=_sample_views(), width=640, height=480)
    gate = CameraCalibration(ROOT)
    payload = gate.run(cfg, session_dir=tmp_path, override_rel=None)
    path = tmp_path / "calibration.json"
    assert path.is_file()
    loaded = load_calibration(path)
    assert loaded["cameras"]["wrist_right"]["extrinsics"]["T"][0][3] == pytest.approx(-0.06)
    cfg2 = apply_calibration_to_config(cfg, loaded)
    assert cfg2.views[1].intrinsics["fx"] == pytest.approx(430.0)


def test_override_merges_intrinsics(tmp_path: Path):
    ov = tmp_path / "ov.yaml"
    ov.write_text(
        "cameras:\n  wrist_right:\n    intrinsics: {fx: 500.0, fy: 501.0, cx: 310.0, cy: 230.0}\n",
        encoding="utf-8",
    )
    # point override via absolute path by writing under configs-like layout
    cfg = MultiCameraConfig(views=_sample_views())
    from teleop_birth.scene.recording.calibration import load_override_yaml, build_calibration

    override = load_override_yaml(ov)
    calib = build_calibration(views=cfg.views, override=override, source="test+override")
    assert calib["cameras"]["wrist_right"]["intrinsics"]["fx"] == pytest.approx(500.0)
    assert calib["cameras"]["third_person"]["intrinsics"]["fx"] == pytest.approx(390.0)


def test_mcap_writes_calibration_topic(tmp_path: Path):
    pytest.importorskip("mcap_protobuf")
    from mcap.reader import make_reader

    from teleop_birth.scene.recording.mcap_writer import McapEpisodeWriter

    calib = build_calibration(views=_sample_views())
    path = tmp_path / "ep.mcap"
    with McapEpisodeWriter(path, task_info={"task": "demo", "calibration": calib}) as w:
        w.write_result({"success": True})
    topics = set()
    with path.open("rb") as f:
        for schema, channel, message in make_reader(f).iter_messages():
            topics.add(channel.topic)
            if channel.topic == "/camera/calibration":
                # Log protobuf — just confirm topic exists
                assert message.data
    assert "/camera/calibration" in topics
    assert "/task/log" in topics
