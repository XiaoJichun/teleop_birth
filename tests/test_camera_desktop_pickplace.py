"""相机插件：D405 腕部 / D455 第三人称 内参外参自动解析。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_camera_plugin_defaults_d405_d455():
    from teleop_birth.scene.plugins.camera import CameraPlugin

    plug = CameraPlugin()
    views = plug.resolve_views(
        ROOT,
        [
            {"name": "third_person", "type": "world"},
            {"name": "wrist_right", "type": "site", "site": "R_g_pinch"},
        ],
    )
    by_name = {v["name"]: v for v in views}
    assert by_name["third_person"]["camera_asset"] == "realsense_d455"
    assert by_name["wrist_right"]["camera_asset"] == "realsense_d405"
    assert by_name["third_person"]["intrinsics"]["fx"] == pytest.approx(390.0)
    assert by_name["wrist_right"]["intrinsics"]["fx"] == pytest.approx(430.0)
    T = np.asarray(by_name["wrist_right"]["T_parent_cam"])
    assert T.shape == (4, 4)
    # 支架默认有平移
    assert float(np.linalg.norm(T[:3, 3])) > 0.01


def test_compose_scene_resolves_cameras_and_desktop():
    from teleop_birth.scene.plugins.scene import compose_scene

    ctx = compose_scene(ROOT, ROOT / "configs" / "scenes" / "dual_ur_pick_place.yaml")
    cams = ctx.extras.get("cameras") or []
    assert len(cams) == 3
    assets = {c["name"]: c["camera_asset"] for c in cams}
    assert assets["third_person"] == "realsense_d455"
    assert assets["wrist_left"] == "realsense_d405"
    assert assets["wrist_right"] == "realsense_d405"
    assert ctx.extras.get("desktop_base") is not None
    assert ctx.name_map.get("desktop_base_arm") == "ur5e"


def test_multicamera_from_scene():
    import yaml

    from teleop_birth.scene.recording.camera import MultiCameraConfig

    scene = yaml.safe_load(
        (ROOT / "configs" / "scenes" / "dual_ur_pick_place.yaml").read_text(encoding="utf-8")
    )
    recording = yaml.safe_load(
        (ROOT / "configs" / "scene" / "recording.yaml").read_text(encoding="utf-8")
    )
    assert isinstance(scene, dict) and isinstance(recording, dict)
    cfg = MultiCameraConfig.from_scene(ROOT, scene, recording=recording)
    assert cfg.enabled
    assert len(cfg.views) == 3
    by = {v.name: v for v in cfg.views}
    assert by["third_person"].camera_asset == "realsense_d455"
    assert by["wrist_right"].intrinsics is not None
    assert by["wrist_right"].intrinsics["fx"] == pytest.approx(430.0)


def test_pick_place_task_name_matches_config():
    from teleop_birth.scene.tasks.pick_place import SemanticPickPlaceTask

    spec = SemanticPickPlaceTask().task_spec()
    assert spec.name == "right_pick_red_cube_to_blue_plate"
    assert "red_cube" in spec.objects
    assert "blue_plate" in spec.objects


def test_pick_place_waypoints_clear_table_and_plate():
    """抓取 TCP 接近自然 pinch 高度（可物理夹取），搬运悬停高于盘子。"""
    from teleop_birth.scene.tasks.pick_place import SemanticPickPlaceTask

    class _Props:
        def get_pose(self, name: str):
            if name == "red_cube":
                return np.array([0.15, 0.40, 0.445, 0, 0, 0, 1], dtype=np.float64)
            return np.array([0.20, 0.35, 0.438, 0, 0, 0, 1], dtype=np.float64)

    table_z = 0.42
    clearance = 0.010
    wps = SemanticPickPlaceTask().build_waypoints(
        _Props(),
        table_top_z=table_z,
        tcp_table_clearance=clearance,
        grasp_aim_up=0.012,
    )
    by = dict(zip(wps.labels, wps.poses))
    assert float(by["descend_open"][2]) >= table_z + clearance - 1e-9
    # 不得抬到指垫够不着方块（旧逻辑 0.47+ 会磁吸）
    assert float(by["descend_open"][2]) < 0.46
    assert "squeeze_hold" in by
    assert float(by["transport"][2]) >= table_z + 0.10
