"""场景物品 / 布局 / MCAP 采集单测。"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
import pytest
from omegaconf import OmegaConf

from teleop_birth.scene_ops.placement.pose_controller import PropPoseController
from teleop_birth.scene_ops.placement.randomizer import LayoutRandomizer
from teleop_birth.scene_ops.props.catalog import PropCatalog
from teleop_birth.scene_ops.recording.mcap_writer import McapEpisodeWriter
from teleop_birth.scene_ops.tasks.pick_place import SemanticPickPlaceTask
from teleop_birth.sim.dual_ur5e_builder import build_dual_ur5e_2f85

ROOT = Path(__file__).resolve().parents[1]


def _scene_with_props(names: list[str]):
    backend = OmegaConf.to_container(
        OmegaConf.load(ROOT / "configs/device/backend/mujoco_dual_ur.yaml"),
        resolve=True,
    )
    paths = OmegaConf.to_container(
        OmegaConf.load(ROOT / "configs/device/paths/default.yaml"),
        resolve=True,
    )
    catalog = PropCatalog(ROOT / "configs/scene/props")
    scene = dict(backend["scene"])
    scene["props"] = [p.to_dict() for p in catalog.resolve_many(names)]
    scene["project_root"] = str(ROOT)
    menagerie = ROOT / str(paths["menagerie"])
    return scene, menagerie


def test_catalog_has_red_cube_and_blue_plate():
    cat = PropCatalog(ROOT / "configs/scene/props")
    assert "red_cube" in cat.list_names()
    assert "blue_plate" in cat.list_names()
    cube = cat.get("red_cube")
    plate = cat.get("blue_plate")
    assert cube.kind == "box"
    assert plate.kind == "plate"
    assert cube.rgba[0] > 0.5 and plate.rgba[2] > 0.5


def test_spawn_red_cube_and_blue_plate():
    scene, menagerie = _scene_with_props(["red_cube", "blue_plate"])
    if not (menagerie / "universal_robots_ur5e/ur5e.xml").is_file():
        pytest.skip("menagerie missing")
    model, name_map = build_dual_ur5e_2f85(menagerie, scene)
    assert set(name_map["prop_names"]) == {"red_cube", "blue_plate"}
    for name in ("red_cube", "blue_plate"):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        assert bid >= 0


def test_prop_pose_controller_moves_objects():
    scene, menagerie = _scene_with_props(["red_cube", "blue_plate"])
    if not (menagerie / "universal_robots_ur5e/ur5e.xml").is_file():
        pytest.skip("menagerie missing")
    model, _ = build_dual_ur5e_2f85(menagerie, scene)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ctrl = PropPoseController(model, data)
    ctrl.discover(["red_cube", "blue_plate"])
    ctrl.set_pose("red_cube", pos=[0.05, 0.40, 0.46])
    pose = ctrl.get_pose("red_cube")
    np.testing.assert_allclose(pose[:3], [0.05, 0.40, 0.46], atol=1e-6)


def test_layout_randomizer_separates_objects():
    cat = PropCatalog()
    props = cat.default_demo_props()
    rnd = LayoutRandomizer.from_table_cfg(
        {"size": [0.55, 0.35, 0.02], "pos_xy": [0.0, 0.4], "z_offset_from_base": -0.02},
        base_height=0.42,
        seed=42,
        arm="right",
    )
    layout = rnd.sample_layout(props)
    a = np.array(layout["red_cube"]["pos"][:2])
    b = np.array(layout["blue_plate"]["pos"][:2])
    assert np.linalg.norm(a - b) >= rnd.workspace.min_separation_m - 1e-6
    # 右臂工作区：x 偏正、y 不太贴基座
    for name in ("red_cube", "blue_plate"):
        xy = np.array(layout[name]["pos"][:2])
        assert xy[0] >= 0.05
        assert xy[1] >= 0.30
        assert rnd._in_reach(xy)


def test_mcap_writer_creates_foxglove_protobuf(tmp_path: Path):
    path = tmp_path / "ep.mcap"
    rgb = np.zeros((48, 64, 3), dtype=np.uint8)
    rgb[:, :] = (20, 120, 220)
    from io import BytesIO
    from PIL import Image

    buf = BytesIO()
    Image.fromarray(rgb, mode="RGB").save(buf, format="JPEG", quality=80)
    jpeg = buf.getvalue()

    with McapEpisodeWriter(path, task_info={"task": "demo"}) as w:
        w.log_step(
            step=0,
            stage="approach",
            ee_poses={"right": np.array([0.2, 0.3, 0.5, 0, 0, 0, 1], dtype=np.float64)},
            prop_poses={"red_cube": [0.1, 0.4, 0.45, 0, 0, 0, 1]},
            gripper={"right": 0.0},
            camera_jpegs={"third_person": jpeg, "wrist_right": jpeg},
        )
    assert path.is_file() and path.stat().st_size > 500
    from mcap.reader import make_reader

    topics = set()
    schemas = set()
    with path.open("rb") as fp:
        for schema, channel, message in make_reader(fp).iter_messages():
            topics.add(channel.topic)
            schemas.add(schema.name)
    assert "/cam/third_person/compressed" in topics
    assert "/cam/wrist_right/compressed" in topics
    assert "/tf" in topics
    assert "/robot/ee/right" in topics
    assert "foxglove.CompressedImage" in schemas
    assert "foxglove.PoseInFrame" in schemas
    assert "foxglove.FrameTransforms" in schemas


def test_semantic_task_waypoints():
    task = SemanticPickPlaceTask()
    spec = task.task_spec()
    assert "红色" in spec.instruction_zh
    assert spec.stages[0].action == "pick"
    assert spec.stages[1].action == "place"
