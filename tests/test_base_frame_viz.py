"""桌面接触 base / 转换后 URDF base 可视化。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_frames_from_dual_ur_scene():
    from teleop_birth.ui.session.base_frame_viz import frames_from_scene

    backend = yaml.safe_load(
        (ROOT / "configs/device/backend/mujoco_dual_ur.yaml").read_text(encoding="utf-8")
    )
    scene = dict(backend["scene"])
    frames = frames_from_scene(scene)
    assert len(frames) == 4  # L/R × contact/urdf
    kinds = {f.kind for f in frames}
    assert kinds == {"contact", "urdf"}
    contacts = [f for f in frames if f.kind == "contact"]
    urdfs = [f for f in frames if f.kind == "urdf"]
    # ur5e pedestal=0 → contact.z == urdf.z
    for c, u in zip(sorted(contacts, key=lambda x: x.origin[0]), sorted(urdfs, key=lambda x: x.origin[0])):
        assert abs(float(c.origin[2]) - float(u.origin[2])) < 1e-6
        assert abs(float(c.origin[0]) - float(u.origin[0])) < 1e-6


def test_frames_artixon_pedestal_lifts_urdf():
    from teleop_birth.ui.session.base_frame_viz import frames_from_scene

    scene = {
        "desktop_base": {
            "arm_id": "artixon_6a",
            "table_top_z": 0.42,
            "xy": [0.0, 0.0],
        },
        "left_arm_xy": [0.05, 0.16],
        "right_arm_xy": [0.05, -0.16],
    }
    frames = frames_from_scene(scene)
    assert len(frames) == 4
    for tag in ("L", "R"):
        c = next(f for f in frames if f.kind == "contact" and f.label.startswith(tag))
        u = next(f for f in frames if f.kind == "urdf" and f.label.startswith(tag))
        assert abs(float(u.origin[2]) - float(c.origin[2]) - 0.02) < 1e-6


def test_add_base_geoms_compile():
    from teleop_birth.ui.scene_builder.draft import SceneDraft
    from teleop_birth.ui.scene_builder.preview import PreviewSession

    draft = SceneDraft(
        mode="catalog",
        robot_id="ur5e",
        robot_title="UR5e",
        robot_category="arm",
        mount_kind="table",
        gripper_id="robotiq_2f85",
        desktop_base_arm="ur5e",
        table_top_z=0.42,
        base_xy=[0.0, 0.0],
    )
    ps = PreviewSession(ROOT, draft)
    assert ps.ensure_compiled()
    model = ps._model
    assert model is not None
    names = []
    import mujoco

    for gid in range(model.ngeom):
        n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        if n.startswith("viz_"):
            names.append(n)
    assert any("contact" in n for n in names)
    assert any("urdf" in n for n in names)
