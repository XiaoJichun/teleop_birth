"""场景无损换臂：接触点固定，仅 arm_id / 腕部 site 变。"""

from __future__ import annotations

import copy

from teleop_birth.scene.ops.swap_arm import (
    extract_contact,
    infer_robot_id,
    swap_scene_robot,
)
from teleop_birth.teleop.frame.desktop_base import get_desktop_base, mount_origin_z


def _sample_ur_scene() -> dict:
    return {
        "id": "demo_ur",
        "type": "dual_ur5e",
        "title": "双UR演示",
        "backend": {"type": "mujoco_dual_ur", "robot_id": "ur5e"},
        "desktop_base": {
            "arm_id": "ur5e",
            "table_top_z": 0.42,
            "xy": [0.05, -0.02],
            "yaw_rad": 0.1,
        },
        "props": ["red_cube", "blue_plate"],
        "prop_specs": [{"name": "red_cube", "kind": "box"}],
        "cameras": [
            {
                "name": "third_person",
                "type": "world",
                "role": "third_person",
                "camera_asset": "realsense_d455",
                "pos": [0.55, -0.75, 1.15],
                "lookat": [0.15, 0.35, 0.45],
            },
            {
                "name": "wrist_right",
                "type": "site",
                "role": "wrist",
                "camera_asset": "realsense_d405",
                "mount": "wrist_camera_bracket",
                "site": "R_g_pinch",
            },
            {
                "name": "wrist_left",
                "type": "site",
                "role": "wrist",
                "camera_asset": "realsense_d405",
                "mount": "wrist_camera_bracket",
                "site": "L_g_pinch",
            },
        ],
        "collect": {"policy": "pick_place", "task": "right_pick_red_cube_to_blue_plate"},
    }


def test_swap_preserves_contact_pose():
    src = _sample_ur_scene()
    before = extract_contact(src)
    result = swap_scene_robot(src, new_robot_id="artixon_arm_6a")
    after = extract_contact(result.scene)
    assert after.table_top_z == before.table_top_z
    assert after.xy == before.xy
    assert after.yaw_rad == before.yaw_rad
    assert result.scene["desktop_base"]["arm_id"] == "artixon_6a"
    assert result.scene["type"] == "logistics_x2arm"
    assert result.scene["props"] == ["red_cube", "blue_plate"]
    assert result.scene["prop_specs"][0]["name"] == "red_cube"


def test_swap_updates_wrist_sites():
    src = _sample_ur_scene()
    result = swap_scene_robot(src, new_robot_id="artixon_arm_6a")
    wrists = {
        c["name"]: c["site"]
        for c in result.scene["cameras"]
        if c.get("role") == "wrist"
    }
    assert wrists["wrist_right"] == "R_tcp_pinch"
    assert wrists["wrist_left"] == "L_tcp_pinch"
    # 第三人称位姿不变
    tp = next(c for c in result.scene["cameras"] if c["role"] == "third_person")
    assert tp["pos"] == [0.55, -0.75, 1.15]


def test_swap_mount_z_follows_pedestal_only():
    src = _sample_ur_scene()
    top = 0.42
    result = swap_scene_robot(src, new_robot_id="artixon_arm_6a")
    assert result.mount_z_old == mount_origin_z(top, get_desktop_base("ur5e"))
    assert result.mount_z_new == mount_origin_z(top, get_desktop_base("artixon_6a"))
    # URDF 原点抬升，接触点本身不变
    assert result.mount_z_new == result.mount_z_old + 0.02


def test_swap_roundtrip_back_to_ur():
    src = _sample_ur_scene()
    mid = swap_scene_robot(src, new_robot_id="artixon_arm_6a").scene
    back = swap_scene_robot(mid, new_robot_id="ur5e")
    assert back.scene["desktop_base"]["arm_id"] == "ur5e"
    assert back.scene["type"] == "dual_ur5e"
    wrists = {
        c["name"]: c["site"]
        for c in back.scene["cameras"]
        if c.get("role") == "wrist"
    }
    assert wrists["wrist_right"] == "R_g_pinch"
    assert extract_contact(back.scene) == extract_contact(src)


def test_infer_robot_id():
    assert infer_robot_id(_sample_ur_scene()) == "ur5e"
    art = copy.deepcopy(_sample_ur_scene())
    art["backend"] = {"type": "logistics_x2arm", "robot_id": "artixon_arm_6a"}
    art["desktop_base"]["arm_id"] = "artixon_6a"
    assert infer_robot_id(art) == "artixon_arm_6a"
