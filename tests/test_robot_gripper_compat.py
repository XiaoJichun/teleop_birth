"""机器人–夹爪兼容过滤。"""

from __future__ import annotations

from pathlib import Path

from teleop_birth.robot import get_robot, grippers_compatible_with, list_robots
from teleop_birth.scene.plugins.gripper import GripperPlugin


def test_preview_robots_only_ready_with_grippers():
    preview = list_robots(preview_only=True)
    ids = {r.id for r in preview}
    assert "ur5e" in ids
    assert "artixon_arm_6a" in ids
    assert "franka_panda" not in ids
    for r in preview:
        assert r.compatible_grippers
        assert r.scene_preview is True


def test_franka_has_no_compatible_grippers():
    assert grippers_compatible_with("franka_panda") == ()
    assert get_robot("franka_panda").scene_preview is False


def test_gripper_plugin_filters_by_robot(tmp_path=None):
    root = Path(__file__).resolve().parents[1]
    ur = GripperPlugin().list_available(root, robot_id="ur5e")
    art = GripperPlugin().list_available(root, robot_id="artixon_arm_6a")
    frank = GripperPlugin().list_available(root, robot_id="franka_panda")
    assert [g["id"] for g in ur] == ["robotiq_2f85"]
    assert [g["id"] for g in art] == ["artixon_parallel"]
    assert frank == []
