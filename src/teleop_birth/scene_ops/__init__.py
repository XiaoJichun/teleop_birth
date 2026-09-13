"""场景物品 / 布局 / 语义采集（对齐 Genie Sim 的 asset→layout→autocollect 分层）。

模块:
  props/       — 物品库与程序化生成（方块、盘子、MJCF）
  placement/   — 同场景下物品位姿控制与随机布局
  tasks/       — 语义任务（pick / place stages）
  collection/  — 自动化采集循环
  recording/   — MCAP（Foxglove 可打开）
"""

from teleop_birth.scene_ops.props.catalog import PropCatalog, PropSpec
from teleop_birth.scene_ops.props.spawner import PropSpawner
from teleop_birth.scene_ops.placement.pose_controller import PropPoseController
from teleop_birth.scene_ops.placement.randomizer import LayoutRandomizer
from teleop_birth.scene_ops.tasks.pick_place import SemanticPickPlaceTask
from teleop_birth.scene_ops.collection.auto_collector import AutoCollector
from teleop_birth.scene_ops.recording.mcap_writer import McapEpisodeWriter

__all__ = [
    "PropCatalog",
    "PropSpec",
    "PropSpawner",
    "PropPoseController",
    "LayoutRandomizer",
    "SemanticPickPlaceTask",
    "AutoCollector",
    "McapEpisodeWriter",
]
