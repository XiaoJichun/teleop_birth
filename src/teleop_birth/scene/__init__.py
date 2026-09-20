"""场景：物品 / 布局 / 采集 / MCAP，以及 plugins / assets。

  collection/ placement/ props/ tasks/ recording/
  plugins/   scene camera gripper sensor physics collect
  assets/    catalog + 本地 URDF/相机
"""

from teleop_birth.scene.config import CollectConfig, GenerateConfig, PlaceConfig
from teleop_birth.scene.props.catalog import PropCatalog, PropSpec
from teleop_birth.scene.props.spawner import PropSpawner
from teleop_birth.scene.placement.pose_controller import PropPoseController
from teleop_birth.scene.placement.randomizer import LayoutRandomizer
from teleop_birth.scene.tasks.pick_place import SemanticPickPlaceTask
from teleop_birth.scene.recording.mcap_writer import McapEpisodeWriter

# AutoCollector / SceneOpsRunner 依赖 embodiment，按需从子模块导入，避免包级循环

__all__ = [
    "PropCatalog",
    "PropSpec",
    "PropSpawner",
    "PropPoseController",
    "LayoutRandomizer",
    "SemanticPickPlaceTask",
    "McapEpisodeWriter",
    "CollectConfig",
    "GenerateConfig",
    "PlaceConfig",
    "AutoCollector",
    "SceneOpsRunner",
]


def __getattr__(name: str):
    if name == "AutoCollector":
        from teleop_birth.scene.collection.auto_collector import AutoCollector

        return AutoCollector
    if name == "SceneOpsRunner":
        from teleop_birth.scene.runner import SceneOpsRunner

        return SceneOpsRunner
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
