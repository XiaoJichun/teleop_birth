"""路径：自变量 ArtiXon Arm-6A 资产。"""

from __future__ import annotations

from pathlib import Path


def package_root() -> Path:
    # robot/arm/artixon → teleop_birth
    return Path(__file__).resolve().parents[3]


def project_root() -> Path:
    return package_root().parents[1]


def assets_dir() -> Path:
    return package_root() / "scene" / "assets" / "robots" / "arm" / "artixon_6a"


def arm_urdf_path() -> Path:
    return assets_dir() / "urdf" / "artixon_arm_6a_parallel_gripper.urdf"


def configs_dir() -> Path:
    return project_root() / "configs" / "robots"


def module_root() -> Path:
    """兼容旧名：指向仓库根（日志/输出基准）。"""
    return project_root()


def default_output_dir() -> Path:
    return project_root() / "logs" / "mcap" / "artixon"
