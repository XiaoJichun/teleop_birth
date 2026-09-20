"""场景操作配置数据类（与 CLI TUI 解耦，供 scene_ops / plugins 共用）。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class GenerateConfig:
    props: list[str] = field(default_factory=lambda: ["red_cube", "blue_plate"])
    render: bool = False
    settle_steps: int = 100

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "GenerateConfig":
        return cls(
            props=list(data.get("props") or ["red_cube", "blue_plate"]),
            render=bool(data.get("render", False)),
            settle_steps=int(data.get("settle_steps", 100)),
        )


@dataclass
class PlaceConfig:
    props: list[str] = field(default_factory=lambda: ["red_cube", "blue_plate"])
    seed: int = 0
    repeats: int = 3
    render: bool = False

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "PlaceConfig":
        return cls(
            props=list(data.get("props") or ["red_cube", "blue_plate"]),
            seed=int(data.get("seed", 0)),
            repeats=int(data.get("repeats", 3)),
            render=bool(data.get("render", False)),
        )


@dataclass
class CollectConfig:
    task: str = "right_pick_red_cube_to_blue_plate"
    episodes: int = 3
    seed: int = 0
    arm: str = "right"
    render: bool = True
    pace_s: float = 0.02
    output_dir: str = "logs/mcap"
    steps_per_segment: int = 35
    hold_steps: int = 0
    sticky_grasp: bool = True
    camera_enabled: bool = True
    # 分辨率 / 采样间隔以 configs/scene/recording.yaml 为准；此处仅兼容旧字段
    camera_width: int = 640
    camera_height: int = 480
    camera_every_n_steps: int = 2
    recording: str = "scene/recording.yaml"
    scene: str = "scenes/dual_ur_pick_place.yaml"
    # 采集前先标定相机内参/外参，并写入 MCAP + session/calibration.json
    calibrate_before_collect: bool = True
    calibration_override: str = "scene/calibration/override.yaml"
    calibration_file: str = ""  # 若填写已有 JSON，则直接加载（跳过解析）
    # 会话向导传入的随机化区域（RandomizeRegion.to_dict）；空则用桌面默认
    randomize_region: dict | None = None
    randomize_props: list[str] = field(default_factory=list)
    dataset_format: str = "mcap"
    backend: str = "device/backend/mujoco_dual_ur.yaml"
    ik: str = "algorithm/ik/dls.yaml"
    end_effector: str = "device/end_effector/gripper.yaml"
    paths: str = "device/paths/default.yaml"

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "CollectConfig":
        cam = data.get("camera") if isinstance(data.get("camera"), dict) else {}
        return cls(
            task=str(data.get("task") or "right_pick_red_cube_to_blue_plate"),
            episodes=int(data.get("episodes", 3)),
            seed=int(data.get("seed", 0)),
            arm=str(data.get("arm") or "right"),
            render=bool(data.get("render", True)),
            pace_s=float(data.get("pace_s", 0.02)),
            output_dir=str(data.get("output_dir") or "logs/mcap"),
            steps_per_segment=int(data.get("steps_per_segment", 35)),
            hold_steps=int(data.get("hold_steps", 0)),
            sticky_grasp=bool(data.get("sticky_grasp", True)),
            camera_enabled=bool(cam.get("enabled", data.get("camera_enabled", True))),
            camera_width=int(cam.get("width", data.get("camera_width", 640))),
            camera_height=int(cam.get("height", data.get("camera_height", 480))),
            camera_every_n_steps=int(cam.get("every_n_steps", data.get("camera_every_n_steps", 2))),
            recording=str(data.get("recording") or "scene/recording.yaml"),
            scene=str(data.get("scene") or "scenes/dual_ur_pick_place.yaml"),
            calibrate_before_collect=bool(data.get("calibrate_before_collect", True)),
            calibration_override=str(
                data.get("calibration_override") or "scene/calibration/override.yaml"
            ),
            calibration_file=str(data.get("calibration_file") or ""),
            randomize_region=(
                dict(data["randomize_region"])
                if isinstance(data.get("randomize_region"), dict)
                else None
            ),
            randomize_props=list(data.get("randomize_props") or []),
            dataset_format=str(data.get("dataset_format") or "mcap"),
            backend=str(data.get("backend") or "device/backend/mujoco_dual_ur.yaml"),
            ik=str(data.get("ik") or "algorithm/ik/dls.yaml"),
            end_effector=str(data.get("end_effector") or "device/end_effector/gripper.yaml"),
            paths=str(data.get("paths") or "device/paths/default.yaml"),
        )
