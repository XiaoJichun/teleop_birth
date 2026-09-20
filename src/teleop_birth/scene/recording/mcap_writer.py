"""Foxglove 兼容 MCAP（protobuf）；topic 名由 configs/scene/recording.yaml 注入。"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

_DEPS: dict | None = None


def _deps() -> dict:
    """采集写 MCAP 时再导入。遥操菜单不应因为缺 collect extra 而启动失败。"""
    global _DEPS
    if _DEPS is not None:
        return _DEPS
    try:
        from foxglove_schemas_protobuf.CompressedImage_pb2 import CompressedImage
        from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
        from foxglove_schemas_protobuf.FrameTransforms_pb2 import FrameTransforms
        from foxglove_schemas_protobuf.Log_pb2 import Log
        from foxglove_schemas_protobuf.PoseInFrame_pb2 import PoseInFrame
        from foxglove_schemas_protobuf.Pose_pb2 import Pose
        from foxglove_schemas_protobuf.Quaternion_pb2 import Quaternion
        from foxglove_schemas_protobuf.Vector3_pb2 import Vector3
        from google.protobuf.timestamp_pb2 import Timestamp
        from mcap_protobuf.writer import Writer as ProtobufMcapWriter
    except ImportError as exc:  # pragma: no cover
        raise ImportError(
            "需要 Foxglove MCAP 依赖：uv sync --extra collect\n"
            "（mcap-protobuf-support / foxglove-schemas-protobuf / Pillow）"
        ) from exc
    _DEPS = {
        "CompressedImage": CompressedImage,
        "FrameTransform": FrameTransform,
        "FrameTransforms": FrameTransforms,
        "Log": Log,
        "PoseInFrame": PoseInFrame,
        "Pose": Pose,
        "Quaternion": Quaternion,
        "Vector3": Vector3,
        "Timestamp": Timestamp,
        "ProtobufMcapWriter": ProtobufMcapWriter,
    }
    return _DEPS


@dataclass
class TopicMap:
    tf: str = "/tf"
    log: str = "/task/log"
    calibration: str = "/camera/calibration"
    ee_left: str = "/robot/ee/left"
    ee_right: str = "/robot/ee/right"
    action_target: str = "/action/target"
    props_prefix: str = "/props/"
    cameras: dict[str, str] = field(
        default_factory=lambda: {
            "third_person": "/cam/third_person/compressed",
            "wrist_left": "/cam/wrist_left/compressed",
            "wrist_right": "/cam/wrist_right/compressed",
        }
    )

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "TopicMap":
        d = dict(data or {})
        cams = dict(d.get("cameras") or {})
        return cls(
            tf=str(d.get("tf") or "/tf"),
            log=str(d.get("log") or "/task/log"),
            calibration=str(d.get("calibration") or "/camera/calibration"),
            ee_left=str(d.get("ee_left") or "/robot/ee/left"),
            ee_right=str(d.get("ee_right") or "/robot/ee/right"),
            action_target=str(d.get("action_target") or "/action/target"),
            props_prefix=str(d.get("props_prefix") or "/props/"),
            cameras={
                "third_person": str(cams.get("third_person") or "/cam/third_person/compressed"),
                "wrist_left": str(cams.get("wrist_left") or "/cam/wrist_left/compressed"),
                "wrist_right": str(cams.get("wrist_right") or "/cam/wrist_right/compressed"),
                **{k: str(v) for k, v in cams.items() if k not in ("third_person", "wrist_left", "wrist_right")},
            },
        )

    def prop_topic(self, name: str) -> str:
        return f"{self.props_prefix}{name}"

    def camera_topic(self, name: str) -> str:
        return self.cameras.get(name) or f"/cam/{name}/compressed"


def _ts(ns: int):
    Timestamp = _deps()["Timestamp"]
    return Timestamp(seconds=int(ns // 1_000_000_000), nanos=int(ns % 1_000_000_000))


def _pose_msg(pose_xyzw: np.ndarray | list[float]):
    d = _deps()
    p = np.asarray(pose_xyzw, dtype=np.float64).reshape(7)
    return d["Pose"](
        position=d["Vector3"](x=float(p[0]), y=float(p[1]), z=float(p[2])),
        orientation=d["Quaternion"](x=float(p[3]), y=float(p[4]), z=float(p[5]), w=float(p[6])),
    )


def _frame_tf(ns: int, parent: str, child: str, pose_xyzw: np.ndarray | list[float]):
    d = _deps()
    p = np.asarray(pose_xyzw, dtype=np.float64).reshape(7)
    return d["FrameTransform"](
        timestamp=_ts(ns),
        parent_frame_id=parent,
        child_frame_id=child,
        translation=d["Vector3"](x=float(p[0]), y=float(p[1]), z=float(p[2])),
        rotation=d["Quaternion"](x=float(p[3]), y=float(p[4]), z=float(p[5]), w=float(p[6])),
    )


class McapEpisodeWriter:
    """写入 Foxglove Studio 可直接打开的 protobuf MCAP。"""

    def __init__(
        self,
        path: str | Path,
        *,
        task_info: dict[str, Any] | None = None,
        topics: TopicMap | None = None,
        dt_ns: int = 20_000_000,
    ) -> None:
        deps = _deps()
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = self.path.open("wb")
        self._writer = deps["ProtobufMcapWriter"](self._fp)
        self.topics = topics or TopicMap()
        self.dt_ns = int(dt_ns)
        self._sim_ns = 0
        self.count = 0
        self.task_info = task_info or {}
        self._write_log(0, "INFO", json.dumps(self.task_info, ensure_ascii=False))
        # 单独话题写入标定，便于数据集工具直接订阅
        calib = self.task_info.get("calibration")
        if isinstance(calib, dict) and calib:
            self.write_calibration(calib, log_time_ns=0)

    def write_calibration(
        self,
        calib: dict[str, Any],
        *,
        log_time_ns: int = 0,
    ) -> None:
        """写入 /camera/calibration（Foxglove Log JSON，含全部机位 K + T）。"""
        Log = _deps()["Log"]
        ns = int(log_time_ns)
        payload = json.dumps({"type": "camera_calibration", **calib}, ensure_ascii=False)
        msg = Log(timestamp=_ts(ns), level=Log.INFO, name="camera_calibration", message=payload)
        self._writer.write_message(
            topic=self.topics.calibration,
            message=msg,
            log_time=ns,
            publish_time=ns,
        )

    def now_ns(self) -> int:
        return int(self._sim_ns)

    def _write_log(self, ns: int, level: str, message: str) -> None:
        Log = _deps()["Log"]
        lvl = {
            "DEBUG": Log.DEBUG,
            "INFO": Log.INFO,
            "WARNING": Log.WARNING,
            "ERROR": Log.ERROR,
            "FATAL": Log.FATAL,
        }.get(level.upper(), Log.INFO)
        msg = Log(timestamp=_ts(ns), level=lvl, name="teleop_birth", message=message)
        self._writer.write_message(
            topic=self.topics.log,
            message=msg,
            log_time=ns,
            publish_time=ns,
        )

    def write_image_jpeg(
        self,
        jpeg_bytes: bytes,
        *,
        camera_name: str,
        frame_id: str | None = None,
        log_time_ns: int | None = None,
    ) -> None:
        ns = int(self._sim_ns if log_time_ns is None else log_time_ns)
        topic = self.topics.camera_topic(camera_name)
        msg = _deps()["CompressedImage"](
            timestamp=_ts(ns),
            frame_id=frame_id or f"cam_{camera_name}",
            format="jpeg",
            data=jpeg_bytes,
        )
        self._writer.write_message(topic=topic, message=msg, log_time=ns, publish_time=ns)

    def log_step(
        self,
        *,
        step: int,
        stage: str,
        ee_poses: dict[str, np.ndarray],
        prop_poses: dict[str, list[float] | np.ndarray],
        target_pose: np.ndarray | None = None,
        gripper: dict[str, float] | None = None,
        extra: dict[str, Any] | None = None,
        camera_jpegs: dict[str, bytes] | None = None,
        log_time_ns: int | None = None,
    ) -> None:
        if log_time_ns is None:
            ns = int(self._sim_ns)
        else:
            ns = int(log_time_ns)
            self._sim_ns = ns

        meta = {
            "step": int(step),
            "stage": stage,
            "gripper": gripper or {},
            "extra": extra or {},
        }
        self._write_log(ns, "INFO", json.dumps(meta, ensure_ascii=False))

        transforms = []
        ee_topic = {"left": self.topics.ee_left, "right": self.topics.ee_right}
        for name, pose in ee_poses.items():
            topic = ee_topic.get(name) or f"/robot/ee/{name}"
            self._writer.write_message(
                topic=topic,
                message=_deps()["PoseInFrame"](timestamp=_ts(ns), frame_id="world", pose=_pose_msg(pose)),
                log_time=ns,
                publish_time=ns,
            )
            transforms.append(_frame_tf(ns, "world", f"ee_{name}", pose))

        for name, pose in prop_poses.items():
            self._writer.write_message(
                topic=self.topics.prop_topic(name),
                message=_deps()["PoseInFrame"](timestamp=_ts(ns), frame_id="world", pose=_pose_msg(pose)),
                log_time=ns,
                publish_time=ns,
            )
            transforms.append(_frame_tf(ns, "world", f"prop_{name}", pose))

        if transforms:
            self._writer.write_message(
                topic=self.topics.tf,
                message=_deps()["FrameTransforms"](transforms=transforms),
                log_time=ns,
                publish_time=ns,
            )

        if target_pose is not None:
            self._writer.write_message(
                topic=self.topics.action_target,
                message=_deps()["PoseInFrame"](
                    timestamp=_ts(ns), frame_id="world", pose=_pose_msg(target_pose)
                ),
                log_time=ns,
                publish_time=ns,
            )

        for cam_name, jpeg in (camera_jpegs or {}).items():
            self.write_image_jpeg(jpeg, camera_name=cam_name, log_time_ns=ns)

        self.count += 1
        self._sim_ns = ns + self.dt_ns

    def write_result(self, payload: dict[str, Any]) -> None:
        self._write_log(self._sim_ns, "INFO", "RESULT " + json.dumps(payload, ensure_ascii=False))

    def close(self) -> None:
        self._writer.finish()
        self._fp.close()

    def __enter__(self) -> "McapEpisodeWriter":
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()
