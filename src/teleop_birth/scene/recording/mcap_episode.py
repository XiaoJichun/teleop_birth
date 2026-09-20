"""从 MCAP 解析 episode 轨迹张量（EE / action / 图像 / 元数据）。"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class EpisodeTensors:
    """解析后的轨迹；缺字段为空数组。"""

    timestamps_ns: np.ndarray = field(default_factory=lambda: np.zeros((0,), dtype=np.int64))
    ee_left: np.ndarray = field(default_factory=lambda: np.zeros((0, 7), dtype=np.float32))
    ee_right: np.ndarray = field(default_factory=lambda: np.zeros((0, 7), dtype=np.float32))
    action: np.ndarray = field(default_factory=lambda: np.zeros((0, 7), dtype=np.float32))
    gripper: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=np.float32))
    images: dict[str, list[np.ndarray]] = field(default_factory=dict)
    logs: list[dict[str, Any]] = field(default_factory=list)
    meta: dict[str, Any] = field(default_factory=dict)

    @property
    def n_steps(self) -> int:
        return int(max(len(self.ee_left), len(self.ee_right), len(self.action), len(self.logs)))


def _pose_from_proto(msg: Any) -> np.ndarray:
    p = msg.pose.position
    q = msg.pose.orientation
    return np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w], dtype=np.float32)


def parse_mcap_episode(mcap_path: Path) -> EpisodeTensors:
    """解码 Foxglove protobuf MCAP；无依赖或损坏时返回空张量 + meta.note。"""
    path = Path(mcap_path)
    out = EpisodeTensors(meta={"source_mcap": path.name})
    if not path.is_file():
        out.meta["error"] = "file_missing"
        return out

    try:
        from mcap.reader import make_reader
        from mcap_protobuf.decoder import DecoderFactory
    except ImportError:
        out.meta["error"] = "mcap_protobuf_missing"
        out.meta["note"] = "需要: uv sync --extra collect"
        return out

    ee_l: list[np.ndarray] = []
    ee_r: list[np.ndarray] = []
    acts: list[np.ndarray] = []
    grips: list[np.ndarray] = []
    ts: list[int] = []
    images: dict[str, list[np.ndarray]] = {}
    logs: list[dict[str, Any]] = []

    try:
        with path.open("rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for _schema, channel, message, proto in reader.iter_decoded_messages():
                topic = channel.topic
                t = int(message.log_time)
                name = type(proto).__name__
                if name == "PoseInFrame":
                    pose = _pose_from_proto(proto)
                    if topic.endswith("/left") or topic == "/robot/ee/left":
                        ee_l.append(pose)
                        ts.append(t)
                    elif topic.endswith("/right") or topic == "/robot/ee/right":
                        ee_r.append(pose)
                    elif "action" in topic or topic == "/action/target":
                        acts.append(pose)
                elif name == "CompressedImage":
                    cam = topic.strip("/").split("/")
                    # /cam/third_person/compressed → third_person
                    cam_name = cam[1] if len(cam) >= 2 and cam[0] == "cam" else topic
                    arr = _decode_jpeg(bytes(proto.data))
                    if arr is not None:
                        images.setdefault(cam_name, []).append(arr)
                elif name == "Log":
                    text = str(getattr(proto, "message", "") or "")
                    if text.startswith("RESULT "):
                        try:
                            out.meta["result"] = json.loads(text[7:])
                        except json.JSONDecodeError:
                            logs.append({"raw": text})
                    else:
                        try:
                            logs.append(json.loads(text))
                        except json.JSONDecodeError:
                            logs.append({"raw": text})
    except Exception as exc:  # noqa: BLE001
        out.meta["error"] = f"parse_failed: {exc}"
        return out

    # 从 log 抽 gripper
    for row in logs:
        g = row.get("gripper") if isinstance(row, dict) else None
        if isinstance(g, dict):
            grips.append(
                np.array(
                    [float(g.get("left", 0.0)), float(g.get("right", 0.0))],
                    dtype=np.float32,
                )
            )
        elif isinstance(g, (list, tuple)) and len(g) >= 2:
            grips.append(np.array([float(g[0]), float(g[1])], dtype=np.float32))

    out.timestamps_ns = np.asarray(ts, dtype=np.int64)
    out.ee_left = _stack(ee_l, 7)
    out.ee_right = _stack(ee_r, 7)
    out.action = _stack(acts, 7)
    out.gripper = _stack(grips, 2)
    out.images = images
    out.logs = logs
    out.meta["n_ee_left"] = int(len(ee_l))
    out.meta["n_ee_right"] = int(len(ee_r))
    out.meta["n_action"] = int(len(acts))
    out.meta["cameras"] = {k: len(v) for k, v in images.items()}
    return out


def _stack(rows: list[np.ndarray], dim: int) -> np.ndarray:
    if not rows:
        return np.zeros((0, dim), dtype=np.float32)
    return np.stack(rows, axis=0).astype(np.float32)


def _decode_jpeg(data: bytes) -> np.ndarray | None:
    if not data:
        return None
    try:
        from io import BytesIO

        from PIL import Image

        img = Image.open(BytesIO(data)).convert("RGB")
        return np.asarray(img, dtype=np.uint8)
    except Exception:
        return None


def observation_state(ep: EpisodeTensors) -> np.ndarray:
    """拼接左右 EE(7+7)=14 作为 observation.state。"""
    n = max(len(ep.ee_left), len(ep.ee_right), 1)
    left = ep.ee_left if len(ep.ee_left) else np.zeros((n, 7), dtype=np.float32)
    right = ep.ee_right if len(ep.ee_right) else np.zeros((n, 7), dtype=np.float32)
    n = min(len(left), len(right))
    return np.concatenate([left[:n], right[:n]], axis=-1)


def action_14(ep: EpisodeTensors) -> np.ndarray:
    """action 优先用 /action/target；否则用下一帧 EE 差分占位。"""
    if len(ep.action):
        # pad to 14 with zeros for missing arm
        a = ep.action
        if a.shape[-1] == 7:
            z = np.zeros((len(a), 7), dtype=np.float32)
            return np.concatenate([a, z], axis=-1)
        return a.astype(np.float32)
    state = observation_state(ep)
    if len(state) < 2:
        return np.zeros((0, 14), dtype=np.float32)
    # 下一帧状态作为伪 action
    return state[1:]
