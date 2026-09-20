"""ROS 数据通讯类：发布观测（相机 + 关节/末端），订阅动作（关节/末端）。

transport=ros 时用 rclpy；未安装或初始化失败则回退 mock（进程内队列，便于单测/无 ROS 环境）。
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path
from threading import Lock
from typing import Any, Literal, Mapping, Sequence

import numpy as np
import yaml

StateMode = Literal["joint", "ee"]
ActionMode = Literal["joint", "ee"]
Transport = Literal["ros", "mock"]


@dataclass
class CommsConfig:
    enabled: bool = True
    transport: Transport = "ros"
    # cameras
    camera_names: list[str] = field(default_factory=list)  # empty = all
    camera_topic_prefix: str = "/teleop_birth/cam"
    camera_encoding: str = "rgb8"
    # state pub
    state_mode: StateMode = "ee"
    joint_topic: str = "/teleop_birth/robot/joint_states"
    ee_left_topic: str = "/teleop_birth/robot/ee_left"
    ee_right_topic: str = "/teleop_birth/robot/ee_right"
    gripper_topic: str = "/teleop_birth/robot/grippers"
    publish_rate_hz: float = 20.0
    # action sub
    action_mode: ActionMode = "ee"
    joint_cmd_topic: str = "/teleop_birth/robot/joint_cmd"
    ee_cmd_left_topic: str = "/teleop_birth/robot/ee_cmd_left"
    ee_cmd_right_topic: str = "/teleop_birth/robot/ee_cmd_right"
    gripper_cmd_topic: str = "/teleop_birth/robot/gripper_cmd"
    on_idle: str = "hold"
    node_name: str = "teleop_birth_comms"
    frame_id: str = "world"

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "CommsConfig":
        pub = dict(data.get("publish") or {})
        sub = dict(data.get("subscribe") or {})
        cams = dict(pub.get("cameras") or {})
        state = dict(pub.get("state") or {})
        return cls(
            enabled=bool(data.get("enabled", True)),
            transport=str(data.get("transport") or "ros").lower(),  # type: ignore[arg-type]
            camera_names=[str(x) for x in (cams.get("names") or [])],
            camera_topic_prefix=str(cams.get("topic_prefix") or "/teleop_birth/cam"),
            camera_encoding=str(cams.get("encoding") or "rgb8"),
            state_mode=str(state.get("mode") or "ee").lower(),  # type: ignore[arg-type]
            joint_topic=str(state.get("joint_topic") or "/teleop_birth/robot/joint_states"),
            ee_left_topic=str(state.get("ee_left_topic") or "/teleop_birth/robot/ee_left"),
            ee_right_topic=str(state.get("ee_right_topic") or "/teleop_birth/robot/ee_right"),
            gripper_topic=str(state.get("gripper_topic") or "/teleop_birth/robot/grippers"),
            publish_rate_hz=float(pub.get("rate_hz") or 20.0),
            action_mode=str(sub.get("mode") or "ee").lower(),  # type: ignore[arg-type]
            joint_cmd_topic=str(sub.get("joint_topic") or "/teleop_birth/robot/joint_cmd"),
            ee_cmd_left_topic=str(sub.get("ee_left_topic") or "/teleop_birth/robot/ee_cmd_left"),
            ee_cmd_right_topic=str(sub.get("ee_right_topic") or "/teleop_birth/robot/ee_cmd_right"),
            gripper_cmd_topic=str(sub.get("gripper_topic") or "/teleop_birth/robot/gripper_cmd"),
            on_idle=str(sub.get("on_idle") or "hold"),
            node_name=str(data.get("node_name") or "teleop_birth_comms"),
            frame_id=str(data.get("frame_id") or "world"),
        )


def load_comms_config(
    project_root: Path,
    *,
    rel: str = "plugins/comms/ros_bridge.yaml",
    overrides: Mapping[str, Any] | None = None,
) -> CommsConfig:
    path = Path(project_root) / "configs" / rel
    raw: dict[str, Any] = {}
    if path.is_file():
        loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if isinstance(loaded, dict):
            raw = loaded
    if overrides:
        raw = _deep_merge(raw, dict(overrides))
    return CommsConfig.from_dict(raw)


def _deep_merge(base: dict[str, Any], over: dict[str, Any]) -> dict[str, Any]:
    out = deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)  # type: ignore[arg-type]
        else:
            out[k] = deepcopy(v)
    return out


@dataclass
class ObservationBundle:
    """一帧观测：相机 RGB + 机器人状态。"""

    timestamp_ns: int = 0
    cameras: dict[str, np.ndarray] = field(default_factory=dict)  # name → HxWx3 uint8
    joint_names: list[str] = field(default_factory=list)
    joint_positions: np.ndarray | None = None
    ee_left: np.ndarray | None = None   # xyz+quat xyzw
    ee_right: np.ndarray | None = None
    left_gripper: float = 0.0
    right_gripper: float = 0.0


@dataclass
class ActionCommand:
    """一帧动作指令。"""

    timestamp_ns: int = 0
    mode: ActionMode = "ee"
    joint_names: list[str] = field(default_factory=list)
    joint_positions: np.ndarray | None = None
    ee_left: np.ndarray | None = None
    ee_right: np.ndarray | None = None
    left_active: bool = False
    right_active: bool = False
    left_gripper: float | None = None
    right_gripper: float | None = None


class RosDataComms:
    """数据通讯类：pub 观测 / sub 动作。

    用法::

        comms = RosDataComms(cfg)
        comms.start()
        comms.publish_observation(obs)
        cmd = comms.poll_action()
        comms.spin_once()
        comms.stop()
    """

    def __init__(self, cfg: CommsConfig) -> None:
        self.cfg = cfg
        self._impl: _TransportBase | None = None
        self._started = False

    @property
    def transport(self) -> str:
        return self._impl.kind if self._impl else self.cfg.transport

    def start(self) -> None:
        if self._started:
            return
        want = self.cfg.transport
        if want == "ros":
            try:
                self._impl = _RosTransport(self.cfg)
                self._impl.start()
            except Exception as exc:  # noqa: BLE001
                print(f"[comms] ROS 不可用（{exc}），回退 mock 传输")
                self._impl = _MockTransport(self.cfg)
                self._impl.start()
        else:
            self._impl = _MockTransport(self.cfg)
            self._impl.start()
        self._started = True
        print(
            f"[comms] 已启动 transport={self.transport}  "
            f"state={self.cfg.state_mode} action={self.cfg.action_mode}"
        )

    def stop(self) -> None:
        if self._impl is not None:
            self._impl.stop()
            self._impl = None
        self._started = False

    def spin_once(self) -> None:
        if self._impl is not None:
            self._impl.spin_once()

    def publish_observation(self, obs: ObservationBundle) -> None:
        if self._impl is None:
            return
        cams = obs.cameras
        if self.cfg.camera_names:
            allow = set(self.cfg.camera_names)
            cams = {k: v for k, v in cams.items() if k in allow}
        filtered = ObservationBundle(
            timestamp_ns=obs.timestamp_ns,
            cameras=cams,
            joint_names=list(obs.joint_names),
            joint_positions=None if obs.joint_positions is None else np.asarray(obs.joint_positions),
            ee_left=None if obs.ee_left is None else np.asarray(obs.ee_left),
            ee_right=None if obs.ee_right is None else np.asarray(obs.ee_right),
            left_gripper=float(obs.left_gripper),
            right_gripper=float(obs.right_gripper),
        )
        self._impl.publish_observation(filtered)

    def poll_action(self) -> ActionCommand | None:
        if self._impl is None:
            return None
        return self._impl.poll_action()

    # —— mock 注入（单测 / 无 ROS 时本地喂动作）——
    def inject_action(self, cmd: ActionCommand) -> None:
        if isinstance(self._impl, _MockTransport):
            self._impl.inject_action(cmd)
        else:
            raise RuntimeError("inject_action 仅 mock 传输可用")


class _TransportBase:
    kind: str = "base"

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def spin_once(self) -> None: ...
    def publish_observation(self, obs: ObservationBundle) -> None: ...
    def poll_action(self) -> ActionCommand | None: ...


class _MockTransport(_TransportBase):
    kind = "mock"

    def __init__(self, cfg: CommsConfig) -> None:
        self.cfg = cfg
        self._lock = Lock()
        self._latest: ActionCommand | None = None
        self._pub_count = 0

    def start(self) -> None:
        return

    def stop(self) -> None:
        return

    def spin_once(self) -> None:
        return

    def publish_observation(self, obs: ObservationBundle) -> None:
        self._pub_count += 1

    def poll_action(self) -> ActionCommand | None:
        with self._lock:
            cmd = self._latest
            self._latest = None
            return cmd

    def inject_action(self, cmd: ActionCommand) -> None:
        with self._lock:
            self._latest = cmd


class _RosTransport(_TransportBase):
    kind = "ros"

    def __init__(self, cfg: CommsConfig) -> None:
        self.cfg = cfg
        self._node = None
        self._rclpy = None
        self._lock = Lock()
        self._latest: ActionCommand | None = None
        self._cam_pubs: dict[str, Any] = {}
        self._joint_pub = None
        self._ee_l_pub = None
        self._ee_r_pub = None
        self._grip_pub = None
        self._Image = None
        self._JointState = None
        self._PoseStamped = None
        self._Float64MultiArray = None
        self._Header = None

    def start(self) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Float64MultiArray, Header

        self._rclpy = rclpy
        self._Image = Image
        self._JointState = JointState
        self._PoseStamped = PoseStamped
        self._Float64MultiArray = Float64MultiArray
        self._Header = Header

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node(self.cfg.node_name)

        # publishers created lazily for cameras; state pubs now
        if self.cfg.state_mode == "joint":
            self._joint_pub = self._node.create_publisher(
                JointState, self.cfg.joint_topic, 10
            )
        else:
            self._ee_l_pub = self._node.create_publisher(
                PoseStamped, self.cfg.ee_left_topic, 10
            )
            self._ee_r_pub = self._node.create_publisher(
                PoseStamped, self.cfg.ee_right_topic, 10
            )
        self._grip_pub = self._node.create_publisher(
            Float64MultiArray, self.cfg.gripper_topic, 10
        )

        if self.cfg.action_mode == "joint":
            self._node.create_subscription(
                JointState, self.cfg.joint_cmd_topic, self._on_joint_cmd, 10
            )
        else:
            self._node.create_subscription(
                PoseStamped, self.cfg.ee_cmd_left_topic, self._on_ee_left, 10
            )
            self._node.create_subscription(
                PoseStamped, self.cfg.ee_cmd_right_topic, self._on_ee_right, 10
            )
        self._node.create_subscription(
            Float64MultiArray, self.cfg.gripper_cmd_topic, self._on_gripper, 10
        )

    def stop(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._rclpy is not None and self._rclpy.ok():
            # 不 shutdown 全局，避免多节点冲突；仅销毁本 node
            pass

    def spin_once(self) -> None:
        if self._rclpy is not None and self._node is not None:
            self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def publish_observation(self, obs: ObservationBundle) -> None:
        assert self._node is not None
        stamp = self._node.get_clock().now().to_msg()
        header = self._Header(stamp=stamp, frame_id=self.cfg.frame_id)

        for name, rgb in obs.cameras.items():
            pub = self._cam_pubs.get(name)
            if pub is None:
                topic = f"{self.cfg.camera_topic_prefix}/{name}/image_raw"
                pub = self._node.create_publisher(self._Image, topic, 10)
                self._cam_pubs[name] = pub
            pub.publish(_rgb_to_image(rgb, header, self._Image, self.cfg.camera_encoding))

        if self.cfg.state_mode == "joint" and self._joint_pub is not None:
            msg = self._JointState()
            msg.header = header
            msg.name = list(obs.joint_names)
            if obs.joint_positions is not None:
                msg.position = [float(x) for x in np.asarray(obs.joint_positions).reshape(-1)]
            self._joint_pub.publish(msg)
        else:
            if obs.ee_left is not None and self._ee_l_pub is not None:
                self._ee_l_pub.publish(_pose_to_stamped(obs.ee_left, header, self._PoseStamped))
            if obs.ee_right is not None and self._ee_r_pub is not None:
                self._ee_r_pub.publish(_pose_to_stamped(obs.ee_right, header, self._PoseStamped))

        if self._grip_pub is not None:
            g = self._Float64MultiArray()
            g.data = [float(obs.left_gripper), float(obs.right_gripper)]
            self._grip_pub.publish(g)

    def poll_action(self) -> ActionCommand | None:
        with self._lock:
            cmd = self._latest
            # 保持最新，不清空：控制环每帧可重复读；用时间戳区分
            return deepcopy(cmd) if cmd is not None else None

    def _on_joint_cmd(self, msg) -> None:
        names = list(msg.name)
        pos = np.asarray(msg.position, dtype=np.float64)
        with self._lock:
            prev = self._latest or ActionCommand(mode="joint")
            prev.mode = "joint"
            prev.joint_names = names
            prev.joint_positions = pos
            prev.left_active = True
            prev.right_active = True
            self._latest = prev

    def _on_ee_left(self, msg) -> None:
        pose = _stamped_to_pose(msg)
        with self._lock:
            prev = self._latest or ActionCommand(mode="ee")
            prev.mode = "ee"
            prev.ee_left = pose
            prev.left_active = True
            self._latest = prev

    def _on_ee_right(self, msg) -> None:
        pose = _stamped_to_pose(msg)
        with self._lock:
            prev = self._latest or ActionCommand(mode="ee")
            prev.mode = "ee"
            prev.ee_right = pose
            prev.right_active = True
            self._latest = prev

    def _on_gripper(self, msg) -> None:
        data = list(msg.data or [])
        with self._lock:
            prev = self._latest or ActionCommand(mode=self.cfg.action_mode)
            if len(data) >= 1:
                prev.left_gripper = float(data[0])
            if len(data) >= 2:
                prev.right_gripper = float(data[1])
            self._latest = prev


def _rgb_to_image(rgb: np.ndarray, header, ImageCls, encoding: str):
    arr = np.asarray(rgb, dtype=np.uint8)
    if arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError(f"期望 HxWx3 RGB，得到 {arr.shape}")
    msg = ImageCls()
    msg.header = header
    msg.height = int(arr.shape[0])
    msg.width = int(arr.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(arr.shape[1] * 3)
    msg.data = arr.tobytes()
    return msg


def _pose_to_stamped(pose: np.ndarray, header, PoseStampedCls):
    p = np.asarray(pose, dtype=np.float64).reshape(7)
    msg = PoseStampedCls()
    msg.header = header
    msg.pose.position.x = float(p[0])
    msg.pose.position.y = float(p[1])
    msg.pose.position.z = float(p[2])
    msg.pose.orientation.x = float(p[3])
    msg.pose.orientation.y = float(p[4])
    msg.pose.orientation.z = float(p[5])
    msg.pose.orientation.w = float(p[6])
    return msg


def _stamped_to_pose(msg) -> np.ndarray:
    return np.array(
        [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ],
        dtype=np.float64,
    )
