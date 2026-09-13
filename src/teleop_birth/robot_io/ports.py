"""Simulation ↔ real robot I/O ports (extractable).

两端口约定（接真机时只换实现，不改 TeleopLoop / Mapper）：

  StateReceiver.receive()  → RobotState   （状态接收）
  ActionPublisher.publish() → 下发动作，可选返回 RobotState

仿真与真机各自实现 RobotEndpoint = (StateReceiver + ActionPublisher)。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np


@dataclass
class RobotState:
    """统一状态快照（仿真或真机）。"""

    timestamp_ns: int = 0
    extremities: Dict[str, np.ndarray] = field(default_factory=dict)
    grippers: Dict[str, float] = field(default_factory=dict)
    joint_positions: Dict[str, np.ndarray] = field(default_factory=dict)
    extra: Dict[str, float] = field(default_factory=dict)

    def extremity(self, name: str) -> Optional[np.ndarray]:
        p = self.extremities.get(name)
        return None if p is None else p.copy()


@dataclass
class RobotAction:
    """统一动作指令（笛卡尔末端 + 夹爪；关节由 embodiment 内解）。"""

    timestamp_ns: int = 0
    extremities: Dict[str, np.ndarray] = field(default_factory=dict)
    active: Dict[str, bool] = field(default_factory=dict)
    grippers: Dict[str, float] = field(default_factory=dict)
    meta: Dict[str, object] = field(default_factory=dict)

    def is_active(self, name: str) -> bool:
        return bool(self.active.get(name, False))


class StateReceiver(ABC):
    """状态接收口 —— 真机侧通常订阅关节/力矩/末端反馈。"""

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def receive(self) -> RobotState: ...


class ActionPublisher(ABC):
    """动作发布口 —— 真机侧通常发布关节目标 / 笛卡尔伺服。"""

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def publish(self, action: RobotAction) -> RobotState:
        """下发动作；同步仿真可直接返回实现后状态，异步真机可返回最近反馈。"""


class RobotEndpoint(ABC):
    """成对封装：一个 embodiment = 状态口 + 动作口。"""

    kind: str = "unknown"  # "sim" | "real" | "mock"

    @property
    @abstractmethod
    def state_port(self) -> StateReceiver: ...

    @property
    @abstractmethod
    def action_port(self) -> ActionPublisher: ...

    def connect(self) -> None:
        self.state_port.connect()
        self.action_port.connect()

    def disconnect(self) -> None:
        self.action_port.disconnect()
        self.state_port.disconnect()
