"""Embodiment base: legacy RobotBackend ABC + BackendState (仍供 TeleopLoop 使用).

新代码请优先使用 teleop_birth.robot_io 双端口：
  StateReceiver / ActionPublisher / RobotEndpoint
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List

from teleop_birth.schema import SparseTarget


@dataclass
class BackendState:
    extremities: Dict[str, object] = field(default_factory=dict)
    left_gripper: float = 0.0
    right_gripper: float = 0.0
    extra: Dict[str, float] = field(default_factory=dict)


class RobotBackend(ABC):
    """兼容层接口。新 embodiment 应实现 RobotEndpoint，再用 EndpointBackendAdapter 接入。"""

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def get_feedback(self) -> Dict[str, object]: ...

    @abstractmethod
    def apply(self, target: SparseTarget) -> BackendState: ...


class LoggingBackend(RobotBackend):
    def __init__(self, inner: RobotBackend) -> None:
        self.inner = inner
        self.commands: List[SparseTarget] = []

    def connect(self) -> None:
        self.inner.connect()

    def disconnect(self) -> None:
        self.inner.disconnect()

    def get_feedback(self) -> Dict[str, object]:
        return self.inner.get_feedback()

    def apply(self, target: SparseTarget) -> BackendState:
        self.commands.append(target)
        return self.inner.apply(target)
