"""Bridge SparseTarget ↔ RobotAction / RobotBackend ↔ RobotEndpoint."""

from __future__ import annotations

from typing import Dict

import numpy as np

from teleop_birth.embodiment.base import BackendState, RobotBackend
from teleop_birth.robot_io.ports import ActionPublisher, RobotAction, RobotEndpoint, RobotState, StateReceiver
from teleop_birth.schema import SparseTarget


class SparseTargetCodec:
    """Mapper 总线 SparseTarget 与 RobotAction / RobotState 互转。"""

    @staticmethod
    def to_action(target: SparseTarget) -> RobotAction:
        return RobotAction(
            timestamp_ns=int(target.timestamp_ns),
            extremities={k: v.copy() for k, v in target.extremities.items()},
            active=dict(target.active),
            grippers={
                "left": float(target.left_gripper),
                "right": float(target.right_gripper),
            },
            meta=dict(target.meta or {}),
        )

    @staticmethod
    def from_state_extremities(state: RobotState) -> Dict[str, np.ndarray]:
        return {k: v.copy() for k, v in state.extremities.items()}

    @staticmethod
    def to_backend_state(state: RobotState) -> BackendState:
        return BackendState(
            extremities={k: v.copy() for k, v in state.extremities.items()},
            left_gripper=float(state.grippers.get("left", 0.0)),
            right_gripper=float(state.grippers.get("right", 0.0)),
            extra=dict(state.extra),
        )


class EndpointBackendAdapter(RobotBackend):
    """让 TeleopLoop 继续吃 RobotBackend：内部走双端口 RobotEndpoint。"""

    def __init__(self, endpoint: RobotEndpoint) -> None:
        self.endpoint = endpoint
        self._codec = SparseTargetCodec()

    def connect(self) -> None:
        self.endpoint.connect()

    def disconnect(self) -> None:
        self.endpoint.disconnect()

    def get_feedback(self) -> Dict[str, np.ndarray]:
        return self._codec.from_state_extremities(self.endpoint.state_port.receive())

    def apply(self, target: SparseTarget) -> BackendState:
        action = self._codec.to_action(target)
        realized = self.endpoint.action_port.publish(action)
        return self._codec.to_backend_state(realized)


class BackendAsEndpoint(RobotEndpoint):
    """把遗留 RobotBackend 适配成双端口（渐进迁移用）。"""

    kind = "legacy"

    def __init__(self, backend: RobotBackend, *, kind: str = "legacy") -> None:
        self._backend = backend
        self.kind = kind
        self._state = _BackendStateReceiver(backend)
        self._action = _BackendActionPublisher(backend, self._state)

    @property
    def state_port(self) -> StateReceiver:
        return self._state

    @property
    def action_port(self) -> ActionPublisher:
        return self._action


class _BackendStateReceiver(StateReceiver):
    def __init__(self, backend: RobotBackend) -> None:
        self._backend = backend
        self._connected = False

    def connect(self) -> None:
        if not self._connected:
            self._backend.connect()
            self._connected = True

    def disconnect(self) -> None:
        if self._connected:
            self._backend.disconnect()
            self._connected = False

    def receive(self) -> RobotState:
        fb = self._backend.get_feedback()
        return RobotState(extremities={k: v.copy() for k, v in fb.items()})


class _BackendActionPublisher(ActionPublisher):
    def __init__(self, backend: RobotBackend, state: _BackendStateReceiver) -> None:
        self._backend = backend
        self._state = state

    def connect(self) -> None:
        self._state.connect()

    def disconnect(self) -> None:
        return

    def publish(self, action: RobotAction) -> RobotState:
        target = SparseTarget(
            timestamp_ns=action.timestamp_ns,
            extremities={k: v.copy() for k, v in action.extremities.items()},
            active=dict(action.active),
            left_gripper=float(action.grippers.get("left", 0.0)),
            right_gripper=float(action.grippers.get("right", 0.0)),
            meta=dict(action.meta),
        )
        st = self._backend.apply(target)
        return RobotState(
            timestamp_ns=action.timestamp_ns,
            extremities={k: v.copy() for k, v in st.extremities.items()},
            grippers={"left": st.left_gripper, "right": st.right_gripper},
            extra=dict(st.extra),
        )
