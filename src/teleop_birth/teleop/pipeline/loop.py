"""Main teleoperation loop — embodiment-agnostic."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional

from teleop_birth.robot import BackendState, RobotBackend
from teleop_birth.teleop.control import LatencyProbe
from teleop_birth.teleop.mapping import Mapper
from teleop_birth.teleop.record import SessionRecorder
from teleop_birth.teleop.schema import SparseTarget, XRObservation
from teleop_birth.teleop.xr import XRSource


@dataclass
class LoopStats:
    steps: int
    latency: dict


class TeleopLoop:
    def __init__(
        self,
        source: XRSource,
        mapper: Mapper,
        backend: RobotBackend,
        *,
        hz: float,
        latency_window: int,
        recorder: Optional[SessionRecorder] = None,
        on_step: Optional[Callable[[XRObservation, SparseTarget, BackendState], None]] = None,
        trace: Optional[object] = None,
    ) -> None:
        if hz <= 0:
            raise ValueError("hz must be > 0")
        self.source = source
        self.mapper = mapper
        self.backend = backend
        self.dt = 1.0 / hz
        self.recorder = recorder
        self.on_step = on_step
        self.trace = trace
        self.latency = LatencyProbe(window=latency_window)
        self._running = False

    def run(self, duration_s: Optional[float] = None, max_steps: Optional[int] = None) -> LoopStats:
        self.source.start()
        if self.trace is not None:
            self.trace.start()
        self.backend.connect()
        self.mapper.reset()
        self._running = True
        steps = 0
        t_end = None if duration_s is None else time.perf_counter() + duration_s

        try:
            while self._running:
                if t_end is not None and time.perf_counter() >= t_end:
                    break
                if max_steps is not None and steps >= max_steps:
                    break

                t0 = time.perf_counter()
                obs = self.source.read()
                if not obs.valid:
                    time.sleep(self.dt)
                    continue
                if self.trace is not None:
                    self.trace.record_pico(obs)

                # Pico 重连后清空 clutch，避免用旧参考点猛拉
                if getattr(obs, "reconnected", False):
                    self.mapper.reset()
                    print("[teleop] 已重连并复位 clutch，请重新长按 Grip 遥操")

                feedback = self.backend.get_feedback()
                target = self.mapper.map(obs, feedback)
                # 掉线时冻结：不推进遥操目标（仅 stream_ok=False）
                if not getattr(obs, "stream_ok", True):
                    for name in list(target.active.keys()):
                        if name in ("left_ee", "right_ee"):
                            target.active[name] = False
                if self.trace is not None:
                    self.trace.record_command(target)
                state = self.backend.apply(target)
                if self.trace is not None:
                    self.trace.record_realized(state)
                sample = self.latency.tick(obs.timestamp_ns)
                if self.on_step is not None:
                    # Attach loop timing for diagnostics without changing callback arity
                    try:
                        self.on_step.last_loop_ms = sample.loop_ms  # type: ignore[attr-defined]
                    except Exception:
                        pass

                if self.recorder is not None:
                    self.recorder.log(obs, target, state, sample)

                if self.on_step is not None:
                    self.on_step(obs, target, state)

                steps += 1
                elapsed = time.perf_counter() - t0
                sleep_t = self.dt - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
        finally:
            self._running = False
            try:
                self.backend.disconnect()
                self.source.stop()
                if self.recorder is not None:
                    self.recorder.close()
            finally:
                if self.trace is not None:
                    self.trace.close()

        return LoopStats(steps=steps, latency=self.latency.summary())

    def stop(self) -> None:
        self._running = False
