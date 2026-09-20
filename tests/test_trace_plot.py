"""Pose trace plotter stays off unless configured, and flags large jumps."""

from __future__ import annotations

import math

import numpy as np

from teleop_birth.teleop.plot.trace import PoseTracePlotter, _Chan, _recv_hz, attach_trace, plotter_from_cfg
from teleop_birth.teleop.schema import XRObservation, identity_pose


def test_recv_hz_uses_sliding_window() -> None:
    chan = _Chan(20)
    for i in range(6):
        chan.push(i * 0.01, (float(i), 0.0, 0.0), True)
    rate = _recv_hz(chan, 1.0)
    assert rate is not None
    assert abs(rate - 100.0) < 1.0


def test_target_hz_follows_loop_hz() -> None:
    plotter = plotter_from_cfg({"enabled": True, "target_hz": 80}, loop_hz=150.0)
    assert plotter.target_hz == 150.0


def test_default_plotter_is_off() -> None:
    plotter = plotter_from_cfg(None)
    assert plotter.enabled is False
    obs = XRObservation(timestamp_ns=1)
    plotter.record_pico(obs)
    plotter.record_pre_ik("left", identity_pose(), active=True)
    plotter.start()
    plotter.close()


def test_jump_warning_prints_once(capsys) -> None:
    plotter = plotter_from_cfg(
        {"enabled": True, "pico": True, "pre_ik": True, "jump_warn_m": 0.01}
    )
    plotter.record_pre_ik("left", np.zeros(7), active=True)
    plotter.record_pre_ik("left", np.array([0.05, 0.0, 0.0, 0, 0, 0, 1]), active=True)
    plotter.record_pre_ik("left", np.array([0.12, 0.0, 0.0, 0, 0, 0, 1]), active=True)
    out = capsys.readouterr().out
    assert "IK 前" in out
    assert "50 mm" in out
    assert out.count("IK 前") == 1


def test_gap_inserts_nan_so_line_breaks() -> None:
    chan = _Chan(10)
    chan.push(0.0, (0.0, 0.0, 0.0), True)
    chan.push(1.0, (1.0, 0.0, 0.0), True)
    assert math.isnan(chan.x[1])
    assert chan.x[2] == 1.0


def test_attach_unwraps_endpoint_adapter() -> None:
    class Raw:
        pass

    class Endpoint:
        def __init__(self) -> None:
            self._backend = Raw()

    class Adapter:
        def __init__(self) -> None:
            self.endpoint = Endpoint()

    adapter = Adapter()
    plotter = PoseTracePlotter(enabled=False)
    attach_trace(adapter, plotter)
    assert adapter.endpoint._backend.trace_plotter is plotter
