"""XR data sources: mock for bring-up, XRoboToolkit for Pico Enterprise."""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Optional

import numpy as np
import yaml

from teleop_birth.teleop.schema import ButtonState, XRObservation, identity_pose, multiply_poses


class XRSource(ABC):
    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def read(self) -> XRObservation: ...


class MockXRSource(XRSource):
    """Synthetic XR stream for protocol / loop bring-up without a headset."""

    def __init__(self, hz: float, radius: float) -> None:
        self.hz = hz
        self.radius = radius
        self._t0 = time.perf_counter()
        self._running = False

    def start(self) -> None:
        self._t0 = time.perf_counter()
        self._running = True

    def stop(self) -> None:
        self._running = False

    def read(self) -> XRObservation:
        t = time.perf_counter() - self._t0
        ang = 0.4 * t
        left = identity_pose()
        right = identity_pose()
        left[0] = -0.25 + self.radius * np.cos(ang)
        left[1] = 0.30 + self.radius * np.sin(ang)
        left[2] = 0.20
        right[0] = 0.25 + self.radius * np.cos(ang + np.pi)
        right[1] = 0.30 + self.radius * np.sin(ang + np.pi)
        right[2] = 0.20
        head = identity_pose()
        head[2] = 1.5
        return XRObservation(
            timestamp_ns=time.time_ns(),
            head=head,
            left_controller=left,
            right_controller=right,
            buttons=ButtonState(
                left_grip=1.0,
                right_grip=1.0,
                left_trigger=0.2 + 0.2 * np.sin(ang),
                right_trigger=0.2 + 0.2 * np.cos(ang),
            ),
            valid=self._running,
        )


class KeyboardClutchMockXRSource(MockXRSource):
    """Same as mock, but grips can be toggled via attributes for tests."""

    def __init__(self, hz: float, radius: float) -> None:
        super().__init__(hz=hz, radius=radius)
        self.left_grip = 1.0
        self.right_grip = 1.0

    def read(self) -> XRObservation:
        obs = super().read()
        obs.buttons.left_grip = self.left_grip
        obs.buttons.right_grip = self.right_grip
        return obs


def _load_setup(setup_config: str | Path, project_root: Path) -> dict[str, Any]:
    path = Path(setup_config)
    if not path.is_absolute():
        path = project_root / path
    if not path.is_file():
        raise FileNotFoundError(f"未找到 XRobo setup 配置: {path}")
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"非法 setup 配置: {path}")
    return data


def check_xrobo_sdk_available(project_root: Path | None = None) -> tuple[bool, str]:
    root = project_root or Path(__file__).resolve().parents[4]
    try:
        from teleop_birth.teleop.xr.env import apply_xrobo_library_path

        apply_xrobo_library_path(root)
        import xrobotoolkit_sdk  # noqa: F401
    except ImportError as exc:
        return False, (
            "未安装 xrobotoolkit_sdk。\n"
            "一键安装（无需 sudo）:\n"
            "  bash scripts/xrobo/install_xrobo.sh\n"
            "或直接:\n"
            "  ./start_pico_sim.sh\n"
            f"原始错误: {exc}"
        )
    return True, "ok"


class XRoboSource(XRSource):
    """Adapter over xrobotoolkit_sdk；API 名称来自 configs/setup/xrobo.yaml。"""

    def __init__(
        self,
        *,
        include_body: bool,
        setup_config: str,
        project_root: Path,
    ) -> None:
        self.include_body = include_body
        self.project_root = Path(project_root)
        self.setup = _load_setup(setup_config, self.project_root)
        self.api: dict[str, str] = dict(self.setup.get("api") or {})
        self.hint_doc = str((self.setup.get("preflight") or {}).get("hint_doc") or "")
        rc = dict(self.setup.get("reconnect") or {})
        # 默认关闭自动重连：用位姿静止判掉线会误杀（手不动也会重连）
        self.auto_reconnect = bool(rc.get("enabled", False))
        self.stale_s = float(rc.get("stale_s", 5.0))
        self.reconnect_cooldown_s = float(rc.get("cooldown_s", 8.0))
        self._sdk = None
        self._running = False
        self._last_ts_change_t = time.perf_counter()
        self._last_timestamp_ns: Optional[int] = None
        self._last_reconnect_t = 0.0
        self._stream_ok = True
        self._just_reconnected = False
        self._warned_static = False

    def start(self) -> None:
        from teleop_birth.teleop.xr.env import apply_xrobo_library_path

        apply_xrobo_library_path(self.project_root)
        ok, msg = check_xrobo_sdk_available(self.project_root)
        if not ok:
            hint = f"\n详见: {self.hint_doc}" if self.hint_doc else ""
            raise ImportError(msg + hint)

        import xrobotoolkit_sdk as xrt  # type: ignore

        self._sdk = xrt
        self._sdk_init()
        self._running = True
        self._last_ts_change_t = time.perf_counter()
        self._last_timestamp_ns = None
        self._stream_ok = True
        self._warned_static = False

    def _sdk_init(self) -> None:
        assert self._sdk is not None
        init_name = self.api.get("init", "init")
        if hasattr(self._sdk, init_name):
            getattr(self._sdk, init_name)()

    def _sdk_close(self) -> None:
        if self._sdk is None:
            return
        close_name = self.api.get("close", "close")
        if hasattr(self._sdk, close_name):
            try:
                getattr(self._sdk, close_name)()
            except Exception:
                pass

    def stop(self) -> None:
        self._running = False
        self._sdk_close()

    def _try_reconnect(self) -> bool:
        now = time.perf_counter()
        if now - self._last_reconnect_t < self.reconnect_cooldown_s:
            return False
        self._last_reconnect_t = now
        print(
            "[xrobo] XR 时间戳长时间不更新，尝试重连 SDK…"
            "（请保持头显 WORKING + Send；连上后重新长按 Grip）"
        )
        try:
            self._sdk_close()
            time.sleep(0.4)
            self._sdk_init()
            self._just_reconnected = True
            self._last_ts_change_t = time.perf_counter()
            self._last_timestamp_ns = None
            self._stream_ok = True
            self._warned_static = False
            print("[xrobo] SDK 已重新 init")
            return True
        except Exception as exc:
            print(f"[xrobo] 重连失败: {exc}")
            self._stream_ok = False
            return False

    def _update_link_watch(self, timestamp_ns: int, left: np.ndarray, right: np.ndarray) -> None:
        """用 SDK 时间戳判断链路，不用位姿（手静止时位姿不变是正常的）。"""
        now = time.perf_counter()
        if self._last_timestamp_ns is None or int(timestamp_ns) != int(self._last_timestamp_ns):
            self._last_timestamp_ns = int(timestamp_ns)
            self._last_ts_change_t = now
            if not self._stream_ok:
                print("[xrobo] Pico 时间戳已恢复")
            self._stream_ok = True
            self._warned_static = False
            return

        # 时间戳不变：可能掉线，或 Send 关闭后 SDK 仍返回旧值
        idle = now - self._last_ts_change_t
        if idle < self.stale_s:
            return
        was_ok = self._stream_ok
        self._stream_ok = False
        if was_ok and not self._warned_static:
            print(
                f"[xrobo] 已 {idle:.1f}s 无新时间戳 → 可能掉线或 Send 关闭 "
                f"(xrL≈{left[0]:+.2f},{left[1]:+.2f},{left[2]:+.2f})"
            )
            self._warned_static = True
        if self.auto_reconnect:
            self._try_reconnect()

    def read(self) -> XRObservation:
        if not self._running or self._sdk is None:
            return XRObservation(timestamp_ns=time.time_ns(), valid=False, stream_ok=False)

        xrt = self._sdk
        api = self.api
        head = self._call_pose(xrt, api["head_pose"])
        left = self._call_pose(xrt, api["left_controller_pose"])
        right = self._call_pose(xrt, api["right_controller_pose"])

        ts_fn = api.get("timestamp_ns", "get_time_stamp_ns")
        if hasattr(xrt, ts_fn):
            timestamp_ns = int(getattr(xrt, ts_fn)())
        else:
            timestamp_ns = time.time_ns()

        left_axis = self._call_axis_pair(xrt, api.get("left_axis", "get_left_axis"))
        right_axis = self._call_axis_pair(xrt, api.get("right_axis", "get_right_axis"))

        buttons = ButtonState(
            left_grip=float(self._call_scalar(xrt, api["left_grip"])),
            right_grip=float(self._call_scalar(xrt, api["right_grip"])),
            left_trigger=float(self._call_scalar(xrt, api["left_trigger"])),
            right_trigger=float(self._call_scalar(xrt, api["right_trigger"])),
            button_a=bool(self._call_scalar(xrt, api["button_a"])),
            button_b=bool(self._call_scalar(xrt, api["button_b"])),
            button_x=bool(self._call_scalar(xrt, api.get("button_x", "get_X_button"))),
            button_y=bool(self._call_scalar(xrt, api.get("button_y", "get_Y_button"))),
            left_menu=bool(self._call_scalar(xrt, api.get("left_menu", "get_left_menu_button"))),
            right_menu=bool(self._call_scalar(xrt, api.get("right_menu", "get_right_menu_button"))),
            left_axis=left_axis,
            right_axis=right_axis,
            left_axis_click=bool(
                self._call_scalar(xrt, api.get("left_axis_click", "get_left_axis_click"))
            ),
            right_axis_click=bool(
                self._call_scalar(xrt, api.get("right_axis_click", "get_right_axis_click"))
            ),
        )

        body = None
        if self.include_body:
            avail_fn = api.get("body_available", "is_body_data_available")
            poses_fn = api.get("body_poses", "get_body_joints_pose")
            if hasattr(xrt, avail_fn) and getattr(xrt, avail_fn)():
                body = np.asarray(getattr(xrt, poses_fn)(), dtype=np.float64)
                if body.ndim == 1:
                    body = body.reshape(-1, 7)

        self._update_link_watch(timestamp_ns, left, right)
        reconnected = self._just_reconnected
        self._just_reconnected = False

        return XRObservation(
            timestamp_ns=timestamp_ns,
            head=head,
            left_controller=left,
            right_controller=right,
            body_joints=body,
            buttons=buttons,
            valid=True,
            stream_ok=self._stream_ok,
            reconnected=reconnected,
        )

    @staticmethod
    def _call_pose(xrt: Any, fn_name: str) -> np.ndarray:
        if not hasattr(xrt, fn_name):
            return identity_pose()
        return np.asarray(getattr(xrt, fn_name)(), dtype=np.float64)

    @staticmethod
    def _call_scalar(xrt: Any, fn_name: str) -> float:
        if not hasattr(xrt, fn_name):
            return 0.0
        return float(getattr(xrt, fn_name)())

    @staticmethod
    def _call_axis_pair(xrt: Any, fn_name: str) -> tuple[float, float]:
        if not hasattr(xrt, fn_name):
            return (0.0, 0.0)
        val = getattr(xrt, fn_name)()
        arr = np.asarray(val, dtype=np.float64).reshape(-1)
        if arr.size >= 2:
            return (float(arr[0]), float(arr[1]))
        return (0.0, 0.0)


def apply_workspace_offset(pose: np.ndarray, offset: np.ndarray) -> np.ndarray:
    return multiply_poses(offset, pose)
