"""Pico 原始位姿与 IK 前目标的滚动轨迹图。

控制环只入队样本。matplotlib 跑在 spawn 出的子进程里，避免卡住 150 Hz
循环，也避免和 MuJoCo viewer 抢同一条 GUI 线程。
"""

from __future__ import annotations

import math
import os
import time
from collections import deque
from datetime import datetime
from multiprocessing import get_context
from pathlib import Path
from typing import Any


_SIDES = ("left", "right")
_KINDS = ("pico", "pre_ik", "post_ik")
_GAP_S = 0.2
_ROW_META = {
    "pico": {
        "title": "PICO raw   |   just received from Pico",
        "short": "PICO",
        "window": "PICO raw (just received)",
        "face": "#e7f0fa",
        "color": "#1f4e79",
    },
    "pre_ik": {
        "title": "PRE-IK   |   command just before IK solve",
        "short": "PRE-IK",
        "window": "PRE-IK (before IK solve)",
        "face": "#fff3df",
        "color": "#9a4d00",
    },
    "post_ik": {
        "title": "POST-IK   |   robot TCP after IK solve",
        "short": "POST-IK",
        "window": "POST-IK (after IK solve)",
        "face": "#e8f7e8",
        "color": "#1b6b3a",
    },
}


class _Chan:
    """Rolling xyz series. A NaN sample breaks the line across time gaps."""

    def __init__(self, window: int) -> None:
        self.t: deque[float] = deque(maxlen=window)
        self.x: deque[float] = deque(maxlen=window)
        self.y: deque[float] = deque(maxlen=window)
        self.z: deque[float] = deque(maxlen=window)
        self.active: deque[bool] = deque(maxlen=window)

    def push(self, t: float, xyz: tuple[float, float, float], active: bool, *, gap_s: float = _GAP_S) -> None:
        if self.t and (t - self.t[-1]) > gap_s:
            self.t.append(t)
            self.x.append(math.nan)
            self.y.append(math.nan)
            self.z.append(math.nan)
            self.active.append(False)
        self.t.append(t)
        self.x.append(xyz[0])
        self.y.append(xyz[1])
        self.z.append(xyz[2])
        self.active.append(bool(active))


def _key(kind: str, side: str) -> str:
    return f"{kind}_{side}"


def _xyz(pose: Any) -> tuple[float, float, float]:
    return (float(pose[0]), float(pose[1]), float(pose[2]))


def _plot_worker(queue, cfg: dict[str, Any]) -> None:
    """Child process: own the figure. Sentinel None shuts it down."""
    window = max(int(cfg.get("window", 600)), 8)
    refresh_hz = max(float(cfg.get("refresh_hz", 12.0)), 1.0)
    warn_m = float(cfg.get("jump_warn_m", 0.08))
    target_hz = float(cfg.get("target_hz") or 0.0)
    rate_window_s = max(float(cfg.get("rate_window_s", 1.0)), 0.2)
    save_png = bool(cfg.get("save_png", False))
    save_dir = Path(str(cfg.get("save_dir") or "logs/plots"))
    interactive = bool(cfg.get("interactive", False))
    show = {k: bool(cfg.get(k, True)) for k in _KINDS}

    plt, live = _load_pyplot(interactive)
    if plt is None:
        return

    chans = {_key(k, s): _Chan(window) for k in _KINDS for s in _SIDES}
    rows = [k for k in _KINDS if show[k]]
    if not rows:
        return

    figs: dict[str, object] = {}
    axes_by_kind: dict[str, object] = {}
    try:
        for kind in rows:
            meta = _ROW_META[kind]
            fig, axes = plt.subplots(1, 4, figsize=(14.0, 3.15), sharex=True)
            fig.patch.set_facecolor(meta["face"])
            _reserve_header(fig)
            try:
                fig.canvas.manager.set_window_title(meta["window"])
            except Exception:
                pass
            figs[kind] = fig
            axes_by_kind[kind] = axes
    except Exception as exc:
        print(f"[plot] 无法创建窗口，轨迹图已跳过。{exc}", flush=True)
        return
    dirty = False
    last_draw = 0.0
    last_save = 0.0
    period = 1.0 / refresh_hz
    if not live:
        save_dir.mkdir(parents=True, exist_ok=True)
        names = ", ".join(str(save_dir / f"trace_{kind}.png") for kind in rows)
        print(f"[plot] 无交互窗口，轨迹图写到 {names}", flush=True)
    else:
        print(
            "[plot] 已打开三个窗口：蓝 PICO raw（刚收到） / 橙 PRE-IK（求解前） / 绿 POST-IK（求解后 TCP）",
            flush=True,
        )
        try:
            plt.show(block=False)
        except Exception:
            live = False

    try:
        while True:
            batch = _drain(queue, timeout=period)
            stop = False
            for item in batch:
                if item == ("__stop__",):
                    stop = True
                    break
                kind, side, t, xyz, active = item
                chan = chans.get(_key(str(kind), str(side)))
                if chan is not None:
                    chan.push(float(t), xyz, bool(active))
                    dirty = True
            now = time.perf_counter()
            if dirty and (now - last_draw) >= period:
                _paint(figs, axes_by_kind, rows, chans, warn_m, target_hz, rate_window_s)
                if live:
                    try:
                        for fig in figs.values():
                            fig.canvas.draw()
                            fig.canvas.flush_events()
                        plt.pause(0.001)
                    except Exception:
                        live = False
                if (not live or save_png) and (now - last_save) >= 1.0:
                    save_dir.mkdir(parents=True, exist_ok=True)
                    for kind, fig in figs.items():
                        fig.savefig(save_dir / f"trace_{kind}.png", dpi=110)
                    last_save = now
                last_draw = now
                dirty = False
            if stop:
                break
    finally:
        try:
            if figs:
                _paint(figs, axes_by_kind, rows, chans, warn_m, target_hz, rate_window_s)
            _save_session(figs, save_dir)
        except Exception as exc:
            print(f"[plot] 保存轨迹图失败: {exc}", flush=True)
        for fig in figs.values():
            try:
                plt.close(fig)
            except Exception:
                pass


def _load_pyplot(interactive: bool):
    try:
        import matplotlib
    except Exception as exc:
        print(f"[plot] 未安装 matplotlib，轨迹图已跳过（uv sync --extra plot）。{exc}", flush=True)
        return None, False

    if interactive:
        for name in ("TkAgg", "QtAgg"):
            try:
                matplotlib.use(name, force=True)
                import matplotlib.pyplot as plt

                return plt, True
            except Exception:
                continue
        print("[plot] 交互后端不可用，改为保存 PNG", flush=True)

    try:
        matplotlib.use("Agg", force=True)
        import matplotlib.pyplot as plt

        return plt, False
    except Exception as exc:
        print(f"[plot] matplotlib 初始化失败，轨迹图已跳过。{exc}", flush=True)
        return None, False


def _drain(queue, *, timeout: float) -> list:
    """Pull every queued sample so a slow draw cannot drop the IK traces."""
    batch: list = []
    try:
        batch.append(queue.get(timeout=timeout))
    except Exception:
        return batch
    while len(batch) < 8000:
        try:
            batch.append(queue.get_nowait())
        except Exception:
            break
    return batch


def _recv_hz(chan: _Chan | None, window_s: float) -> float | None:
    """Sliding-window rate of valid Pico samples."""
    if chan is None or len(chan.t) < 2:
        return None
    t_end = chan.t[-1]
    times = [t for t, x in zip(chan.t, chan.x) if not math.isnan(x) and t >= t_end - window_s]
    if len(times) < 2:
        return None
    span = times[-1] - times[0]
    if span <= 1e-6:
        return None
    return (len(times) - 1) / span


def _reserve_header(fig) -> None:
    """One compact header row, plots fill the rest of the figure."""
    fig.subplots_adjust(left=0.05, right=0.995, top=0.88, bottom=0.16, wspace=0.28)


def _paint(figs, axes_by_kind, rows, chans, warn_m, target_hz, rate_window_s) -> None:
    recv = _recv_hz(chans.get("pico_left"), rate_window_s)
    for kind in rows:
        _draw_row(axes_by_kind[kind], kind, chans, warn_m)
        _set_header(figs[kind], kind, chans, target_hz=target_hz, recv_hz=recv)


def _save_session(figs: dict, save_dir: Path) -> Path | None:
    if not figs:
        return None
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session = save_dir / stamp
    session.mkdir(parents=True, exist_ok=True)
    for kind, fig in figs.items():
        fig.savefig(session / f"{kind}.png", dpi=120)
        fig.savefig(save_dir / f"trace_{kind}.png", dpi=120)
    print(f"[plot] 遥操结束，轨迹图已保存: {session}", flush=True)
    return session


def _fmt_xyz(tag: str, xyz: tuple[float, float, float] | None) -> str:
    if xyz is None:
        return f"{tag}   n/a"
    return f"{tag}   x={xyz[0]:+.3f}   y={xyz[1]:+.3f}   z={xyz[2]:+.3f}"


def _set_header(fig, kind: str, chans: dict[str, _Chan], *, target_hz: float, recv_hz: float | None) -> None:
    """Single header row, sitting just above the axes."""
    for artist in list(fig.texts):
        artist.remove()
    meta = _ROW_META[kind]
    set_txt = f"set {target_hz:.0f} Hz" if target_hz > 0 else "set n/a"
    recv_txt = f"recv {recv_hz:.1f} Hz" if recv_hz is not None else "recv -- Hz"
    left = _fmt_xyz("L", _last_xyz(chans[_key(kind, "left")]))
    right = _fmt_xyz("R", _last_xyz(chans[_key(kind, "right")]))
    fig.text(0.05, 0.985, meta["short"], fontsize=11, fontweight="bold", color=meta["color"], va="top", ha="left")
    fig.text(0.13, 0.985, f"{set_txt}   {recv_txt}", fontsize=10, color=meta["color"], va="top", ha="left")
    fig.text(0.38, 0.985, left, fontsize=9, color="C0", va="top", ha="left")
    fig.text(0.68, 0.985, right, fontsize=9, color="C1", va="top", ha="left")
    _reserve_header(fig)


def _last_xyz(chan: _Chan) -> tuple[float, float, float] | None:
    for i in range(len(chan.x) - 1, -1, -1):
        xyz = (chan.x[i], chan.y[i], chan.z[i])
        if not any(math.isnan(v) for v in xyz):
            return xyz
    return None


def _draw_row(axes_row, kind: str, chans: dict[str, _Chan], warn_m: float) -> None:
    meta = _ROW_META[kind]
    comps = (("x (m)", "x"), ("y (m)", "y"), ("z (m)", "z"))
    for c, (ylabel, attr) in enumerate(comps):
        ax = axes_row[c]
        ax.clear()
        ax.set_facecolor(meta["face"])
        plotted: list[float] = []
        for side, color in (("left", "C0"), ("right", "C1")):
            chan = chans[_key(kind, side)]
            if kind == "pre_ik":
                _plot_masked(ax, chan, attr, color, active_want=True, style="-")
                _plot_masked(ax, chan, attr, color, active_want=False, style=":")
            else:
                _plot_masked(ax, chan, attr, color, active_want=True, style="-")
            plotted.extend(_finite(chan, attr))
        ax.set_ylabel(ylabel, fontsize=10, color=meta["color"])
        _fit_ylim(ax, plotted)
        ax.grid(True, alpha=0.3)
    ax = axes_row[3]
    ax.clear()
    ax.set_facecolor(meta["face"])
    for side, color in (("left", "C0"), ("right", "C1")):
        ts, ds = _deltas(chans[_key(kind, side)])
        if ts:
            ax.plot(ts, ds, color=color, lw=1.0)
    if warn_m > 0:
        ax.axhline(warn_m, color="red", lw=0.7, ls=":")
    ax.set_ylabel("|dp| (m)", fontsize=10, color=meta["color"])
    ax.set_xlabel("t (s)")
    axes_row[0].set_xlabel("t (s)")
    ax.grid(True, alpha=0.3)


def _finite(chan: _Chan, attr: str) -> list[float]:
    return [v for v in getattr(chan, attr) if not math.isnan(v)]


def _fit_ylim(ax, values: list[float]) -> None:
    if not values:
        return
    lo, hi = min(values), max(values)
    pad = max(0.008, (hi - lo) * 0.12)
    ax.set_ylim(lo - pad, hi + pad)


def _plot_masked(
    ax,
    chan: _Chan,
    attr: str,
    color: str,
    *,
    active_want: bool,
    style: str,
    label: str | None = None,
) -> None:
    if not chan.t:
        return
    ts = list(chan.t)
    vs = list(getattr(chan, attr))
    act = list(chan.active)
    out = [v if bool(a) == active_want else math.nan for v, a in zip(vs, act)]
    if all(math.isnan(v) for v in out):
        return
    ax.plot(ts, out, color=color, ls=style, lw=1.1, label=label)


def _deltas(chan: _Chan) -> tuple[list[float], list[float]]:
    ts: list[float] = []
    ds: list[float] = []
    prev = None
    prev_t = None
    for i, t in enumerate(chan.t):
        xyz = (chan.x[i], chan.y[i], chan.z[i])
        if any(math.isnan(v) for v in xyz):
            prev = None
            continue
        if prev is not None and prev_t is not None and (t - prev_t) <= _GAP_S:
            ds.append(math.dist(prev, xyz))
            ts.append(t)
        prev = xyz
        prev_t = t
    return ts, ds


class PoseTracePlotter:
    """Rolling Pico / pre-IK traces. `enabled=False` is a no-op (default)."""

    def __init__(
        self,
        *,
        enabled: bool = False,
        pico: bool = True,
        pre_ik: bool = True,
        post_ik: bool = True,
        window: int = 600,
        refresh_hz: float = 12.0,
        jump_warn_m: float = 0.08,
        save_png: bool = False,
        save_dir: str | Path = "logs/plots",
        target_hz: float = 0.0,
        rate_window_s: float = 1.0,
    ) -> None:
        self.pico = bool(pico)
        self.pre_ik = bool(pre_ik)
        self.post_ik = bool(post_ik)
        self.enabled = bool(enabled) and (self.pico or self.pre_ik or self.post_ik)
        self.window = max(int(window), 8)
        self.refresh_hz = float(refresh_hz)
        self.jump_warn_m = float(jump_warn_m)
        self.save_png = bool(save_png)
        self.save_dir = Path(save_dir)
        self.target_hz = float(target_hz)
        self.rate_window_s = max(float(rate_window_s), 0.2)
        self._t0 = time.perf_counter()
        self._last: dict[str, tuple[float, tuple[float, float, float]]] = {}
        self._last_warn = -1e9
        self._q = None
        self._proc = None

    def start(self) -> None:
        if not self.enabled or self._proc is not None:
            return
        interactive = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        save_png = self.save_png or not interactive
        if not interactive:
            print(
                f"[plot] 无 DISPLAY，轨迹图将写到 "
                f"{self.save_dir / 'trace_pico.png'} 和 {self.save_dir / 'trace_pre_ik.png'}",
                flush=True,
            )
        else:
            print("[plot] 正在打开轨迹图（Pico 原始 / IK 前）。关闭请设 plot.enabled=false", flush=True)
        ctx = get_context("spawn")
        self._q = ctx.Queue(maxsize=4096)
        self._proc = ctx.Process(
            target=_plot_worker,
            args=(
                self._q,
                {
                    "window": self.window,
                    "refresh_hz": self.refresh_hz,
                    "jump_warn_m": self.jump_warn_m,
                    "target_hz": self.target_hz,
                    "rate_window_s": self.rate_window_s,
                    "save_png": save_png,
                    "save_dir": str(self.save_dir),
                    "interactive": interactive,
                    "pico": self.pico,
                    "pre_ik": self.pre_ik,
                    "post_ik": self.post_ik,
                },
            ),
            daemon=True,
        )
        self._proc.start()
        self._t0 = time.perf_counter()

    def close(self) -> None:
        q, proc = self._q, self._proc
        self._q = None
        self._proc = None
        if q is not None:
            try:
                q.put(("__stop__",), timeout=0.2)
            except Exception:
                pass
        if proc is not None:
            proc.join(timeout=5.0)
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=0.5)

    def record_pico(self, obs: Any) -> None:
        if not self.enabled or not self.pico:
            return
        self._sample("pico", "left", getattr(obs, "left_controller", None), active=True)
        self._sample("pico", "right", getattr(obs, "right_controller", None), active=True)

    def record_command(self, target: Any) -> None:
        """Mapper 目标，送进 IK 之前。未握持时也画，避免求解前窗口空白。"""
        if not self.enabled or not self.pre_ik or target is None:
            return
        extremities = getattr(target, "extremities", None) or {}
        for side, name in (("left", "left_ee"), ("right", "right_ee")):
            pose = extremities.get(name)
            if pose is None:
                continue
            active = bool(target.is_active(name)) if hasattr(target, "is_active") else True
            self.record_pre_ik(side, pose, active=active)

    def record_realized(self, state: Any) -> None:
        """IK 求解并步进仿真之后的 TCP。"""
        if not self.enabled or not self.post_ik or state is None:
            return
        extremities = getattr(state, "extremities", None) or {}
        for side, name in (("left", "left_ee"), ("right", "right_ee")):
            pose = extremities.get(name)
            if pose is not None:
                self._sample("post_ik", side, pose, active=True)

    def record_pre_ik(self, side: str, pose: Any, *, active: bool = True) -> None:
        if not self.enabled or not self.pre_ik:
            return
        if side not in _SIDES or pose is None:
            return
        self._sample("pre_ik", side, pose, active=active)

    def _sample(self, kind: str, side: str, pose: Any, *, active: bool) -> None:
        if pose is None:
            return
        xyz = _xyz(pose)
        t = time.perf_counter() - self._t0
        self._warn_jump(kind, side, t, xyz)
        q = self._q
        if q is None:
            return
        try:
            q.put_nowait((kind, side, t, xyz, bool(active)))
        except Exception:
            return

    def _warn_jump(self, kind: str, side: str, t: float, xyz: tuple[float, float, float]) -> None:
        if self.jump_warn_m <= 0:
            return
        key = _key(kind, side)
        prev = self._last.get(key)
        self._last[key] = (t, xyz)
        if prev is None:
            return
        pt, pxyz = prev
        if (t - pt) > _GAP_S:
            return
        dp = math.dist(pxyz, xyz)
        if dp < self.jump_warn_m:
            return
        if (t - self._last_warn) < 0.4:
            return
        self._last_warn = t
        label = "Pico 原始" if kind == "pico" else "IK 前"
        hand = "左" if side == "left" else "右"
        print(f"[plot] {label} {hand} 相邻跳变 {dp * 1000:.0f} mm", flush=True)


def plotter_from_cfg(
    cfg: dict | None,
    *,
    project_root: Path | None = None,
    loop_hz: float | None = None,
) -> PoseTracePlotter:
    c = dict(cfg or {})
    save_dir = Path(str(c.get("save_dir") or "logs/plots"))
    if project_root is not None and not save_dir.is_absolute():
        save_dir = project_root / save_dir
    # 图上的设定频率就是本次控制环 hz，不再单独配一份，避免 50/120/150 对不上。
    target_hz = float(loop_hz or 0.0)
    return PoseTracePlotter(
        enabled=bool(c.get("enabled", False)),
        pico=bool(c.get("pico", True)),
        pre_ik=bool(c.get("pre_ik", True)),
        post_ik=bool(c.get("post_ik", True)),
        window=int(c.get("window", 600)),
        refresh_hz=float(c.get("refresh_hz", 12.0)),
        jump_warn_m=float(c.get("jump_warn_m", 0.08)),
        save_png=bool(c.get("save_png", False)),
        save_dir=save_dir,
        target_hz=target_hz,
        rate_window_s=float(c.get("rate_window_s", 1.0)),
    )


def attach_trace(backend: Any, plotter: PoseTracePlotter) -> None:
    """Hang the plotter on the raw backend (unwrap EndpointBackendAdapter)."""
    raw = backend
    endpoint = getattr(backend, "endpoint", None)
    if endpoint is not None and hasattr(endpoint, "_backend"):
        raw = endpoint._backend
    setattr(raw, "trace_plotter", plotter)


def emit_pre_ik(owner: Any, side: str, pose: Any, *, active: bool) -> None:
    plotter = getattr(owner, "trace_plotter", None)
    if plotter is None or pose is None:
        return
    plotter.record_pre_ik(side, pose, active=active)
