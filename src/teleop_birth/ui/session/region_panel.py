"""随机化区域显隐：优先 MuJoCo overlay；PyQt 面板仅主线程且已有 QApplication 时才开。

采集循环 / MuJoCo viewer 回调里若在后台线程调用 ``ensure_region_ui``，
绝不能 ``QApplication.exec()``（会报 Must be called from the main thread 并卡住）。
"""

from __future__ import annotations

import threading
from typing import Any, Optional


_PANEL_WINDOW = None  # 保持引用防 GC
_PANEL_WARNED = False


def _is_main_thread() -> bool:
    return threading.current_thread() is threading.main_thread()


def start_region_panel(
    region: Any = None,
    *,
    title: str = "随机化区域",
) -> None:
    """尝试打开显隐面板；非主线程或无 QApp 时跳过（不影响 MuJoCo 绿盒）。"""
    global _PANEL_WINDOW, _PANEL_WARNED

    try:
        from teleop_birth.ui.session.region_viz import get_region_overlay

        ov = get_region_overlay()
        if region is not None:
            ov.set_region(region)
            ov.set_visible(True)
    except Exception as exc:  # noqa: BLE001
        print(f"  [region-panel] overlay 跳过: {exc}")
        return

    if not _is_main_thread():
        global_warn = False
        if not _PANEL_WARNED:
            _PANEL_WARNED = True
            global_warn = True
        if global_warn:
            print(
                "  [region-panel] 非主线程，跳过 PyQt 显隐面板"
                "（MuJoCo 中绿盒仍可用，可用快捷键/配置关显）"
            )
        return

    try:
        from PyQt6.QtWidgets import (
            QApplication,
            QCheckBox,
            QLabel,
            QMainWindow,
            QVBoxLayout,
            QWidget,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"  [region-panel] PyQt 不可用，跳过显隐面板: {exc}")
        return

    app = QApplication.instance()
    if app is None:
        # 采集场景不要在这里新建 QApplication 并 exec——会堵主线程
        if not _PANEL_WARNED:
            _PANEL_WARNED = True
            print(
                "  [region-panel] 无运行中的 Qt 应用，跳过显隐面板"
                "（MuJoCo 绿盒仍显示）"
            )
        return

    # 已有面板则只刷新区域
    if _PANEL_WINDOW is not None:
        try:
            if _PANEL_WINDOW.isVisible():
                return
        except Exception:
            _PANEL_WINDOW = None

    from teleop_birth.ui.session.region_viz import get_region_overlay

    ov = get_region_overlay()

    win = QMainWindow()
    win.setWindowTitle(title)
    win.setMinimumWidth(320)
    central = QWidget()
    lay = QVBoxLayout(central)
    tip = QLabel(
        "半透明绿盒 = 随机化区域（相对机器人 base）。\n"
        "取消勾选则在 MuJoCo 中隐藏；勾选则显示。"
    )
    tip.setWordWrap(True)
    lay.addWidget(tip)
    try:
        r = getattr(ov, "_region", None)
        if r is not None:
            b = r.world_bounds()
            lay.addWidget(
                QLabel(
                    f"世界范围 x=[{b['x_min']:.3f},{b['x_max']:.3f}] "
                    f"y=[{b['y_min']:.3f},{b['y_max']:.3f}]"
                )
            )
    except Exception:
        pass
    cb = QCheckBox("显示随机化区域")
    cb.setChecked(bool(ov.visible))
    cb.setStyleSheet("font-weight:bold; padding:8px;")
    cb.toggled.connect(lambda checked: ov.set_visible(bool(checked)))
    lay.addWidget(cb)
    lay.addStretch(1)
    win.setCentralWidget(central)
    win.resize(360, 160)
    win.show()
    _PANEL_WINDOW = win
    # 不调用 app.exec()：由调用方（工坊等）已有事件循环负责


def ensure_region_ui(region: Any = None) -> None:
    """挂载区域到 MuJoCo overlay；条件允许时再开 PyQt 显隐面板。"""
    from teleop_birth.ui.session.region_viz import get_region_overlay

    ov = get_region_overlay()
    if region is not None:
        try:
            ov.set_region(region)
        except Exception as exc:  # noqa: BLE001
            # dict 区域可能需转换；失败也不阻塞采集
            print(f"  [region] set_region 跳过: {exc}")
        ov.set_visible(True)
    start_region_panel(region)
