"""Qt 中文输入法：让系统 IME 在输入框内直接输出汉字。

必须在创建 QApplication 之前调用 configure_qt_ime()。

本环境常见失效原因：
1. LANG/LC_ALL 为空或 POSIX → Qt inputMethod locale=C
2. 无 DBUS_SESSION_BUS_ADDRESS → Qt ibus 插件连不上输入法
3. 未导出 IBUS_ADDRESS
"""

from __future__ import annotations

import locale as _locale
import os
import shutil
import subprocess
import time
from pathlib import Path


def configure_qt_ime(*, verbose: bool = True) -> str:
    """UTF-8 中文 locale + session dbus + ibus，供框内直接出汉字。"""
    loc = _ensure_utf8_locale()
    _ensure_plugin_path()
    _ensure_session_dbus(verbose=verbose)

    os.environ["QT_IM_MODULE"] = "ibus"
    os.environ["XMODIFIERS"] = "@im=ibus"
    os.environ["GTK_IM_MODULE"] = "ibus"
    # 关键：Qt 只要看到 IBUS_USE_PORTAL 这个变量名（即使=0）就会走 portal，
    # 本机无有效 portal 时会出现 invalid portal bus，框内无法出中文。
    os.environ.pop("IBUS_USE_PORTAL", None)

    _ensure_ibus_running(verbose=verbose)
    _export_ibus_address(verbose=verbose)
    switch_to_chinese_ime(verbose=verbose)

    if verbose:
        eng = current_engine()
        dbus_ok = bool(os.environ.get("DBUS_SESSION_BUS_ADDRESS"))
        ibus_ok = bool(os.environ.get("IBUS_ADDRESS"))
        print(
            f"  Qt IME: ibus / locale={loc} / 引擎={eng or '?'} "
            f"/ dbus={'ok' if dbus_ok else '缺'} / ibus_addr={'ok' if ibus_ok else '缺'}"
        )
    return "ibus"


def apply_qt_locale(app=None) -> str:
    """QApplication 创建后设为简体中文。"""
    try:
        from PyQt6.QtCore import QLocale

        ql = QLocale(QLocale.Language.Chinese, QLocale.Country.China)
        QLocale.setDefault(ql)
        if app is not None:
            try:
                app.inputMethod().setLocale(ql)
            except Exception:
                pass
        return ql.name()
    except Exception:
        return ""


def switch_to_chinese_ime(*, verbose: bool = False) -> bool:
    """切换到 ibus 中文引擎（libpinyin），使输入框可直接出汉字。"""
    return switch_to_pinyin(verbose=verbose)


def switch_to_pinyin(*, verbose: bool = False) -> bool:
    """兼容旧名：切到智能拼音引擎。"""
    if not shutil.which("ibus"):
        return False
    if current_engine() == "libpinyin":
        if verbose:
            print("  Qt IME: 已是 libpinyin")
        return True
    for _ in range(5):
        subprocess.run(
            ["ibus", "engine", "libpinyin"],
            capture_output=True,
            check=False,
        )
        if current_engine() == "libpinyin":
            if verbose:
                print("  Qt IME: 已切换到 libpinyin")
            return True
        time.sleep(0.25)
    ok = current_engine() == "libpinyin"
    if verbose and not ok:
        print("  Qt IME: 切换 libpinyin 未成功，请执行: ibus engine libpinyin")
    return ok


def current_engine() -> str:
    try:
        r = subprocess.run(
            ["ibus", "engine"],
            capture_output=True,
            text=True,
            check=False,
        )
        return (r.stdout or "").strip()
    except Exception:
        return ""


def enable_widget_ime(widget) -> None:
    """确保控件接收系统输入法事件。"""
    try:
        from PyQt6.QtCore import Qt

        widget.setAttribute(Qt.WidgetAttribute.WA_InputMethodEnabled, True)
        widget.setInputMethodHints(Qt.InputMethodHint.ImhNone)
        widget.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        try:
            widget.setAttribute(Qt.WidgetAttribute.WA_KeyCompression, False)
        except Exception:
            pass
    except Exception:
        pass


def _ensure_utf8_locale() -> str:
    """优先 zh_CN.UTF-8（即使环境是 en_US，也强制中文 locale 以便 IME）。"""
    candidates = (
        "zh_CN.UTF-8",
        "zh_CN.utf8",
        "C.UTF-8",
        "C.utf8",
        "en_US.UTF-8",
        "en_US.utf8",
    )
    chosen = ""
    for name in candidates:
        try:
            _locale.setlocale(_locale.LC_ALL, name)
            chosen = name
            break
        except _locale.Error:
            continue
    if not chosen:
        chosen = "C.UTF-8"
        os.environ.setdefault("LANG", chosen)
        os.environ.setdefault("LC_ALL", chosen)
        return chosen

    os.environ["LANG"] = chosen
    os.environ["LC_ALL"] = chosen
    os.environ["LC_CTYPE"] = chosen
    if chosen.lower().startswith("zh"):
        os.environ["LANGUAGE"] = "zh_CN:zh"
    return chosen


def _ensure_plugin_path() -> None:
    try:
        import PyQt6

        plug = Path(PyQt6.__file__).resolve().parent / "Qt6" / "plugins"
        cur = os.environ.get("QT_PLUGIN_PATH", "")
        parts = [p for p in cur.split(":") if p]
        if str(plug) not in parts:
            os.environ["QT_PLUGIN_PATH"] = str(plug) + ((":" + cur) if cur else "")
    except Exception:
        pass


def _ensure_session_dbus(*, verbose: bool = False) -> None:
    """Qt ibus 插件依赖 session dbus；Cursor/docker 终端里经常没有。"""
    if os.environ.get("DBUS_SESSION_BUS_ADDRESS"):
        return
    if not shutil.which("dbus-launch"):
        if verbose:
            print("  Qt IME: 无 dbus-launch，请安装: apt install -y dbus-x11")
        return
    try:
        r = subprocess.run(
            ["dbus-launch", "--sh-syntax"],
            capture_output=True,
            text=True,
            check=False,
        )
        if r.returncode != 0:
            return
        for line in (r.stdout or "").splitlines():
            line = line.strip().rstrip(";")
            if line.startswith("export "):
                continue
            for key in ("DBUS_SESSION_BUS_ADDRESS", "DBUS_SESSION_BUS_PID"):
                prefix = key + "="
                if line.startswith(prefix):
                    raw = line[len(prefix) :].strip().strip("'").strip('"')
                    if raw:
                        os.environ[key] = raw
        if verbose and os.environ.get("DBUS_SESSION_BUS_ADDRESS"):
            print("  Qt IME: 已启动 session dbus")
    except Exception as exc:
        if verbose:
            print(f"  Qt IME: dbus-launch 失败 ({exc})")


def _export_ibus_address(*, verbose: bool = False) -> None:
    addr = ""
    if shutil.which("ibus"):
        try:
            r = subprocess.run(
                ["ibus", "address"],
                capture_output=True,
                text=True,
                check=False,
            )
            addr = (r.stdout or "").strip()
        except Exception:
            addr = ""
    if not addr:
        bus_dir = Path.home() / ".config" / "ibus" / "bus"
        if bus_dir.is_dir():
            for p in sorted(bus_dir.glob("*"), key=lambda x: x.stat().st_mtime, reverse=True):
                try:
                    text = p.read_text(encoding="utf-8", errors="ignore")
                except Exception:
                    continue
                for line in text.splitlines():
                    if line.startswith("IBUS_ADDRESS="):
                        addr = line.split("=", 1)[1].strip()
                        break
                if addr:
                    break
    if addr:
        os.environ["IBUS_ADDRESS"] = addr
        if verbose:
            print("  Qt IME: 已导出 IBUS_ADDRESS")


def _ensure_ibus_running(*, verbose: bool = False) -> None:
    if not shutil.which("ibus-daemon"):
        if verbose:
            print("  Qt IME: 请安装: apt install -y ibus ibus-libpinyin")
        return
    try:
        r = subprocess.run(
            ["pgrep", "-x", "ibus-daemon"],
            capture_output=True,
            check=False,
        )
        if r.returncode != 0:
            subprocess.Popen(
                ["ibus-daemon", "-drx"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env={**os.environ},
            )
            time.sleep(0.6)
            if verbose:
                print("  Qt IME: 已启动 ibus-daemon")
        subprocess.run(["ibus", "write-cache"], capture_output=True, check=False)
    except Exception as exc:
        if verbose:
            print(f"  Qt IME: ibus 启动跳过 ({exc})")
