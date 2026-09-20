"""PyQt / Matplotlib 中文字体：缺 CJK 时界面会显示乱码或方块。

优先加载 mplfonts 自带字体（SimHei / Noto Sans CJK SC），再回退系统文泉驿/Noto。
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

_APPLIED = False
_FONT_FAMILY: Optional[str] = None

# 系统字体族名候选（fc-list）
_SYSTEM_FAMILIES = (
    "Noto Sans CJK SC",
    "Noto Sans CJK JP",  # 部分环境只有 JP/KR 包也能显示部分汉字
    "WenQuanYi Micro Hei",
    "文泉驿微米黑",
    "Source Han Sans SC",
    "Source Han Sans CN",
    "SimHei",
    "Microsoft YaHei",
    "PingFang SC",
)


def apply_chinese_fonts(app=None) -> str:
    """为当前进程设置中文字体；返回选用的 family 名。可重复调用。"""
    global _APPLIED, _FONT_FAMILY
    family = _ensure_qt_font(app)
    _ensure_matplotlib_font(family)
    _APPLIED = True
    _FONT_FAMILY = family
    return family


def _ensure_qt_font(app=None) -> str:
    from PyQt6.QtGui import QFont, QFontDatabase
    from PyQt6.QtWidgets import QApplication

    # 1) 注册 mplfonts 内嵌文件（不依赖系统 apt 字体）
    registered: list[str] = []
    for path in _mplfonts_font_files():
        fid = QFontDatabase.addApplicationFont(str(path))
        if fid >= 0:
            families = QFontDatabase.applicationFontFamilies(fid)
            registered.extend(families)

    # 2) 挑选可用 family
    prefer = [
        "Noto Sans CJK SC",
        "Noto Sans CJK SC Regular",
        "NotoSansCJKsc-Regular",
        "SimHei",
        *registered,
        *_SYSTEM_FAMILIES,
    ]
    available = set(QFontDatabase.families())
    # 模糊匹配：family 名可能带 Regular 后缀
    chosen = None
    for name in prefer:
        if name in available:
            chosen = name
            break
        for fam in available:
            if name.lower().replace(" ", "") in fam.lower().replace(" ", ""):
                chosen = fam
                break
        if chosen:
            break
    if chosen is None:
        # 仍无 CJK：用第一个已注册的，否则 DejaVu（会乱码，但至少不崩）
        chosen = registered[0] if registered else "Sans Serif"

    font = QFont(chosen, 10)
    font.setStyleHint(QFont.StyleHint.SansSerif)
    QApplication.setFont(font)
    if app is not None:
        app.setFont(font)
    return chosen


def _mplfonts_font_files() -> list[Path]:
    out: list[Path] = []
    try:
        import mplfonts

        root = Path(mplfonts.__file__).resolve().parent / "fonts"
    except Exception:
        root = None
    if root is not None and root.is_dir():
        prefer_names = (
            "SimHei.ttf",
            "NotoSansCJKsc-Regular.otf",
            "NotoSerifCJKsc-Regular.otf",
            "SourceHanSerifSC-Regular.otf",
        )
        for name in prefer_names:
            p = root / name
            if p.is_file():
                out.append(p)
        if not out:
            out.extend(sorted(root.glob("*.ttf")))
            out.extend(sorted(root.glob("*.otf")))
    # 系统 CJK（Qt 有时不会自动枚举 TTC，需显式 addApplicationFont）
    for p in (
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
        Path("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"),
        Path("/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc"),
    ):
        if p.is_file():
            out.append(p)
    return out


def _ensure_matplotlib_font(preferred_family: str | None = None) -> None:
    try:
        import logging

        logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)
    except Exception:
        pass
    try:
        from mplfonts import use_font

        # mplfonts 内置名
        for name in ("Noto Sans CJK SC", "SimHei", "Noto Serif CJK SC"):
            try:
                use_font(name)
                return
            except Exception:
                continue
    except Exception:
        pass
    try:
        import matplotlib
        from matplotlib import font_manager

        # 直接注册 ttf/otf，避免 generic sans-serif 找不到 CJK
        for path in _mplfonts_font_files():
            if path.suffix.lower() in (".ttf", ".otf"):
                try:
                    font_manager.fontManager.addfont(str(path))
                except Exception:
                    continue
        matplotlib.rcParams["axes.unicode_minus"] = False
        family = preferred_family or "SimHei"
        matplotlib.rcParams["font.sans-serif"] = [
            family,
            "SimHei",
            "Noto Sans CJK SC",
            "WenQuanYi Micro Hei",
            "DejaVu Sans",
        ]
        matplotlib.rcParams["font.family"] = "sans-serif"
    except Exception:
        pass
