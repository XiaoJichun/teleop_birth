"""交互式终端公共组件（方向键菜单 / 确认 / 文本输入）。仿 data_process.cli.tui。"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Sequence, TypeVar

import questionary
from questionary import Choice, Style

T = TypeVar("T")

STYLE = Style(
    [
        ("qmark", "fg:cyan bold"),
        ("question", "bold"),
        ("answer", "fg:green"),
        ("pointer", "fg:cyan bold"),
        ("highlighted", "fg:cyan bold"),
        ("selected", "fg:green"),
        ("separator", "fg:ansiblack"),
        ("instruction", "fg:#888888"),
        ("text", ""),
        ("disabled", "fg:#858585 italic"),
    ]
)

CANCEL = object()


def banner(title: str, width: int = 72) -> None:
    line = "=" * width
    print(f"\n{line}\n{title}\n{line}")


def section(title: str, width: int = 72) -> None:
    print(f"\n{'-' * width}\n{title}\n{'-' * width}")


def select(
    title: str,
    options: Sequence[tuple[T, str]],
    *,
    default: T | None = None,
    allow_cancel: bool = True,
    disabled: Mapping[T, str] | None = None,
) -> T | None:
    if not options:
        raise RuntimeError(f"{title}：没有可选项")

    choices: list[Choice] = []
    for value, label in options:
        reason = disabled.get(value) if disabled else None
        if reason:
            choices.append(Choice(title=label, value=value, disabled=reason))
        else:
            choices.append(Choice(title=label, value=value))

    default_choice = None
    if default is not None:
        for choice in choices:
            if choice.value == default and not choice.disabled:
                default_choice = choice
                break

    kwargs: dict = {
        "message": title,
        "choices": choices,
        "style": STYLE,
        "instruction": "(↑↓ 选择，Enter 确认" + ("，Ctrl+C 取消)" if allow_cancel else ")"),
    }
    if default_choice is not None:
        kwargs["default"] = default_choice

    try:
        return questionary.select(**kwargs).ask()
    except KeyboardInterrupt:
        return None


def confirm(title: str, *, default: bool = True) -> bool | None:
    try:
        return questionary.confirm(
            title,
            default=default,
            style=STYLE,
            auto_enter=False,
        ).ask()
    except KeyboardInterrupt:
        return None


def text(title: str, *, default: str = "", validate=None) -> str | None:
    try:
        return questionary.text(
            title,
            default=default,
            validate=validate,
            style=STYLE,
        ).ask()
    except KeyboardInterrupt:
        return None


def path_text(
    title: str,
    *,
    default: str = "",
    must_exist: bool = False,
    expect_file: bool | None = None,
) -> Path | None:
    def _validate(raw: str) -> bool | str:
        value = (raw or "").strip()
        if not value:
            return "路径不能为空"
        path = Path(value).expanduser()
        if must_exist and not path.exists():
            return f"路径不存在: {path}"
        if expect_file is True and path.exists() and not path.is_file():
            return f"需要文件路径: {path}"
        if expect_file is False and path.exists() and not path.is_dir():
            return f"需要目录路径: {path}"
        return True

    raw = text(title, default=default, validate=_validate)
    if raw is None:
        return None
    return Path(raw.strip()).expanduser()


def press_enter(message: str = "按回车返回主菜单…") -> None:
    try:
        questionary.press_any_key_to_continue(message, style=STYLE).ask()
    except KeyboardInterrupt:
        pass
