"""交互式终端主菜单（./start.sh / uv run birth-teleop）。"""

from __future__ import annotations

import sys

from birth_teleop.cli import tui
from birth_teleop.cli.util import load_yaml, project_root, resolve_handler


def main() -> None:
    root = project_root()
    menu_path = root / "configs" / "menu.yaml"
    if not menu_path.is_file():
        print(f"错误: 未找到菜单配置 {menu_path}")
        sys.exit(1)
    if not (root / "run_teleop.py").is_file():
        print(f"错误: 未找到 run_teleop.py（期望在项目根）: {root}")
        sys.exit(1)

    menu = load_yaml(menu_path)
    actions = menu.get("actions") or []
    if not isinstance(actions, list) or not actions:
        print(f"错误: {menu_path} 中 actions 为空")
        sys.exit(1)

    while True:
        tui.banner(str(menu.get("banner") or "birth_teleop"))
        print(f"  项目目录: {root}")
        print(f"  {menu.get('instructions') or ''}")

        options: list[tuple[str, str]] = []
        handler_map: dict[str, str | None] = {}
        for item in actions:
            if not isinstance(item, dict):
                continue
            aid = str(item.get("id") or "").strip()
            label = str(item.get("label") or aid)
            if not aid:
                continue
            options.append((aid, label))
            raw_handler = item.get("handler")
            handler_map[aid] = str(raw_handler) if raw_handler else None

        choice = tui.select("请选择功能", options, allow_cancel=True)
        if choice is None:
            print(f"\n{menu.get('exit_message') or '再见。'}")
            return

        handler_spec = handler_map.get(choice)
        if handler_spec is None:
            # menu.yaml 中 handler: null → 退出
            print(f"\n{menu.get('exit_message') or '再见。'}")
            return

        try:
            handler = resolve_handler(handler_spec)
            assert handler is not None
            handler(root)
        except KeyboardInterrupt:
            print("\n已中断当前任务。")
        except Exception as exc:  # noqa: BLE001
            print(f"\n执行失败: {exc}")

        tui.press_enter(str(menu.get("press_enter_message") or "按回车返回主菜单…"))


if __name__ == "__main__":
    main()
