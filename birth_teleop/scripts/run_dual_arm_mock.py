"""已弃用：请使用 ./start.sh 或 uv run python run_teleop.py。"""

from __future__ import annotations


def main() -> None:
    raise SystemExit(
        "此脚本已弃用。请使用:\n"
        "  ./start.sh\n"
        "或:\n"
        "  uv run python run_teleop.py source=mock backend=mock_dual_arm "
        "runtime.interactive_confirm=false runtime.duration_s=3"
    )


if __name__ == "__main__":
    main()
