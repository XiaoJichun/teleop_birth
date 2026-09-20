"""ArtiXon 物流采集子包。"""

from __future__ import annotations

__all__ = ["LogisticsCollector"]


def __getattr__(name: str):
    if name == "LogisticsCollector":
        from teleop_birth.robot.arm.artixon.collection.collector import LogisticsCollector

        return LogisticsCollector
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
