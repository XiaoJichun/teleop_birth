"""语义任务规格（对齐 Genie Sim stages: pick / place / reset）。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass
class TaskStage:
    action: str  # pick | place | reset | wait
    description: str = ""
    active_object: Optional[str] = None
    passive_object: Optional[str] = None
    arm: str = "right"  # left | right
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass
class TaskSpec:
    name: str
    instruction_zh: str
    instruction_en: str
    objects: list[str]
    stages: list[TaskStage]
    episodes: int = 5
    seed: int = 0
    success_xy_tol_m: float = 0.06
    success_z_tol_m: float = 0.08

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "TaskSpec":
        stages = [
            TaskStage(
                action=str(s["action"]),
                description=str(s.get("description") or s.get("action_text") or ""),
                active_object=(s.get("active") or {}).get("object_id")
                if isinstance(s.get("active"), dict)
                else s.get("active_object"),
                passive_object=(s.get("passive") or {}).get("object_id")
                if isinstance(s.get("passive"), dict)
                else s.get("passive_object"),
                arm=str(s.get("arm") or (s.get("extra_params") or {}).get("arm") or "right"),
                extra=dict(s.get("extra") or s.get("extra_params") or {}),
            )
            for s in (data.get("stages") or [])
        ]
        return cls(
            name=str(data.get("name") or data.get("task") or "task"),
            instruction_zh=str(
                (data.get("task_description") or {}).get("task_text")
                or data.get("instruction_zh")
                or ""
            ),
            instruction_en=str(
                (data.get("task_description") or {}).get("english_task_text")
                or data.get("instruction_en")
                or ""
            ),
            objects=list(data.get("objects") or []),
            stages=stages,
            episodes=int(data.get("episodes", 5)),
            seed=int(data.get("seed", 0)),
            success_xy_tol_m=float(data.get("success_xy_tol_m", 0.06)),
            success_z_tol_m=float(data.get("success_z_tol_m", 0.08)),
        )
