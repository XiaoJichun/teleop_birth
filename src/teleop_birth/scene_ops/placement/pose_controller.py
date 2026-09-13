"""运行时控制场景中 freejoint 物品的位姿。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Optional

import mujoco
import numpy as np


@dataclass
class FreeBodyHandle:
    name: str
    body_id: int
    joint_id: int
    qpos_adr: int


class PropPoseController:
    """在相同场景中移动已生成物品（MuJoCo freejoint: xyz + quat wxyz）。"""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model = model
        self.data = data
        self._handles: Dict[str, FreeBodyHandle] = {}

    def discover(self, names: Iterable[str] | None = None) -> list[str]:
        """按 body 名注册 freejoint；names=None 时扫描全部 freejoint body。"""
        found: list[str] = []
        if names is None:
            for j in range(self.model.njnt):
                if self.model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
                    continue
                bid = int(self.model.jnt_bodyid[j])
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid) or f"body_{bid}"
                self._register(name, bid, j)
                found.append(name)
            return found

        for name in names:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid < 0:
                raise KeyError(f"场景中没有 body: {name}")
            jid = self._freejoint_of_body(bid)
            if jid is None:
                raise KeyError(f"body {name} 没有 freejoint，无法运行时改位姿")
            self._register(name, bid, jid)
            found.append(name)
        return found

    def _register(self, name: str, body_id: int, joint_id: int) -> None:
        adr = int(self.model.jnt_qposadr[joint_id])
        self._handles[name] = FreeBodyHandle(
            name=name, body_id=body_id, joint_id=joint_id, qpos_adr=adr
        )

    def _freejoint_of_body(self, body_id: int) -> Optional[int]:
        for j in range(self.model.njnt):
            if int(self.model.jnt_bodyid[j]) == body_id and self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                return j
        return None

    def list_props(self) -> list[str]:
        return sorted(self._handles)

    def get_pose(self, name: str) -> np.ndarray:
        """返回 xyz + quat xyzw（与工程统一）。"""
        h = self._handles[name]
        q = self.data.qpos[h.qpos_adr : h.qpos_adr + 7].copy()
        # MuJoCo freejoint: [x,y,z, qw,qx,qy,qz]
        return np.array([q[0], q[1], q[2], q[4], q[5], q[6], q[3]], dtype=np.float64)

    def set_pose(
        self,
        name: str,
        *,
        pos: np.ndarray | list[float],
        quat_xyzw: np.ndarray | list[float] | None = None,
        zero_vel: bool = True,
    ) -> None:
        h = self._handles[name]
        p = np.asarray(pos, dtype=np.float64).reshape(3)
        if quat_xyzw is None:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
        else:
            q = np.asarray(quat_xyzw, dtype=np.float64).reshape(4)
            qx, qy, qz, qw = float(q[0]), float(q[1]), float(q[2]), float(q[3])
            n = max(np.linalg.norm([qw, qx, qy, qz]), 1e-12)
            qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
        self.data.qpos[h.qpos_adr : h.qpos_adr + 7] = [p[0], p[1], p[2], qw, qx, qy, qz]
        if zero_vel:
            # freejoint qvel = 6
            j = h.joint_id
            vadr = int(self.model.jnt_dofadr[j])
            self.data.qvel[vadr : vadr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def set_many(self, poses: dict[str, dict]) -> None:
        for name, cfg in poses.items():
            self.set_pose(
                name,
                pos=cfg["pos"],
                quat_xyzw=cfg.get("quat_xyzw"),
                zero_vel=bool(cfg.get("zero_vel", True)),
            )

    def snapshot(self) -> dict[str, list[float]]:
        return {n: self.get_pose(n).tolist() for n in self._handles}
