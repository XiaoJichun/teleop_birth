"""freejoint 包裹位姿控制。"""

from __future__ import annotations

from typing import Dict, Iterable

import mujoco
import numpy as np


class PropPoseController:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model = model
        self.data = data
        self._qadr: Dict[str, int] = {}

    def discover(self, names: Iterable[str]) -> None:
        for name in names:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid < 0:
                raise KeyError(name)
            jid = None
            for j in range(self.model.njnt):
                if int(self.model.jnt_bodyid[j]) == bid and self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                    jid = j
                    break
            if jid is None:
                raise KeyError(f"{name} 无 freejoint")
            self._qadr[name] = int(self.model.jnt_qposadr[jid])

    def get_pose(self, name: str) -> np.ndarray:
        adr = self._qadr[name]
        q = self.data.qpos[adr : adr + 7]
        # mujoco: xyz qw qx qy qz → xyzw
        return np.array([q[0], q[1], q[2], q[4], q[5], q[6], q[3]], dtype=np.float64)

    def set_pose(
        self,
        name: str,
        *,
        pos: np.ndarray | list[float],
        quat_xyzw: np.ndarray | list[float] | None = None,
        zero_vel: bool = True,
    ) -> None:
        adr = self._qadr[name]
        p = np.asarray(pos, dtype=np.float64).reshape(3)
        if quat_xyzw is None:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
        else:
            qx, qy, qz, qw = [float(x) for x in np.asarray(quat_xyzw).reshape(4)]
        self.data.qpos[adr : adr + 7] = [p[0], p[1], p[2], qw, qx, qy, qz]
        if zero_vel:
            # freejoint dof = 6
            jid = None
            for j in range(self.model.njnt):
                if int(self.model.jnt_qposadr[j]) == adr:
                    jid = j
                    break
            if jid is not None:
                da = int(self.model.jnt_dofadr[jid])
                self.data.qvel[da : da + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def snapshot(self) -> dict[str, list[float]]:
        return {n: self.get_pose(n).tolist() for n in self._qadr}
