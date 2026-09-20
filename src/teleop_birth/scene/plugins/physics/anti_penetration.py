"""可复用防穿模模块：臂↔物、物↔桌、物↔物。

设计：
- 包裹 contype=2 / affinity=1：与桌面碰、与臂不碰（避免夹爪物理穿插抖动）
- 抓住时运动学钉住；松手后钉回桌面高度
- 每步：物-物分离 + 桌面支撑 + TCP 近距推开

新场景接入：
  mgr = AntiPenetrationManager(model, data, table_top_z=0.44)
  mgr.register_prop("pkg_0", half_extent_z=0.02)
  mgr.register_tcp_site("L_tcp_pinch")
  mgr.on_grasp("pkg_0")
  mgr.step()
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import mujoco
import numpy as np


# 碰撞分组：桌/地=1，包裹=2；臂保持默认 1
PROP_CONTYPE = 2
PROP_CONAFFINITY = 1  # 只与桌面/地面碰，不与机械臂碰


@dataclass
class _GeomBackup:
    ids: list[int] = field(default_factory=list)
    contype: list[int] = field(default_factory=list)
    conaffinity: list[int] = field(default_factory=list)
    half_z: float = 0.02


class AntiPenetrationManager:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        table_top_z: float = 0.44,
        min_separation_m: float = 0.08,
        soft_push: float = 0.8,
        tcp_clearance_m: float = 0.06,
    ) -> None:
        self.model = model
        self.data = data
        self.table_top_z = float(table_top_z)
        self.min_separation_m = float(min_separation_m)
        self.soft_push = float(soft_push)
        self.tcp_clearance_m = float(tcp_clearance_m)
        self._props: dict[str, _GeomBackup] = {}
        self._held: set[str] = set()
        self._qadr: dict[str, int] = {}
        self._tcp_sids: list[int] = []
        # 额外支撑面：(xy 半范围中心, 半尺寸xy, 顶面 z)
        self._supports: list[tuple[np.ndarray, np.ndarray, float]] = []
        self._finger_bids: list[int] = []
        self.finger_clearance_m = 0.055
        # 接近/抓取/拨送中的目标：不做指垫推开，避免把盒子推离夹爪
        self._approach_exempt: set[str] = set()

    def register_prop(self, body_name: str, *, half_extent_z: float | None = None) -> None:
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if bid < 0:
            raise KeyError(body_name)
        bak = _GeomBackup()
        hz = 0.02
        for gid in range(self.model.ngeom):
            if int(self.model.geom_bodyid[gid]) != bid:
                continue
            # 跳过纯视觉面单
            gname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
            if gname.endswith("_label"):
                continue
            bak.ids.append(gid)
            # 强制包裹碰撞组：与臂不碰
            self.model.geom_contype[gid] = PROP_CONTYPE
            self.model.geom_conaffinity[gid] = PROP_CONAFFINITY
            bak.contype.append(PROP_CONTYPE)
            bak.conaffinity.append(PROP_CONAFFINITY)
            if self.model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_BOX:
                hz = max(hz, float(self.model.geom_size[gid][2]))
        if half_extent_z is not None:
            hz = float(half_extent_z)
        bak.half_z = hz
        self._props[body_name] = bak
        for j in range(self.model.njnt):
            if int(self.model.jnt_bodyid[j]) == bid and self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                self._qadr[body_name] = int(self.model.jnt_qposadr[j])
                break

    def register_tcp_site(self, site_name: str) -> None:
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if sid < 0:
            raise KeyError(site_name)
        self._tcp_sids.append(sid)

    def register_finger_bodies(self, *body_names: str) -> None:
        for name in body_names:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid >= 0:
                self._finger_bids.append(bid)

    def register_support_plane(
        self,
        *,
        center_xy: tuple[float, float] | list[float],
        half_xy: tuple[float, float] | list[float],
        top_z: float,
    ) -> None:
        """注册局部支撑面（如传送带顶面），防止包裹沉入皮带。"""
        self._supports.append(
            (
                np.asarray(center_xy, dtype=np.float64).reshape(2),
                np.asarray(half_xy, dtype=np.float64).reshape(2),
                float(top_z),
            )
        )

    def register_robot_prefix(self, prefix: str) -> None:
        # 臂保持默认碰撞；包裹已不与臂碰。保留接口兼容。
        return

    def register_robot_bodies(self, body_names: Iterable[str]) -> None:
        return

    def on_grasp(self, prop_name: str) -> None:
        """抓住：关掉包裹全部碰撞（运动学跟随，避免任何接触穿透）。"""
        bak = self._props[prop_name]
        for gid in bak.ids:
            self.model.geom_contype[gid] = 0
            self.model.geom_conaffinity[gid] = 0
        self._held.add(prop_name)
        self._approach_exempt.add(prop_name)

    def begin_approach(self, prop_name: str) -> None:
        self._approach_exempt.add(prop_name)

    def end_approach(self, prop_name: str) -> None:
        if prop_name not in self._held:
            self._approach_exempt.discard(prop_name)

    def on_release(self, prop_name: str) -> None:
        bak = self._props[prop_name]
        for i, gid in enumerate(bak.ids):
            self.model.geom_contype[gid] = bak.contype[i]
            self.model.geom_conaffinity[gid] = bak.conaffinity[i]
        self._held.discard(prop_name)
        self._approach_exempt.discard(prop_name)
        self.snap_to_table(prop_name)

    def snap_to_table(self, prop_name: str) -> None:
        if prop_name not in self._qadr:
            return
        bak = self._props[prop_name]
        adr = self._qadr[prop_name]
        self.data.qpos[adr + 2] = self.table_top_z + bak.half_z + 0.001
        # 清速度
        for j in range(self.model.njnt):
            if int(self.model.jnt_qposadr[j]) == adr:
                da = int(self.model.jnt_dofadr[j])
                self.data.qvel[da : da + 6] = 0.0
                break

    def _prop_pos(self, name: str) -> np.ndarray:
        adr = self._qadr[name]
        return self.data.qpos[adr : adr + 3].copy()

    def _set_prop_pos(self, name: str, pos: np.ndarray) -> None:
        adr = self._qadr[name]
        self.data.qpos[adr : adr + 3] = pos

    def separate_props(self) -> None:
        names = [n for n in self._props if n not in self._held and n in self._qadr]
        if len(names) < 2:
            return
        min_d = self.min_separation_m
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = names[i], names[j]
                pa, pb = self._prop_pos(a), self._prop_pos(b)
                dxy = pa[:2] - pb[:2]
                dist = float(np.linalg.norm(dxy))
                if dist < 1e-8:
                    dxy = np.array([1.0, 0.0])
                    dist = 1e-8
                if dist >= min_d:
                    continue
                overlap = min_d - dist
                nrm = dxy / dist
                push = 0.5 * overlap * self.soft_push * nrm
                pa[:2] = pa[:2] + push
                pb[:2] = pb[:2] - push
                self._set_prop_pos(a, pa)
                self._set_prop_pos(b, pb)

    def support_on_table(self) -> None:
        """自由包裹压回桌面/传送带高度，避免沉入穿模。"""
        for name, bak in self._props.items():
            if name in self._held or name not in self._qadr:
                continue
            adr = self._qadr[name]
            xy = self.data.qpos[adr : adr + 2]
            z_min = self.table_top_z + bak.half_z + 0.001
            for cxy, hxy, top_z in self._supports:
                if abs(float(xy[0] - cxy[0])) <= float(hxy[0]) and abs(float(xy[1] - cxy[1])) <= float(
                    hxy[1]
                ):
                    z_min = max(z_min, top_z + bak.half_z + 0.001)
            if float(self.data.qpos[adr + 2]) < z_min:
                self.data.qpos[adr + 2] = z_min

    def clear_tcp_overlap(self) -> None:
        """未抓住时，若 TCP 几乎插入包裹中心则水平轻推（阈值很小，避免干扰拨送）。"""
        if not self._tcp_sids:
            return
        r = min(self.tcp_clearance_m, 0.035)
        for name in self._props:
            if name in self._held or name in self._approach_exempt or name not in self._qadr:
                continue
            p = self._prop_pos(name)
            for sid in self._tcp_sids:
                tcp = self.data.site_xpos[sid]
                d = p - tcp
                dist = float(np.linalg.norm(d))
                if dist >= r or dist < 1e-8:
                    continue
                # 仅当 TCP 明显高于盒心时视为「插进」，侧向拨送不触发
                if float(tcp[2] - p[2]) < 0.02:
                    continue
                nrm = d / dist
                p = tcp + nrm * r
                p[2] = max(p[2], self.table_top_z + self._props[name].half_z + 0.002)
                self._set_prop_pos(name, p)

    def clear_finger_overlap(self) -> None:
        """未抓住时，指垫侵入包裹则把包裹水平推开（臂↔物视觉穿模）。"""
        if not self._finger_bids:
            return
        r = self.finger_clearance_m
        for name in self._props:
            if name in self._held or name in self._approach_exempt or name not in self._qadr:
                continue
            p = self._prop_pos(name)
            for bid in self._finger_bids:
                fp = self.data.xpos[bid]
                d = p - fp
                d[2] *= 0.25  # 主要水平推
                dist = float(np.linalg.norm(d))
                if dist >= r or dist < 1e-8:
                    continue
                nrm = d / dist
                p = p + nrm * (r - dist) * self.soft_push
                p[2] = max(p[2], self.table_top_z + self._props[name].half_z + 0.002)
                self._set_prop_pos(name, p)

    def step(self) -> None:
        self.support_on_table()
        self.separate_props()
        self.clear_finger_overlap()
        self.clear_tcp_overlap()
        mujoco.mj_forward(self.model, self.data)
