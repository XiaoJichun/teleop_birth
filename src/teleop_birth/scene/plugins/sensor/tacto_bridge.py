"""MuJoCo ↔ TACTO（vision-based tactile）桥接。

TACTO 官方针对 PyBullet；本模块用 MuJoCo 射线深度图 + tacto.Renderer.render_from_depth，
在 Arm-6A 指垫上仿真 DIGIT 风格触觉图像。

仓库：https://github.com/facebookresearch/tacto （third_party/tacto）
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


def _load_tacto_renderer(tacto_root: Path):
    """只加载 Renderer，避开 tacto.__init__ → Sensor → urdfpy（与 py3.12/networkx3 冲突）。"""
    import importlib.util
    import types

    conf = tacto_root / "tacto" / "config_digit.yml"
    if not conf.is_file():
        raise FileNotFoundError(conf)

    # 伪造包名，避免执行 tacto/__init__.py
    if "tacto" not in sys.modules:
        pkg = types.ModuleType("tacto")
        pkg.__path__ = [str(tacto_root / "tacto")]
        sys.modules["tacto"] = pkg

    path = tacto_root / "tacto" / "renderer.py"
    if "tacto.renderer" in sys.modules:
        mod = sys.modules["tacto.renderer"]
    else:
        spec = importlib.util.spec_from_file_location("tacto.renderer", path)
        assert spec and spec.loader
        mod = importlib.util.module_from_spec(spec)
        sys.modules["tacto.renderer"] = mod
        spec.loader.exec_module(mod)
    return mod.Renderer, str(conf)


def _ensure_tacto_on_path(repo_root: Path | None = None) -> Path:
    """把 third_party/tacto 加入 path，返回包根目录。"""
    if repo_root is not None:
        tacto_root = Path(repo_root) / "third_party" / "tacto"
        if not (tacto_root / "tacto" / "__init__.py").is_file():
            raise FileNotFoundError(tacto_root)
    else:
        cur = Path(__file__).resolve().parent
        tacto_root = None
        for _ in range(10):
            cand = cur / "third_party" / "tacto"
            if (cand / "tacto" / "__init__.py").is_file():
                tacto_root = cand
                break
            cur = cur.parent
        if tacto_root is None:
            raise FileNotFoundError(
                "未找到 third_party/tacto。请执行: bash scripts/bootstrap_third_party.sh"
            )

    root_s = str(tacto_root)
    if root_s not in sys.path:
        sys.path.insert(0, root_s)
    return tacto_root


@dataclass
class TactileSensorSpec:
    """单个指垫触觉传感器。"""

    name: str
    body_name: str
    # 指垫局部：射线起点相对 body；法向朝接触侧
    local_origin: tuple[float, float, float] = (0.028, 0.0, 0.0)
    local_normal: tuple[float, float, float] = (0.0, 1.0, 0.0)  # 左右指互指
    gel_half_w: float = 0.012  # 沿 pad 宽
    gel_half_h: float = 0.010  # 沿 pad 高
    max_depth_m: float = 0.012


@dataclass
class TactoTouchBundle:
    """一次渲染结果。"""

    colors: dict[str, np.ndarray] = field(default_factory=dict)
    depths: dict[str, np.ndarray] = field(default_factory=dict)
    contact: dict[str, bool] = field(default_factory=dict)


class MujocoTactoSensors:
    """在若干指垫 body 上挂 DIGIT 渲染器。"""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        specs: list[TactileSensorSpec],
        *,
        repo_root: Path | None = None,
        width: int = 120,
        height: int = 160,
        enabled: bool = True,
        show_window: bool = False,
        every_n: int = 2,
    ) -> None:
        self.model = model
        self.data = data
        self.specs = list(specs)
        self.enabled = bool(enabled)
        self.show_window = bool(show_window)
        self.every_n = max(1, int(every_n))
        self.width = int(width)
        self.height = int(height)
        self._tick = 0
        self._renderer = None
        self._cv2 = None
        self._body_id: dict[str, int] = {}
        self._last = TactoTouchBundle()

        for s in self.specs:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, s.body_name)
            if bid < 0:
                raise KeyError(f"tactile body 不存在: {s.body_name}")
            self._body_id[s.name] = bid

        if not self.enabled:
            return

        self._renderer = None
        self._fallback = False
        tacto_root = _ensure_tacto_on_path(repo_root)

        # 按环境尝试 OpenGL 后端：默认(pyglet) → egl → osmesa
        platforms = [None, "egl", "osmesa"]
        last_err: Exception | None = None
        for plat in platforms:
            if plat is None:
                os.environ.pop("PYOPENGL_PLATFORM", None)
            else:
                os.environ["PYOPENGL_PLATFORM"] = plat
            try:
                # 清掉上次失败留下的 GL/渲染模块
                for mod in list(sys.modules):
                    if (
                        mod == "tacto"
                        or mod.startswith("tacto.")
                        or mod == "pyrender"
                        or mod.startswith("pyrender.")
                        or mod == "OpenGL"
                        or mod.startswith("OpenGL.")
                    ):
                        del sys.modules[mod]
                Renderer, conf_path = _load_tacto_renderer(tacto_root)
                self._renderer = Renderer(
                    width=self.width,
                    height=self.height,
                    background=None,
                    config_path=conf_path,
                )
                print(f"[tacto] Renderer OK (PYOPENGL_PLATFORM={plat or 'default'})")
                last_err = None
                break
            except Exception as exc:
                last_err = exc
                self._renderer = None
                continue

        if self._renderer is None:
            print(f"[tacto] OpenGL 渲染不可用，使用深度图回退显示: {last_err}")
            self._fallback = True

        if self.show_window:
            try:
                import cv2

                self._cv2 = cv2
            except ImportError:
                print("[tacto] 未安装 opencv，无法弹窗；uv sync --extra tacto")
                self.show_window = False

    @staticmethod
    def default_arm6a_specs() -> list[TactileSensorSpec]:
        """左右臂各两指内侧垫（法向互指）。"""
        specs: list[TactileSensorSpec] = []
        for pref in ("L_", "R_"):
            # left finger pad：局部 -Y 朝向夹缝；right finger：+Y
            specs.append(
                TactileSensorSpec(
                    name=f"{pref}digit_left",
                    body_name=f"{pref}arm_gripper_left_link",
                    local_origin=(0.028, -0.014, 0.0),
                    local_normal=(0.0, -1.0, 0.0),
                )
            )
            specs.append(
                TactileSensorSpec(
                    name=f"{pref}digit_right",
                    body_name=f"{pref}arm_gripper_right_link",
                    local_origin=(0.028, 0.014, 0.0),
                    local_normal=(0.0, 1.0, 0.0),
                )
            )
        return specs

    def _depthmap_from_rays(self, spec: TactileSensorSpec) -> tuple[np.ndarray, bool]:
        """从指垫平面沿法向打网格射线 → 深度图（接触越深值越大）。"""
        bid = self._body_id[spec.name]
        xpos = self.data.xpos[bid].copy()
        xmat = self.data.xmat[bid].reshape(3, 3).copy()
        origin_l = np.asarray(spec.local_origin, dtype=np.float64)
        normal_l = np.asarray(spec.local_normal, dtype=np.float64)
        normal_l /= max(float(np.linalg.norm(normal_l)), 1e-8)
        t1_l = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        t1_l = t1_l - normal_l * float(np.dot(t1_l, normal_l))
        if float(np.linalg.norm(t1_l)) < 1e-6:
            t1_l = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            t1_l = t1_l - normal_l * float(np.dot(t1_l, normal_l))
        t1_l /= max(float(np.linalg.norm(t1_l)), 1e-8)
        t2_l = np.cross(normal_l, t1_l)
        t2_l /= max(float(np.linalg.norm(t2_l)), 1e-8)

        origin_w = xpos + xmat @ origin_l
        normal_w = xmat @ normal_l
        t1_w = xmat @ t1_l
        t2_w = xmat @ t2_l

        # 粗网格射线，再放大到 DIGIT 分辨率（性能）
        gh, gw = 32, 24
        us = np.linspace(-spec.gel_half_w, spec.gel_half_w, gw)
        vs = np.linspace(-spec.gel_half_h, spec.gel_half_h, gh)
        depth = np.zeros((gh, gw), dtype=np.float64)
        hit_any = False
        geomid = np.zeros(1, dtype=np.int32)
        max_d = float(spec.max_depth_m)

        for yi, v in enumerate(vs):
            for xi, u in enumerate(us):
                pnt = origin_w + u * t1_w + v * t2_w
                dist = mujoco.mj_ray(
                    self.model,
                    self.data,
                    pnt,
                    normal_w,
                    None,
                    True,
                    bid,
                    geomid,
                )
                if dist >= 0.0 and dist < max_d:
                    depth[yi, xi] = max_d - float(dist)
                    hit_any = True

        if hit_any:
            try:
                from scipy import ndimage

                depth = ndimage.gaussian_filter(depth, sigma=0.7)
            except Exception:
                pass
        # 放大到 renderer 尺寸
        try:
            import cv2

            depth = cv2.resize(depth, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
        except Exception:
            ys = np.linspace(0, gh - 1, self.height)
            xs = np.linspace(0, gw - 1, self.width)
            yi = np.clip(ys.astype(int), 0, gh - 1)
            xi = np.clip(xs.astype(int), 0, gw - 1)
            depth = depth[np.ix_(yi, xi)]
        return depth, hit_any

    def render(self) -> TactoTouchBundle:
        self._tick += 1
        if not self.enabled:
            return self._last
        if (self._tick % self.every_n) != 0:
            return self._last

        out = TactoTouchBundle()
        for spec in self.specs:
            depthmap, contacted = self._depthmap_from_rays(spec)
            out.contact[spec.name] = contacted
            if self._renderer is not None:
                color, depth = self._renderer.render_from_depth(
                    depthmap if contacted else np.zeros_like(depthmap),
                    noise=bool(contacted),
                    calibration=False,
                    scale=1.0,
                )
                out.colors[spec.name] = np.asarray(color)
                out.depths[spec.name] = np.asarray(depth)
            else:
                # 回退：把深度映射成伪 DIGIT 色图（无 OpenGL 时仍可见接触）
                out.colors[spec.name] = self._fallback_color(depthmap)
                out.depths[spec.name] = depthmap

        self._last = out
        if self.show_window and self._cv2 is not None:
            self._show(out)
        return out

    @staticmethod
    def _fallback_color(depthmap: np.ndarray) -> np.ndarray:
        d = np.asarray(depthmap, dtype=np.float64)
        d = d / max(float(d.max()), 1e-6)
        h, w = d.shape
        # 凝胶底色 + 接触亮斑
        base = np.zeros((h, w, 3), dtype=np.float64)
        base[..., 0] = 40 + 80 * d
        base[..., 1] = 90 + 140 * d
        base[..., 2] = 120 + 100 * d
        return np.clip(base, 0, 255).astype(np.uint8)

    def _show(self, bundle: TactoTouchBundle) -> None:
        assert self._cv2 is not None
        tiles = []
        for name in sorted(bundle.colors.keys()):
            img = bundle.colors[name]
            if img.dtype != np.uint8:
                img = np.clip(img, 0, 255).astype(np.uint8)
            # BGR for cv2
            bgr = self._cv2.cvtColor(img, self._cv2.COLOR_RGB2BGR)
            label = name.replace("digit_", "")
            self._cv2.putText(
                bgr, label, (4, 14), self._cv2.FONT_HERSHEY_SIMPLEX, 0.4, (20, 220, 20), 1
            )
            tiles.append(bgr)
        if not tiles:
            return
        # 2x2 or 1xN
        if len(tiles) == 4:
            row0 = np.hstack(tiles[0:2])
            row1 = np.hstack(tiles[2:4])
            canvas = np.vstack([row0, row1])
        else:
            canvas = np.hstack(tiles)
        self._cv2.imshow("TACTO DIGIT (MuJoCo)", canvas)
        self._cv2.waitKey(1)

    def close(self) -> None:
        if self._cv2 is not None:
            try:
                self._cv2.destroyAllWindows()
            except Exception:
                pass
        self._renderer = None
