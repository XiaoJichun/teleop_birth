"""场景脚本执行器：读 GenerateConfig / PlaceConfig / CollectConfig 并运行。"""

from __future__ import annotations

from pathlib import Path

import mujoco
from omegaconf import OmegaConf

from teleop_birth.scene.config import CollectConfig, GenerateConfig, PlaceConfig
from teleop_birth.scene.collection.auto_collector import AutoCollector
from teleop_birth.scene.placement.pose_controller import PropPoseController
from teleop_birth.scene.placement.randomizer import LayoutRandomizer
from teleop_birth.scene.props.catalog import PropCatalog
from teleop_birth.scene.props.spawner import PropSpawner
from teleop_birth.robot.arm.ur5e.builder import build_dual_ur5e_2f85


class SceneOpsRunner:
    """类封装：生成 / 摆放 / 采集（交互与 CLI 脚本共用）。"""

    def __init__(self, project_root: Path) -> None:
        self.project_root = Path(project_root)
        self.catalog = PropCatalog(self.project_root / "configs" / "scene" / "props")

    def _load_backend_scene(self, prop_names: list[str]) -> tuple[dict, Path]:
        backend = OmegaConf.to_container(
            OmegaConf.load(self.project_root / "configs/device/backend/mujoco_dual_ur.yaml"),
            resolve=True,
        )
        paths = OmegaConf.to_container(
            OmegaConf.load(self.project_root / "configs/device/paths/default.yaml"),
            resolve=True,
        )
        assert isinstance(backend, dict) and isinstance(paths, dict)
        scene = dict(backend["scene"])
        specs = self.catalog.resolve_many(prop_names)
        scene["props"] = [p.to_dict() for p in specs]
        scene["project_root"] = str(self.project_root)
        menagerie = self.project_root / str(paths.get("menagerie", "third_party/mujoco_menagerie"))
        return scene, menagerie

    def run_generate(self, cfg: GenerateConfig) -> None:
        scene, menagerie = self._load_backend_scene(cfg.props)
        model, name_map = build_dual_ur5e_2f85(menagerie, scene)
        data = mujoco.MjData(model)
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
        for _ in range(int(cfg.settle_steps)):
            mujoco.mj_step(model, data)

        specs = self.catalog.resolve_many(cfg.props)
        print("[generate_props] 已生成:")
        for p in PropSpawner.summarize(specs):
            print(f"  - {p['name']}  kind={p['kind']}  rgba={p['rgba']}  pos={p['pos']}")
        print(f"  name_map.prop_names = {name_map.get('prop_names')}")

        if cfg.render:
            try:
                from mujoco import viewer

                print("[generate_props] 打开 viewer（关闭窗口结束）")
                viewer.launch(model, data)
            except Exception as exc:
                print(f"[generate_props] viewer 不可用: {exc}")

    def run_place(self, cfg: PlaceConfig) -> None:
        import numpy as np

        scene, menagerie = self._load_backend_scene(cfg.props)
        model, _ = build_dual_ur5e_2f85(menagerie, scene)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        ctrl = PropPoseController(model, data)
        ctrl.discover(cfg.props)
        specs = self.catalog.resolve_many(cfg.props)
        rnd = LayoutRandomizer.from_table_cfg(
            dict(scene["table"]), float(scene["base_height"]), seed=cfg.seed
        )

        for i in range(int(cfg.repeats)):
            rnd.rng = np.random.default_rng(cfg.seed + i * 17)
            layout = rnd.apply(ctrl, specs)
            for _ in range(30):
                mujoco.mj_step(model, data)
            print(f"[place_props] layout#{i}:")
            for name, item in layout.items():
                print(f"  {name}: pos={item['pos']}")

        if cfg.render:
            try:
                from mujoco import viewer

                viewer.launch(model, data)
            except Exception as exc:
                print(f"[place_props] viewer 不可用: {exc}")

    def run_collect(self, cfg: CollectConfig) -> None:
        collector = AutoCollector.from_config(self.project_root, cfg)
        try:
            results = collector.run()
        finally:
            collector.close()
        ok_n = sum(1 for r in results if r.success)
        print(f"[collect] done: {ok_n}/{len(results)} success")
        for r in results:
            print(f"  ep{r.episode}: success={r.success} → {r.path}")
