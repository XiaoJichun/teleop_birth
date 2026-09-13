"""cuRobo GPU IK adapter — https://curobo.org/

Optional heavy dependency (CUDA + torch). Uses bundled ur5e.yml when available.
Teleop targets TCP; we convert to tool0/flange in the arm base frame like SSIK.
"""

from __future__ import annotations

from typing import Any, Optional

import numpy as np

from teleop_birth.backend.ik.frames import (
    read_arm_q,
    tcp_des_to_flange_in_base,
    write_arm_q,
)


class CuroboIKSolver:
    def __init__(
        self,
        ik_cfg: dict[str, Any],
        *,
        model,
        data,
        fallback: Optional[Any] = None,
    ) -> None:
        try:
            import torch
            from curobo.types.base import TensorDeviceType
            from curobo.types.math import Pose
            from curobo.types.robot import RobotConfig
            from curobo.util_file import get_robot_configs_path, join_path, load_yaml
            from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
        except ImportError as exc:
            raise ImportError(
                "未安装 cuRobo。请按 https://curobo.org/ 安装（需 CUDA），例如:\n"
                "  # 参见官方 Installation；勿与本仓库默认依赖混装强绑\n"
                "  uv pip install torch  # + 按文档装 curobo\n"
                f"原始错误: {exc}"
            ) from exc

        cc = dict(ik_cfg.get("curobo") or {})
        robot_yml = str(cc.get("robot_yml", "ur5e.yml"))
        yml_path = cc.get("robot_yml_path")
        if yml_path:
            cfg_file = load_yaml(str(yml_path))
        else:
            cfg_file = load_yaml(join_path(get_robot_configs_path(), robot_yml))

        tensor_args = TensorDeviceType()
        if "robot_cfg" in cfg_file:
            robot_cfg = RobotConfig.from_dict(cfg_file["robot_cfg"], tensor_args)
        else:
            kin = cfg_file.get("kinematics") or cfg_file
            robot_cfg = RobotConfig.from_basic(
                kin["urdf_path"],
                kin["base_link"],
                kin["ee_link"],
                tensor_args,
            )

        ik_config = IKSolverConfig.load_from_robot_config(
            robot_cfg,
            None,
            rotation_threshold=float(cc.get("rotation_threshold", 0.05)),
            position_threshold=float(cc.get("position_threshold", 0.005)),
            num_seeds=int(cc.get("num_seeds", 20)),
            self_collision_check=bool(cc.get("self_collision_check", False)),
            self_collision_opt=bool(cc.get("self_collision_opt", False)),
            tensor_args=tensor_args,
            use_cuda_graph=bool(cc.get("use_cuda_graph", True)),
        )
        self._ik = IKSolver(ik_config)
        self._Pose = Pose
        self._torch = torch
        self.model = model
        self.data = data
        self.fallback = fallback
        self._device = tensor_args.device
        print(f"[ik] cuRobo IKSolver ready (robot_yml={robot_yml}, device={self._device})")

    def solve(self, handles, pose_des: np.ndarray, *, position_only: bool) -> bool:
        T = tcp_des_to_flange_in_base(
            model=self.model,
            data=self.data,
            base_bid=handles.base_bid,
            flange_sid=handles.flange_sid,
            tcp_sid=handles.tcp_sid,
            T_flange_tcp=handles.T_flange_tcp,
            pose_des=pose_des,
            position_only=position_only,
        )
        # cuRobo Pose: position + quaternion **wxyz**
        from teleop_birth.schema import matrix_to_pose

        pose7 = matrix_to_pose(T)  # xyz + xyzw
        pos = self._torch.tensor(pose7[:3], device=self._device, dtype=self._torch.float32).view(1, 3)
        q_xyzw = pose7[3:]
        q_wxyz = self._torch.tensor(
            [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]],
            device=self._device,
            dtype=self._torch.float32,
        ).view(1, 4)
        goal = self._Pose(pos, q_wxyz)

        q_seed = read_arm_q(self.model, self.data, handles.arm_aids)
        seed = self._torch.tensor(q_seed, device=self._device, dtype=self._torch.float32).view(1, 1, -1)

        try:
            result = self._ik.solve_single(goal, seed_config=seed)
            ok = bool(result.success.item()) if hasattr(result.success, "item") else bool(result.success)
            if not ok:
                if self.fallback is not None:
                    return self.fallback.solve(handles, pose_des, position_only=position_only)
                return False
            q = result.solution.detach().cpu().numpy().reshape(-1)[: len(handles.arm_aids)]
            write_arm_q(self.model, self.data, handles.arm_aids, q)
            return True
        except Exception as exc:  # noqa: BLE001
            if self.fallback is not None:
                return self.fallback.solve(handles, pose_des, position_only=position_only)
            raise RuntimeError(f"curobo.solve_single 失败: {exc}") from exc
