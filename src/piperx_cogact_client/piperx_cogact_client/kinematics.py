"""复用驱动的 C++ FK/IK，将世界系 TCP 轨迹转换为带零偏标定的关节目标。"""
from __future__ import annotations

import math
import time
from typing import Any, Mapping, Sequence

import numpy as np
from piper_x_driver import (
    ArmGravityModel,
    FleetConfig,
    IkResult,
    default_config_path,
    forward_kinematics,
    inverse_kinematics,
)

from .cogact_api import ActionChunk, inverse_rigid, pose_xyzw_to_matrix
from .config import ArmConfig, IkConfig

# TCP 是闭合手指内侧接触面的中心；带力传感器的夹爪沿 Link6 的 +Z 多出 29.8 mm。
DEFAULT_LINK6_TO_TCP_Z_M: dict[str, float] = {
    "gripper": 0.13503,
    "gripper_with_wrench": 0.16483,
    "teaching_handle": 0.0795,
}


class DriverKinematics:
    """Provides per-arm calibrated FK and 6-DOF C++ IK backed by piper_x_driver."""

    def __init__(self, arm_configs: Sequence[ArmConfig]):
        self.gravity_models: dict[str, ArmGravityModel | None] = {}
        self.end_effectors: dict[str, str] = {}
        fleet = None
        try:
            fleet = FleetConfig(default_config_path())
        except Exception:
            fleet = None
        for cfg in arm_configs:
            if fleet is not None:
                try:
                    drv_arm = fleet.arm(cfg.id)
                    self.gravity_models[cfg.id] = fleet.gravity_model(drv_arm)
                    self.end_effectors[cfg.id] = drv_arm.end_effector
                    continue
                except Exception:
                    pass
            self.gravity_models[cfg.id] = None
            self.end_effectors[cfg.id] = "gripper_with_wrench"

    def default_flange_to_tcp_z_m(self, arm_id: str) -> float:
        eff = self.end_effectors.get(arm_id, "gripper_with_wrench")
        return DEFAULT_LINK6_TO_TCP_Z_M.get(eff, 0.16483)

    def fk_link6(self, arm_id: str, joints_rad: Sequence[float]) -> np.ndarray:
        model = self.gravity_models.get(arm_id)
        if model is not None:
            return model.forward_kinematics(joints_rad)
        return forward_kinematics(joints_rad)

    def solve_ik_link6(
        self,
        arm_id: str,
        t_base_link6: np.ndarray,
        q_seed: Sequence[float],
        ik_cfg: IkConfig,
    ) -> IkResult:
        model = self.gravity_models.get(arm_id)
        fn = model.inverse_kinematics if model is not None else inverse_kinematics
        res = fn(
            t_base_link6,
            q_seed=q_seed,
            damping=ik_cfg.damping,
            max_iters=ik_cfg.max_iters,
            pos_tol_m=ik_cfg.pos_tol_m,
            rot_tol_rad=ik_cfg.rot_tol_rad,
            return_result=True,
        )
        assert isinstance(res, IkResult)
        return res

    def resolve_chunk_ik(
        self,
        chunk: ActionChunk,
        *,
        arm_configs: Sequence[ArmConfig],
        calibration_data: Mapping[str, Any],
        seed_joints_by_arm: Mapping[str, Sequence[float]],
        ik_cfg: IkConfig,
    ) -> dict[str, Any]:
        """Solve continuous 6-DOF joint targets across all steps in an ActionChunk."""
        t0 = time.perf_counter()
        all_converged = True
        max_pos_mm = 0.0
        max_rot_deg = 0.0
        solved_steps = 0

        for arm_cfg in arm_configs:
            role = arm_cfg.role
            w_b = calibration_data["world_from_base"][role]
            f_tcp = calibration_data["flange_from_tcp"][role]
            q_seed = tuple(float(v) for v in seed_joints_by_arm[arm_cfg.id][:6])
            transforms = None

            for step in chunk.steps:
                target = step.by_role(role)
                if target.joint_positions_rad is not None:
                    q_seed = tuple(float(v) for v in target.joint_positions_rad[:6])
                    continue
                if target.tcp_pose_world_xyzw is None:
                    raise ValueError(f"Step {step.step_index} ({role}) contains neither joint nor TCP target")
                # 静态外参每臂每批只求逆一次，避免每步矩阵与四元数的往返转换。
                if transforms is None:
                    transforms = (np.asarray(inverse_rigid(w_b)), np.asarray(inverse_rigid(f_tcp)))
                base_from_world, tcp_from_link6 = transforms
                t_base_link6 = base_from_world @ pose_xyzw_to_matrix(target.tcp_pose_world_xyzw) @ tcp_from_link6
                res = self.solve_ik_link6(arm_cfg.id, t_base_link6, q_seed, ik_cfg)
                pos_mm = res.position_error_m * 1000.0
                rot_deg = math.degrees(res.orientation_error_rad)
                target.joint_positions_rad = list(res.joints)
                target.ik_converged = bool(res.converged)
                target.ik_pos_err_mm = round(pos_mm, 3)
                target.ik_rot_err_deg = round(rot_deg, 3)
                # 首步使用实测关节角，后续沿用上一步解以保持同一 IK 分支。
                q_seed = res.joints
                all_converged = all_converged and bool(res.converged)
                max_pos_mm = max(max_pos_mm, pos_mm)
                max_rot_deg = max(max_rot_deg, rot_deg)
                solved_steps += 1

        solve_ms = (time.perf_counter() - t0) * 1000.0
        summary = {
            "solved_targets": solved_steps,
            "solve_ms": round(solve_ms, 2),
            "all_converged": all_converged,
            "max_pos_err_mm": round(max_pos_mm, 3),
            "max_rot_deg": round(max_rot_deg, 3),
        }
        chunk.ik_summary = summary
        return summary

