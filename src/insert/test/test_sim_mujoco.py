"""Focused MuJoCo tests for PiPER-X + PX6D-75A wrist force/torque sensor simulation."""

import math
import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')

from piper_x_driver.gravity import forward_kinematics

from insert.policy import forward_tip_pose, quat_to_mat
from insert.sim_mujoco import PiperWrenchInsertSim, SimSceneConfig, run_episode


def test_wrench_urdf_fk_and_paxini_sensor_zero_moment_transport():
    sim = PiperWrenchInsertSim(arm_gravity_mode='matched', gravity_compensation_mode='analytic')

    # 1. Verify exact 6-DOF FK & frame alignment between MuJoCo URDF and PiPER-X SDK across poses
    for q in ([0.0, 0.85, -1.35, 0.0, 0.50, 0.0], [0.2, 1.1, -1.2, -0.3, 0.4, 0.6], sim.q_inserted):
        sim.set_sdk_joints(q)
        p_mj, q_mj = sim.true_tip_pose_in_base()
        p_fk, q_fk = forward_tip_pose(q, tip_offset_m=sim.scene_cfg.tip_offset_m, gravity_model=sim.gravity_model)
        assert p_mj == pytest.approx(p_fk, abs=1e-7)
        assert quat_to_mat(q_mj) == pytest.approx(quat_to_mat(q_fk), abs=1e-7)

        t_ft_sdk = forward_kinematics(q, (0.0, 0.0, 0.0195))
        assert sim.data.site_xpos[sim.ft_site_id] == pytest.approx(t_ft_sdk[:3, 3], abs=1e-7)
        assert sim.data.site_xmat[sim.ft_site_id].reshape(3, 3) == pytest.approx(t_ft_sdk[:3, :3], abs=1e-7)

    # 2. Verify dynamic downstream gravity elimination and pure-force zero-moment transport at the 19.5 cm tip
    sim.reset_episode(initial_offset_h=(0.0, 0.0, 0.025), tare_at_start=False)
    plug_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, 'grasped_plug')
    f_t_applied = np.array([4.0, -3.0, 5.0], dtype=np.float64)
    sim.paxini_filter.reset()
    for _ in range(1200):
        _, q_bt = sim.true_tip_pose_in_base()
        f_world = quat_to_mat(q_bt) @ f_t_applied
        r_com_to_tip = sim.data.site_xpos[sim.tip_site_id] - sim.data.xipos[plug_id]
        sim.data.xfrc_applied[plug_id, :3] = f_world
        sim.data.xfrc_applied[plug_id, 3:] = np.cross(r_com_to_tip, f_world)
        sim.data.ctrl[:6] = sim.compute_joint_torques()
        mujoco.mj_step(sim.model, sim.data)
        sim.ingest_sensor_substep()
    w_tip = sim.paxini_filter.tip_wrench(tip_offset_m=sim.scene_cfg.tip_offset_m)
    assert w_tip[:3] == pytest.approx(f_t_applied, abs=0.02)
    assert w_tip[3:6] == pytest.approx([0.0, 0.0, 0.0], abs=0.005)


def test_joint_mit_tilt_divergence_vs_cartesian_tip_impedance_convergence():
    init_p = (0.005, -0.004, 0.008)
    init_r = (math.radians(2.0), math.radians(-1.5), math.radians(2.5))

    # Joint-space MIT PD fails to insert due to off-diagonal tip stiffness coupling across the tool lever arm
    sim_joint = PiperWrenchInsertSim(
        scene_cfg=SimSceneConfig(shape='hexagon', clearance_m=0.0005),
        impedance_mode='joint_mit',
        gravity_compensation_mode='analytic',
        arm_gravity_mode='matched',
    )
    rep_joint = run_episode(sim_joint, steps=160, initial_offset_h=init_p, initial_rotvec_h=init_r)
    assert not rep_joint['succeeded']
    assert rep_joint['max_depth_mm'] < 1.0
    assert rep_joint['max_tilt_deg'] > 12.0

    # Same actor.onnx + same URDF arm & PX6D-75A sensor with 200 Hz decoupled tip Cartesian impedance
    # and 10 ms CAN feedback delay inserts cleanly without oscillation across variable tool offsets
    for tip_off in ((0.0, 0.0, 0.195), (0.010, -0.006, 0.150)):
        sim_cart = PiperWrenchInsertSim(
            scene_cfg=SimSceneConfig(shape='hexagon', clearance_m=0.0005, tip_offset_m=tip_off),
            impedance_mode='cartesian_tip',
            inner_control_hz=200,
            can_delay_substeps=10,
            gravity_compensation_mode='analytic',
            arm_gravity_mode='matched',
        )
        rep_cart = run_episode(sim_cart, steps=260, initial_offset_h=init_p, initial_rotvec_h=init_r)
        assert rep_cart['succeeded']
        assert rep_cart['final_tilt_deg'] < 2.5
        assert rep_cart['max_depth_mm'] >= 14.5
        assert rep_cart['max_force_n'] < 15.0
