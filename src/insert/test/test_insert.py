"""Focused verification for InsertAnything ONNX policy deployment and CAN4 dashboard workflow."""

import json
import math
import threading
from urllib.request import Request, urlopen

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import WrenchStamped
from piper_x_driver.gravity import _fast_fk, end_effector_wrench, geometric_jacobian
from piper_x_interfaces.msg import GripperCommand, MitCommand
from sensor_msgs.msg import JointState

from insert.policy import (
    DEFAULT_TIP_LENGTH_M,
    DEFAULT_TIP_OFFSET_M,
    DRIVER_WRENCH_DT_S,
    TRAIN_WRENCH_DT_S,
    TRAIN_WRENCH_FILTER_TAU_S,
    WRENCH_FILTER_TAU_S,
    GoalPose,
    InsertAnythingPolicy,
    WrenchFilter,
    actor_observation,
    canonical_quat,
    default_onnx_path,
    flange_wrench_to_tip_wrench,
    forward_tip_pose,
    quat_inv,
    quat_mul,
    rotate,
    rotvec_quat,
    solve_tip_ik,
)
from insert.web import JOINT_NAMES, InsertDashboardNode, create_http_server


def test_transforms_wrench_and_onnx_policy_convergence():
    """Verify ONNX path in insert package, 200Hz equivalent filter, tip wrench, and downward convergence."""
    onnx_file = default_onnx_path()
    assert onnx_file.is_file() and onnx_file.parent.name == 'insert'

    assert DEFAULT_TIP_LENGTH_M == pytest.approx(0.195)
    assert DEFAULT_TIP_OFFSET_M == pytest.approx((0.0, 0.0, 0.195))
    alpha_1khz = 1.0 - math.exp(-TRAIN_WRENCH_DT_S / TRAIN_WRENCH_FILTER_TAU_S)
    alpha_200hz = 1.0 - math.exp(-DRIVER_WRENCH_DT_S / WRENCH_FILTER_TAU_S)
    assert (alpha_200hz / DRIVER_WRENCH_DT_S) == pytest.approx(alpha_1khz / TRAIN_WRENCH_DT_S, rel=1e-9)

    p_ht = np.array([0.004, -0.003, 0.015], dtype=np.float64)
    q_ht = rotvec_quat([0.02, -0.015, 0.04])
    wrench_t = np.array([1.5, -2.0, 4.0, 0.05, -0.04, 0.02], dtype=np.float64)
    obs = actor_observation(p_ht, q_ht, (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), wrench_t)
    assert obs.shape == (1, 13) and obs.dtype == np.float32
    assert obs[0, :3] == pytest.approx(-p_ht, abs=1e-6)
    assert obs[0, 3:7] == pytest.approx(canonical_quat(quat_inv(q_ht)), abs=1e-6)

    tip_len = DEFAULT_TIP_LENGTH_M
    wf = WrenchFilter()
    wf.update([10.0, -6.0, 0.0, 6.0 * tip_len, 10.0 * tip_len, 0.0], timestamp=1.0)
    tip_w = wf.tip_wrench(tip_length_m=tip_len)
    assert tip_w[:3] == pytest.approx([10.0, 6.0, 0.0], abs=1e-6)
    assert tip_w[3:6] == pytest.approx([0.0, 0.0, 0.0], abs=1e-6)

    q_down = (-1.316, math.radians(110.0), math.radians(-90.0), math.radians(74.6), 0.0, 0.0)
    j_tip = geometric_jacobian(q_down, tool_offset_m=(0.0, 0.0, tip_len), frame='gripper')
    true_tip_force = np.array([2.0, -1.0, -5.0, 0.0, 0.0, 0.0], dtype=np.float64)
    w_flange = end_effector_wrench(
        j_tip.T @ true_tip_force, q_down, tool_offset_m=(0.0, 0.0, 0.0), damping=1e-2, frame='gripper',
    )
    w_tip_rec = flange_wrench_to_tip_wrench(w_flange, q_down, tip_length_m=tip_len)
    assert w_tip_rec[:3] == pytest.approx(true_tip_force[:3], abs=1e-2)
    assert w_tip_rec[3:6] == pytest.approx([0.0, 0.0, 0.0], abs=1e-3)

    policy = InsertAnythingPolicy(onnx_file)
    p_bh, q_bh = forward_tip_pose(q_down, tip_length_m=tip_len)
    goal = GoalPose(
        joints=q_down,
        p_bh=tuple(float(v) for v in p_bh),
        q_bh=tuple(float(v) for v in q_bh),
        tip_length_m=tip_len,
        recorded_at=0.0,
    )
    p_start_b = p_bh + np.array([0.003, -0.002, 0.020])
    q_curr = np.array(solve_tip_ik(q_down, p_start_b, q_bh, tip_length_m=tip_len, max_iters=20))
    droop = np.array([0.0, 0.006, -0.008, -0.015, 0.0, 0.0])
    kp = np.array([45.0, 45.0, 45.0, 15.0, 10.0, 6.0])
    q_servo = q_curr.copy()
    wf.reset()

    for _ in range(80):
        w_t = wf.tip_wrench(tip_length_m=tip_len)
        step_res = policy.step(q_curr, goal, w_t, tip_length_m=tip_len)
        q_cmd = np.array(step_res.q_cmd)
        for _ in range(4):
            q_curr = q_curr + 0.25 * (q_cmd + droop - q_curr)
            q_servo = q_servo + 0.25 * (q_cmd - q_servo)
            tau_drv = kp * (q_curr - q_cmd)
            w_f = end_effector_wrench(tau_drv, q_curr, tool_offset_m=(0.0, 0.0, 0.0), damping=1e-2, frame='gripper')
            w_tip_g = flange_wrench_to_tip_wrench(
                w_f, q_curr, tip_length_m=tip_len, q_cmd=q_cmd, q_servo=q_servo, kp=kp,
            )
            wf.update(w_tip_g, dt=0.005, already_at_tip=True)

    _, _, _, rots = _fast_fk(q_curr, (0.0, 0.0, tip_len))
    assert np.asarray(rots[5])[2, 2] < -0.98
    assert step_res.p_ht[2] < 0.002
    assert float(np.linalg.norm(step_res.p_ht[:2])) < 0.003


def test_dashboard_can4_gripper_record_and_policy_workflow():
    """Verify end-to-end dashboard workflow on CAN4 (/arm_4): gripper switch, drag, record goal, and execute policy."""
    rclpy.init()
    node = helper = server = None
    try:
        node = InsertDashboardNode(arm_selector='can4')
        assert node.arm_id == 'arm_4' and node.arm_namespace == '/arm_4' and node.can_label == 'can4'

        helper = rclpy.create_node('test_insert_helper')
        received_mit: list[MitCommand] = []
        received_grip: list[GripperCommand] = []
        helper.create_subscription(MitCommand, '/arm_4/command/mit', received_mit.append, 10)
        helper.create_subscription(GripperCommand, '/arm_4/command/gripper', received_grip.append, 10)

        server = create_http_server(node, host='127.0.0.1', port=0)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        base_url = f'http://127.0.0.1:{server.server_port}'

        def post(path: str, payload: dict) -> dict:
            req = Request(
                base_url + path,
                data=json.dumps(payload).encode('utf-8'),
                headers={'Content-Type': 'application/json'},
                method='POST',
            )
            with urlopen(req, timeout=2.0) as resp:
                return json.loads(resp.read().decode('utf-8'))

        q_inserted = [0.0, 0.85, -1.35, 0.0, 0.50, 0.0]
        node._on_joints(JointState(name=list(JOINT_NAMES), position=[*q_inserted, 0.012]))
        w_msg = WrenchStamped()
        w_msg.wrench.force.z = -1.5
        node._on_wrench(w_msg)

        res_close = post('/api/gripper', {'action': 'close', 'width_m': 0.0, 'effort': 2.5})
        assert res_close['ok'] is True and res_close['gripper_state'] == 'closed'

        res_drag = post('/api/drag', {})
        assert res_drag['mode'] == 'drag'
        node._tick()
        for _ in range(5):
            rclpy.spin_once(helper, timeout_sec=0.01)
        assert received_grip and received_grip[-1].effort == pytest.approx(2.5)
        assert received_mit and list(received_mit[-1].kp) == [0.0] * 6

        res_goal = post('/api/goal/record', {'tip_offset_cm': [0.0, 0.0, 19.5]})
        assert res_goal['ok'] is True
        assert res_goal['goal']['joints_rad'] == pytest.approx(q_inserted)
        assert res_goal['goal']['tip_offset_cm'] == pytest.approx([0.0, 0.0, 19.5])

        p_bh = np.array(res_goal['goal']['position_m'])
        q_bh = np.array(res_goal['goal']['quaternion_wxyz'])
        p_lifted = p_bh + rotate(q_bh, [0.002, -0.001, 0.012])
        q_lifted = solve_tip_ik(q_inserted, p_lifted, q_bh, tip_length_m=0.195, max_iters=8)
        node._on_joints(JointState(name=list(JOINT_NAMES), position=[*q_lifted, 0.012]))

        res_start = post('/api/policy/start', {})
        assert res_start['mode'] == 'policy'
        node._tick()
        for _ in range(5):
            rclpy.spin_once(helper, timeout_sec=0.01)

        with urlopen(base_url + '/api/status', timeout=2.0) as resp:
            status = json.loads(resp.read().decode('utf-8'))
        assert status['policy']['mode'] == 'policy' and status['policy']['steps'] == 1
        assert status['arm']['hole_relative']['p_ht_mm'][2] == pytest.approx(12.0, abs=0.2)
        assert list(received_mit[-1].kp) == list(node.policy_kp)
        assert status['policy']['last_step']['p_target_h_mm'][2] < status['arm']['hole_relative']['p_ht_mm'][2]

        res_stop = post('/api/policy/stop', {'mode': 'drag'})
        assert res_stop['mode'] == 'drag'
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
        if helper is not None:
            helper.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
