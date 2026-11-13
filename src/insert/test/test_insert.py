"""Focused verification for InsertAnything ONNX policy deployment and CAN4 dashboard workflow."""

import json
import math
import threading
import time
from urllib.request import Request, urlopen
from unittest.mock import patch

import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import WrenchStamped
from piper_x_driver.gravity import _fast_fk, end_effector_wrench, geometric_jacobian
from piper_x_interfaces.msg import GripperCommand, MitCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from insert.policy import (
    DEFAULT_TIP_LENGTH_M,
    DEFAULT_TIP_OFFSET_M,
    JOINT_LOWER,
    JOINT_UPPER,
    PAXINI_SENSOR_OFFSET_M,
    TRAIN_WRENCH_FILTER_TAU_S,
    WRENCH_FILTER_TAU_S,
    GoalPose,
    InsertAnythingPolicy,
    RuleBasedInsertPolicy,
    WrenchFilter,
    actor_observation,
    canonical_quat,
    default_onnx_path,
    flange_wrench_to_tip_wrench,
    forward_tip_pose,
    hole_target_to_base,
    quat_mul,
    quat_inv,
    relative_pose_in_hole,
    rotate,
    rotvec_quat,
    solve_tip_ik,
)
from insert.web import JOINT_NAMES, InsertDashboardNode, create_http_server
from insert.agent_flow.geometry import (
    BoardSpec,
    calibrate_external_camera,
    compute_alignment_plan,
    detect_charuco_pose,
    invert_se3,
    project_base_points,
    rt_to_mat,
    solve_axis_from_two_views,
    triangulate_two_rays,
    wrist_camera_pose_in_base,
)
from insert.agent_flow.server import AgentInsertNode, create_agent_http_server


def test_wrench_filter_preserves_response_across_feedback_rates():
    expected = 1.0 - math.exp(-0.1 / TRAIN_WRENCH_FILTER_TAU_S)
    for hz in (30, 200, 500, 1000):
        filt = WrenchFilter()
        filt.update(np.zeros(6), timestamp=0.0)
        for sample in range(1, int(hz * 0.1) + 1):
            filt.update(np.ones(6), timestamp=sample / hz)
        assert filt.filtered_s == pytest.approx(np.full(6, expected), abs=1e-12)
        assert filt.alpha == pytest.approx(1.0 - math.exp(-1.0 / hz / filt.tau))

    # Switching rates and irregular intervals preserve the same elapsed-time decay.
    filt = WrenchFilter()
    filt.update(np.zeros(6), timestamp=0.0)
    for timestamp in (0.005, 0.010, 0.012, 0.015, 0.016, 0.020):
        filt.update(np.ones(6), timestamp=timestamp)
    assert filt.filtered_s == pytest.approx(np.full(6, 1.0 - math.exp(-0.020 / filt.tau)))


def test_feedback_and_stop_continue_during_policy_computation(monkeypatch):
    rclpy.init()
    node = InsertDashboardNode(namespace_override='/insert_receive_test', wrench_source='paxini')
    node._timer.cancel()
    q = (0.0, 0.85, -1.35, 0.0, 0.50, 0.0)
    node.joint_positions = q
    node.goal = node._build_goal(q, time.monotonic())
    node.control_state = {'mode': 'mit'}
    node.mode = 'policy'
    node.last_cmd_joints = q
    helper = rclpy.create_node('insert_receive_test_sensor')
    estimation_requests = []

    def set_estimation(request, response):
        estimation_requests.append(request.data)
        response.success = True
        return response

    helper.create_service(SetBool, '/insert_receive_test/set_wrench_estimation', set_estimation)
    publisher = helper.create_publisher(WrenchStamped, '/insert_receive_test/wrench_stamped', 10)
    joints = helper.create_publisher(JointState, '/insert_receive_test/joint_states_feedback', 10)
    entered, release = threading.Event(), threading.Event()
    original = node.policy.step
    sent = []
    monkeypatch.setattr(node, '_publish_mit', lambda *args: sent.append(args))

    def slow_policy(*args, **kwargs):
        entered.set()
        release.wait(timeout=3.0)
        return original(*args, **kwargs)

    monkeypatch.setattr(node.policy, 'step', slow_policy)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.add_node(helper)
    node._timer.reset()
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert entered.wait(timeout=2.0)
        deadline = time.monotonic() + 1.0
        message = WrenchStamped()
        message.wrench.force.z = 4.0
        moved = (0.01,) + q[1:]
        joint_message = JointState(name=list(JOINT_NAMES), position=[*moved, 0.0])
        while time.monotonic() < deadline:
            publisher.publish(message)
            joints.publish(joint_message)
            if node.paxini_received_at is not None and node.joint_positions == moved:
                break
            time.sleep(0.005)
        assert node.paxini_received_at is not None
        assert node.joint_positions == moved
        assert not release.is_set()
        assert sent and sent[-1][0] == q
        assert node.stop_policy('idle')['mode'] == 'idle'
        sent_at_stop = len(sent)
        for source, wanted in (('estimated', True), ('paxini', False)):
            node.update_parameters({'wrench_source': source})
            deadline = time.monotonic() + 1.0
            while (not estimation_requests or estimation_requests[-1] != wanted) and time.monotonic() < deadline:
                time.sleep(0.005)
            assert estimation_requests[-1] == wanted
        node._timer.cancel()
    finally:
        node._timer.cancel()
        release.set()
        executor.shutdown()
        thread.join(timeout=2.0)
        helper.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()
    assert len(sent) == sent_at_stop  # Neither heartbeat nor a late result sends after stop.


def test_transforms_wrench_and_onnx_policy_convergence():
    """Verify ONNX path, tip wrench, and downward convergence."""
    onnx_file = default_onnx_path()
    assert onnx_file.is_file() and onnx_file.parent.name == 'insert'

    assert DEFAULT_TIP_LENGTH_M == pytest.approx(0.195)
    assert DEFAULT_TIP_OFFSET_M == pytest.approx((0.0, 0.0, 0.195))
    assert WRENCH_FILTER_TAU_S == TRAIN_WRENCH_FILTER_TAU_S

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
    pax_lever = tip_len - PAXINI_SENSOR_OFFSET_M[2]
    wf_pax = WrenchFilter(sensor_offset_m=PAXINI_SENSOR_OFFSET_M)
    wf_pax.update([10.0, -6.0, 0.0, 6.0 * pax_lever, 10.0 * pax_lever, 0.0], timestamp=1.0)
    assert wf_pax.tip_wrench(tip_length_m=tip_len) == pytest.approx([10.0, 6.0, 0.0, 0.0, 0.0, 0.0], abs=1e-6)

    q_down = (-1.316, math.radians(110.0), math.radians(-90.0), math.radians(74.6), 0.0, 0.0)
    j_tip = geometric_jacobian(q_down, tool_offset_m=(0.0, 0.0, tip_len), frame='gripper')
    true_tip_wrench = np.array([2.0, -1.0, -5.0, 0.4, -0.25, 0.15], dtype=np.float64)
    w_flange = end_effector_wrench(
        j_tip.T @ true_tip_wrench, q_down, tool_offset_m=(0.0, 0.0, 0.0), damping=1e-2, frame='gripper',
    )
    w_tip_rec = flange_wrench_to_tip_wrench(w_flange, q_down, tip_length_m=tip_len)
    assert w_tip_rec[:3] == pytest.approx(true_tip_wrench[:3], abs=2e-2)
    assert w_tip_rec[3:6] == pytest.approx(true_tip_wrench[3:6], abs=1e-2)
    q_sing = (0.0, math.radians(145.572), 0.0, math.radians(0.44), math.radians(89.0), 0.0)
    assert np.isfinite(flange_wrench_to_tip_wrench([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], q_sing, tip_length_m=tip_len)).all()

    policy = InsertAnythingPolicy(onnx_file)
    p_bh, q_bh = forward_tip_pose(q_down, tip_length_m=tip_len)
    goal = GoalPose(
        joints=q_down,
        p_bh=(float(p_bh[0]), float(p_bh[1]), float(p_bh[2])),
        q_bh=(float(q_bh[0]), float(q_bh[1]), float(q_bh[2]), float(q_bh[3])),
        tip_length_m=tip_len,
        recorded_at=0.0,
    )
    p_start_b = p_bh + np.array([0.003, -0.002, 0.020])
    q_curr = np.array(solve_tip_ik(q_down, p_start_b, q_bh, tip_length_m=tip_len, max_iters=20))
    wf.reset()

    step_res = None
    for _ in range(80):
        w_t = wf.tip_wrench(tip_length_m=tip_len)
        step_res = policy.step(q_curr, goal, w_t, tip_length_m=tip_len)
        q_cmd = np.array(step_res.q_cmd)
        for _ in range(4):
            q_curr = q_curr + 0.25 * (q_cmd - q_curr)
            wf.update(np.zeros(6), dt=0.005, already_at_tip=True)

    assert step_res is not None
    _, _, _, rots = _fast_fk(q_curr, (0.0, 0.0, tip_len))
    assert np.asarray(rots[5])[2, 2] < -0.98
    assert step_res.p_ht[2] < 0.002
    assert float(np.linalg.norm(step_res.p_ht[:2])) < 0.0035

    # Verify two-stage RuleBasedInsertPolicy: align XY + orientation first, then press Z straight down
    rule = RuleBasedInsertPolicy()
    p_off_b, q_off_b = hole_target_to_base([0.004, -0.003, 0.020], rotvec_quat([0.04, -0.03, 0.0]), p_bh, q_bh)
    q_rule = np.array(solve_tip_ik(q_down, p_off_b, q_off_b, tip_length_m=tip_len, max_iters=20))
    saw_align = False
    saw_press = False
    r_step = None
    for _ in range(150):
        r_step = rule.step(q_rule, goal, np.zeros(6), tip_length_m=tip_len)
        if rule.stage == 'align':
            saw_align = True
            assert r_step.p_target_h[2] == pytest.approx(0.020, abs=5e-4)
        else:
            saw_press = True
            assert rule.cmd_p_h is not None
            assert float(np.linalg.norm(rule.cmd_p_h[:2])) == pytest.approx(0.0, abs=1e-9)
        q_rule = q_rule + 0.5 * (np.array(r_step.q_cmd) - q_rule)
    assert saw_align and saw_press and r_step is not None
    assert r_step.p_ht[2] < 0.0005 and float(np.linalg.norm(r_step.p_ht[:2])) < 0.0005

    # Verify rim-blocked spiral breaks joint/contact stiction under ~1.6 N axial compliance and enters offset hole
    rule_rim = RuleBasedInsertPolicy()
    p_rim_b, q_rim_b = hole_target_to_base([0.0, 0.0, 0.012], [1.0, 0.0, 0.0, 0.0], p_bh, q_bh)
    q_rim = np.array(solve_tip_ik(q_down, p_rim_b, q_rim_b, tip_length_m=tip_len))
    true_hole_xy = np.array([0.0018, -0.0012])
    rs = None
    for _ in range(550):
        pb_r, qb_r = forward_tip_pose(q_rim, tip_length_m=tip_len)
        ph_r, _ = relative_pose_in_hole(pb_r, qb_r, p_bh, q_bh)
        in_hole = float(np.linalg.norm(ph_r[:2] - true_hole_xy)) < 0.0005
        fz_r = 3.0 if (not in_hole and ph_r[2] <= 0.0121) else 0.0
        rs = rule_rim.step(q_rim, goal, [0.0, 0.0, fz_r, 0.0, 0.0, 0.0], tip_length_m=tip_len)
        dq_r = np.array(rs.q_cmd) - q_rim
        dq_eff = np.sign(dq_r) * np.maximum(0.0, np.abs(dq_r) - 0.004)
        q_next = q_rim + 0.25 * dq_eff
        pb_n, qb_n = forward_tip_pose(q_next, tip_length_m=tip_len)
        ph_n, qh_n = relative_pose_in_hole(pb_n, qb_n, p_bh, q_bh)
        if ph_r[2] < 0.0115:
            # Inside cylindrical hole bore: wall confines XY within 0.5 mm of true_hole_xy
            off_xy = ph_n[:2] - true_hole_xy
            off_norm = float(np.linalg.norm(off_xy))
            if off_norm > 0.0005:
                ph_n[:2] = true_hole_xy + off_xy * (0.0005 / off_norm)
            ph_n[2] = max(0.0, float(ph_n[2]))
            pbc, qbc = hole_target_to_base(ph_n, qh_n, p_bh, q_bh)
            q_next = np.array(solve_tip_ik(q_rim, pbc, qbc, tip_length_m=tip_len))
        elif float(np.linalg.norm(ph_n[:2] - true_hole_xy)) >= 0.0005 and ph_n[2] < 0.012:
            ph_n[2] = 0.012
            pbc, qbc = hole_target_to_base(ph_n, qh_n, p_bh, q_bh)
            q_next = np.array(solve_tip_ik(q_rim, pbc, qbc, tip_length_m=tip_len))
        q_rim = q_next
        if rs.p_ht[2] < 0.0005:
            break
    assert rs is not None and rule_rim.spiral_phase > 1.0 and rs.p_ht[2] < 0.0005

    # When requested pitch exceeds J4's +89 deg physical limit, IK must respect JOINT_UPPER without lifting tip XYZ
    q_near_j4 = (-1.316, math.radians(113.0), math.radians(-98.0), math.radians(85.0), 0.0, 0.0)
    p_lim_0, q_lim_0 = forward_tip_pose(q_near_j4, tip_length_m=tip_len)
    _, axes_lim, _, _ = _fast_fk(q_near_j4, (0.0, 0.0, tip_len))
    p_lim_des = p_lim_0 + np.array([0.0, 0.0, -0.001])
    q_lim_des = canonical_quat(quat_mul(rotvec_quat(np.asarray(axes_lim[3]) * math.radians(8.0)), q_lim_0))
    q_lim_sol = np.asarray(solve_tip_ik(q_near_j4, p_lim_des, q_lim_des, tip_length_m=tip_len))
    assert np.all(q_lim_sol >= JOINT_LOWER) and np.all(q_lim_sol <= JOINT_UPPER)
    p_lim_chk, _ = forward_tip_pose(q_lim_sol, tip_length_m=tip_len)
    assert p_lim_chk == pytest.approx(p_lim_des, abs=1e-4)


def test_dashboard_can4_gripper_record_and_policy_workflow():
    """Verify end-to-end dashboard workflow on CAN4 (/arm_4): gripper switch, drag, record goal, and execute policy."""
    rclpy.init()
    node = helper = server = None
    try:
        node = InsertDashboardNode(arm_selector='can4', wrench_source='estimated')
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
        pax_msg = WrenchStamped()
        pax_msg.wrench.force.x = 2.0
        pax_msg.wrench.force.y = -3.0
        pax_msg.wrench.force.z = -6.0
        pax_msg.wrench.torque.x = 0.08
        pax_msg.wrench.torque.y = -0.12
        pax_msg.wrench.torque.z = 0.03
        node._on_paxini_wrench(pax_msg)

        # Sample stamps carry 200 Hz estimated force and 500 Hz sensor feedback,
        # even when callbacks arrive together without those wall-clock delays.
        for message, callback, interval_ns in (
            (w_msg, node._on_wrench, 5_000_000),
            (pax_msg, node._on_paxini_wrench, 2_000_000),
        ):
            message.header.stamp.sec = 100
            message.header.stamp.nanosec = 0
            callback(message)
            message.header.stamp.nanosec = interval_ns
            callback(message)
        estimated_alpha = node.estimated_wrench_filter.alpha
        sensor_alpha = node.paxini_wrench_filter.alpha
        assert estimated_alpha == pytest.approx(1.0 - math.exp(-0.005 / 0.015))
        assert sensor_alpha == pytest.approx(1.0 - math.exp(-0.002 / 0.015))
        node.wrench_received_at = time.monotonic() - 0.06
        time.sleep(0.002)
        node._on_control_state(String(data=json.dumps({'end_wrench': [0.0, 0.0, -2.0, 0.0, 0.0, 0.0]})))
        assert node.estimated_wrench_filter.alpha is not None and 0.0 < node.estimated_wrench_filter.alpha < 1.0
        time.sleep(0.002)
        w_msg.header.stamp.sec = 1_794_000_000
        node._on_wrench(w_msg)
        assert node.estimated_wrench_filter.alpha is not None and 0.0 < node.estimated_wrench_filter.alpha < 0.5
        w_msg.header.stamp.nanosec = 10_000_000
        node._on_wrench(w_msg)
        assert node.estimated_wrench_filter.alpha == pytest.approx(estimated_alpha)

        res_src = post('/api/parameters', {'wrench_source': 'paxini'})
        assert res_src['ok'] is True and res_src['wrench_source'] == 'paxini'
        assert res_src['filter_alpha'] == pytest.approx(sensor_alpha)
        assert res_src['filter_sample_hz'] == pytest.approx(500.0)
        with patch.object(node, '_ingest_flange_wrench', side_effect=AssertionError('Inactive source recomputed')):
            for stamp in (10_000_000, 15_000_000):
                w_msg.header.stamp.nanosec = stamp
                node._on_wrench(w_msg)
        assert post('/api/parameters', {'wrench_source': 'estimated'})['filter_alpha'] == pytest.approx(estimated_alpha)
        post('/api/parameters', {'wrench_source': 'paxini'})
        with urlopen(base_url + '/api/status', timeout=2.0) as resp:
            status_before = json.loads(resp.read().decode('utf-8'))
        assert status_before['wrench']['source'] == 'paxini'
        assert status_before['wrench']['force_norm_n'] > 1.0

        res_close = post('/api/gripper', {'action': 'close', 'width_m': 0.0, 'effort': 2.5})
        assert res_close['ok'] is True and res_close['gripper_state'] == 'closed'
        assert res_close['effort'] == pytest.approx(2.0)

        res_drag = post('/api/drag', {})
        assert res_drag['mode'] == 'drag'
        node._tick()
        node._publish_last_target()
        for _ in range(5):
            rclpy.spin_once(helper, timeout_sec=0.01)
        assert received_grip and received_grip[-1].effort == pytest.approx(2.0)
        res_eff = post('/api/gripper', {'action': 'effort', 'effort': 0.8})
        assert res_eff['ok'] is True and res_eff['effort'] == pytest.approx(0.8)
        for _ in range(5):
            rclpy.spin_once(helper, timeout_sec=0.01)
        assert received_grip[-1].effort == pytest.approx(0.8)
        assert received_mit and list(received_mit[-1].kp) == [0.0] * 6

        res_goal = post('/api/goal/record', {'tip_offset_cm': [0.0, 0.0, 19.5]})
        assert res_goal['ok'] is True
        assert res_goal['goal']['joints_rad'] == pytest.approx(q_inserted)
        assert res_goal['goal']['tip_offset_cm'] == pytest.approx([0.0, 0.0, 19.5])

        p_bh = np.array(res_goal['goal']['position_m'])
        q_bh = np.array(res_goal['goal']['quaternion_wxyz'])
        p_too_far = p_bh + rotate(q_bh, [0.0, 0.0, 0.095])
        q_too_far = solve_tip_ik(q_inserted, p_too_far, q_bh, tip_length_m=0.195, max_iters=12)
        node._on_joints(JointState(name=list(JOINT_NAMES), position=[*q_too_far, 0.012]))
        with pytest.raises( Exception ):
            post('/api/policy/start', {})
        assert node.mode == 'drag'
        # Rule-based insertion is allowed beyond 8 cm
        res_rule_far = post('/api/rule/start', {})
        assert res_rule_far['ok'] is True and res_rule_far['controller'] == 'rule'
        post('/api/policy/stop', {'mode': 'drag'})

        p_lifted = p_bh + rotate(q_bh, [0.002, -0.001, 0.012])
        q_lifted = solve_tip_ik(q_inserted, p_lifted, q_bh, tip_length_m=0.195, max_iters=8)
        node._on_joints(JointState(name=list(JOINT_NAMES), position=[*q_lifted, 0.012]))

        res_start = post('/api/policy/start', {})
        assert res_start['mode'] == 'policy'
        assert res_start['wrench_source'] == 'paxini'
        # Temporary gravity elimination at policy start zeroes the current wrench reading
        with urlopen(base_url + '/api/status', timeout=2.0) as resp:
            status_tared = json.loads(resp.read().decode('utf-8'))
        assert status_tared['wrench']['tip_t'] == pytest.approx([0.0] * 6, abs=1e-9)
        assert status_tared['wrench']['force_norm_n'] == pytest.approx(0.0, abs=1e-9)

        node._tick()
        node._publish_last_target()
        for _ in range(5):
            rclpy.spin_once(helper, timeout_sec=0.01)

        with urlopen(base_url + '/api/status', timeout=2.0) as resp:
            status = json.loads(resp.read().decode('utf-8'))
        assert status['policy']['mode'] == 'policy' and status['policy']['steps'] == 1
        assert status['arm']['hole_relative']['p_ht_mm'][2] == pytest.approx(12.0, abs=0.2)
        assert list(received_mit[-1].kp) == [-1.0] * 6
        assert list(received_mit[-1].kd) == [-1.0] * 6
        assert status['policy']['last_step']['p_target_h_mm'][2] < status['arm']['hole_relative']['p_ht_mm'][2]

        res_stop = post('/api/policy/stop', {'mode': 'drag'})
        assert res_stop['mode'] == 'drag'

        # Next policy start re-eliminates gravity at the new current reading
        pax_msg.wrench.force.z = -9.5
        node._on_paxini_wrench(pax_msg)
        with urlopen(base_url + '/api/status', timeout=2.0) as resp:
            status_drifted = json.loads(resp.read().decode('utf-8'))
        assert abs(status_drifted['wrench']['tip_t'][2]) > 3.0
        res_restart = post('/api/policy/start', {})
        assert res_restart['ok'] is True
        with urlopen(base_url + '/api/status', timeout=2.0) as resp:
            status_rezeroed = json.loads(resp.read().decode('utf-8'))
        assert status_rezeroed['wrench']['tip_t'] == pytest.approx([0.0] * 6, abs=1e-9)
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
        if helper is not None:
            helper.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def test_dual_camera_calibration_and_3d_alignment_geometry():
    K1 = np.array([[1000.0, 0.0, 960.0], [0.0, 980.0, 540.0], [0.0, 0.0, 1.0]])
    K2 = np.array([[930.0, 0.0, 960.0], [0.0, 930.0, 540.0], [0.0, 0.0, 1.0]])
    dist0 = np.zeros(5, dtype=np.float64)

    # Known camera poses in arm_4 base_link looking toward workspace center [0.32, -0.10, 0.05]
    def look_at_cam(eye, target):
        z_c = np.asarray(target, float) - np.asarray(eye, float)
        z_c /= np.linalg.norm(z_c)
        up = np.array([0.0, 0.0, 1.0])
        x_c = np.cross(z_c, up)
        x_c /= np.linalg.norm(x_c)
        y_c = np.cross(z_c, x_c)
        return rt_to_mat(np.column_stack([x_c, y_c, z_c]), eye)

    t_base_h1_gt = look_at_cam([0.28, -0.58, 0.52], [0.32, -0.10, 0.04])
    t_base_h2_gt = look_at_cam([0.72, -0.08, 0.48], [0.32, -0.10, 0.04])
    t_base_board_gt = rt_to_mat(np.eye(3), [0.24, -0.18, 0.0])

    spec = BoardSpec()
    from insert.agent_flow.geometry import _board_chessboard_corners, _build_charuco_board
    _, board, _ = _build_charuco_board(spec)
    obj_pts = _board_chessboard_corners(board)
    pts_b = (obj_pts @ t_base_board_gt[:3, :3].T) + t_base_board_gt[:3, 3]
    uv1 = project_base_points(pts_b, K1, dist0, t_base_h1_gt)
    t_h1_board = invert_se3(t_base_h1_gt) @ t_base_board_gt
    cal1 = calibrate_external_camera(
        [{'T_base_board': t_base_board_gt, 'T_head_board': t_h1_board, 'corners_2d': uv1, 'object_3d': obj_pts}],
        K1, dist0,
    )
    assert np.allclose(cal1['T_base_cam'], t_base_h1_gt, atol=1e-5)
    assert cal1['reproj_rms_px'] < 1e-3

    # Dual-view 3D hole & tilted insertion axis recovery + 0.5 cm pre-insert alignment
    p_hole_gt = np.array([0.33, -0.12, 0.08], dtype=np.float64)
    a_insert_gt = np.array([0.25, 0.15, -0.9565563], dtype=np.float64)
    a_insert_gt /= np.linalg.norm(a_insert_gt)
    p_axis_gt = p_hole_gt + 0.06 * a_insert_gt

    uv_cam1 = project_base_points(np.stack([p_hole_gt, p_axis_gt]), K1, dist0, t_base_h1_gt)
    uv_cam2 = project_base_points(np.stack([p_hole_gt, p_axis_gt]), K2, dist0, t_base_h2_gt)

    from insert.agent_flow.geometry import pixel_to_base_ray
    o1, d1 = pixel_to_base_ray(uv_cam1[0, 0], uv_cam1[0, 1], K1, dist0, t_base_h1_gt)
    o2, d2 = pixel_to_base_ray(uv_cam2[0, 0], uv_cam2[0, 1], K2, dist0, t_base_h2_gt)
    tri = triangulate_two_rays(o1, d1, o2, d2)
    assert tri['point_b'] == pytest.approx(p_hole_gt, abs=1e-6)
    assert tri['ray_gap_mm'] == pytest.approx(0.0, abs=1e-4)

    ax = solve_axis_from_two_views(
        tri['point_b'],
        tuple(uv_cam1[0]), tuple(uv_cam1[1]), K1, dist0, t_base_h1_gt,
        tuple(uv_cam2[0]), tuple(uv_cam2[1]), K2, dist0, t_base_h2_gt,
    )
    assert ax['insert_axis_b'] == pytest.approx(a_insert_gt, abs=1e-5)

    q_init = (0.0, 0.90, -1.35, 0.0, 0.45, 0.0)
    plan = compute_alignment_plan(
        hole_entrance_b=tri['point_b'],
        insert_axis_b=ax['insert_axis_b'],
        joints_rad=q_init,
        tip_offset_m=(0.0, 0.0, 0.195),
        standoff_m=0.005,
        insert_depth_m=0.020,
    )
    assert np.asarray(plan['pre_insert_m']) == pytest.approx(p_hole_gt - 0.005 * a_insert_gt, abs=1e-6)
    assert rotate(plan['q_bh_wxyz'], [0.0, 0.0, -1.0]) == pytest.approx(a_insert_gt, abs=1e-6)
    p_ht_pre, q_ht_pre = relative_pose_in_hole(
        plan['pre_insert_m'], plan['q_bh_wxyz'], plan['goal_inserted_m'], plan['q_bh_wxyz'],
    )
    assert p_ht_pre == pytest.approx([0.0, 0.0, 0.025], abs=1e-6)
    assert q_ht_pre == pytest.approx([1.0, 0.0, 0.0, 0.0], abs=1e-6)


def test_agent_flow_web_perception_and_pre_insert_move(tmp_path, monkeypatch):
    rclpy.init()
    node = None
    server = None
    try:
        ext_file = tmp_path / 'extrinsics.yaml'
        node = AgentInsertNode(
            namespace_override='/agent_flow_test',
            wrench_source='paxini',
            extrinsics_path=ext_file,
        )
        node._timer.cancel()
        node._command_timer.cancel()
        assert node.mode == 'drag'

        q_start = (0.0, 1.45, -1.25, 0.0, 1.20, 0.0)
        node._on_joints(JointState(name=list(JOINT_NAMES), position=[*q_start, 0.010]))
        node.control_state = {'mode': 'mit', 'mit_ready': True}

        # Inject two calibrated external cameras
        K = np.array([[950.0, 0.0, 960.0], [0.0, 950.0, 540.0], [0.0, 0.0, 1.0]])
        dist0 = np.zeros(5, dtype=np.float64)
        t_h1 = rt_to_mat(np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]), [0.26, -0.05, 0.65])
        t_h2 = rt_to_mat(np.array([[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]), [0.75, -0.05, 0.25])
        node.extrinsics['head_1'] = {'T_base_cam': t_h1, 'translation_mm': [260.0, -50.0, 650.0], 'rpy_deg': [180.0, 0.0, 0.0]}
        node.extrinsics['head_2'] = {'T_base_cam': t_h2, 'translation_mm': [750.0, -50.0, 250.0], 'rpy_deg': [90.0, 0.0, 90.0]}

        blank = np.zeros((1080, 1920, 3), dtype=np.uint8)
        monkeypatch.setattr(node, 'grab_head_bgr', lambda cam, fresh=True, timeout=3.5: blank.copy())
        monkeypatch.setattr(
            node, 'get_head_intrinsic',
            lambda cam: {'available': True, 'camera_matrix': K.reshape(-1).tolist(), 'distortion_coefficients': dist0.tolist()},
        )

        p_tip_fk, _ = forward_tip_pose(q_start, tip_offset_m=node.tip_offset_m, gravity_model=node.gravity_model)
        p_hole_target = p_tip_fk + np.array([0.015, -0.010, -0.030])
        a_ins_target = np.array([0.0, 0.0, -1.0])
        uv1 = project_base_points(np.stack([p_tip_fk, p_tip_fk + [0, 0, 0.05], p_hole_target, p_hole_target + 0.05 * a_ins_target]), K, dist0, t_h1)
        uv2 = project_base_points(np.stack([p_tip_fk, p_tip_fk + [0, 0, 0.05], p_hole_target, p_hole_target + 0.05 * a_ins_target]), K, dist0, t_h2)

        captured_calls = []
        def fake_perceive(img1, img2, **kwargs):
            captured_calls.append(kwargs)
            return {
                'raw_numeric': {
                    'symmetry_deg': 180.0,
                    'cam1': {
                        'plug_tip': [float(uv1[0, 0]), float(uv1[0, 1])],
                        'plug_axis_point': [float(uv1[1, 0]), float(uv1[1, 1])],
                        'plug_orient_point': [float(uv1[0, 0] + 40.0), float(uv1[0, 1])],
                        'hole_center': [float(uv1[2, 0]), float(uv1[2, 1])],
                        'hole_insert_point': [float(uv1[3, 0]), float(uv1[3, 1])],
                        'hole_orient_point': [float(uv1[2, 0] + 40.0), float(uv1[2, 1])],
                    },
                    'cam2': {
                        'plug_tip': [float(uv2[0, 0]), float(uv2[0, 1])],
                        'plug_axis_point': [float(uv2[1, 0]), float(uv2[1, 1])],
                        'plug_orient_point': [float(uv2[0, 0]), float(uv2[0, 1] + 40.0)],
                        'hole_center': [float(uv2[2, 0]), float(uv2[2, 1])],
                        'hole_insert_point': [float(uv2[3, 0]), float(uv2[3, 1])],
                        'hole_orient_point': [float(uv2[2, 0]), float(uv2[2, 1] + 40.0)],
                    },
                },
                'symmetry_deg': 180.0,
                'cam1': {
                    'plug_tip': [float(uv1[0, 0]), float(uv1[0, 1])],
                    'plug_axis_point': [float(uv1[1, 0]), float(uv1[1, 1])],
                    'plug_orient_point': [float(uv1[0, 0] + 40.0), float(uv1[0, 1])],
                    'hole_center': [float(uv1[2, 0]), float(uv1[2, 1])],
                    'hole_insert_point': [float(uv1[3, 0]), float(uv1[3, 1])],
                    'hole_orient_point': [float(uv1[2, 0] + 40.0), float(uv1[2, 1])],
                },
                'cam2': {
                    'plug_tip': [float(uv2[0, 0]), float(uv2[0, 1])],
                    'plug_axis_point': [float(uv2[1, 0]), float(uv2[1, 1])],
                    'plug_orient_point': [float(uv2[0, 0]), float(uv2[0, 1] + 40.0)],
                    'hole_center': [float(uv2[2, 0]), float(uv2[2, 1])],
                    'hole_insert_point': [float(uv2[3, 0]), float(uv2[3, 1])],
                    'hole_orient_point': [float(uv2[2, 0]), float(uv2[2, 1] + 40.0)],
                },
                'model': kwargs.get('model', 'gpt-6.1-sol'),
                'reasoning_effort': kwargs.get('reasoning_effort', 'max'),
                'reasoning_steps': 2,
                'latency_s': 1.2,
                'object_prompt': kwargs['object_prompt'],
                'output_prompt': kwargs['output_prompt'],
            }

        monkeypatch.setattr('insert.agent_flow.server.perceive_dual_views', fake_perceive)

        server = create_agent_http_server(node, host='127.0.0.1', port=0)
        port = server.server_address[1]
        threading.Thread(target=server.serve_forever, daemon=True).start()
        base_url = f'http://127.0.0.1:{port}'

        def post(path, payload):
            req = Request(base_url + path, data=json.dumps(payload).encode('utf-8'), headers={'Content-Type': 'application/json'}, method='POST')
            with urlopen(req, timeout=5.0) as r:
                return json.loads(r.read().decode('utf-8'))

        with urlopen(base_url + '/', timeout=2.0) as r_html:
            assert 'Agent 视觉引导全流程插孔控制台' in r_html.read().decode('utf-8')

        perc = post('/api/agent/perceive', {'standoff_cm': 0.5, 'insert_depth_cm': 2.0})
        assert len(captured_calls) == 1
        assert captured_calls[0]['model'] == 'gpt-6.1-sol'
        assert captured_calls[0]['reasoning_effort'] == 'max'
        assert '把黑色夹爪捏着的插头的末端' in captured_calls[0]['object_prompt']
        assert 'raw_llm_output' in perc and perc['raw_llm_output']['symmetry_deg'] == 180.0
        assert np.asarray(perc['hole_entrance_m']) == pytest.approx(p_hole_target, abs=1e-4)
        assert np.asarray(perc['insert_axis_unit']) == pytest.approx(a_ins_target, abs=1e-4)

        with urlopen(base_url + perc['images']['head_1'], timeout=2.0) as r_img:
            assert len(r_img.read()) > 1000

        # Stage 3: smooth move to 0.5 cm in front of hole
        mv = post('/api/move/pre_insert', {'standoff_cm': 0.5, 'insert_depth_cm': 2.0, 'duration_s': 0.5})
        assert mv['ok'] is True and node.mode == 'move'
        t_base = node._move_start_time
        for step_idx in range(1, 11):
            node._move_start_time = time.monotonic() - (0.05 * step_idx)
            node._tick()
        assert node.mode == 'hold'
        assert node.hold_joints is not None
        p_after, _ = forward_tip_pose(node.hold_joints, tip_offset_m=node.tip_offset_m, gravity_model=node.gravity_model)
        assert p_after == pytest.approx(p_hole_target - 0.005 * a_ins_target, abs=1e-4)

        # Stage 4: start rule-based or policy-based insertion from 0.5 cm pre-insert pose
        node._on_joints(JointState(name=list(JOINT_NAMES), position=[*node.hold_joints, 0.010]))
        res_rule = post('/api/rule/start', {})
        assert res_rule['ok'] is True and res_rule['controller'] == 'rule'
        post('/api/policy/stop', {'mode': 'hold'})
        res_onnx = post('/api/policy/start', {'controller': 'onnx'})
        assert res_onnx['ok'] is True and res_onnx['controller'] == 'onnx'
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
        if node is not None:
            node.shutdown_streams()
            node.destroy_node()
        rclpy.try_shutdown()
