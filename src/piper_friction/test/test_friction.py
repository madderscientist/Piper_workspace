"""End-to-end tests for friction modeling, parameter identification, and ROS 2 sweep collection."""

import json
import threading
from urllib.request import Request, urlopen

import numpy as np
from piper_friction.identification import (
    SweepSample,
    identify_joint_friction,
    load_samples_csv,
    save_samples_csv,
)
from piper_friction.model import (
    ArmFrictionModel,
    JointFrictionModel,
    SpatialHarmonic,
    load_friction_model,
    save_friction_model,
)
from piper_friction.web import FrictionWorkbenchNode, create_http_server
from piper_x_driver.configuration import default_config_path
import pytest
import rclpy


def _synthesize_sweeps(
    truth: JointFrictionModel,
    speeds: tuple[float, ...] = (0.02, 0.05, 0.1, 0.25, 0.5, 1.0, 1.6),
    q_min: float = -0.6,
    q_max: float = 0.6,
    joint: int = 2,
) -> list[SweepSample]:
    samples: list[SweepSample] = []
    t = 0.0
    span = q_max - q_min
    for speed in speeds:
        n_steps = max(400, int((span / speed) * 200))
        dt = (span / speed) / n_steps
        for direction in (1.0, -1.0):
            qs = (
                np.linspace(q_min, q_max, n_steps)
                if direction > 0
                else np.linspace(q_max, q_min, n_steps)
            )
            dq = direction * speed
            for idx, q in enumerate(qs):
                tau_g = 2.5 * np.cos(q)
                # Inject an initial acceleration transient (first 15% of samples) with large inertial torque
                if speed >= 0.5 and idx < int(0.15 * n_steps):
                    ramp = (idx + 1) / (0.15 * n_steps)
                    dq_actual = direction * speed * (0.2 + 0.6 * ramp)
                    tau_inertial = direction * 4.5
                else:
                    dq_actual = dq
                    tau_inertial = 0.0
                tau_f = truth.torque(float(q), float(dq_actual))
                samples.append(SweepSample(
                    t=t,
                    joint=joint,
                    q=float(q),
                    dq=float(dq_actual),
                    tau_meas=float(tau_g + tau_f + tau_inertial),
                    q_des=float(q),
                    dq_des=float(dq),
                    tau_gravity=float(tau_g),
                ))
                t += dt
            # Settle points between passes
            for _ in range(10):
                samples.append(SweepSample(
                    t=t, joint=joint, q=float(qs[-1]), dq=0.0,
                    tau_meas=float(2.5 * np.cos(qs[-1])),
                    q_des=float(qs[-1]), dq_des=0.0,
                    tau_gravity=float(2.5 * np.cos(qs[-1])),
                ))
                t += 0.01
    return samples


def test_identify_stribeck_and_periodic_harmonics_and_yaml_roundtrip(tmp_path):
    truth = JointFrictionModel(
        fc=0.35,
        fs=0.80,
        vs=0.12,
        delta=2.0,
        fv=0.15,
        v0=0.005,
        vr=0.35,
        harmonics=(
            SpatialHarmonic(omega=36.0, a=0.14, b=-0.08, c=0.09, d=0.05),
            SpatialHarmonic(omega=72.0, a=-0.04, b=0.06, c=0.03, d=-0.04),
        ),
    )
    raw_samples = _synthesize_sweeps(truth)
    csv_path = save_samples_csv(raw_samples, tmp_path / 'j2_sweep.csv')
    loaded_samples = load_samples_csv(csv_path)

    report = identify_joint_friction(loaded_samples, max_harmonics=2)
    est = report.model
    assert isinstance(est, JointFrictionModel)

    assert est.fc == pytest.approx(truth.fc, abs=0.02)
    assert est.fs == pytest.approx(truth.fs, abs=0.03)
    assert est.vs == pytest.approx(truth.vs, abs=0.02)
    assert est.fv == pytest.approx(truth.fv, abs=0.02)
    assert est.vr == pytest.approx(truth.vr, abs=0.03)

    by_omega = {round(h.omega): h for h in est.harmonics}
    assert set(by_omega) == {36, 72}
    for omega, truth_h in ((36, truth.harmonics[0]), (72, truth.harmonics[1])):
        est_h = by_omega[omega]
        assert est_h.a == pytest.approx(truth_h.a, abs=0.015)
        assert est_h.b == pytest.approx(truth_h.b, abs=0.015)
        assert est_h.c == pytest.approx(truth_h.c, abs=0.015)
        assert est_h.d == pytest.approx(truth_h.d, abs=0.015)

    assert report.rmse_total < 0.015
    assert report.rmse_stribeck < 0.01
    assert report.rmse_cogging < 0.01
    assert report.rmse_modulation < 0.01

    yaml_path = tmp_path / 'friction.yaml'
    arm_model = ArmFrictionModel(arm_id='arm_1').with_joint(2, est)
    save_friction_model(arm_model, yaml_path)
    restored = load_friction_model(yaml_path)
    q_test = (0.0, 0.25, -0.3, 0.0, 0.0, 0.0)
    dq_test = (0.0, 0.04, 0.0, 0.0, 0.0, 0.0)
    torques = restored.torques(q_test, dq_test)
    assert torques[0] == 0.0
    assert torques[1] == pytest.approx(truth.torque(0.25, 0.04), abs=0.02)


def test_web_workbench_demo_all_and_queue_requests(tmp_path):
    rclpy.init()
    web_node = FrictionWorkbenchNode(config_file=default_config_path())
    server = create_http_server(web_node, host='127.0.0.1', port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f'http://127.0.0.1:{server.server_port}'
    try:
        html = urlopen(f'{base_url}/', timeout=3).read().decode('utf-8')
        assert '关节摩擦力与周期性齿轮阻碍标定工作台' in html
        gripper_html = urlopen(f'{base_url}/gripper.html', timeout=3).read().decode('utf-8')
        assert '夹爪 / 示教手柄非周期摩擦力标定工作台' in gripper_html

        req = Request(
            f'{base_url}/api/fit',
            data=json.dumps({'source': 'demo_all', 'arm_id': 'arm_1', 'joint': 1}).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        fit_resp = json.loads(urlopen(req, timeout=30).read().decode('utf-8'))
        assert fit_resp['ok'] is True
        assert set(fit_resp['joint_fits'].keys()) == {'1', '2', '3', '4', '5', '6'}

        req_g = Request(
            f'{base_url}/api/fit',
            data=json.dumps({'source': 'demo', 'arm_id': 'arm_1', 'joint': 7}).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        fit_g_resp = json.loads(urlopen(req_g, timeout=30).read().decode('utf-8'))
        assert fit_g_resp['ok'] is True
        assert fit_g_resp['fit']['is_gripper'] is True
        assert len(fit_g_resp['fit']['profile_knots']) >= 10
        assert fit_g_resp['fit']['metrics']['rmse_total'] < 0.03
        assert '7' in fit_g_resp['joint_fits']

        yaml_out = tmp_path / 'seq_arm1.yaml'
        sweep_req = Request(
            f'{base_url}/api/sweep/start',
            data=json.dumps({
                'namespace': '/arm_1',
                'joints_plan': [
                    {'joint': 1, 'q_min': -0.2, 'q_max': 0.2},
                    {'joint': 2, 'q_min': 0.3, 'q_max': 0.7},
                ],
                'speeds': [0.4, 1.0],
                'approach_speed': 2.0,
                'settle_time': 0.01,
                'fit_output': str(yaml_out),
            }).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        start_resp = json.loads(urlopen(sweep_req, timeout=3).read().decode('utf-8'))
        assert start_resp['ok'] is True
        assert start_resp['status']['sweep']['queue_joints'] == [1, 2]

        # Starting a second arm concurrently succeeds without blocking /arm_1
        sweep_req_2 = Request(
            f'{base_url}/api/sweep/start',
            data=json.dumps({
                'namespace': '/arm_2',
                'joint': 1,
                'q_min': -0.2,
                'q_max': 0.2,
                'speeds': [0.4],
            }).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        start_resp_2 = json.loads(urlopen(sweep_req_2, timeout=3).read().decode('utf-8'))
        assert start_resp_2['ok'] is True
        assert start_resp_2['status']['sweeps_by_ns']['/arm_2']['phase'] == 'waiting_feedback'
    finally:
        server.shutdown()
        server.server_close()
        web_node.destroy_node()
        rclpy.shutdown()

