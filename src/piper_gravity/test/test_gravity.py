"""End-to-end tests for bidirectional quasi-static gravity & static friction calibration."""

import json
import shutil
import threading
import time
from urllib.request import Request, urlopen
from urllib.error import HTTPError

from piper_gravity.cli import evaluate_main, fit_main
from piper_gravity.collector import ProbeCollectorNode, ProbeConfig, realized_gains
from piper_gravity.identification import (
    ProbeSample,
    identify_gripper_payload,
    identify_joint_gravity,
    load_probe_samples_csv,
    save_probe_samples_csv,
)
from piper_gravity.model import (
    ArmGravityModel,
    JointGravityModel,
    gravity_torques,
    load_gravity_model,
    save_gravity_model,
)
from piper_gravity.web import GravityWorkbenchNode, create_http_server, synthesize_demo_probe_samples
from conftest import Feedback, MockDriverNode as DriverNode
from piper_x_driver.configuration import default_config_path, FleetConfig
from piper_x_driver.gravity import estimate_joint_torques
from piper_x_driver.terminal import terminal_for_model
import numpy as np
import yaml
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter


def payload_samples(model, added_mass=0.16):
    nominal = model.with_added_mass(0)
    truth = model.with_added_mass(added_mass)
    rows = {}
    for joint, limits in ((2, (0.55, 1.2)), (3, (-1.8, -1.15)), (5, (-0.4, 0.4))):
        rows[joint] = []
        for direction in (1, -1):
            for angle in np.linspace(*limits, 7):
                q = [0.0, 0.9, -1.5, 0.1, -0.1, 0.2]
                q[joint - 1] = angle
                torque = truth.torques(q)[joint - 1] + direction * 0.12
                rows[joint].append(ProbeSample(
                    t=len(rows[joint]) * 0.1, joint=joint, direction=direction,
                    q_target=angle, q=angle, dq=0.0, tau_meas=torque, tau_kp=torque,
                    tau_gravity=nominal.torques(q)[joint - 1], q_full=tuple(q)))
    return rows


def test_composite_payload_mass_cli_and_inertia_roundtrip(tmp_path):
    fleet = FleetConfig(default_config_path())
    arm3_model = fleet.gravity_model('arm_3')
    assert arm3_model is not None
    baseline = arm3_model.with_added_mass(0)
    samples = payload_samples(baseline)
    fitted, reports = identify_gripper_payload(samples, baseline)
    assert fitted.added_mass_kg == pytest.approx(0.16, abs=1e-7)
    assert all(fitted.joint(j).scale == baseline.joint(j).scale for j in range(1, 7))
    assert all(report.rmse_calibrated < 1e-7 for report in reports.values())
    assert all(report.static_friction_mean == pytest.approx(0.12) for report in reports.values())
    terminal = terminal_for_model('gripper_with_wrench')
    assert terminal is not None
    shape = terminal.components['gripper_base']
    nominal, loaded = baseline.effector_body(), fitted.effector_body()
    def about_origin(body):
        c = np.asarray(body.com_m)
        return body.inertia + body.mass_kg * (np.dot(c, c) * np.eye(3) - np.outer(c, c))
    expected = 0.16 * (shape.inertia / shape.mass_kg + (
        np.dot(shape.com_m, shape.com_m) * np.eye(3) - np.outer(shape.com_m, shape.com_m)))
    np.testing.assert_allclose(about_origin(loaded) - about_origin(nominal), expected, atol=1e-12)
    paths = [save_probe_samples_csv(rows, tmp_path / f'j{j}.csv') for j, rows in samples.items()]
    output = tmp_path / 'arm3.yaml'
    assert fit_main([*map(str, paths), '--arm-id', 'arm_3', '-o', str(output)]) == 0
    restored = load_gravity_model(output)
    assert restored.added_mass_kg == pytest.approx(0.16, abs=1e-6)
    q = (0.0, 0.8, -1.3, 0.1, -0.2, 0.4)
    assert restored.torques(q) == pytest.approx(baseline.with_added_mass(0.16).torques(q))
    assert restored.inertias(q) == pytest.approx(fitted.inertias(q))
    assert estimate_joint_torques(
        q, (0,) * 6, q, (0,) * 6, (0,) * 6, (0,) * 6,
        acceleration=(1,) * 6, gravity_model=restored,
    ) == pytest.approx(restored.inertias(q))
    config = yaml.safe_load(default_config_path().read_text())
    config['arms'][2]['gravity_config'] = str(output)
    config_path = tmp_path / 'arms.yaml'
    config_path.write_text(yaml.safe_dump(config))
    loaded_fleet = FleetConfig(config_path)
    loaded_arm3 = loaded_fleet.gravity_model('arm_3')
    assert loaded_arm3 is not None
    assert loaded_arm3.added_mass_kg == pytest.approx(0.16)
    payload = loaded_fleet.public(loaded_fleet.arm('arm_3'))['gravity_payload']
    assert payload['total_mass_kg'] == pytest.approx(nominal.mass_kg + 0.16)


def test_identify_gravity_and_static_friction_and_yaml_roundtrip(tmp_path):
    raw_samples = synthesize_demo_probe_samples(joint=2, num_points=13)
    csv_path = save_probe_samples_csv(raw_samples, tmp_path / 'j2_probe.csv')
    loaded_samples = load_probe_samples_csv(csv_path)

    report = identify_joint_gravity(loaded_samples)
    est = report.model

    assert est.scale == pytest.approx(1.08, abs=0.04)
    assert est.angle_offset == pytest.approx(-0.015, abs=0.015)
    assert est.static_friction == pytest.approx(0.72, abs=0.02)
    assert report.rmse_calibrated < report.rmse_cad * 0.35

    # Upright J1 must lock scale=1.0 and angle_offset=0.0
    j1_samples = synthesize_demo_probe_samples(joint=1, num_points=9)
    j1_report = identify_joint_gravity(j1_samples)
    assert j1_report.model.scale == 1.0
    assert j1_report.model.angle_offset == 0.0
    assert j1_report.model.bias == 0.0
    assert j1_report.model.static_friction == pytest.approx(0.38, abs=0.02)

    yaml_path = tmp_path / 'gravity.yaml'
    arm_model = ArmGravityModel(arm_id='arm_1').with_joint(2, est)
    save_gravity_model(arm_model, yaml_path)
    restored = load_gravity_model(yaml_path)
    q_test = (0.0, 0.75, -1.45, 0.15, -0.10, 0.0)
    torques = restored.torques(q_test)
    cad = gravity_torques(q_test)
    assert torques[0] == pytest.approx(cad[0])
    assert torques[1] != pytest.approx(cad[1])

    # Inertia tensors and pose-dependent joint inertias scale proportionally without changing shape
    from piper_gravity.model import joint_inertias
    q_eff = (q_test[0], q_test[1] + est.angle_offset, *q_test[2:])
    base_inertias = joint_inertias(q_eff)
    scaled_inertias = restored.inertias(q_test)
    assert scaled_inertias[0] == pytest.approx(base_inertias[0])
    assert scaled_inertias[1] == pytest.approx(est.scale * base_inertias[1])
    tensors_base = ArmGravityModel(arm_id='arm_1').scaled_inertia_tensors()
    tensors_cal = restored.scaled_inertia_tensors()
    assert tensors_cal[1] == pytest.approx(est.scale * tensors_base[1])


@pytest.mark.usefixtures('fake_arm')
def test_ros_probe_collector_and_cli_pipeline(tmp_path):
    cfg_path = tmp_path / 'arms.yaml'
    shutil.copyfile(default_config_path(), cfg_path)
    rclpy.init()
    executor = SingleThreadedExecutor()
    driver = DriverNode(
        namespace='/arm_1',
        parameter_overrides=[Parameter('config_file', value=str(cfg_path))],
    )
    arm = driver.arm
    effector = driver.control.effector
    assert effector is not None
    driver.control.adaptive_stiffness_scale_applied = driver.control.s_a
    kp_real, _ = realized_gains(2, 30.0, 6.58)
    true_scale = 1.10
    true_fs = 0.65

    def simulated_snapshot():
        vel = [0.0] * 6
        if arm.mit_output is not None:
            vel = list(arm.mit_output.velocity)
        pos = list(arm.position)
        q_ref = collector.current_q_des if collector.current_kind == 'measure' else pos[1]
        eval_pos = (pos[0], q_ref, pos[2], pos[3], pos[4], pos[5])
        g_cad = gravity_torques(eval_pos, mass_kg=effector.mass_kg, com_m=effector.com_m)
        torques = list(g_cad)
        direction = 1.0 if collector.current_direction >= 0 else -1.0
        true_tau2 = true_scale * g_cad[1] + direction * true_fs
        torques[1] = true_tau2
        if collector.current_kind == 'measure':
            g_mod = driver.control.gravity_model
            kp_hw = (
                realized_gains(2, arm.mit_output.kp[1], arm.mit_output.kd[1])[0]
                if arm.mit_output is not None and arm.mit_output.kp[1] > 0.0
                else kp_real
            )
            for _ in range(4):
                eval_settled = (pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
                g_settled = gravity_torques(
                    eval_settled, mass_kg=effector.mass_kg, com_m=effector.com_m
                )
                g_app = (
                    g_settled[1]
                    if g_mod is None
                    else g_mod.torques(eval_settled, mass_kg=effector.mass_kg, com_m=effector.com_m)[1]
                )
                true_tau2 = true_scale * g_settled[1] + direction * true_fs
                pos[1] = q_ref - (true_tau2 - g_app) / kp_hw
            torques[1] = true_tau2
        return Feedback(
            tuple(pos), arm.status.copy(), arm.enabled, (0.0,) * 11,
            (0.2, 0.0, 0.2, 0.0, 0.0, 0.0), tuple(vel), tuple(torques),
        )

    collector = ProbeCollectorNode(ProbeConfig(
        namespace='/arm_1',
        joint=2,
        q_min=0.60,
        q_max=1.25,
        num_points=5,
        pre_travel=0.04,
        approach_speed=1.5,
        settle_time=0.10,
        rate_hz=200.0,
    ))
    arm.snapshot = simulated_snapshot
    executor.add_node(driver)
    executor.add_node(collector)
    try:
        deadline = time.monotonic() + 15.0
        while not collector.completed and not collector.error and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.005)
        assert collector.completed, collector.error
        assert len(collector.samples) >= 10
        # Compensation scales are restored upon completion
        assert driver.control.gravity_scale == 1.0
        assert driver.control.friction_scale == FleetConfig(cfg_path).arm('arm_1').friction_scale
        assert driver.control.s_a == FleetConfig(cfg_path).arm('arm_1').s_a
        assert FleetConfig(cfg_path).arm('arm_1').gravity_scale == 1.0

        csv_file = save_probe_samples_csv(collector.samples, tmp_path / 'probe_j2.csv')
        yaml_file = tmp_path / 'arm1_gravity.yaml'
        rc = fit_main([str(csv_file), '-o', str(yaml_file)])
        assert rc == 0
        assert yaml_file.exists()
        fitted = load_gravity_model(yaml_file).joint(2)
        assert fitted.scale == pytest.approx(true_scale, abs=0.03)
        assert fitted.static_friction == pytest.approx(true_fs, abs=0.03)

        rc_eval = evaluate_main(['-m', str(yaml_file), '-q', '0', '0.6', '-1.2', '0', '0', '0'])
        assert rc_eval == 0
    finally:
        executor.shutdown(timeout_sec=1)
        collector.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()


@pytest.mark.usefixtures('fake_arm')
def test_gravity_web_workbench(tmp_path):
    cfg_path = tmp_path / 'arms.yaml'
    shutil.copyfile(default_config_path(), cfg_path)
    rclpy.init()
    executor = SingleThreadedExecutor()
    driver = DriverNode(
        namespace='/arm_1',
        parameter_overrides=[Parameter('config_file', value=str(cfg_path))],
    )
    web_node = GravityWorkbenchNode(config_file=cfg_path)
    executor.add_node(driver)
    executor.add_node(web_node)
    server = create_http_server(web_node, host='127.0.0.1', port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f'http://127.0.0.1:{server.server_port}'
    try:
        html = urlopen(f'{base_url}/', timeout=3).read().decode('utf-8')
        assert '关节重力补偿与静摩擦标定工作台' in html

        req = Request(
            f'{base_url}/api/fit',
            data=json.dumps({'source': 'demo_all', 'arm_id': 'arm_1'}).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        fit_resp = json.loads(urlopen(req, timeout=10).read().decode('utf-8'))
        assert fit_resp['ok'] is True
        assert set(fit_resp['joint_fits'].keys()) == {'1', '2', '3', '4', '5', '6'}

        # Composite terminals reuse all measured joints to fit one gripper-only added mass.
        from piper_gravity.web import format_probe_samples_csv_text
        for samples in payload_samples(web_node.arm_models['arm_3']).values():
            req = Request(
                f'{base_url}/api/fit',
                data=json.dumps({
                    'source': 'csv_text', 'arm_id': 'arm_3',
                    'csv_text': format_probe_samples_csv_text(samples),
                    'output_yaml': str(tmp_path / 'arm3_gravity.yaml'),
                }).encode('utf-8'),
                headers={'Content-Type': 'application/json'}, method='POST')
            try:
                response = json.loads(urlopen(req, timeout=10).read().decode('utf-8'))
            except HTTPError as error:
                pytest.fail(error.read().decode('utf-8'))
            assert response['fit']['payload']['added_mass_kg'] == pytest.approx(0.16, abs=1e-6)
        assert load_gravity_model(tmp_path / 'arm3_gravity.yaml').added_mass_kg == pytest.approx(0.16)

        # Starting arm_1 and arm_2 concurrently succeeds without blocking each other
        for ns in ('/arm_1', '/arm_2'):
            start_req = Request(
                f'{base_url}/api/probe/start',
                data=json.dumps({
                    'namespace': ns,
                    'joint': 2,
                    'q_min': 0.6,
                    'q_max': 1.2,
                    'num_points': 3,
                }).encode('utf-8'),
                headers={'Content-Type': 'application/json'},
                method='POST',
            )
            start_resp = json.loads(urlopen(start_req, timeout=3).read().decode('utf-8'))
            assert start_resp['ok'] is True
        status_resp = json.loads(urlopen(f'{base_url}/api/status', timeout=3).read().decode('utf-8'))
        assert status_resp['probes_by_ns']['/arm_1']['phase'] == 'waiting_feedback'
        assert status_resp['probes_by_ns']['/arm_2']['phase'] == 'waiting_feedback'
    finally:
        server.shutdown()
        server.server_close()
        executor.shutdown(timeout_sec=1)
        web_node.destroy_node()
        driver.destroy_node()
        rclpy.shutdown()
