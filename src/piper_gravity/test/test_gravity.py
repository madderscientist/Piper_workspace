"""End-to-end tests for bidirectional quasi-static gravity & static friction calibration."""

import json
import shutil
import threading
import time
from urllib.request import Request, urlopen

from piper_gravity.cli import evaluate_main, fit_main
from piper_gravity.collector import ProbeCollectorNode, ProbeConfig, realized_gains
from piper_gravity.identification import (
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
from piper_x_driver.backend import Feedback
from piper_x_driver.configuration import default_config_path, FleetConfig
from piper_x_driver.node import DriverNode
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter


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
                pos[1] = q_ref - (true_tau2 - g_app) / kp_real
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
