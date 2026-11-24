"""End-to-end tests for bidirectional quasi-static gravity & static friction calibration."""

from dataclasses import replace
import json
import threading
from urllib.request import Request, urlopen

from piper_gravity.cli import fit_main
from piper_gravity.identification import (
    ProbeSample,
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
from piper_x_driver.configuration import default_config_path, FleetConfig
from piper_x_driver.gravity import estimate_joint_torques
import numpy as np
import yaml
import pytest
import rclpy


def composite_samples(baseline, scales):
    truth = baseline
    for joint, scale in scales.items():
        truth = truth.with_joint(joint, JointGravityModel(scale=scale))
    rows = {}
    for joint, limits in ((2, (0.55, 1.2)), (3, (-1.8, -1.15))):
        rows[joint] = []
        for direction in (1, -1):
            for angle in np.linspace(*limits, 7):
                q = [0.0, 0.9, -1.5, 0.1, -0.1, 0.2]
                q[joint - 1] = angle
                torque = truth.torques(q)[joint - 1] + direction * 0.12
                rows[joint].append(ProbeSample(
                    t=len(rows[joint]) * 0.1, joint=joint, direction=direction,
                    q_target=angle, q=angle, dq=0.0, tau_meas=torque, tau_kp=torque,
                    tau_gravity=baseline.torques(q)[joint - 1], q_full=tuple(q)))
    return rows


def test_composite_effector_fits_joints_and_keeps_configured_added_mass(tmp_path):
    fleet = FleetConfig(default_config_path())
    baseline = replace(fleet.gravity_model('arm_3'), joints={}).with_added_mass(0.1)
    scales = {2: 1.08, 3: 0.95}
    samples = composite_samples(baseline, scales)
    paths = [save_probe_samples_csv(rows, tmp_path / f'j{j}.csv') for j, rows in samples.items()]
    output = tmp_path / 'arm3.yaml'
    save_gravity_model(baseline, output)  # added_mass_kg configured by hand before fitting
    assert fit_main([*map(str, paths), '--arm-id', 'arm_3', '-o', str(output)]) == 0
    restored = load_gravity_model(output)
    assert restored.added_mass_kg == pytest.approx(0.1)
    for joint, scale in scales.items():
        assert restored.joint(joint).scale == pytest.approx(scale, abs=2e-3)
        assert restored.joint(joint).static_friction == pytest.approx(0.12, abs=1e-3)
    q = (0.0, 0.8, -1.3, 0.1, -0.2, 0.4)
    assert estimate_joint_torques(
        q, (0,) * 6, q, (0,) * 6, (0,) * 6, (0,) * 6,
        acceleration=(1,) * 6, gravity_model=restored,
    ) == pytest.approx(restored.inertias(q))
    config_path = tmp_path / 'arms'
    config_path.mkdir()
    arm3 = yaml.safe_load((default_config_path() / 'arm_3.yaml').read_text())
    (config_path / 'arm_3.yaml').write_text(yaml.safe_dump({**arm3, 'gravity_config': str(output)}))
    loaded_fleet = FleetConfig(config_path)
    payload = loaded_fleet.public(loaded_fleet.arm('arm_3'))['gravity_payload']
    assert payload['total_mass_kg'] == pytest.approx(baseline.with_added_mass(0).effector_body().mass_kg + 0.1)


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


def test_gravity_web_workbench(tmp_path):
    rclpy.init()
    web_node = GravityWorkbenchNode(config_file=default_config_path())
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
        web_node.destroy_node()
        rclpy.shutdown()
