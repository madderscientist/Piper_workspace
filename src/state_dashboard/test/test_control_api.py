import json
import math
from pathlib import Path
import threading
from typing import Any
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from piper_x_interfaces.srv import CalibrateEffector, ConfigureArm, SetMode
import pytest
from rclpy.task import Future
from std_srvs.srv import Trigger

from state_dashboard.control import ArmController, TeachingController
from state_dashboard.dashboard import DashboardServer, DEFAULT_NAMESPACES, TelemetryStore


class Client:
    def __init__(self):
        self.ready = True
        self.calls = []
        self.success = True
        self.pending = False

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        future = Future()
        if not self.pending:
            if isinstance(request, CalibrateEffector.Request):
                response = CalibrateEffector.Response(
                    success=self.success, message='test response')
            elif isinstance(request, ConfigureArm.Request):
                response = ConfigureArm.Response(success=self.success, message='test response')
            elif isinstance(request, SetMode.Request):
                response = SetMode.Response(success=self.success, message='test response')
            elif isinstance(request, Trigger.Request):
                response = Trigger.Response(success=self.success, message='teaching ok')
            else:
                raise TypeError(f'Unexpected request {type(request)}')
            future.set_result(response)
        return future


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def controller() -> Any:
    pubs = {}
    ctrl: Any = ArmController(
        [Client() for _ in range(4)], [Client() for _ in range(4)],
        [Client() for _ in range(4)],
        lambda idx, kind: pubs.setdefault((idx, kind), Publisher()),
        timeout=0.01)
    ctrl.pubs = pubs
    return ctrl


@pytest.fixture
def teaching() -> TeachingController:
    return TeachingController(Client(), Client(), Client(), timeout=0.01)


@pytest.fixture
def http(controller, teaching):
    store = TelemetryStore(list(DEFAULT_NAMESPACES))
    assets = Path(__file__).resolve().parents[1] / 'dashboard'
    with DashboardServer(
        ('127.0.0.1', 0), store, assets,
        arm_controller=controller, teaching_controller=teaching,
    ) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            yield f'http://127.0.0.1:{server.server_port}', server.control_token
        finally:
            server.shutdown()
            thread.join(timeout=3)


def post(http, index, action, body, token=None):
    path = f'/api/arms/{index}/{action}' if index is not None else f'/api/teaching/{action}'
    url, correct_token = http
    request = Request(
        f'{url}{path}', data=json.dumps(body).encode(),
        headers={'Content-Type': 'application/json',
                 'X-Piper-Control-Token': correct_token if token is None else token})
    try:
        with urlopen(request, timeout=3) as response:
            return response.status, json.load(response)
    except HTTPError as error:
        return error.code, json.load(error)


def test_configure_and_calibrate_dispatch_only_to_selected_arm(http, controller):
    for index in range(1, 5):
        status, payload = post(http, index, 'configure', {
            'end_effector': 'teaching_handle', 'gravity_scale': 0.4, 'friction_scale': 0.8})
        assert status == 200 and payload['success']
        req = controller.config_clients[index - 1].calls[-1]
        assert req.gravity_scale == 0.4 and req.friction_scale == 0.8

        status, payload = post(http, index, 'calibrate', {'confirm_closed': True})
        assert status == 200 and payload['success']
        assert controller.calibration_clients[index - 1].calls[-1].confirm_closed is True

    assert [len(c.calls) for c in controller.config_clients] == [1, 1, 1, 1]
    assert [len(c.calls) for c in controller.calibration_clients] == [1, 1, 1, 1]


def test_mit_session_commands_and_mesh_endpoints(http, controller):
    url, _ = http
    with urlopen(url + '/mesh.js', timeout=3) as response:
        assert response.status == 200 and b'PiperMesh' in response.read()
    with urlopen(url + '/meshes/base_link.STL', timeout=3) as response:
        assert response.status == 200 and len(response.read()) > 84

    for angle_deg in (-2.0, 195.0):
        position = [0.0, math.radians(angle_deg), -0.5, 0.0, 0.0, 0.0]
        status, _ = post(http, 2, 'mit/command', {
            'position': position, 'velocity': [0.0] * 6,
            'kp': [-1.0] * 6, 'kd': [-1.0] * 6, 'torque': [0.0] * 6,
        })
        assert status == 200
        assert list(controller.pubs[(1, 'mit')].messages[-1].position) == position

    status, _ = post(http, 1, 'mode', {'mode': 'joint'})
    assert status == 200 and controller.mode_clients[0].calls[-1].mode == 'joint'
    status, _ = post(http, 1, 'joints', {
        'position': [0.1, 0.2, -0.3, 0.0, 0.0, 0.0], 'speed_percent': 25})
    assert status == 200 and controller.pubs[(0, 'joints')].messages[-1].speed_percent == 25
    status, _ = post(http, 1, 'cartesian', {
        'pose': [0.2, 0.0, 0.2, 0.0, 0.0, 0.0], 'speed_percent': 30})
    assert status == 200 and controller.pubs[(0, 'cartesian')].messages[-1].speed_percent == 30
    status, _ = post(http, 1, 'gripper', {'width': 0.04, 'effort': 1.5})
    assert status == 200 and controller.pubs[(0, 'gripper')].messages[-1].width == 0.04
    status, _ = post(http, 1, 'mit/command', {
        'position': [0.0] * 6, 'velocity': [0.0] * 6,
        'kp': [10.0] * 6, 'kd': [0.5] * 6, 'torque': [0.0] * 6,
    })
    assert status == 200 and list(controller.pubs[(0, 'mit')].messages[-1].kp) == [10.0] * 6


def test_teaching_validation_and_failure_handling(http, controller, teaching):
    assert post(http, None, 'start', {})[0] == 200
    assert post(http, None, 'stop', {})[0] == 200
    assert post(http, None, 'mode', {'mode': 'bilateral'})[0] == 200
    assert teaching.mode_client.calls[-1].mode == 'bilateral'
    assert post(http, None, 'start', {'extra': 1})[0] == 400
    assert post(http, None, 'mode', {'mode': 'invalid'})[0] == 400

    assert post(http, 1, 'mode', {'mode': 'mit'}, token='wrong')[0] == 403
    for action, body in [
        ('configure', {'end_effector': 'gripper', 'gravity_scale': True}),
        ('configure', {'end_effector': 'gripper', 'gravity_scale': 1.01}),
        ('configure', {'end_effector': 'gripper'}),
        ('calibrate', {'confirm_closed': False}),
        ('calibrate', {'confirm_closed': True, 'extra': 1}),
        ('mit/command', {'position': [0.0]}),
        ('mit/command', {'position': [20.0] * 6, 'velocity': [0.0] * 6, 'kp': [1.0] * 6, 'kd': [0.1] * 6, 'torque': [0.0] * 6}),
        ('mode', {'mode': 'invalid'}),
    ]:
        assert post(http, 1, action, body)[0] == 400
    assert post(http, 1, 'mit/start', {})[0] == 404
    assert not any(
        c.calls for c in (
            controller.config_clients + controller.calibration_clients
        )
    )

    for failure, code in [('unavailable', 503), ('rejected', 409), ('timeout', 504)]:
        for action, body, attr in [
            ('mode', {'mode': 'mit'}, 'mode_clients'),
            ('calibrate', {'confirm_closed': True}, 'calibration_clients'),
        ]:
            client = getattr(controller, attr)[0]
            client.calls.clear()
            client.ready = failure != 'unavailable'
            client.success = failure != 'rejected'
            client.pending = failure == 'timeout'
            status, result = controller.request(0, action, body)
            assert status == code and not result['success']
            assert len(client.calls) == (0 if failure == 'unavailable' else 1)
