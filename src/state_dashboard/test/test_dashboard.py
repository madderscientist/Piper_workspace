"""Dashboard unit, HTTP and isolated ROS subscription tests; no CAN access."""

from concurrent.futures import ThreadPoolExecutor
from importlib.metadata import distribution
import json
import math
from pathlib import Path
import socket
import threading
import time
from typing import Any
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from state_dashboard.dashboard import (
    DashboardNode,
    DashboardServer,
    DEFAULT_NAMESPACES,
    EnableController,
    joint_payload,
    main,
    normalize_namespaces,
    pose_payload,
    status_payload,
    TelemetryStore,
)
from piper_x_interfaces.msg import ArmStatus
from piper_x_interfaces.srv import Enable
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState


@pytest.fixture
def store():
    return TelemetryStore(list(DEFAULT_NAMESPACES))


def test_package_entry_point_assets_and_namespaces(capsys):
    entry = next(
        entry for entry in distribution('state_dashboard').entry_points
        if entry.group == 'console_scripts' and entry.name == 'state_dashboard')
    assert entry.value == 'state_dashboard.dashboard:main'
    assert entry.load() is main
    with pytest.raises(SystemExit) as exc:
        main(['--help'])
    assert exc.value.code == 0 and '--port' in capsys.readouterr().out

    assets = Path(get_package_share_directory('state_dashboard')) / 'dashboard'
    assert all((assets / filename).is_file()
               for filename in ('index.html', 'app.js', 'controls.js', 'mesh.js', 'styles.css'))
    assert normalize_namespaces(['one/', '/two', '/three/', 'four']) == [
        '/one', '/two', '/three', '/four',
    ]
    for bad in (
        ['one', 'two', 'three'],
        ['one', 'one', 'three', 'four'],
        ['/', 'two', 'three', 'four'],
        ['bad name', 'two', 'three', 'four'],
        ['1arm', 'two', 'three', 'four'],
    ):
        with pytest.raises(ValueError):
            normalize_namespaces(bad)


def test_telemetry_payloads_and_power_state():
    message = JointState()
    message.name = ['joint6', 'joint5', 'joint4', 'joint3', 'joint2', 'joint1', 'gripper']
    message.position = [math.pi, 0.5, 0.4, 0.3, 0.2, 0.1, 0.04]
    # The existing Piper driver omits the trailing gripper velocity.
    message.velocity = [6.0, 5.0, 4.0, 3.0, 2.0, 1.0]
    payload = joint_payload(message)
    assert payload['position_deg'][5] == pytest.approx(180)
    assert payload['position_deg'][0] == pytest.approx(math.degrees(0.1))
    assert payload['velocity_rad_s'] == [1, 2, 3, 4, 5, 6]
    assert payload['effort_nm'] == [None] * 6
    assert payload['gripper_m'] == pytest.approx(0.04)

    for value in (math.nan, math.inf, -math.inf):
        with pytest.raises(ValueError, match='non-finite'):
            joint_payload(JointState(name=['joint1'], position=[value]))
    with pytest.raises(ValueError, match='No Piper'):
        joint_payload(JointState(name=['unrelated'], position=[0.0]))
    with pytest.raises(ValueError, match='same Piper joint'):
        joint_payload(JointState(name=['joint1', 'joint_1'], position=[0.0, 0.0]))

    payload = joint_payload(JointState(name=['joint_2'], position=[1.0]))
    assert payload['position_deg'][0] is None
    assert payload['position_deg'][1] == pytest.approx(math.degrees(1.0))
    assert payload['gripper_m'] is None

    message = PoseStamped()
    message.pose.position.x = 0.123
    message.pose.orientation.z = math.sin(math.pi / 4)
    message.pose.orientation.w = math.cos(math.pi / 4)
    payload = pose_payload(message)
    assert payload['position_mm'] == pytest.approx([123, 0, 0])
    assert payload['rpy_deg'] == pytest.approx([0, 0, 90])
    invalid = PoseStamped()
    invalid.pose.orientation.w = 0.0
    with pytest.raises(ValueError, match='zero-length'):
        pose_payload(invalid)

    message = ArmStatus(err_code=257, arm_status=1)
    message.joint_1_angle_limit = True
    message.communication_status_joint_1 = True
    payload = status_payload(message)
    assert payload['err_code'] == 257
    assert payload['arm_status'] == 1
    assert payload['angle_limits'] == [True, False, False, False, False, False]
    assert payload['communication_errors'] == [True, False, False, False, False, False]
    for enabled, valid, expected in [
        ([False] * 6, [True] * 6, 'disabled'),
        ([True] * 6, [True] * 6, 'enabled'),
        ([True, False, False, False, False, False], [True] * 6, 'mixed'),
        ([False] * 6, [False] * 6, 'unknown'),
        ([True] * 6, [True, True, False, True, True, True], 'unknown'),
    ]:
        out = status_payload(ArmStatus(motor_enabled=enabled, motor_enable_valid=valid))
        assert out['power_state'] == expected


def test_telemetry_store_freshness_frequency_and_concurrency():
    now = [0.0]
    store = TelemetryStore(list(DEFAULT_NAMESPACES), clock=lambda: now[0])
    initial = store.snapshot()
    assert initial['read_only'] is True and len(initial['arms']) == 4
    assert all(
        arm['connection'] == 'waiting' and arm['joints'] is None
        and all(s['age_ms'] is None for s in arm['streams'].values())
        for arm in initial['arms']
    )
    store.update(0, 'joints', {'position_deg': [0.0]})
    assert store.snapshot()['arms'][0]['connection'] == 'partial'
    store.update(0, 'status', {})
    store.update(0, 'pose', {})
    for stamp in (0.5, 1.0, 2.0):
        now[0] = stamp
        store.update(0, 'joints', {'position_deg': [1.0]})
    arm0 = store.snapshot()['arms'][0]
    assert arm0['connection'] == 'live' and arm0['streams']['joints']['hz'] == 1.5

    store.update_control(0, {'debug_active': False, 'last_error': 'err1'})
    now[0] = 2.3
    assert store.snapshot()['arms'][0]['control_fresh']
    now[0] = 2.300001
    assert not store.snapshot()['arms'][0]['control_fresh']
    assert store.snapshot()['arms'][0]['connection'] == 'partial'
    now[0] = 4.001
    assert store.snapshot()['arms'][0]['connection'] == 'stale'
    now[0] = 2.3
    assert store.reject(0, 'joints', 'Invalid sample')
    assert not store.reject(0, 'joints', 'Invalid sample')
    arm = store.snapshot()['arms'][0]
    assert arm['streams']['joints']['error'] == 'Invalid sample'
    assert not arm['streams']['joints']['fresh']
    assert arm['joints']['position_deg'] == [1.0]

    def write(index):
        for value in range(15):
            store.update(index, 'joints', {'value': value})
            json.dumps(store.snapshot(), allow_nan=False)
    with ThreadPoolExecutor(max_workers=4) as executor:
        list(executor.map(write, range(4)))
    snapshot = store.snapshot()
    snapshot['arms'][0]['joints']['value'] = -1
    assert store.snapshot()['arms'][0]['joints']['value'] == 14


@pytest.fixture
def http_server(store):
    assets = Path(__file__).resolve().parents[1] / 'dashboard'
    with DashboardServer(('127.0.0.1', 0), store, assets) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            yield 'http://127.0.0.1:{}'.format(server.server_port)
        finally:
            server.shutdown()
            thread.join(timeout=5)


def test_http_assets_and_readonly_snapshot(http_server):
    for path in ('/', '/app.js', '/controls.js', '/styles.css', '/api/status'):
        with urlopen(http_server + path, timeout=3) as response:
            assert response.status == 200
            assert response.headers['Cache-Control'] == 'no-store'
            assert response.headers['X-Content-Type-Options'] == 'nosniff'
            assert "frame-ancestors 'none'" in response.headers['Content-Security-Policy']
            assert response.read()
    with urlopen(http_server + '/api/status', timeout=3) as response:
        assert len(json.load(response)['arms']) == 4
    with urlopen(Request(http_server + '/', method='HEAD'), timeout=3) as response:
        assert response.read() == b''
    for path in ('/../setup.py', '/setup.py', '/api/control'):
        with pytest.raises(HTTPError) as error:
            urlopen(http_server + path, timeout=3)
        assert error.value.code == 404
    with pytest.raises(HTTPError) as error:
        urlopen(Request(http_server + '/api/status', data=b'{}'), timeout=3)
    assert error.value.code == 404
    for action in ('enable', 'disable'):
        with pytest.raises(HTTPError) as error:
            urlopen(Request(http_server + '/api/arms/1/' + action, data=b'{}'), timeout=3)
        assert error.value.code == 403


class FakeEnableClient:
    """Capture requests without ever creating a real ROS client or CAN socket."""

    def __init__(self, response=True, ready=True):
        self.ready = ready
        self.response = response
        self.requests = []
        self.future = Future()
        self.futures = []
        self.called = threading.Event()

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        if self.requests:
            self.future = Future()
        self.requests.append(request)
        self.futures.append(self.future)
        self.called.set()
        if isinstance(self.response, Exception):
            self.future.set_exception(self.response)
        elif isinstance(self.response, Enable.Response):
            self.future.set_result(self.response)
        elif self.response is not None:
            self.future.set_result(Enable.Response(enable_response=self.response))
        return self.future


def populate_feedback(store):
    for index in range(4):
        store.update(index, 'status', status_payload(
            ArmStatus(motor_enable_valid=[True] * 6)))
        store.update(index, 'joints', joint_payload(
            JointState(name=['joint1'], position=[0.0])))
        store.update(index, 'pose', pose_payload(PoseStamped()))


@pytest.fixture
def enable_setup(store):
    populate_feedback(store)
    clients = [FakeEnableClient() for _ in range(4)]
    return store, clients, EnableController(store, clients, timeout=0.1)


def test_enable_and_disable_routing_and_safety_blocks(enable_setup):
    _, clients, controller = enable_setup
    for index in range(4):
        code, result = controller.enable(index)
        assert code == 200 and result['success'] is True and result['arm_id'] == index + 1
        assert clients[index].requests[-1].enable_request is True
        code, result = controller.disable(index)
        assert code == 200 and result['success'] is True and result['action'] == 'disable'
        assert clients[index].requests[-1].enable_request is False

    for condition, expected in [
        ('waiting', 409), ('stale', 409), ('fault', 409),
        ('limit', 409), ('communication', 409), ('unavailable', 503),
        ('enabled', 409), ('mixed', 409), ('unknown', 409),
    ]:
        now = [0.0]
        s = TelemetryStore(list(DEFAULT_NAMESPACES), clock=lambda: now[0])
        c_list = [FakeEnableClient() for _ in range(4)]
        ctrl = EnableController(s, c_list, timeout=0.1)
        if condition != 'waiting':
            populate_feedback(s)
        if condition == 'stale':
            now[0] = 2.001
        elif condition == 'fault':
            s.update(0, 'status', status_payload(ArmStatus(arm_status=1)))
        elif condition == 'limit':
            s.update(0, 'status', status_payload(ArmStatus(joint_1_angle_limit=True)))
        elif condition == 'communication':
            s.update(0, 'status', status_payload(ArmStatus(communication_status_joint_3=True)))
        elif condition == 'unavailable':
            c_list[0].ready = False
        elif condition in ('enabled', 'mixed', 'unknown'):
            msg: Any = ArmStatus(
                motor_enable_valid=[condition != 'unknown'] * 6,
                motor_enabled=[condition == 'enabled'] * 6)
            if condition == 'mixed':
                msg.motor_enabled[0] = True
            s.update(0, 'status', status_payload(msg))
        code, res = ctrl.enable(0)
        assert code == expected and res['success'] is False
        assert not any(c.requests for c in c_list)
        if condition in ('waiting', 'stale', 'fault'):
            assert ctrl.disable(0)[0] == 200


def test_enable_failures_concurrency_and_disable_priority(enable_setup, monkeypatch):
    _, clients, controller = enable_setup
    for response in (False, RuntimeError('Service unavailable')):
        clients[2].response = response
        code, result = controller.enable(2)
        assert code == 502 and result['success'] is False
        assert controller.snapshot()[2]['outcome'] == 'failed'

    detail = 'Cannot enable with invalid measured gripper width: -10.800 mm; expected 0..100 mm'
    clients[1].response = Enable.Response(enable_response=False, message=detail)
    for action, label in (('enable', '使能'), ('disable', '失能')):
        code, result = getattr(controller, action)(1)
        assert code == 502 and label in result['message'] and detail in result['message']

    clients[3].response = None
    clients[3].future.cancel()
    assert controller.enable(3)[0] == 502 and not controller.snapshot()[3]['pending']

    def fail(_):
        raise RuntimeError('ROS down')

    monkeypatch.setattr(clients[3], 'call_async', fail)
    code, result = controller.enable(3)
    assert code == 503 and 'ROS down' in result['message']

    clients[0].response = None
    controller.timeout = 3
    with ThreadPoolExecutor(max_workers=2) as worker:
        enabling = worker.submit(controller.enable, 0)
        assert clients[0].called.wait(1)
        disabling = worker.submit(controller.disable, 0)
        deadline = time.monotonic() + 1
        while len(clients[0].requests) < 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert [r.enable_request for r in clients[0].requests] == [True, False]
        assert controller.disable(0)[0] == 409 and controller.enable(0)[0] == 409
        clients[0].futures[0].set_result(Enable.Response(enable_response=True))
        assert enabling.result(timeout=1)[0] == 200
        clients[0].futures[1].set_result(Enable.Response(enable_response=True))
        assert disabling.result(timeout=1)[0] == 200


@pytest.fixture
def control_http_server(enable_setup):
    store, clients, controller = enable_setup
    assets = Path(__file__).resolve().parents[1] / 'dashboard'
    with DashboardServer(('127.0.0.1', 0), store, assets, controller) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            yield 'http://127.0.0.1:{}'.format(server.server_port), server, clients
        finally:
            server.shutdown()
            thread.join(timeout=5)


def enable_http_request(url, server, path='/api/arms/2/enable', headers=None, data=b'{}'):
    request_headers = {
        'Content-Type': 'application/json',
        'X-Piper-Control-Token': server.control_token,
        'Origin': url,
    }
    if headers:
        request_headers.update(headers)
    return Request(url + path, data=data, headers=request_headers, method='POST')


def test_http_power_routes_and_request_validation(control_http_server):
    url, server, clients = control_http_server
    with urlopen(url + '/api/status', timeout=3) as response:
        snapshot = json.load(response)
    assert snapshot['read_only'] is False and snapshot['control_token'] == server.control_token

    with pytest.raises(HTTPError) as error:
        urlopen(url + '/api/arms/2/enable', timeout=3)
    assert error.value.code == 404 and not any(c.requests for c in clients)

    for headers, expected in [
        ({'X-Piper-Control-Token': ''}, 403),
        ({'X-Piper-Control-Token': 'wrong'}, 403),
        ({'Origin': 'https://untrusted.invalid'}, 403),
        ({'Origin': 'null'}, 403),
        ({'Origin': 'http://['}, 403),
        ({'Sec-Fetch-Site': 'cross-site'}, 403),
        ({'Content-Type': 'text/plain'}, 400),
        ({'Content-Length': '0'}, 400),
        ({'Content-Length': '129'}, 400),
        ({'Content-Length': 'invalid'}, 400),
    ]:
        with pytest.raises(HTTPError) as error:
            urlopen(enable_http_request(url, server, headers=headers), timeout=3)
        assert error.value.code == expected

    with socket.create_connection(server.server_address, timeout=3) as connection:
        packet = (
            f'POST /api/arms/2/enable HTTP/1.0\r\n'
            f'Host: {server.server_address[0]}:{server.server_port}\r\n'
            f'X-Piper-Control-Token: {server.control_token}\r\n'
            'Content-Type: application/json\r\n'
            'Transfer-Encoding: chunked\r\n\r\n2\r\n{}\r\n0\r\n\r\n'
        )
        connection.sendall(packet.encode('ascii'))
        with connection.makefile('rb') as response:
            assert int(response.readline().split()[1]) == 400

    for data in (b'[]', b'null', b'{"enable":true}', b'{'):
        with pytest.raises(HTTPError) as error:
            urlopen(enable_http_request(url, server, data=data), timeout=3)
        assert error.value.code == 400

    for path in (
        '/api/arms/0/enable', '/api/arms/5/enable',
        '/api/arms/1/stop', '/api/arms/1/enable?arm=2',
    ):
        with pytest.raises(HTTPError) as error:
            urlopen(enable_http_request(url, server, path=path), timeout=3)
        assert error.value.code == 404
    assert not any(c.requests for c in clients)

    with urlopen(enable_http_request(url, server), timeout=3) as response:
        assert json.load(response)['success'] is True
    req_disable = enable_http_request(url, server, path='/api/arms/4/disable')
    with urlopen(req_disable, timeout=3) as response:
        assert json.load(response)['action'] == 'disable'
    assert [len(c.requests) for c in clients] == [0, 1, 0, 1]

    detail = 'Measured gripper width -10.800 mm is outside 0..100 mm'
    clients[1].response = Enable.Response(enable_response=False, message=detail)
    with pytest.raises(HTTPError) as error:
        urlopen(enable_http_request(url, server, path='/api/arms/2/enable'), timeout=3)
    assert error.value.code == 502
    result = json.load(error.value)
    assert detail in result['message']


def test_ros_subscriptions_and_enable_service_integration():
    namespaces = [f'/dashboard_test/arm_{i}' for i in range(1, 5)]
    rclpy.init(args=[])
    dashboard = DashboardNode(namespaces)
    source = Node('dashboard_test_source')
    executor = SingleThreadedExecutor()
    executor.add_node(dashboard)
    executor.add_node(source)
    calls = []
    try:
        publishers = []
        for index, namespace in enumerate(namespaces):
            joints = JointState(name=['joint1'], position=[float(index + 1)])
            status = ArmStatus(ctrl_mode=index, motor_enable_valid=[True] * 6)
            pose = PoseStamped()
            pose.pose.orientation.w = 1.0
            for message, topic in [
                (joints, 'joint_states_feedback'), (status, 'arm_status'),
                (pose, 'end_pose_stamped'),
            ]:
                publisher = source.create_publisher(type(message), namespace + '/' + topic, 10)
                publishers.append((publisher, message))

            def handle(request, response, index=index):
                calls.append((index, request.enable_request))
                response.enable_response = index != 3
                return response
            source.create_service(Enable, namespace + '/enable_srv', handle)

        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            for publisher, message in publishers:
                publisher.publish(message)
            executor.spin_once(timeout_sec=0.05)
            snapshot = dashboard.store.snapshot()
            if all(arm['connection'] == 'live' for arm in snapshot['arms']):
                break
        assert all(arm['connection'] == 'live' for arm in snapshot['arms'])
        topics = dashboard.get_publisher_names_and_types_by_node(
            dashboard.get_name(), dashboard.get_namespace())
        assert {name for name, _ in topics} <= {'/rosout', '/parameter_events'}

        for client in dashboard.enable_controller.clients:
            assert client.wait_for_service(timeout_sec=3)
        with ThreadPoolExecutor(max_workers=1) as worker:
            for index in (0, 3):
                future = worker.submit(dashboard.enable_controller.enable, index)
                deadline = time.monotonic() + 4
                while not future.done() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.02)
                code, result = future.result(timeout=1)
                assert code == (502 if index == 3 else 200)
                assert result['success'] is (index != 3)
        assert calls == [(0, True), (3, True)]
    finally:
        executor.shutdown()
        dashboard.destroy_node()
        source.destroy_node()
        rclpy.try_shutdown()
