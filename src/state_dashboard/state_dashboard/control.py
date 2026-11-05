"""Bounded HTTP-to-ROS requests; actuator streaming remains in the driver."""

import logging
import math
import threading
import time

from piper_x_interfaces.msg import CartesianCommand, GripperCommand, JointCommand, MitCommand
from piper_x_interfaces.srv import CalibrateEffector, ConfigureArm, DebugMit, SetMode
from piper_x_driver.configuration import bounded
from piper_x_driver.controller import LOWER, MODES, UPPER, vector
from std_srvs.srv import Trigger


LOGGER = logging.getLogger('state_dashboard.control')


def _finite_list(values, name: str, length: int = 6) -> list[float]:
    if (not isinstance(values, list) or len(values) != length
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in values)):
        raise ValueError(f'{name} requires {length} finite numbers')
    return [float(v) for v in values]


def _speed_percent(value) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 100:
        raise ValueError('speed_percent must be an integer in [1, 100]')
    return value


def drain_client(client) -> None:
    client_handle = getattr(client, 'handle', None)
    srv_type = getattr(client, 'srv_type', None)
    if client_handle is None or srv_type is None or not hasattr(client, 'get_pending_request'):
        return
    while True:
        with client_handle:
            hdr, resp = client_handle.take_response(srv_type.Response)
        if hdr is None:
            break
        try:
            fut = client.get_pending_request(hdr.request_id.sequence_number)
        except KeyError:
            pass
        else:
            fut.set_result(resp)


def _call_service(client, request, timeout: float, action: str = ''):
    if client is None or not client.service_is_ready():
        return ArmController._error(503, 'ROS control service unavailable')
    future = client.call_async(request)
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if getattr(client, 'handle', None) is not None and hasattr(client, 'get_pending_request'):
        deadline = time.monotonic() + timeout
        while not done.is_set() and time.monotonic() < deadline:
            drain_client(client)
            if not done.is_set():
                done.wait(0.002)
    else:
        done.wait(timeout)
    if not done.is_set():
        message = 'ROS response timed out; result unknown, do not retry automatically.'
        if action == 'calibrate':
            message += ' Zeroing may already have reached the device; inspect feedback.'
        elif action.startswith('mit/'):
            message += ' MIT sessions expire without heartbeats.'
        return ArmController._error(504, message)
    if future.cancelled() or future.exception() is not None:
        return ArmController._error(
            502, f'ROS request failed: {future.exception()}'
            if not future.cancelled() else 'ROS request cancelled')
    response = future.result()
    if not response.success:
        return ArmController._error(409, response.message)
    result = {'success': True, 'message': response.message}
    if isinstance(response, DebugMit.Response) and response.session:
        result['session'] = response.session
        result['position'] = list(response.position)
    return 200, result


class ArmController:
    def __init__(
        self, config_clients, debug_clients, calibration_clients,
        mode_clients=None, command_publisher=None, timeout=2.0, log_callback=None,
    ):
        self.config_clients = config_clients
        self.debug_clients = debug_clients
        self.calibration_clients = calibration_clients
        self.mode_clients = mode_clients
        self.command_publisher = command_publisher
        self.timeout = timeout
        self.log_callback = log_callback
        self._service_locks = [threading.Lock() for _ in range(max(4, len(config_clients)))]

    def _log(self, index: int, action: str, status: int, message: str) -> None:
        if self.log_callback is not None:
            level = 'INFO' if status == 200 else 'ERROR'
            self.log_callback(level, f'arm_{index + 1}.{action}', message)

    def _publish(self, index: int, topic: str, message, summary: str):
        if self.command_publisher is None:
            return self._error(503, 'ROS command publisher unavailable')
        self.command_publisher(index, topic).publish(message)
        self._log(index, topic, 200, summary)
        return 200, {'success': True, 'message': summary}

    def request(self, index, action, body):
        try:
            if not isinstance(body, dict):
                raise ValueError('JSON object required')
            if action == 'calibrate':
                if set(body) != {'confirm_closed'} or body['confirm_closed'] is not True:
                    raise ValueError('confirm_closed must be explicitly true')
                client = self.calibration_clients[index]
                request = CalibrateEffector.Request(confirm_closed=True)
            elif action == 'configure':
                allowed = {'end_effector', 'gravity_scale', 'friction_scale'}
                if (not {'end_effector', 'gravity_scale'}.issubset(body)
                        or not set(body).issubset(allowed)):
                    raise ValueError('end_effector and gravity_scale required')
                if not isinstance(body['end_effector'], str):
                    raise ValueError('end_effector must be a string')
                scale = bounded(body['gravity_scale'], 'gravity_scale', 0, 1)
                req_kwargs = {'end_effector': body['end_effector'], 'gravity_scale': scale}
                if 'friction_scale' in body:
                    req_kwargs['friction_scale'] = bounded(
                        body['friction_scale'], 'friction_scale', 0, 1)
                client = self.config_clients[index]
                request = ConfigureArm.Request(**req_kwargs)
            elif action == 'mode':
                if set(body) != {'mode'} or body['mode'] not in MODES:
                    raise ValueError(f'mode must be one of {MODES}')
                client = None if self.mode_clients is None else self.mode_clients[index]
                request = SetMode.Request(mode=body['mode'])
            elif action == 'joints':
                if set(body) != {'position', 'speed_percent'}:
                    raise ValueError('position and speed_percent required')
                return self._publish(
                    index, 'joints',
                    JointCommand(
                        position=_finite_list(body['position'], 'position'),
                        speed_percent=_speed_percent(body['speed_percent']),
                    ),
                    '已发送六轴关节目标（command/joints）',
                )
            elif action == 'cartesian':
                if set(body) != {'pose', 'speed_percent'}:
                    raise ValueError('pose and speed_percent required')
                return self._publish(
                    index, 'cartesian',
                    CartesianCommand(
                        pose=_finite_list(body['pose'], 'pose'),
                        speed_percent=_speed_percent(body['speed_percent']),
                    ),
                    '已发送笛卡尔位姿目标（command/cartesian）',
                )
            elif action == 'gripper':
                if set(body) != {'width', 'effort'}:
                    raise ValueError('width and effort required')
                return self._publish(
                    index, 'gripper',
                    GripperCommand(
                        width=bounded(body['width'], 'width', 0.0, 0.1),
                        effort=bounded(body['effort'], 'effort', 0.0, 5.0),
                    ),
                    '已发送夹爪指令（command/gripper）',
                )
            elif action == 'mit/command':
                fields = ('position', 'velocity', 'kp', 'kd', 'torque')
                if set(body) != set(fields):
                    raise ValueError('Complete MIT command fields required')
                arrays = {name: _finite_list(body[name], name) for name in fields}
                vector(arrays['position'], 'MIT position encoding', LOWER, UPPER)
                return self._publish(
                    index, 'mit',
                    MitCommand(
                        position=arrays['position'], velocity=arrays['velocity'],
                        kp=[bounded(v, 'kp', 0.0, 500.0) for v in arrays['kp']],
                        kd=[bounded(v, 'kd', 0.0, 100.0) for v in arrays['kd']],
                        torque=arrays['torque'],
                    ),
                    '已发送自定义 MIT 指令（command/mit）',
                )
            else:
                operation = action.removeprefix('mit/')
                fields = {
                    'start': set(), 'target': {'session', 'position'}, 'stop': {'session'},
                }
                if operation not in fields or set(body) != fields[operation]:
                    raise ValueError('Invalid MIT operation fields')
                request = DebugMit.Request(action=operation)
                if operation != 'start':
                    session = body['session']
                    if not isinstance(session, str) or not 1 <= len(session) <= 128:
                        raise ValueError('Valid MIT session required')
                    request.session = session
                if operation == 'target':
                    position = _finite_list(body['position'], 'Six numeric joint positions')
                    vector(position, 'MIT position encoding', LOWER, UPPER)
                    request.position = position
                client = self.debug_clients[index]
            with self._service_locks[index]:
                status, result = _call_service(client, request, self.timeout, action)
            self._log(index, action, status, str(result.get('message', '')))
            return status, result
        except ValueError as error:
            self._log(index, action, 400, str(error))
            return self._error(400, str(error))
        except RuntimeError as error:
            self._log(index, action, 503, str(error))
            return self._error(503, str(error))

    @staticmethod
    def _error(status, message):
        LOGGER.warning('%s', message)
        return status, {'success': False, 'message': message}


class TeachingController:
    def __init__(self, start_client, stop_client, mode_client, timeout=2.0):
        self.start_client = start_client
        self.stop_client = stop_client
        self.mode_client = mode_client
        self.timeout = timeout

    def request(self, action, body):
        try:
            if not isinstance(body, dict):
                raise ValueError('JSON object required')
            if action in ('start', 'stop'):
                if body:
                    raise ValueError(f'{action} does not accept parameters')
                client = self.start_client if action == 'start' else self.stop_client
                return _call_service(client, Trigger.Request(), self.timeout)
            if action == 'mode':
                if set(body) != {'mode'} or body['mode'] not in ('master_slave', 'bilateral'):
                    raise ValueError('mode must be master_slave or bilateral')
                return _call_service(
                    self.mode_client, SetMode.Request(mode=body['mode']), self.timeout)
            raise ValueError('Unknown teaching operation')
        except ValueError as error:
            return ArmController._error(400, str(error))
        except RuntimeError as error:
            return ArmController._error(503, str(error))
