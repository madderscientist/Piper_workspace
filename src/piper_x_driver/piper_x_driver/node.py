"""ROS adapter; all actuator access stays in this one SDK-owning process."""

from concurrent.futures import ThreadPoolExecutor
from functools import partial
import json
import os
import select
import signal
import time
from typing import Any

from can import CanError
from geometry_msgs.msg import PoseStamped
from piper_x_interfaces.msg import (
    ArmStatus, CartesianCommand, GripperCommand, JointCommand, MitCommand,
)
from piper_x_interfaces.srv import (
    CalibrateEffector, ConfigureArm, DebugMit, Enable, SetMode,
)
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import yaml

from .backend import fresh, SdkArm
from .controller import Controller, SLIDER_LOWER, SLIDER_UPPER
from .configuration import default_config_path, FleetConfig


_JOINT_NAMES = [f'joint{i}' for i in range(1, 7)] + ['gripper']


class DriverNode(Node):
    def __init__(self, **kwargs):
        super().__init__('piper_x_driver', **kwargs)
        self.declare_parameter('config_file', str(default_config_path()))
        self.fleet = FleetConfig(str(self.get_parameter('config_file').value))
        namespace = self.get_namespace()
        default_arm = next(
            (arm.id for arm in self.fleet.arms if arm.namespace == namespace),
            self.fleet.arms[0].id if namespace == '/' else '')
        self.declare_parameter('arm_id', default_arm)
        self.arm_config = self.fleet.arm(str(self.get_parameter('arm_id').value))
        if namespace not in ('/', self.arm_config.namespace):
            raise ValueError('Driver namespace does not match configured arm')
        self.arm: Any = SdkArm(self.arm_config.can_port)
        self.control = Controller(
            self.arm, effector=self.fleet.effectors[self.arm_config.end_effector],
            gravity_scale=self.arm_config.gravity_scale,
            debug_kp=self.arm_config.debug_kp, debug_kd=self.arm_config.debug_kd)
        self.last_error = ''
        self.teaching_active = False
        self.teaching_received = 0.0
        self.power_request = None
        self.power_serial = 0
        self.calibration_active = False
        self.calibration_confirm_after = None
        self.calibration_worker = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix='effector-zero')
        self.power_group = ReentrantCallbackGroup()
        self.joints_pub = self.create_publisher(JointState, 'joint_states_feedback', 10)
        self.status_pub = self.create_publisher(ArmStatus, 'arm_status', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'end_pose_stamped', 10)
        self.control_pub = self.create_publisher(String, 'control_state', 10)
        self.create_service(Enable, 'enable_srv', self._enable, callback_group=self.power_group)
        self.create_service(CalibrateEffector, 'calibrate_effector', self._calibrate_effector,
                            callback_group=self.power_group)
        self.create_service(SetMode, 'set_mode', self._mode)
        self.create_service(ConfigureArm, 'configure_arm', self._configure)
        self.create_service(DebugMit, 'debug_mit', self._debug)
        for kind, topic, handler in (
            (JointCommand, 'command/joints', self.control.joints),
            (CartesianCommand, 'command/cartesian', self.control.cartesian),
            (MitCommand, 'command/mit', self.control.mit),
            (GripperCommand, 'command/gripper', self.control.gripper),
        ):
            self.create_subscription(kind, topic, partial(self._command, handler), 1)
        self.create_subscription(Bool, '/bilateral_teaching/active', self._teaching, 1)
        self.teaching_start_client = self.create_client(Trigger, '/bilateral_teaching/start')
        try:
            self.control.start_mit()
        except (ValueError, RuntimeError, CanError) as error:
            self._error(error)
        self.create_timer(0.02, self._publish)
        self.get_logger().info('REAL HARDWARE: started in MIT mode with gravity compensation')

    def _error(self, error):
        message = str(error)
        if message != self.last_error:
            self.get_logger().error(message)
        self.last_error = message

    def _teaching(self, message):
        self.teaching_active = message.data
        self.teaching_received = time.monotonic()

    def _command(self, handler, message):
        try:
            if self.calibration_active:
                raise ValueError('Effector calibration in progress; commands rejected')
            if self.control.debug_session:
                raise ValueError('MIT debugger owns this arm; external commands rejected')
            handler(message)
        except (ValueError, RuntimeError, CanError) as error:
            self._error(error)

    def _configure(self, request, response):
        try:
            if self.calibration_active:
                raise ValueError('Wait for effector calibration to finish before configuring')
            if request.end_effector != self.arm_config.end_effector:
                sample = self.arm.snapshot()
                if (self.teaching_active or self.control.mode == 'mit'
                        or any(sample.enabled)
                        or not fresh(sample.age[3:4] + sample.age[5:])):
                    raise ValueError('Stop MIT/teaching and confirm all seven axes disabled '
                                     'before changing the end effector')
            updated = self.fleet.save_arm(
                self.arm_config, request.end_effector, request.gravity_scale)
            self.fleet = updated
            self.arm_config = updated.arm(self.arm_config.id)
            self.control.effector = updated.effectors[self.arm_config.end_effector]
            self.control.gravity_scale = self.arm_config.gravity_scale
            response.success = True
            response.message = 'Configuration saved and applied; no power or mode change'
        except (ValueError, RuntimeError, OSError, yaml.YAMLError) as error:
            self._error(error)
            response.message = str(error)
        self._publish_control_state()
        return response

    def _debug(self, request, response):
        try:
            if self.calibration_active:
                raise ValueError('Wait for effector calibration to finish before MIT debugging')
            if self.teaching_active:
                raise ValueError('Teaching owns the arm; stop teaching before MIT debugging')
            if request.action == 'start':
                if self.power_request is not None:
                    raise ValueError('Wait for the current power request to finish')
                response.session = self.control.debug_start()
                response.position = list(self.control.debug_target)
            elif request.action == 'target':
                self.control.debug_update(request.session, request.position)
            elif request.action == 'stop':
                self.control.debug_stop(request.session)
            else:
                raise ValueError('Unknown MIT debugging action')
            response.success = True
            response.message = 'Request accepted; inspect actual mode and enable feedback'
        except (ValueError, RuntimeError, CanError) as error:
            self._error(error)
            response.message = str(error)
        self._publish_control_state()
        return response

    def _calibration_feedback(self, sample=None):
        if self.power_request is not None or self.teaching_active:
            raise ValueError(
                'Stop teaching and wait for power requests to finish before calibration')
        if (self.control.mode != 'standby' or self.control.debug_session
                or (self.control.mit_received is not None
                    and self.control.clock() - self.control.mit_received <= 0.2)):
            raise ValueError('Stop MIT/other controllers and select standby before calibration')
        sample = self.control.healthy(enabled=False, sample=sample)
        if any(sample.enabled):
            raise ValueError('Fresh feedback must confirm all six joints and effector disabled')
        status = sample.status
        if (status['arm_status'] != 0 or status['teach_status'] not in (0, 2, 6)
                or status['ctrl_mode'] not in (0, 1, 2)
                or status['mode_feedback'] not in (0, 1, 2, 3, 5)):
            raise ValueError('Leave active native teaching/MIT/remote control before calibration')
        return sample

    def _calibration_block_reason(self, sample=None):
        if self.calibration_active:
            return 'Effector calibration in progress; do not repeat the request'
        try:
            self._calibration_feedback(sample)
        except (ValueError, RuntimeError, CanError) as error:
            return str(error)
        return None

    async def _calibrate_effector(self, request, response):
        if not request.confirm_closed:
            response.message = (
                'Explicit confirmation of the physically closed, unloaded zero required')
            self._error(response.message)
            return response
        reason = self._calibration_block_reason()
        if reason:
            response.message = reason
            self._error(reason)
            return response
        self.calibration_active = True
        self.calibration_confirm_after = None
        serial = self.power_serial
        future = Future()
        timer = None
        try:
            self.get_logger().info(
                'Operator requested effector zero calibration; all axes disabled')
            # The SDK waits synchronously for CAN acknowledgement; keep ROS callbacks responsive.
            work = self.calibration_worker.submit(self.arm.calibrate_effector)
            timer = self.create_timer(
                0.02, lambda: self._check_calibration(work, future, serial),
                callback_group=self.power_group)
            self._publish_control_state()
            width: Any = await future
            response.success = True
            response.message = (
                f'Device acknowledged effector zero calibration; '
                f'fresh opening {width * 1000:.3f} mm; '
                'all seven axes disabled, no enable or motion targets sent. '
                'Power-cycle persistence not verified.')
            self.get_logger().info(response.message)
            self.last_error = ''
        except (ValueError, RuntimeError, CanError, OSError) as error:
            response.message = f'Calibration failed or unconfirmed: {error}; no automatic retry'
            self._error(response.message)
        finally:
            if timer is not None:
                self.destroy_timer(timer)
            self.calibration_active = False
            self._publish_control_state()
        return response

    def _check_calibration(self, work, future, serial):
        if future.done() or not work.done():
            return
        try:
            if self.calibration_confirm_after is None:
                work.result()
                self.calibration_confirm_after = time.monotonic()
            if serial != self.power_serial:
                raise ValueError(
                    'Calibration superseded by a power request; inspect actual feedback')
            sample = self._calibration_feedback()
            elapsed = time.monotonic() - self.calibration_confirm_after
            if (all(0 <= age <= elapsed for age in sample.age)
                    and 0 <= sample.position[6] <= 0.0001):
                future.set_result(sample.position[6])
            elif elapsed > 0.5:
                raise ValueError(
                    'Firmware acknowledged calibration but fresh disabled feedback with '
                    'opening 0..0.1 mm was not confirmed within 0.5 s; result unverified')
        except (ValueError, RuntimeError, CanError, OSError) as error:
            future.set_exception(error)

    def _publish_control_state(self, sample=None):
        self.control_pub.publish(String(data=json.dumps({
            'configuration': self.fleet.public(self.arm_config),
            'mode': self.control.mode,
            'mit_ready': self.control.mode == 'mit' and self.control.mit_ready,
            'debug_active': bool(self.control.debug_session),
            'debug_target': self.control.debug_target if self.control.debug_session else None,
            'joint_lower': list(SLIDER_LOWER), 'joint_upper': list(SLIDER_UPPER),
            'gravity_torque': list(self.control.gravity),
            'output_torque': list(self.control.output_torque),
            'gravity_scale_applied': self.control.gravity_scale_applied,
            'holding': bool(self.control.hold_reason),
            'hold_reason': self.control.hold_reason,
            'calibration_active': self.calibration_active,
            'calibration_blocked_reason': self._calibration_block_reason(sample),
            'last_error': self.last_error,
        }, allow_nan=False)))

    async def _enable(self, request, response):
        if request.enable_request and self.calibration_active:
            response.message = 'Effector calibration in progress; enabling rejected'
            return response
        if request.enable_request and self.control.debug_session:
            response.message = 'MIT debugger is active; use stop/disable first'
            return response
        try:
            started = time.monotonic()
            self.power_serial += 1
            serial = self.power_serial
            self.power_request = request.enable_request
            self.control.power(request.enable_request)
            # Yield to the same executor so stop/feedback callbacks remain responsive.
            future = Future()
            timer = self.create_timer(0.02, lambda: self._check_power(
                request.enable_request, started, future, serial), callback_group=self.power_group)
            try:
                await future
            finally:
                self.destroy_timer(timer)
            response.enable_response = True
            response.message = 'Fresh feedback confirms all seven enable flags'
        except (ValueError, RuntimeError, CanError) as error:
            self._error(error)
            response.message = str(error)
        finally:
            if self.power_serial == serial:
                self.power_request = None
        return response

    def _check_power(self, enabled, started, future, serial):
        if future.done():
            return
        if enabled and serial != self.power_serial:
            future.set_exception(ValueError('Power request superseded by a later request'))
            return
        try:
            sample = self.arm.snapshot()
            power_ages = sample.age[3:4] + sample.age[5:]
            fresh_after_request = fresh(power_ages) and all(
                0 <= age <= time.monotonic() - started for age in power_ages)
            if not enabled and self.control.mode == 'drag':
                future.set_result(True)
            elif (fresh_after_request
                    and (all(sample.enabled[:6]) if enabled else not any(sample.enabled))):
                future.set_result(True)
            elif time.monotonic() - started > 3.0:
                future.set_exception(ValueError('Power request not confirmed within 3 s'))
        except (ValueError, RuntimeError, CanError) as error:
            if time.monotonic() - started > 3.0:
                future.set_exception(error)

    def _mode(self, request, response):
        try:
            if self.calibration_active:
                raise ValueError('Wait for effector calibration to finish before changing modes')
            self.control.set_mode(request.mode)
            response.success = True
            response.message = 'Request sent; inspect ctrl_mode/mode_feedback/teach_status'
        except (ValueError, RuntimeError, CanError) as error:
            self._error(error)
            response.message = str(error)
        return response

    def _publish(self):
        if self.teaching_active and time.monotonic() - self.teaching_received > 0.3:
            self.teaching_active = False
            self.get_logger().error('Teaching heartbeat expired')
        try:
            sample = self.control.watchdog()
        except (ValueError, RuntimeError, CanError) as error:
            self._error(error)
            try:
                sample = self.arm.snapshot()
            except (ValueError, RuntimeError, CanError) as feedback_error:
                self._error(f'{error}; feedback unavailable: {feedback_error}')
                self._publish_control_state()
                return
        self._publish_control_state(sample)
        stamp = self.get_clock().now().to_msg()
        if sample.status is not None and 0 <= sample.age[4] <= 1.0:
            message = ArmStatus(**sample.status)
            message.header.stamp = stamp
            message.feedback_age = list(sample.age)
            message.motor_enabled = list(sample.enabled[:6])
            message.motor_enable_valid = [0 <= age <= 1.0 for age in sample.age[5:]]
            message.gripper_enabled = sample.enabled[6]
            message.gripper_valid = 0 <= sample.age[3] <= 1.0
            message.teaching_active = self.teaching_active
            message.last_error = self.last_error
            for index in range(6):
                setattr(message, f'joint_{index + 1}_angle_limit',
                        bool(message.err_code & (1 << (index + 8))))
                setattr(message, f'communication_status_joint_{index + 1}',
                        bool(message.err_code & (1 << index)))
            self.status_pub.publish(message)
        if sample.position is not None and fresh(sample.age[:4]):
            message = JointState()
            message.header.stamp = stamp
            message.name = _JOINT_NAMES
            message.position = list(sample.position)
            message.velocity, message.effort = list(sample.velocity), list(sample.torque)
            self.joints_pub.publish(message)
        if sample.pose is not None:
            message = PoseStamped()
            message.header.stamp, message.header.frame_id = stamp, 'base_link'
            (message.pose.position.x, message.pose.position.y,
             message.pose.position.z) = sample.pose[:3]
            quaternion = Rotation.from_euler('xyz', sample.pose[3:]).as_quat()
            (message.pose.orientation.x, message.pose.orientation.y,
             message.pose.orientation.z, message.pose.orientation.w) = map(float, quaternion)
            self.pose_pub.publish(message)

    def destroy_node(self):
        try:
            if self.control.mode != 'drag' and (self.control.mode == 'mit' or self.arm._powered):
                self.arm._enter_drag()
                self.control._clear_power_state()
                self.control.mode = 'drag'
                if self.context.ok():
                    self._publish_control_state()
        except (ValueError, RuntimeError, CanError) as error:
            self._error(f'Driver shutdown: {error}; actual motor power unconfirmed')
        self.calibration_worker.shutdown(wait=True)
        try:
            self.arm.close()
        except (ValueError, RuntimeError, CanError) as error:
            self._error(f'Backend shutdown: {error}; actual motor power unconfirmed')
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    shutdown_requested = False

    def request_shutdown(*_):
        nonlocal shutdown_requested
        shutdown_requested = True

    previous_handlers = {
        signum: signal.signal(signum, request_shutdown)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    tty_fd = None
    try:
        node = DriverNode()
        if node.arm_config.id == node.fleet.arms[0].id:
            try:
                tty_fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
            except OSError:
                tty_fd = None
        while rclpy.ok() and not shutdown_requested:
            if tty_fd is not None and select.select([tty_fd], [], [], 0.0)[0]:
                try:
                    if any(ch in os.read(tty_fd, 1024) for ch in (b'\n', b'\r')):
                        if node.teaching_start_client.service_is_ready():
                            node.get_logger().info('Enter pressed: requesting teaching retry')
                            node.teaching_start_client.call_async(Trigger.Request())
                except OSError:
                    pass
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if tty_fd is not None:
            try:
                os.close(tty_fd)
            except OSError:
                pass
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def power_off_main(args=None):
    rclpy.init(args=args)
    node = Node('piper_x_power_off')
    node.declare_parameter('config_file', str(default_config_path()))
    fleet = FleetConfig(str(node.get_parameter('config_file').value))
    clients = [
        (arm, node.create_client(Enable, f'{arm.namespace}/enable_srv'))
        for arm in fleet.arms
    ]
    try:
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline and not all(c.service_is_ready() for _, c in clients):
            rclpy.spin_once(node, timeout_sec=0.05)
        futures = []
        offline_arms = []
        for arm, client in clients:
            if client.service_is_ready():
                futures.append((arm, client.call_async(Enable.Request(enable_request=False))))
            else:
                offline_arms.append(arm)
        wait_until = time.monotonic() + 5.0
        while futures and time.monotonic() < wait_until and not all(f.done() for _, f in futures):
            rclpy.spin_once(node, timeout_sec=0.05)
        for arm, future in futures:
            if future.done() and future.exception() is None and future.result().enable_response:
                node.get_logger().info(
                    f'{arm.id} ({arm.can_port}): powered off via running driver')
            else:
                offline_arms.append(arm)
        for arm in offline_arms:
            sdk_arm = SdkArm(arm.can_port)
            try:
                sdk_arm.power(False, 0.0)
                node.get_logger().info(f'{arm.id} ({arm.can_port}): powered off directly')
            finally:
                sdk_arm.close()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
