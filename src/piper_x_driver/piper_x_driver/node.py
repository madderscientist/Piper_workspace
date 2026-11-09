"""ROS adapter; all actuator access stays in this one SDK-owning process."""

from concurrent.futures import ThreadPoolExecutor
from functools import partial
import json
import math
import os
import select
import signal
import time
from typing import Any

from can import CanError
from geometry_msgs.msg import PoseStamped, WrenchStamped
from piper_x_interfaces.msg import (
    ArmStatus, CartesianCommand, GripperCommand, JointCommand, MitCommand,
)
from piper_x_interfaces.srv import (
    CalibrateEffector, ConfigureArm, DebugMit, Enable, SetMode,
)
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
import yaml

from .backend import fresh, SdkArm
from .controller import Controller, RateMeter, SLIDER_LOWER, SLIDER_UPPER
from .configuration import bounded, default_config_path, FleetConfig


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
        self.declare_parameter('rate_hz', 200.0)
        self.declare_parameter('telemetry_hz', 200.0)
        self.declare_parameter('wrench_estimation_enabled', False)
        self.rate_hz = bounded(
            float(self.get_parameter('rate_hz').value), 'rate_hz', 10.0, 1000.0,
        )
        self.telemetry_hz = bounded(
            float(self.get_parameter('telemetry_hz').value),
            'telemetry_hz',
            10.0,
            self.rate_hz,
        )
        self.arm_config = self.fleet.arm(str(self.get_parameter('arm_id').value))
        self._public_config = self.fleet.public(self.arm_config)
        self._public_config_prefix = (
            '{"configuration": ' + json.dumps(self._public_config, allow_nan=False) + ', '
        )
        if namespace not in ('/', self.arm_config.namespace):
            raise ValueError('Driver namespace does not match configured arm')
        self.declare_parameter(
            'friction_error_epsilon', float(self.arm_config.friction_error_epsilon),
        )
        self.declare_parameter(
            'friction_velocity_epsilon', float(self.arm_config.friction_velocity_epsilon),
        )
        self.declare_parameter(
            'target_velocity_cutoff_hz', float(self.arm_config.target_velocity_cutoff_hz),
        )
        self.declare_parameter(
            's_a', float(self.arm_config.s_a),
        )
        self.declare_parameter(
            'adaptive_stiffness_scale', float(self.arm_config.adaptive_stiffness_scale),
        )
        self.declare_parameter(
            'adaptive_stiffness_b', float(self.arm_config.adaptive_stiffness_b),
        )
        self.declare_parameter(
            'adaptive_stiffness_power', float(self.arm_config.adaptive_stiffness_power),
        )
        self.declare_parameter(
            'offset_ramp_s', float(self.arm_config.offset_ramp_s),
        )
        self.arm: Any = SdkArm(self.arm_config.can_port)
        self.control = Controller(
            self.arm, effector=self.fleet.effectors[self.arm_config.end_effector],
            gravity_scale=self.arm_config.gravity_scale,
            gravity_model=self.fleet.gravity_model(self.arm_config),
            friction_scale=self.arm_config.friction_scale,
            friction_model=self.fleet.friction_model(self.arm_config),
            friction_error_epsilon=float(self.get_parameter('friction_error_epsilon').value),
            friction_velocity_epsilon=float(self.get_parameter('friction_velocity_epsilon').value),
            target_velocity_cutoff_hz=float(self.get_parameter('target_velocity_cutoff_hz').value),
            s_a=float(self.get_parameter('s_a').value),
            adaptive_stiffness_b=float(self.get_parameter('adaptive_stiffness_b').value),
            adaptive_stiffness_power=float(self.get_parameter('adaptive_stiffness_power').value),
            offset_ramp_s=float(self.get_parameter('offset_ramp_s').value),
            kp=self.arm_config.kp, kd=self.arm_config.kd)
        self.add_on_set_parameters_callback(self._on_parameters_set)
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
        self.wrench_pub = self.create_publisher(WrenchStamped, 'end_wrench_stamped', 10)
        self.control_pub = self.create_publisher(String, 'control_state', 10)
        self._joint_msg = JointState(name=_JOINT_NAMES)
        self._pose_msg = PoseStamped()
        self._pose_msg.header.frame_id = 'base_link'
        self._wrench_msg = WrenchStamped()
        self._wrench_msg.header.frame_id = 'gripper'
        self._status_msg = ArmStatus()
        self._control_msg = String()
        self._loop_rate = RateMeter()
        self._can_rx_rate = RateMeter()
        self._joints_pub_rate = RateMeter()
        self._wrench_pub_rate = RateMeter()
        self._pose_pub_rate = RateMeter()
        self._status_pub_rate = RateMeter()
        self._control_pub_rate = RateMeter()
        self._last_can_rx_updates = getattr(self.arm, 'can_rx_updates', 0)
        self._last_control_state_pub = 0.0
        self._last_state_key = None
        self._last_err_code = None
        self._wrench_estimation_active = self.wrench_estimation_enabled
        self._wrench_estimate_ready = False
        self.create_service(Enable, 'enable_srv', self._enable, callback_group=self.power_group)
        self.create_service(CalibrateEffector, 'calibrate_effector', self._calibrate_effector,
                            callback_group=self.power_group)
        self._sync_services = (
            self.create_service(SetMode, 'set_mode', self._mode),
            self.create_service(ConfigureArm, 'configure_arm', self._configure),
            self.create_service(DebugMit, 'debug_mit', self._debug),
            self.create_service(SetBool, 'set_wrench_estimation', self._set_wrench_estimation),
        )
        realtime_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        for kind, topic, handler, qos in (
            (JointCommand, 'command/joints', self.control.joints, 10),
            (CartesianCommand, 'command/cartesian', self.control.cartesian, 10),
            (MitCommand, 'command/mit', self.control.mit, realtime_sub_qos),
            (GripperCommand, 'command/gripper', self.control.gripper, realtime_sub_qos),
        ):
            self.create_subscription(kind, topic, partial(self._command, handler), qos)
        self.create_subscription(
            Bool, '/bilateral_teaching/active', self._teaching, realtime_sub_qos)
        self.teaching_start_client = self.create_client(Trigger, '/bilateral_teaching/start')
        try:
            self.control.start_mit()
        except (ValueError, RuntimeError, CanError) as error:
            self._error(error)
        self._main_timer = self.create_timer(1.0 / self.telemetry_hz, self._publish)
        self.get_logger().info(
            f'REAL HARDWARE: started in MIT mode with gravity compensation '
            f'(inner={self.rate_hz:g} Hz, telemetry={self.telemetry_hz:g} Hz)'
        )

    @property
    def wrench_estimation_enabled(self):
        return bool(self.get_parameter('wrench_estimation_enabled').value)

    @property
    def wrench_estimate_available(self):
        return (self.wrench_estimation_enabled and self._wrench_estimate_ready
                and self.control.mode == 'mit' and self.control.mit_ready
                and not self.control.hold_reason)

    def _sync_wrench_estimation(self):
        enabled = self.wrench_estimation_enabled
        if enabled != self._wrench_estimation_active:
            self.control.reset_wrench_estimation()
            self._wrench_pub_rate.clear()
            self._wrench_estimate_ready = False
            self._wrench_estimation_active = enabled
        return enabled

    def _on_parameters_set(self, parameters):
        result = SetParametersResult(successful=True)
        updates = {}
        for param in parameters:
            name = param.name
            if name not in (
                'friction_error_epsilon',
                'friction_velocity_epsilon',
                'target_velocity_cutoff_hz',
                's_a',
                'adaptive_stiffness_scale',
                'adaptive_stiffness_b',
                'adaptive_stiffness_power',
                'offset_ramp_s',
            ):
                continue
            raw = param.value
            if isinstance(raw, bool) or not isinstance(raw, (int, float)):
                result.successful = False
                result.reason = f'{name} must be a finite number'
                return result
            updates[name] = float(raw)
        if updates:
            try:
                self.control.reconfigure(**updates)
            except ValueError as error:
                result.successful = False
                result.reason = str(error)
        return result

    def _set_wrench_estimation(self, request, response):
        result = self.set_parameters([Parameter('wrench_estimation_enabled', value=request.data)])[0]
        response.success = result.successful
        response.message = result.reason or ('Wrench estimation enabled' if request.data else 'Wrench estimation disabled')
        self._sync_wrench_estimation()
        self._publish_control_state()
        return response

    def _error(self, error):
        message = str(error)
        if self._error_key(message) != self._error_key(self.last_error):
            self.get_logger().error(message)
        self.last_error = message

    @staticmethod
    def _error_key(message):
        # Ages change every tick during one feedback outage. Preserve the full
        # diagnostic, but avoid flooding logs and triggering extra telemetry.
        if message.startswith('Complete fresh joint/gripper/status/enable feedback required:'):
            return 'feedback_stale'
        return message

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
            self.last_error = ''
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
            raw_friction_scale = getattr(request, 'friction_scale', -1.0)
            if raw_friction_scale < -1.0:
                # 标定过程临时覆盖运行时 friction_scale（如 -2.0 表示临时置 0，或 -(2.0 + s)），不写盘修改 arms.yaml
                temp_scale = bounded(-raw_friction_scale - 2.0, 'friction_scale', 0, 1)
                self.control.reconfigure(friction_scale=temp_scale)
                self.last_error = ''
                response.success = True
                response.message = 'Temporary runtime friction_scale applied without modifying arms.yaml'
                self._publish_control_state()
                return response
            friction_scale = (
                self.arm_config.friction_scale
                if raw_friction_scale == -1.0
                else raw_friction_scale
            )
            updated = self.fleet.save_arm(
                self.arm_config, request.end_effector, request.gravity_scale, friction_scale)
            self.fleet = updated
            self.arm_config = updated.arm(self.arm_config.id)
            self._public_config = updated.public(self.arm_config)
            self._public_config_prefix = (
                '{"configuration": ' + json.dumps(self._public_config, allow_nan=False) + ', '
            )
            self.control.reconfigure(
                effector=updated.effectors[self.arm_config.end_effector],
                gravity_scale=self.arm_config.gravity_scale,
                gravity_model=updated.gravity_model(self.arm_config),
                friction_scale=self.arm_config.friction_scale,
                friction_model=updated.friction_model(self.arm_config),
                friction_error_epsilon=self.arm_config.friction_error_epsilon,
                friction_velocity_epsilon=self.arm_config.friction_velocity_epsilon,
                target_velocity_cutoff_hz=self.arm_config.target_velocity_cutoff_hz,
                s_a=self.arm_config.s_a,
                adaptive_stiffness_b=self.arm_config.adaptive_stiffness_b,
                adaptive_stiffness_power=self.arm_config.adaptive_stiffness_power,
                offset_ramp_s=self.arm_config.offset_ramp_s,
                update_models=True,
            )
            self.last_error = ''
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
            self.last_error = ''
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
        if self.power_request is not None or self.teaching_active:
            return 'Stop teaching and wait for power requests to finish before calibration'
        if (self.control.mode != 'standby' or self.control.debug_session
                or (self.control.mit_received is not None
                    and self.control.clock() - self.control.mit_received <= 0.2)):
            return 'Stop MIT/other controllers and select standby before calibration'
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
        now = time.monotonic()
        self._control_pub_rate.mark(now)
        self._last_control_state_pub = now
        ctrl_now = self.control.clock()
        rates = {
            'target_hz': float(self.rate_hz),
            'telemetry_target_hz': float(self.telemetry_hz),
            'can_rx_hz': self._can_rx_rate.hz(now),
            'loop_hz': self._loop_rate.hz(now),
            'comp_hz': self.control.comp_rate.hz(ctrl_now),
            'cmd_tx_hz': self.control.cmd_tx_rate.hz(ctrl_now),
            'mit_tx_hz': self.control.mit_tx_rate.hz(ctrl_now),
            'grip_tx_hz': self.control.grip_tx_rate.hz(ctrl_now),
            'ros_cmd_hz': self.control.ros_cmd_rate.hz(ctrl_now),
            'joints_pub_hz': self._joints_pub_rate.hz(now),
            'wrench_pub_hz': self._wrench_pub_rate.hz(now),
            'pose_pub_hz': self._pose_pub_rate.hz(now),
            'status_pub_hz': self._status_pub_rate.hz(now),
            'control_pub_hz': self._control_pub_rate.hz(now),
        }
        body = json.dumps({
            'mode': self.control.mode,
            'mit_ready': self.control.mode == 'mit' and self.control.mit_ready,
            'debug_active': bool(self.control.debug_session),
            'debug_target': self.control.debug_target if self.control.debug_session else None,
            'joint_lower': list(SLIDER_LOWER), 'joint_upper': list(SLIDER_UPPER),
            'gravity_torque': list(self.control.gravity),
            'friction_torque': list(self.control.friction),
            'gripper_friction': float(self.control.gripper_friction),
            'gripper_velocity': (
                0.0 if sample is None else float(getattr(sample, 'gripper_velocity', 0.0))
            ),
            'gripper_force': (
                0.0 if sample is None else float(getattr(sample, 'gripper_force', 0.0))
            ),
            'output_torque': list(self.control.output_torque),
            'applied_kp': list(self.control.applied_kp),
            'applied_kd': list(self.control.applied_kd),
            'wrench_estimation_enabled': self.wrench_estimation_enabled,
            'wrench_estimate_available': self.wrench_estimate_available,
            'inertia_torque': list(self.control.inertia_torque) if self.wrench_estimate_available else None,
            'external_torque': list(self.control.external_torque) if self.wrench_estimate_available else None,
            'end_wrench': list(self.control.end_wrench) if self.wrench_estimate_available else None,
            'end_force': list(self.control.end_force) if self.wrench_estimate_available else None,
            'end_torque': list(self.control.end_torque) if self.wrench_estimate_available else None,
            'gravity_scale_applied': self.control.gravity_scale_applied,
            'friction_scale_applied': self.control.friction_scale_applied,
            's_a_applied': self.control.adaptive_stiffness_scale_applied,
            'adaptive_stiffness_scale_applied': self.control.adaptive_stiffness_scale_applied,
            'holding': bool(self.control.hold_reason),
            'hold_reason': self.control.hold_reason,
            'calibration_active': self.calibration_active,
            'calibration_blocked_reason': self._calibration_block_reason(sample),
            'last_error': self.last_error,
            'rates': rates,
        }, allow_nan=False)
        self._control_msg.data = self._public_config_prefix + body[1:]
        self.control_pub.publish(self._control_msg)

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
            self.last_error = ''
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
            self.control.set_mode(request.mode, preload_mit=True)
            self.last_error = ''
            response.success = True
            response.message = 'Request sent; inspect ctrl_mode/mode_feedback/teach_status'
        except (ValueError, RuntimeError, CanError) as error:
            self._error(error)
            response.message = str(error)
        self._publish_control_state()
        return response

    def _record_can_rx(self, sample, now: float) -> None:
        rx_updates = getattr(self.arm, 'can_rx_updates', None)
        if rx_updates is not None and getattr(self.arm, '_real_sdk', lambda: False)():
            if rx_updates != self._last_can_rx_updates:
                self._last_can_rx_updates = rx_updates
                self._can_rx_rate.mark(now)
        elif sample is not None and sample.position is not None and fresh(sample.age[:4]):
            self._can_rx_rate.mark(now)

    def _fast_control_step(self) -> None:
        """Inner control step: run MIT watchdog, gravity/friction compensation, and CAN TX only."""
        now = time.monotonic()
        self._loop_rate.mark(now)
        try:
            sample = self.control.watchdog(compute_wrench=False)
            self._record_can_rx(sample, now)
        except (ValueError, RuntimeError, CanError) as error:
            self._error(error)

    def _publish(self):
        now = time.monotonic()
        estimate = self._sync_wrench_estimation()
        self._loop_rate.mark(now)
        if self.teaching_active and now - self.teaching_received > 0.3:
            self.teaching_active = False
            self.get_logger().error('Teaching heartbeat expired')
        try:
            sample = self.control.watchdog(compute_wrench=estimate)
            if estimate and self.control.mode == 'mit' and self.control.mit_ready:
                self._wrench_estimate_ready = True
            if 'retaining last MIT output' in self.last_error:
                self.last_error = ''
        except (ValueError, RuntimeError, CanError) as error:
            self._error(error)
            try:
                sample = self.arm.snapshot()
            except (ValueError, RuntimeError, CanError) as feedback_error:
                self._error(f'{error}; feedback unavailable: {feedback_error}')
                self._publish_control_state()
                return
        self._record_can_rx(sample, now)

        state_key = (
            self.control.mode,
            self.control.mit_ready,
            bool(self.control.hold_reason),
            self._error_key(self.last_error),
            self.wrench_estimation_enabled,
            self.wrench_estimate_available,
            bool(self.control.debug_session),
            self.calibration_active,
            sample.enabled,
        )
        publish_slow = (
            self._last_control_state_pub == 0.0
            or state_key != self._last_state_key
            or now - self._last_control_state_pub >= 0.0195
        )
        if publish_slow:
            self._last_state_key = state_key
            self._publish_control_state(sample)

        stamp = self.get_clock().now().to_msg()
        if publish_slow and sample.status is not None and 0 <= sample.age[4] <= 1.0:
            message = self._status_msg
            st = sample.status
            message.ctrl_mode = st['ctrl_mode']
            message.arm_status = st['arm_status']
            message.mode_feedback = st['mode_feedback']
            message.teach_status = st['teach_status']
            message.motion_status = st['motion_status']
            message.trajectory_num = st['trajectory_num']
            err = st['err_code']
            message.err_code = err
            message.header.stamp = stamp
            message.feedback_age = list(sample.age)
            message.motor_enabled = list(sample.enabled[:6])
            message.motor_enable_valid = [0 <= age <= 1.0 for age in sample.age[5:]]
            message.gripper_enabled = sample.enabled[6]
            message.gripper_valid = 0 <= sample.age[3] <= 1.0
            message.teaching_active = self.teaching_active
            message.last_error = self.last_error
            if err != self._last_err_code:
                self._last_err_code = err
                for idx in range(6):
                    setattr(message, f'joint_{idx + 1}_angle_limit', bool(err & (1 << (idx + 8))))
                    setattr(message, f'communication_status_joint_{idx + 1}', bool(err & (1 << idx)))
            self.status_pub.publish(message)
            self._status_pub_rate.mark(now)
        if sample.position is not None and fresh(sample.age[:4]):
            j_msg = self._joint_msg
            j_msg.header.stamp = stamp
            j_msg.position = list(sample.position)
            j_msg.velocity, j_msg.effort = list(sample.velocity), list(sample.torque)
            self.joints_pub.publish(j_msg)
            self._joints_pub_rate.mark(now)
        if publish_slow and sample.pose is not None:
            p_msg = self._pose_msg
            p_msg.header.stamp = stamp
            (p_msg.pose.position.x, p_msg.pose.position.y,
             p_msg.pose.position.z) = sample.pose[:3]
            rx, ry, rz = (0.5 * v for v in sample.pose[3:])
            cx, sx = math.cos(rx), math.sin(rx)
            cy, sy = math.cos(ry), math.sin(ry)
            cz, sz = math.cos(rz), math.sin(rz)
            p_msg.pose.orientation.x = float(sx * cy * cz - cx * sy * sz)
            p_msg.pose.orientation.y = float(cx * sy * cz + sx * cy * sz)
            p_msg.pose.orientation.z = float(cx * cy * sz - sx * sy * cz)
            p_msg.pose.orientation.w = float(cx * cy * cz + sx * sy * sz)
            self.pose_pub.publish(p_msg)
            self._pose_pub_rate.mark(now)
        if (self.wrench_estimate_available
                and sample.position is not None and fresh(sample.age[:3])):
            wrench_msg = self._wrench_msg
            wrench_msg.header.stamp = stamp
            (wrench_msg.wrench.force.x, wrench_msg.wrench.force.y,
             wrench_msg.wrench.force.z) = self.control.end_force
            (wrench_msg.wrench.torque.x, wrench_msg.wrench.torque.y,
             wrench_msg.wrench.torque.z) = self.control.end_torque
            self.wrench_pub.publish(wrench_msg)
            self._wrench_pub_rate.mark(now)

    def destroy_node(self):
        try:
            if self.control.shutdown_to_drag() and self.context.ok():
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
    executor = None
    try:
        node = DriverNode()
        if getattr(node, '_main_timer', None) is not None:
            node.destroy_timer(node._main_timer)
            node._main_timer = None
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        if node.arm_config.id == node.fleet.arms[0].id:
            try:
                tty_fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
            except OSError:
                tty_fd = None
        subs = list(node.subscriptions)
        sync_srvs = getattr(node, '_sync_services', ())
        period = 1.0 / node.rate_hz
        telemetry_stride = max(1, round(node.rate_hz / node.telemetry_hz))
        spin_stride = max(1, round(node.rate_hz / 100.0))
        next_tick = time.monotonic()
        step = 0
        while rclpy.ok() and not shutdown_requested:
            for sub in subs:
                while True:
                    with sub.handle:
                        msg_info = sub.handle.take_message(sub.msg_type, sub.raw)
                    if msg_info is None:
                        break
                    sub.callback(msg_info[0])
            for srv in sync_srvs:
                while True:
                    with srv.handle:
                        req_and_hdr = srv.handle.service_take_request(srv.srv_type.Request)
                    if req_and_hdr is None or req_and_hdr[0] is None:
                        break
                    req_msg, hdr = req_and_hdr
                    if req_msg is not None and hdr is not None:
                        resp_msg = srv.callback(req_msg, srv.srv_type.Response())
                        srv.send_response(resp_msg, hdr)
            if step % telemetry_stride == 0:
                node._publish()
            else:
                node._fast_control_step()
            step += 1
            if step % spin_stride == 0 or node.power_request is not None or node.calibration_active:
                if tty_fd is not None and select.select([tty_fd], [], [], 0.0)[0]:
                    try:
                        if any(ch in os.read(tty_fd, 1024) for ch in (b'\n', b'\r')):
                            if node.teaching_start_client.service_is_ready():
                                node.get_logger().info('Enter pressed: requesting teaching retry')
                                node.teaching_start_client.call_async(Trigger.Request())
                    except OSError:
                        pass
                executor.spin_once(timeout_sec=0.0)
            next_tick += period
            now = time.monotonic()
            sleep_s = next_tick - now
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            elif now - next_tick > 0.02:
                next_tick = now
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if tty_fd is not None:
            try:
                os.close(tty_fd)
            except OSError:
                pass
        if executor is not None:
            executor.shutdown()
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
