"""Bilateral teaching as a ROS client: no SDK objects, CAN sockets or handover."""

from dataclasses import replace
import json
import math
import os
import select
import signal
import time

from piper_x_interfaces.msg import ArmStatus, GripperCommand, MitCommand
from piper_x_interfaces.srv import Enable, SetMode
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from piper_x_driver.configuration import default_config_path, FleetConfig

from .bilateral import (
    ArmSample, BilateralCoupling, JOINT_NAMES, MASTER_INDICES, TeachingConfig,
    TeachingFault, measured_references,
)


class TeachingNode(Node):
    def __init__(self, **kwargs):
        super().__init__('bilateral_teaching', **kwargs)
        self.declare_parameter('config_file', str(default_config_path()))
        self.fleet = FleetConfig(str(self.get_parameter('config_file').value))
        self.declare_parameter('dry_run', True)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('rate_hz', 200.0)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.rate_hz = max(10.0, min(1000.0, float(self.get_parameter('rate_hz').value)))
        defaults = TeachingConfig()
        values = {}
        for name in defaults.__dataclass_fields__:
            value = getattr(defaults, name)
            self.declare_parameter(name, list(value) if isinstance(value, tuple) else value)
            param_val = self.get_parameter(name).value
            values[name] = (
                tuple(param_val)
                if isinstance(value, tuple) and param_val is not None
                else param_val
            )
        self.config = TeachingConfig(**values)
        self.coupling = BilateralCoupling(self.config)
        self.phase = 'preview' if self.dry_run else 'idle'
        self.reason = 'Waiting for all four driver streams'
        self.positions: list[tuple[float, ...] | None] = [None] * 4
        self.velocities: list[tuple[float, ...]] = [(0.0,) * 6] * 4
        self.statuses: list[ArmStatus | None] = [None] * 4
        self.joint_received: list[float | None] = [None] * 4
        self.status_received: list[float | None] = [None] * 4
        self.power_clients, self.mode_clients = [], []
        self.mit_pubs, self.gripper_pubs, self.target_pubs = [], [], []
        self.pending = []
        self.deadline = 0.0
        self._preload_since = 0.0
        self.holding = False
        self.retry_requested = False
        self._last_tick = time.monotonic()
        self._last_state_pub = 0.0
        self._last_state_key = None
        self._mit_msgs = [MitCommand() for _ in range(4)]
        self._grip_msgs = [GripperCommand() for _ in range(4)]
        self._target_msgs = [JointState(name=list(JOINT_NAMES)) for _ in range(4)]
        self._active_msg = Bool()
        self._state_msg = String()
        realtime_sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        for index in range(4):
            prefix = self.fleet.arms[index].namespace
            self.create_subscription(
                JointState, prefix + '/joint_states_feedback',
                lambda message, i=index: self._joints(i, message), realtime_sub_qos)
            self.create_subscription(
                ArmStatus, prefix + '/arm_status',
                lambda message, i=index: self._status(i, message), realtime_sub_qos)
            self.create_subscription(
                String, prefix + '/control_state',
                lambda message, i=index: self._driver_control(i, message), realtime_sub_qos)
            self.power_clients.append(self.create_client(Enable, prefix + '/enable_srv'))
            self.mode_clients.append(self.create_client(SetMode, prefix + '/set_mode'))
            self.mit_pubs.append(self.create_publisher(MitCommand, prefix + '/command/mit', 10))
            self.gripper_pubs.append(self.create_publisher(
                GripperCommand, prefix + '/command/gripper', 10))
            self.target_pubs.append(self.create_publisher(
                JointState, f'/bilateral_teaching/arm_{index + 1}/target', 10))
        self.state_pub = self.create_publisher(String, '/bilateral_teaching/state', 10)
        self.active_pub = self.create_publisher(Bool, '/bilateral_teaching/active', 10)
        self.create_service(Trigger, '/bilateral_teaching/start', self._start)
        self.create_service(Trigger, '/bilateral_teaching/stop', self._stop_service)
        self.create_service(SetMode, '/bilateral_teaching/set_mode', self._set_mode)
        self.create_timer(1.0 / self.rate_hz, self._tick)

    def _joints(self, index, message):
        if tuple(message.name) == JOINT_NAMES and len(message.position) == 7:
            self.positions[index] = tuple(message.position)
            if len(message.velocity) >= 6:
                vel = tuple(float(v) for v in message.velocity[:6])
                self.velocities[index] = (
                    vel if all(math.isfinite(v) for v in vel) else (0.0,) * 6
                )
            else:
                self.velocities[index] = (0.0,) * 6
            self.joint_received[index] = time.monotonic()
            return
        names = list(message.name)
        if len(names) != 7 or set(names) != set(JOINT_NAMES) or len(message.position) != 7:
            self.get_logger().error(f'arm_{index + 1}: seven unique positions required')
            self.joint_received[index] = None
            return
        order = [names.index(name) for name in JOINT_NAMES]
        self.positions[index] = tuple(message.position[i] for i in order)
        if len(message.velocity) >= 6 and all(i < len(message.velocity) for i in order[:6]):
            velocity = tuple(float(message.velocity[i]) for i in order[:6])
            self.velocities[index] = (
                velocity if all(math.isfinite(v) for v in velocity) else (0.0,) * 6)
        else:
            self.velocities[index] = (0.0,) * 6
        self.joint_received[index] = time.monotonic()

    def _status(self, index, message):
        self.statuses[index] = message
        self.status_received[index] = time.monotonic()

    def _driver_control(self, index, message):
        try:
            state = json.loads(message.data)
            if not isinstance(state, dict):
                raise ValueError('control_state must be a JSON object')
        except (ValueError, TypeError) as error:
            self.get_logger().error(f'arm_{index + 1}: invalid control state: {error}')
            return
        mode = state.get('mode')
        if ((mode == 'drag' and self.phase in (
                'enabling', 'preloading', 'starting', 'engaging', 'active'))
                or (mode == 'standby' and self.phase in ('engaging', 'active'))):
            reason = f'arm_{index + 1}: driver stop requested'
            self.get_logger().error(reason)
            self.stop(reason, fault=True)

    def samples(self, now):
        result = []
        for index in range(4):
            status, received = self.statuses[index], self.status_received[index]
            joints, position = self.joint_received[index], self.positions[index]
            if (status is None or received is None or joints is None or position is None
                    or not 0 <= now - joints <= self.config.feedback_timeout):
                raise TeachingFault(f'arm_{index + 1}: missing/expired ROS feedback')
            ages = tuple(status.feedback_age)
            stamps = tuple(
                None if age == -1 else received - age for age in ages)
            enabled = tuple(status.motor_enabled) + (status.gripper_enabled,)
            fault = (
                f'arm_status={status.arm_status}, err_code={status.err_code}'
                if status.arm_status not in (0, 11) or status.err_code else '')
            result.append(ArmSample(
                position, stamps, enabled, fault,
                status.mode_feedback, self.velocities[index]))
        return tuple(result)

    def request_retry(self):
        self.retry_requested = True

    def _start(self, _, response):
        try:
            if self.dry_run:
                raise TeachingFault('Preview never actuates; restart with dry_run:=false')
            now = time.monotonic()
            samples = self.samples(now)
            self.coupling.stop('Restarting initialization')
            self.holding = False
            self.coupling.check_start(samples, now, require_enabled=False)
            clients = self.power_clients + self.mode_clients
            if not all(client.service_is_ready() for client in clients):
                raise TeachingFault('All four driver power/mode services must be available')
            self.pending = [
                client.call_async(Enable.Request(enable_request=True))
                for client in self.power_clients
            ]
            self.phase, self.deadline = 'enabling', now + 5.0
            self.reason = 'Start accepted; waiting for fresh enable feedback'
            response.success = True
        except (TeachingFault, ValueError, RuntimeError) as error:
            self.reason = str(error)
            self.get_logger().error(self.reason)
        response.message = self.reason
        return response

    def _results(self, field):
        if not all(future.done() for future in self.pending):
            return False
        for future in self.pending:
            if future.cancelled():
                raise TeachingFault('Driver request was cancelled')
            if future.exception() is not None:
                raise TeachingFault(f'Driver request failed: {future.exception()}')
            response = future.result()
            if not getattr(response, field):
                raise TeachingFault(response.message)
        return True

    def stop(self, reason, fault=False):
        self.coupling.stop(reason)
        self.holding = False
        self.reason = reason
        self.pending = []
        self.phase = 'preview' if self.dry_run else ('fault' if fault else 'stopped')
        self.active_pub.publish(Bool(data=False))

    def _stop_service(self, _, response):
        self.stop('Explicit stop requested; drivers remain in MIT gravity compensation')
        response.success = True
        response.message = 'Teaching stopped; drivers remain in MIT gravity compensation'
        return response

    def _set_mode(self, request, response):
        try:
            if self.phase not in ('preview', 'idle', 'stopped', 'fault'):
                raise TeachingFault(f'Cannot change mode while in phase {self.phase}')
            self.config = replace(self.config, mode=request.mode)
            self.coupling = BilateralCoupling(self.config)
            response.success = True
            response.message = f'Teaching mode set to {self.config.mode}'
        except (TeachingFault, ValueError) as error:
            self.reason = str(error)
            self.get_logger().error(self.reason)
            response.message = self.reason
        return response

    def _send(self, samples, targets, kp, velocities=((0.0,) * 6,) * 4):
        kp_list, kd_list = list(kp), list(self.config.kd)
        zero_6 = [0.0] * 6
        for index, (sample, target, velocity) in enumerate(zip(samples, targets, velocities)):
            free_master = self.config.mode == 'master_slave' and index in MASTER_INDICES
            follow = self.coupling.engaged and not free_master
            zero_gain = self.coupling.engaged and free_master
            m_msg = self._mit_msgs[index]
            m_msg.position = list(target[:6])
            m_msg.velocity = list(velocity[:6])
            m_msg.kp = zero_6 if zero_gain else kp_list
            m_msg.kd = zero_6 if zero_gain else kd_list
            m_msg.torque = zero_6
            self.mit_pubs[index].publish(m_msg)
            g_msg = self._grip_msgs[index]
            g_msg.width = float(target[6] if follow else sample.position[6])
            g_msg.effort = float(self.config.gripper_effort if follow else 0.0)
            self.gripper_pubs[index].publish(g_msg)

    def _tick(self):
        now = time.monotonic()
        elapsed = max(0.0, now - self._last_tick)
        self._last_tick = now
        prev_phase = self.phase
        self.holding = False
        block = ''
        refs = None
        try:
            samples = self.samples(now)
            refs = measured_references(samples, self.config, now)
            if self.retry_requested:
                self.retry_requested = False
                self.get_logger().info('Enter pressed: restarting initialization sequence')
                self._start(Trigger.Request(), Trigger.Response())
            elif self.auto_start and not self.dry_run and self.phase == 'idle':
                try:
                    self.coupling.check_start(samples, now, require_enabled=False)
                    response = self._start(Trigger.Request(), Trigger.Response())
                    if response.success:
                        self.auto_start = False
                except TeachingFault as error:
                    block = str(error)
            if (self.phase == 'enabling' and self._results('enable_response')
                    and all(all(sample.enabled[:6]) for sample in samples)):
                self.coupling.start(samples, now)
                self.phase, self._preload_since = 'preloading', now
            if self.phase in ('preloading', 'starting', 'engaging', 'active'):
                ready = all(
                    sample.mode == 6 and status is not None and status.ctrl_mode == 1
                    for sample, status in zip(samples, self.statuses))
                if self.phase == 'preloading':
                    if now > self._preload_since:
                        self.coupling.step(samples, now, False)
                    self._send(samples, self.coupling.targets, self.config.kp)
                    if now - self._preload_since >= 0.1:
                        self.pending = [
                            client.call_async(SetMode.Request(mode='mit'))
                            for client in self.mode_clients
                        ]
                        self.phase, self.deadline = 'starting', now + 3.0
                else:
                    if self.phase == 'starting':
                        accepted = self._results('success')
                        if accepted and ready:
                            self.phase = 'engaging'
                        elif accepted:
                            self.pending = [
                                client.call_async(SetMode.Request(mode='mit'))
                                for client, sample, status in zip(
                                    self.mode_clients, samples, self.statuses)
                                if status is None or status.ctrl_mode != 1 or sample.mode != 6
                            ]
                    if self.phase == 'engaging' and self.coupling.engaged:
                        self.coupling._last_tick = now
                        self._send(
                            samples, self.coupling.targets,
                            self.config.kp,
                            ((0.0,) * 6,) * 4,
                        )
                        self.phase = 'active'
                    else:
                        targets, kp = self.coupling.step(
                            samples, now, ready and self.phase != 'starting')
                        self._send(samples, targets, kp, self.coupling.velocities)
                self.reason = self.coupling.reason
            if self.phase in ('enabling', 'starting') and now > self.deadline:
                raise TeachingFault(f'{self.phase} acknowledgement timeout')
            if self.phase != prev_phase and self.phase in ('enabling', 'engaging', 'active'):
                self.get_logger().info(
                    f'Mode={self.config.mode} phase={self.phase}: {self.reason}')
        except (TeachingFault, ValueError, RuntimeError) as error:
            block = str(error)
            if self.phase in ('enabling', 'preloading', 'starting', 'engaging', 'active'):
                reason = f'{block}; retaining previous outputs, no disable requested'
                if reason != self.reason:
                    self.get_logger().error(reason)
                self.holding, self.reason = True, reason
                self.coupling.pause(reason, now)
                if self.phase in ('enabling', 'starting'):
                    self.deadline += elapsed
        if refs is not None:
            for publisher, t_msg, target in zip(self.target_pubs, self._target_msgs, refs):
                t_msg.position = list(target)
                publisher.publish(t_msg)
        state_key = (self.phase, self.holding, self.reason, block, self.config.mode)
        if (
            self._last_state_pub == 0.0
            or state_key != self._last_state_key
            or now - self._last_state_pub >= 0.0195
        ):
            self._last_state_pub = now
            self._last_state_key = state_key
            self._active_msg.data = self.phase in ('engaging', 'active')
            self.active_pub.publish(self._active_msg)
            self._state_msg.data = json.dumps({
                'dry_run': self.dry_run, 'mode': self.config.mode,
                'phase': self.phase, 'reason': self.reason,
                'holding': self.holding,
                'start_block_reason': block, 'references': refs, 'pairs': [[1, 3], [2, 4]],
                'master_arms': [1, 2] if self.config.mode == 'master_slave' else [],
                'slave_arms': [3, 4] if self.config.mode == 'master_slave' else [],
                'engage_max_error_deg': self.coupling.engage_error_deg,
            })
            self.state_pub.publish(self._state_msg)


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
        node = TeachingNode()
        for tmr in list(node.timers):
            node.destroy_timer(tmr)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            tty_fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
            node.get_logger().info('Press Enter at any time to retry initialization')
        except OSError:
            tty_fd = None
        subs = list(node.subscriptions)
        period = 1.0 / node.rate_hz
        spin_stride = max(1, round(node.rate_hz / 50.0))
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
            node._tick()
            step += 1
            if step % spin_stride == 0 or node.pending:
                if tty_fd is not None and select.select([tty_fd], [], [], 0.0)[0]:
                    try:
                        if any(ch in os.read(tty_fd, 1024) for ch in (b'\n', b'\r')):
                            node.request_retry()
                    except OSError:
                        pass
                executor.spin_once(timeout_sec=0.0)
            next_tick += period
            now = time.monotonic()
            if next_tick > now:
                time.sleep(next_tick - now)
            elif now - next_tick > 0.05:
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
            if rclpy.ok() and node.phase in (
                    'enabling', 'preloading', 'starting', 'engaging', 'active'):
                node.stop('Teaching process exiting')
                node.active_pub.publish(Bool(data=False))
            node.destroy_node()
        rclpy.try_shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
