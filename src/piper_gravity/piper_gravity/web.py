"""HTTP + ROS 2 workbench for the 5-step bidirectional gravity & static friction calibration workflow."""

import argparse
import csv
from dataclasses import dataclass, field
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
import threading
import time
from typing import Any, Sequence
from urllib.parse import urlsplit

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import numpy as np
from piper_x_driver.configuration import bounded, default_config_path, FleetConfig
from piper_x_driver.gravity import (
    ArmGravityModel,
    JointGravityModel,
    gravity_torques,
    save_gravity_model,
)
from piper_x_interfaces.msg import ArmStatus, MitCommand
from piper_x_interfaces.srv import ConfigureArm, Enable, SetMode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml

from .cli import _sync_static_friction_to_yaml
from .collector import (
    DEFAULT_PROBE_KP,
    JOINT_NAMES,
    ProbeConfig,
    build_probe_schedule,
    realized_gains,
    resolve_safe_anchor,
)
from .identification import (
    CSV_COLUMNS,
    GravityIdentificationReport,
    ProbeSample,
    identify_gripper_payload,
    identify_joint_gravity,
    parse_probe_samples_csv_reader,
    save_probe_samples_csv,
)

DEFAULT_JOINT_RANGES: dict[int, tuple[float, float]] = {
    1: (-0.50, 0.50),
    2: (0.55, 1.20),
    3: (-1.80, -1.15),
    4: (-0.45, 0.45),
    5: (-0.40, 0.40),
    6: (-0.50, 0.50),
}


def web_root_path() -> Path:
    try:
        share = Path(get_package_share_directory('piper_gravity')) / 'web'
        if (share / 'index.html').is_file():
            return share
    except PackageNotFoundError:
        pass
    return Path(__file__).resolve().parent.parent / 'web'


def synthesize_demo_probe_samples(
    joint: int = 2,
    num_points: int = 11,
    q_min: float | None = None,
    q_max: float | None = None,
    arm_model: ArmGravityModel | None = None,
) -> list[ProbeSample]:
    """Generate realistic synthetic bidirectional quasi-static probe data for J1..J6."""
    default_lo, default_hi = DEFAULT_JOINT_RANGES.get(joint, (-0.5, 0.5))
    lo = default_lo if q_min is None else float(q_min)
    hi = default_hi if q_max is None else float(q_max)

    true_scale = (1.0, 1.08, 1.11, 1.04, 0.94, 1.0)[joint - 1]
    true_offset = (0.0, -0.015, 0.012, -0.010, 0.008, 0.0)[joint - 1]
    true_bias = (0.0, 0.0, 0.0, 0.0, 0.0, 0.01)[joint - 1]
    true_fs = (0.38, 0.72, 0.70, 0.11, 0.085, 0.075)[joint - 1]

    base_pose = [0.0, 0.75, -1.45, 0.15, -0.10, 0.0]
    waypoints = np.linspace(lo, hi, num_points)
    kp_real, _ = realized_gains(joint, DEFAULT_PROBE_KP[joint - 1], 3.0)
    rng = np.random.default_rng(100 + joint)

    samples: list[ProbeSample] = []
    t = 0.0
    for direction in (+1, -1):
        pts = waypoints if direction > 0 else waypoints[::-1]
        for wp in pts:
            for _ in range(4):
                q_actual = float(wp + rng.normal(0.0, 0.0005))
                q_vec = list(base_pose)
                q_vec[joint - 1] = q_actual
                cad_g = gravity_torques(tuple(q_vec))[joint - 1]

                q_shifted = list(base_pose)
                q_shifted[joint - 1] = q_actual + true_offset
                true_g = true_scale * gravity_torques(tuple(q_shifted))[joint - 1] + true_bias
                if arm_model is not None and arm_model.effector_model:
                    cad_g = arm_model.with_added_mass(0).torques(q_vec)[joint - 1]
                    true_g = arm_model.with_added_mass(0.16).torques(q_vec)[joint - 1]
                tau_eq = float(true_g + direction * true_fs + rng.normal(0.0, 0.006))
                q_cmd = q_actual + tau_eq / max(1.0, kp_real)
                samples.append(ProbeSample(
                    t=t,
                    joint=joint,
                    direction=direction,
                    q_target=float( q_cmd),
                    q=q_actual,
                    dq=0.0,
                    tau_meas=tau_eq,
                    tau_kp=tau_eq,
                    tau_gravity=float(cad_g),
                    q_full=tuple(q_vec),
                ))
                t += 0.05
    return samples


def parse_probe_samples_csv_text(text: str) -> list[ProbeSample]:
    return parse_probe_samples_csv_reader(csv.DictReader(io.StringIO(text)))


def format_probe_samples_csv_text(samples: Sequence[ProbeSample]) -> str:
    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(CSV_COLUMNS)
    for s in samples:
        qf = list(s.q_full[:6]) if len(s.q_full) >= 6 else [0.0] * 6
        if len(s.q_full) < 6 and 1 <= s.joint <= 6:
            qf[s.joint - 1] = s.q
        writer.writerow([
            f'{s.t:.6f}',
            int(s.joint),
            int(s.direction),
            f'{s.q_target:.8f}',
            f'{s.q:.8f}',
            f'{s.dq:.8f}',
            f'{s.tau_meas:.8f}',
            f'{s.tau_kp:.8f}',
            f'{s.tau_gravity:.8f}',
            *(f'{val:.8f}' for val in qf),
        ])
    return buf.getvalue()


def serialize_gravity_report(
    report: GravityIdentificationReport,
    arm_model: ArmGravityModel,
    max_curve_points: int = 200,
) -> dict[str, Any]:
    """Convert GravityIdentificationReport + ArmGravityModel into JSON-serializable UI payload."""
    m = report.model
    d = report.decomposition
    n_grid = len(d.theta_grid)
    stride = max(1, n_grid // max_curve_points)
    idx = np.arange(0, n_grid, stride)

    yaml_text = yaml.safe_dump(arm_model.to_dict(), allow_unicode=True, sort_keys=False)

    return {
        'joint': int(report.joint),
        'arm_id': arm_model.arm_id,
        'model': {
            **m.to_dict(),
            'angle_offset_deg': float(math.degrees(m.angle_offset)),
        },
        'metrics': {
            'rmse_cad': float(report.rmse_cad),
            'rmse_calibrated': float(report.rmse_calibrated),
            'static_friction_mean': float(report.static_friction_mean),
            'static_friction_p85': float(report.static_friction_p85),
            'grid_points': int(n_grid),
            'theta_min': float(d.theta_grid[0]),
            'theta_max': float(d.theta_grid[-1]),
        },
        'payload': None if report.added_mass_kg is None else {
            'added_mass_kg': report.added_mass_kg,
            'total_mass_kg': report.total_effector_mass_kg,
            'mass_identified': report.mass_identified,
        },
        'curves': {
            'theta_grid': [float(x) for x in d.theta_grid[idx]],
            'tau_pos': [float(x) for x in d.tau_pos[idx]],
            'tau_neg': [float(x) for x in d.tau_neg[idx]],
            'gravity_measured': [float(x) for x in d.gravity_measured[idx]],
            'gravity_cad': [float(x) for x in d.gravity_cad[idx]],
            'gravity_fitted': [float(x) for x in d.gravity_fitted[idx]],
            'stiction_profile': [float(x) for x in d.stiction_profile[idx]],
        },
        'yaml_text': yaml_text,
    }


@dataclass
class _ArmProbeRunner:
    """Independent per-arm bidirectional probe state machine."""

    queue: list[ProbeConfig] = field(default_factory=list)
    queue_index: int = 0
    config: ProbeConfig | None = None
    phase: str = 'idle'
    error: str = ''
    initial: tuple[float, ...] | None = None
    anchor: tuple[float, ...] | None = None
    schedule: list = field(default_factory=list)
    segment_index: int = 0
    segment_started: float = 0.0
    probe_started: float = 0.0
    current_q_des: float = 0.0
    current_dq_des: float = 0.0
    current_direction: int = 0
    current_kind: str = ''
    saved_gravity_scale: float = 1.0
    saved_friction_scale: float = 0.5
    pending_future: Any = None
    output_csv: str = ''
    fit_output: str = ''
    samples: list[ProbeSample] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        total_segs = len(self.schedule)
        seg_idx = min(self.segment_index, total_segs)
        seg_kind = (
            self.schedule[self.segment_index].kind
            if 0 <= self.segment_index < total_segs else ''
        )
        return {
            'phase': self.phase,
            'error': self.error,
            'segment_index': seg_idx,
            'segment_total': total_segs,
            'segment_kind': seg_kind,
            'queue_index': self.queue_index,
            'queue_total': len(self.queue),
            'queue_joints': [c.joint for c in self.queue],
            'sample_count': len(self.samples),
            'current_q_des': self.current_q_des,
            'current_dq_des': self.current_dq_des,
            'current_direction': self.current_direction,
            'config': None if self.config is None else {
                'namespace': self.config.namespace,
                'joint': self.config.joint,
                'q_min': self.config.q_min,
                'q_max': self.config.q_max,
                'num_points': self.config.num_points,
            },
        }


class GravityWorkbenchNode(Node):
    """ROS 2 node backing the gravity & static friction calibration web workbench."""

    def __init__(self, config_file: str | Path | None = None, **kwargs):
        super().__init__('piper_gravity_web', **kwargs)
        self.lock = threading.Lock()
        cfg_path = Path(config_file) if config_file else default_config_path()
        self.fleet = FleetConfig(cfg_path)
        self.arms_by_ns = {arm.namespace: arm for arm in self.fleet.arms}

        self.positions: dict[str, tuple[float, ...]] = {}
        self.velocities: dict[str, tuple[float, ...]] = {}
        self.torques: dict[str, tuple[float, ...]] = {}
        self.joint_stamps: dict[str, float] = {}
        self.statuses: dict[str, ArmStatus] = {}
        self.status_stamps: dict[str, float] = {}
        self.control_states: dict[str, dict[str, Any]] = {}
        self.effector_kwargs: dict[str, dict[str, Any]] = {}

        self.mit_pubs: dict[str, Any] = {}
        self.config_clients: dict[str, Any] = {}
        self.enable_clients: dict[str, Any] = {}
        self.mode_clients: dict[str, Any] = {}

        for arm in self.fleet.arms:
            ns = arm.namespace
            eff = self.fleet.effectors[arm.end_effector]
            self.effector_kwargs[ns] = {'mass_kg': eff.mass_kg, 'com_m': eff.com_m}
            self.create_subscription(
                JointState, f'{ns}/joint_states_feedback',
                partial(self._on_joints, ns), 10,
            )
            self.create_subscription(
                ArmStatus, f'{ns}/arm_status',
                partial(self._on_status, ns), 10,
            )
            self.create_subscription(
                String, f'{ns}/control_state',
                partial(self._on_control_state, ns), 10,
            )
            self.mit_pubs[ns] = self.create_publisher(MitCommand, f'{ns}/command/mit', 1)
            self.config_clients[ns] = self.create_client(ConfigureArm, f'{ns}/configure_arm')
            self.enable_clients[ns] = self.create_client(Enable, f'{ns}/enable_srv')
            self.mode_clients[ns] = self.create_client(SetMode, f'{ns}/set_mode')

        self.runners: dict[str, _ArmProbeRunner] = {
            arm.namespace: _ArmProbeRunner() for arm in self.fleet.arms
        }
        self.active_namespace: str = self.fleet.arms[0].namespace
        self.samples: list[ProbeSample] = []
        self.joint_samples: dict[str, dict[int, list[ProbeSample]]] = {
            arm.id: {} for arm in self.fleet.arms
        }
        self.joint_fits: dict[str, dict[int, dict[str, Any]]] = {
            arm.id: {} for arm in self.fleet.arms
        }
        self.latest_fit: dict[str, Any] | None = None
        self.arm_models: dict[str, ArmGravityModel] = {
            arm.id: (self.fleet.gravity_model(arm) or ArmGravityModel(arm_id=arm.id))
            for arm in self.fleet.arms
        }

        self.create_timer(0.01, self._tick_probe)

    @property
    def probe_phase(self) -> str:
        return self.runners[self.active_namespace].phase

    @property
    def probe_error(self) -> str:
        return self.runners[self.active_namespace].error

    def _on_joints(self, ns: str, message: JointState) -> None:
        names = tuple(message.name)
        if len(message.position) != 7:
            return
        if names == JOINT_NAMES:
            order = range(7)
        else:
            if len(names) != 7 or set(names) != set(JOINT_NAMES):
                return
            order = [names.index(name) for name in JOINT_NAMES]
        pos = tuple(float(message.position[i]) for i in order)
        vel = (
            tuple(float(message.velocity[i]) for i in order[:6])
            if len(message.velocity) >= 6 else (0.0,) * 6
        )
        tau = (
            tuple(float(message.effort[i]) for i in order[:6])
            if len(message.effort) >= 6 else (0.0,) * 6
        )
        now = time.monotonic()
        with self.lock:
            self.positions[ns] = pos
            self.velocities[ns] = vel
            self.torques[ns] = tau
            self.joint_stamps[ns] = now
            runner = self.runners.get(ns)
            if (runner is not None
                    and runner.phase == 'running'
                    and runner.config is not None
                    and runner.current_kind == 'measure'
                    and runner.current_direction in (-1, 1)):
                seg = runner.schedule[runner.segment_index]
                elapsed = now - runner.segment_started
                j_idx = runner.config.joint - 1
                dq_val = vel[j_idx]
                if elapsed >= 0.4 * seg.duration and abs(dq_val) <= 0.03:
                    assert runner.config.kd is not None
                    ctrl_state = self.control_states.get(ns, {})
                    kp_vec = ctrl_state.get('applied_kp')
                    kd_vec = ctrl_state.get('applied_kd')
                    kp_cmd = (
                        float(kp_vec[j_idx])
                        if isinstance(kp_vec, list) and len(kp_vec) >= 6 and float(kp_vec[j_idx]) > 0.0
                        else runner.config.kp[j_idx]
                    )
                    kd_cmd = (
                        float(kd_vec[j_idx])
                        if isinstance(kd_vec, list) and len(kd_vec) >= 6 and float(kd_vec[j_idx]) >= 0.0
                        else runner.config.kd[j_idx]
                    )
                    kp_real, kd_real = realized_gains(
                        runner.config.joint, kp_cmd, kd_cmd,
                    )
                    q_val = pos[j_idx]
                    eff_kw = self.effector_kwargs.get(ns, {})
                    g_torques = gravity_torques(pos[:6], **eff_kw)
                    g_vec = ctrl_state.get('gravity_torque')
                    if isinstance(g_vec, list) and len(g_vec) >= 6:
                        g_applied = float(g_vec[j_idx])
                    else:
                        arm_obj = self.arms_by_ns.get(ns)
                        g_model = self.arm_models.get(arm_obj.id) if arm_obj is not None else None
                        g_active = (
                            g_torques[j_idx]
                            if g_model is None
                            else g_model.torques(pos[:6], **eff_kw)[j_idx]
                        )
                        g_scale_app = float(ctrl_state.get('gravity_scale_applied', 1.0))
                        g_applied = g_scale_app * float(g_active)
                    tau_kp = g_applied + kp_real * (runner.current_q_des - q_val) - kd_real * dq_val
                    sample = ProbeSample(
                        t=now - runner.probe_started,
                        joint=runner.config.joint,
                        direction=runner.current_direction,
                        q_target=runner.current_q_des,
                        q=q_val,
                        dq=dq_val,
                        tau_meas=tau[j_idx],
                        tau_kp=float(tau_kp),
                        tau_gravity=float(g_torques[j_idx]),
                        q_full=pos[:6],
                    )
                    runner.samples.append(sample)
                    if ns == self.active_namespace:
                        self.samples = runner.samples

    def _on_status(self, ns: str, message: ArmStatus) -> None:
        with self.lock:
            self.statuses[ns] = message
            self.status_stamps[ns] = time.monotonic()

    def _on_control_state(self, ns: str, message: String) -> None:
        try:
            state = json.loads(message.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            self.control_states[ns] = state
            eff = state.get('configuration', {}).get('effector', {})
            if 'mass_kg' in eff and 'com_m' in eff:
                self.effector_kwargs[ns] = {
                    'mass_kg': float(eff['mass_kg']),
                    'com_m': tuple(float(x) for x in eff['com_m']),
                }

    def _publish_mit_locked(
        self,
        cfg: ProbeConfig,
        anchor: tuple[float, ...],
        q_j: float,
        dq_j: float,
        anchor_blend: float = 1.0,
        initial: tuple[float, ...] | None = None,
    ) -> None:
        assert cfg.kd is not None
        if initial is not None and anchor_blend < 1.0:
            pos = [
                float(initial[i] + anchor_blend * (anchor[i] - initial[i]))
                for i in range(6)
            ]
        else:
            pos = list(anchor[:6])
        vel = [0.0] * 6
        j_idx = cfg.joint - 1
        pos[j_idx] = float(q_joint := q_j)
        vel[j_idx] = float(dq_j)
        msg = MitCommand()
        msg.position = pos
        msg.velocity = vel
        msg.kp = list(cfg.kp)
        msg.kd = list(cfg.kd)
        msg.torque = [0.0] * 6
        self.mit_pubs[cfg.namespace].publish(msg)

    def start_probe(
        self,
        configs: ProbeConfig | Sequence[ProbeConfig],
        output_csv: str = '',
        fit_output: str = '',
    ) -> dict[str, Any]:
        queue = [configs] if isinstance(configs, ProbeConfig) else list(configs)
        if not queue:
            raise ValueError('At least one joint probe configuration is required')
        namespaces = {c.namespace for c in queue}
        if len(namespaces) != 1:
            raise ValueError('All queued joint probes must target the same CAN port / arm')
        ns = next(iter(namespaces))
        with self.lock:
            if ns not in self.arms_by_ns:
                raise ValueError(f'Unknown namespace {ns}')
            runner = self.runners[ns]
            if runner.phase in (
                'waiting_feedback', 'zeroing_compensation', 'enabling',
                'setting_mode', 'running', 'restoring_compensation',
            ):
                raise ValueError(f'{ns} already has an active probe; stop it first')
            arm = self.arms_by_ns[ns]
            ctrl = self.control_states.get(ns, {})
            cfg_dict = ctrl.get('configuration', {})
            runner.saved_gravity_scale = float(cfg_dict.get('gravity_scale', arm.gravity_scale))
            runner.saved_friction_scale = float(cfg_dict.get('friction_scale', arm.friction_scale))
            runner.queue = queue
            runner.queue_index = 0
            runner.config = queue[0]
            runner.phase = 'waiting_feedback'
            runner.error = ''
            runner.initial = None
            runner.anchor = None
            runner.schedule = []
            runner.segment_index = 0
            runner.samples = []
            runner.output_csv = output_csv.strip()
            runner.fit_output = fit_output.strip()
            self.active_namespace = ns
            self.samples = runner.samples
        return self.status_payload()

    def stop_probe(self, namespace: str | None = None) -> dict[str, Any]:
        to_restore: list[tuple[str, float, float]] = []
        with self.lock:
            target_ns_list = (
                [namespace]
                if namespace and namespace in self.runners
                else list(self.runners.keys())
            )
            for ns in target_ns_list:
                runner = self.runners[ns]
                if (runner.config is not None
                        and runner.anchor is not None
                        and ns in self.positions):
                    cur_pos = self.positions[ns][:6]
                    self._publish_mit_locked(
                        runner.config, cur_pos, cur_pos[runner.config.joint - 1], 0.0,
                    )
                    to_restore.append((ns, runner.saved_gravity_scale, runner.saved_friction_scale))
                runner.phase = 'stopped'
                runner.queue = []
                runner.pending_future = None
        for ns, g_s, f_s in to_restore:
            self._restore_probe_scales(ns, g_s, f_s)
        return self.status_payload()

    def _fail_probe_locked(self, runner: _ArmProbeRunner, reason: str, ns: str = '') -> None:
        if ns and runner.phase not in ('idle', 'waiting_feedback', 'completed', 'failed', 'stopped'):
            client = self.config_clients.get(ns)
            if client is not None and client.service_is_ready():
                arm = self.arms_by_ns[ns]
                ctrl = self.control_states.get(ns, {})
                cfg_dict = ctrl.get('configuration', {})
                end_eff = str(cfg_dict.get('end_effector') or arm.end_effector)
                restore_f = (
                    -4.0 - runner.saved_friction_scale
                    if abs(runner.saved_gravity_scale - 1.0) <= 1e-9
                    else runner.saved_friction_scale
                )
                client.call_async(ConfigureArm.Request(
                    end_effector=end_eff,
                    gravity_scale=runner.saved_gravity_scale,
                    friction_scale=restore_f,
                ))
        runner.error = reason
        runner.phase = 'failed'
        runner.queue = []
        self.get_logger().error(reason)

    def _csv_path_for_joint(self, base_csv: str, joint: int, multi: bool) -> str:
        if not base_csv:
            return ''
        if not multi:
            return base_csv
        p = Path(base_csv)
        stem = p.stem
        if not stem.endswith(f'_j{joint}'):
            stem = f'{stem}_j{joint}'
        return str(p.with_name(f'{stem}{p.suffix or ".csv"}'))

    def request_compensation_scales(
        self,
        namespace: str,
        gravity_scale: float,
        friction_scale: float,
    ) -> bool:
        g_scale = bounded(gravity_scale, 'gravity_scale', 0, 1)
        f_scale = bounded(friction_scale, 'friction_scale', 0, 1)
        with self.lock:
            if namespace not in self.arms_by_ns:
                raise ValueError(f'Unknown namespace {namespace}')
            arm = self.arms_by_ns[namespace]
            ctrl = self.control_states.get(namespace, {})
            cfg_dict = ctrl.get('configuration', {})
            end_eff = str(cfg_dict.get('end_effector') or arm.end_effector)
            client = self.config_clients[namespace]
            if not client.service_is_ready():
                return False
            client.call_async(ConfigureArm.Request(
                end_effector=end_eff,
                gravity_scale=g_scale,
                friction_scale=f_scale,
            ))
            return True

    def _restore_probe_scales(self, namespace: str, gravity_scale: float, friction_scale: float) -> bool:
        g_scale = bounded(gravity_scale, 'gravity_scale', 0, 1)
        f_scale = bounded(friction_scale, 'friction_scale', 0, 1)
        if abs(g_scale - 1.0) > 1e-9:
            return self.request_compensation_scales(namespace, g_scale, f_scale)
        with self.lock:
            if namespace not in self.arms_by_ns:
                raise ValueError(f'Unknown namespace {namespace}')
            arm = self.arms_by_ns[namespace]
            ctrl = self.control_states.get(namespace, {})
            cfg_dict = ctrl.get('configuration', {})
            end_eff = str(cfg_dict.get('end_effector') or arm.end_effector)
            client = self.config_clients[namespace]
            if not client.service_is_ready():
                return False
            client.call_async(ConfigureArm.Request(
                end_effector=end_eff,
                gravity_scale=1.0,
                friction_scale=-4.0 - f_scale,
            ))
            return True

    def reload_saved_model(self, arm_id: str, output_yaml: str | Path) -> bool:
        """Reapply the current driver configuration after its active model file is saved."""
        with self.lock:
            arm = self.fleet.arm(arm_id)
            config = self.control_states.get(arm.namespace, {}).get('configuration', {})
            active_path = config.get('gravity_config') or arm.gravity_config
            if not active_path or Path(active_path).resolve() != Path(output_yaml).resolve():
                return False
            g_scale = float(config.get('gravity_scale', arm.gravity_scale))
            f_scale = float(config.get('friction_scale', arm.friction_scale))
            runner = self.runners[arm.namespace]
            if runner.phase == 'restoring_compensation':
                g_scale, f_scale = runner.saved_gravity_scale, runner.saved_friction_scale
        return self.request_compensation_scales(arm.namespace, g_scale, f_scale)

    def _tick_probe(self) -> None:
        completed_jobs: list[tuple[list[ProbeSample], ProbeConfig, str, str, bool]] = []

        with self.lock:
            for ns, runner in self.runners.items():
                if runner.phase not in (
                    'waiting_feedback', 'zeroing_compensation', 'enabling',
                    'setting_mode', 'running', 'restoring_compensation',
                ):
                    continue
                cfg = runner.config
                assert cfg is not None
                pos = self.positions.get(ns)
                status = self.statuses.get(ns)
                if pos is None or status is None:
                    continue
                if status.err_code or status.arm_status not in (0, 11, 12, 13):
                    self._fail_probe_locked(
                        runner,
                        f'Arm fault: arm_status={status.arm_status}, err_code={status.err_code}',
                        ns,
                    )
                    continue

                if runner.phase == 'waiting_feedback':
                    runner.initial = pos[:6]
                    if runner.anchor is None:
                        runner.anchor = (
                            resolve_safe_anchor(runner.initial, cfg.joint)
                            if cfg.use_safe_anchor
                            else runner.initial
                        )
                    runner.current_q_des = runner.initial[cfg.joint - 1]
                    runner.current_dq_des = 0.0
                    cfg_client = self.config_clients[ns]
                    if cfg_client.service_is_ready():
                        arm = self.arms_by_ns[ns]
                        ctrl = self.control_states.get(ns, {})
                        cfg_dict = ctrl.get('configuration', {})
                        end_eff = str(cfg_dict.get('end_effector') or arm.end_effector)
                        if abs(runner.saved_gravity_scale - 1.0) > 1e-9:
                            cfg_client.call_async(ConfigureArm.Request(
                                end_effector=end_eff, gravity_scale=1.0, friction_scale=-1.0,
                            ))
                        runner.pending_future = cfg_client.call_async(ConfigureArm.Request(
                            end_effector=end_eff,
                            gravity_scale=1.0,
                            friction_scale=-2.0,
                        ))
                        runner.phase = 'zeroing_compensation'
                        continue
                    self._check_enable_or_mit_locked(runner, cfg, status)
                    continue

                if runner.phase == 'zeroing_compensation':
                    if runner.pending_future is None or not runner.pending_future.done():
                        continue
                    res = runner.pending_future.result()
                    if res is None or not res.success:
                        self._fail_probe_locked(
                            runner,
                            f'Zeroing compensation failed: {getattr(res, "message", "no response")}',
                            ns,
                        )
                        continue
                    self._check_enable_or_mit_locked(runner, cfg, status)
                    continue

                if runner.phase == 'enabling':
                    if runner.pending_future is None or not runner.pending_future.done():
                        continue
                    res = runner.pending_future.result()
                    if res is None or not res.enable_response:
                        self._fail_probe_locked(
                            runner,
                            f'Enable failed: {getattr(res, "message", "no response")}',
                            ns,
                        )
                        continue
                    self._enter_mit_locked(runner, cfg, status)
                    continue

                if runner.phase == 'setting_mode':
                    assert runner.anchor is not None
                    self._publish_mit_locked(
                        cfg, runner.anchor, runner.current_q_des, 0.0,
                        anchor_blend=0.0, initial=runner.initial,
                    )
                    if runner.pending_future is None or not runner.pending_future.done():
                        continue
                    res = runner.pending_future.result()
                    if res is None or not res.success:
                        self._fail_probe_locked(
                            runner,
                            f'SetMode(mit) failed: {getattr(res, "message", "no response")}',
                            ns,
                        )
                        continue
                    self._start_schedule_locked(runner, cfg)
                    continue

                if runner.phase == 'running':
                    assert runner.anchor is not None
                    now = time.monotonic()
                    advanced = False
                    while runner.segment_index < len(runner.schedule):
                        seg = runner.schedule[runner.segment_index]
                        elapsed = now - runner.segment_started
                        if elapsed <= seg.duration:
                            ratio = elapsed / seg.duration if seg.duration > 0 else 1.0
                            smooth = 0.5 * (1.0 - math.cos(math.pi * ratio))
                            runner.current_q_des = seg.q_start + smooth * (seg.q_end - seg.q_start)
                            runner.current_dq_des = 0.0
                            runner.current_direction = seg.direction
                            runner.current_kind = seg.kind
                            blend = (
                                smooth
                                if (runner.segment_index == 0 and seg.kind == 'positioning')
                                else 1.0
                            )
                            self._publish_mit_locked(
                                cfg, runner.anchor, runner.current_q_des, 0.0,
                                anchor_blend=blend, initial=runner.initial,
                            )
                            advanced = True
                            break
                        runner.segment_index += 1
                        runner.segment_started = now
                    if advanced:
                        continue

                    runner.current_q_des = runner.anchor[cfg.joint - 1]
                    runner.current_dq_des = 0.0
                    runner.current_direction = 0
                    runner.current_kind = ''
                    self._publish_mit_locked(cfg, runner.anchor, runner.current_q_des, 0.0)
                    finished_samples = list(runner.samples)
                    finished_cfg = cfg
                    arm_id = self.arms_by_ns[cfg.namespace].id
                    self.joint_samples.setdefault(arm_id, {})[cfg.joint] = finished_samples
                    multi = len(runner.queue) > 1
                    out_csv = self._csv_path_for_joint(runner.output_csv, cfg.joint, multi)
                    out_fit = runner.fit_output

                    if runner.queue_index + 1 < len(runner.queue):
                        runner.queue_index += 1
                        runner.config = runner.queue[runner.queue_index]
                        runner.initial = pos[:6]
                        runner.anchor = (
                            resolve_safe_anchor(runner.initial, runner.config.joint)
                            if runner.config.use_safe_anchor
                            else runner.initial
                        )
                        runner.samples = []
                        if ns == self.active_namespace:
                            self.samples = runner.samples
                        self._start_schedule_locked(runner, runner.config)
                        has_next_joint = True
                    else:
                        has_next_joint = False
                        cfg_client = self.config_clients[ns]
                        if cfg_client.service_is_ready():
                            arm = self.arms_by_ns[ns]
                            ctrl = self.control_states.get(ns, {})
                            cfg_dict = ctrl.get('configuration', {})
                            end_eff = str(cfg_dict.get('end_effector') or arm.end_effector)
                            restore_f = (
                                -4.0 - runner.saved_friction_scale
                                if abs(runner.saved_gravity_scale - 1.0) <= 1e-9
                                else runner.saved_friction_scale
                            )
                            runner.pending_future = cfg_client.call_async(ConfigureArm.Request(
                                end_effector=end_eff,
                                gravity_scale=runner.saved_gravity_scale,
                                friction_scale=restore_f,
                            ))
                            runner.phase = 'restoring_compensation'
                        else:
                            runner.phase = 'completed'

                    completed_jobs.append((finished_samples, finished_cfg, out_csv, out_fit, has_next_joint))

                elif runner.phase == 'restoring_compensation':
                    assert runner.anchor is not None
                    self._publish_mit_locked(cfg, runner.anchor, runner.current_q_des, 0.0)
                    if runner.pending_future is None or not runner.pending_future.done():
                        continue
                    runner.phase = 'completed'

        for finished_samples, finished_cfg, out_csv, out_fit, has_next_joint in completed_jobs:
            try:
                if out_csv:
                    save_probe_samples_csv(finished_samples, out_csv)
                arm_id = self.arms_by_ns[finished_cfg.namespace].id
                self.run_fit(
                    finished_samples,
                    arm_id=arm_id,
                    output_yaml=out_fit if (out_fit and not has_next_joint) else None,
                )
                if out_fit and has_next_joint:
                    with self.lock:
                        save_gravity_model(self.arm_models[arm_id], out_fit)
            except Exception as error:
                with self.lock:
                    self.runners[finished_cfg.namespace].error = (
                        f'J{finished_cfg.joint} auto-fit warning: {error}'
                    )

    def _check_enable_or_mit_locked(
        self, runner: _ArmProbeRunner, cfg: ProbeConfig, status: ArmStatus,
    ) -> None:
        ns = cfg.namespace
        if not all(status.motor_enabled):
            client = self.enable_clients[ns]
            if not client.service_is_ready():
                return
            runner.pending_future = client.call_async(Enable.Request(enable_request=True))
            runner.phase = 'enabling'
            return
        self._enter_mit_locked(runner, cfg, status)

    def _enter_mit_locked(
        self, runner: _ArmProbeRunner, cfg: ProbeConfig, status: ArmStatus,
    ) -> None:
        assert runner.anchor is not None
        self._publish_mit_locked(
            cfg, runner.anchor, runner.current_q_des, 0.0,
            anchor_blend=0.0, initial=runner.initial,
        )
        if status.ctrl_mode == 1 and status.mode_feedback == 6:
            self._start_schedule_locked(runner, cfg)
            return
        client = self.mode_clients[cfg.namespace]
        if not client.service_is_ready():
            return
        runner.pending_future = client.call_async(SetMode.Request(mode='mit'))
        runner.phase = 'setting_mode'

    def _start_schedule_locked(self, runner: _ArmProbeRunner, cfg: ProbeConfig) -> None:
        assert runner.anchor is not None
        init_p = runner.initial or runner.anchor
        j_idx = cfg.joint - 1
        q_init = init_p[j_idx]
        q_ret = runner.anchor[j_idx]
        anchor_travel = max(
            (abs(runner.anchor[i] - init_p[i]) for i in range(6) if i != j_idx),
            default=0.0,
        )
        runner.schedule = build_probe_schedule(
            q_init, cfg, anchor_travel=anchor_travel, q_return=q_ret,
        )
        runner.segment_index = 0
        runner.probe_started = time.monotonic()
        runner.segment_started = runner.probe_started
        runner.phase = 'running'

    def run_fit(
        self,
        samples: Sequence[ProbeSample],
        *,
        arm_id: str = 'arm_1',
        torque_source: str = 'auto',
        grid_step: float = 0.005,
        trim_ratio: float = 0.08,
        output_yaml: str | Path | None = None,
        update_friction_yaml: str | Path | None = None,
    ) -> dict[str, Any]:
        if not samples:
            raise ValueError('No gravity probe samples provided')
        joint = samples[0].joint
        with self.lock:
            arm = self.fleet.arm(arm_id)
            config = self.control_states.get(arm.namespace, {}).get('configuration', {})
            model_name = config.get('effector', {}).get('model') or self.fleet.effectors[arm.end_effector].model
            current_model = self.arm_models.get(arm_id, ArmGravityModel(arm_id=arm_id)).for_effector(model_name)
            self.joint_samples.setdefault(arm_id, {})[joint] = list(samples)
            all_samples = {j: list(rows) for j, rows in self.joint_samples[arm_id].items()}
        if current_model.effector_model:
            updated_model, reports = identify_gripper_payload(
                all_samples, current_model, torque_source=torque_source,
                grid_step=grid_step, trim_ratio=trim_ratio)
        else:
            report = identify_joint_gravity(
                samples, torque_source=torque_source, grid_step=grid_step, trim_ratio=trim_ratio)
            reports = {report.joint: report}
            updated_model = current_model.with_joint(report.joint, report.model)
        with self.lock:
            self.arm_models[arm_id] = updated_model
            self.fleet.gravity_models[arm_id] = updated_model
            if output_yaml:
                save_gravity_model(updated_model, Path(output_yaml))
            if update_friction_yaml:
                _sync_static_friction_to_yaml(
                    Path(update_friction_yaml),
                    arm_id,
                    {j: report.model.static_friction for j, report in reports.items()},
                )
            for j, report in reports.items():
                self.joint_fits.setdefault(arm_id, {})[j] = serialize_gravity_report(report, updated_model)
            payload = self.joint_fits[arm_id][joint]
            for j_payload in self.joint_fits[arm_id].values():
                j_payload['yaml_text'] = payload['yaml_text']
            self.latest_fit = payload
        if output_yaml:
            self.reload_saved_model(arm_id, output_yaml)
        return payload

    def run_fit_all_demo(
        self,
        arm_id: str = 'arm_1',
        torque_source: str = 'auto',
        output_yaml: str | Path | None = None,
    ) -> dict[str, Any]:
        last_payload: dict[str, Any] = {}
        for j in range(1, 7):
            demo_samples = synthesize_demo_probe_samples(joint=j, arm_model=self.arm_models[arm_id])
            with self.lock:
                self.joint_samples.setdefault(arm_id, {})[j] = demo_samples
                self.samples = demo_samples
            last_payload = self.run_fit(
                demo_samples,
                arm_id=arm_id,
                torque_source=torque_source,
                output_yaml=output_yaml if j == 6 else None,
            )
        with self.lock:
            return {
                'arm_id': arm_id,
                'joint_fits': {str(k): v for k, v in self.joint_fits.get(arm_id, {}).items()},
                'latest_fit': last_payload,
            }

    def status_payload(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            arms_out = []
            for arm in self.fleet.arms:
                ns = arm.namespace
                pos = self.positions.get(ns)
                vel = self.velocities.get(ns, (0.0,) * 6)
                tau = self.torques.get(ns, (0.0,) * 6)
                stamp = self.joint_stamps.get(ns)
                status = self.statuses.get(ns)
                ctrl = self.control_states.get(ns, {})
                online = stamp is not None and (now - stamp) <= 1.0
                g_cad = (
                    gravity_torques(pos[:6], **self.effector_kwargs.get(ns, {}))
                    if pos is not None else (0.0,) * 6
                )
                arm_model = self.arm_models.get(arm.id, ArmGravityModel(arm_id=arm.id))
                g_cal = (
                    arm_model.torques(pos[:6], **self.effector_kwargs.get(ns, {}))
                    if pos is not None else (0.0,) * 6
                )
                arms_out.append({
                    'id': arm.id,
                    'name': arm.name,
                    'namespace': ns,
                    'can_port': arm.can_port,
                    'online': online,
                    'position': list(pos[:6]) if pos is not None else None,
                    'velocity': list(vel[:6]),
                    'torque': list(tau[:6]),
                    'gravity_cad': list(g_cad),
                    'gravity_calibrated': list(g_cal),
                    'enabled': list(status.motor_enabled) if status is not None else [False] * 6,
                    'mode': ctrl.get('mode', 'standby'),
                    'gravity_scale_applied': ctrl.get('gravity_scale_applied', arm.gravity_scale),
                    'friction_scale_applied': ctrl.get('friction_scale_applied', arm.friction_scale),
                    'probe_phase': self.runners[ns].phase,
                    'calibrated_joints': sorted(self.arm_models.get(arm.id, ArmGravityModel(arm.id)).joints.keys()),
                })

            probes_by_ns = {ns: runner.to_dict() for ns, runner in self.runners.items()}
            active_runner = self.runners.get(self.active_namespace, next(iter(self.runners.values())))
            return {
                'arms': arms_out,
                'default_ranges': {str(k): list(v) for k, v in DEFAULT_JOINT_RANGES.items()},
                'probe': active_runner.to_dict(),
                'probes_by_ns': probes_by_ns,
                'latest_fit': self.latest_fit,
                'joint_fits': {
                    arm_id: {str(j): fit for j, fit in fits.items()}
                    for arm_id, fits in self.joint_fits.items()
                },
                'models': {
                    arm_id: yaml.safe_dump(model.to_dict(), allow_unicode=True, sort_keys=False)
                    for arm_id, model in self.arm_models.items()
                },
            }


class _WorkbenchHandler(BaseHTTPRequestHandler):
    node: GravityWorkbenchNode
    static_dir: Path

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, status: int, text: str, content_type: str) -> None:
        body = text.encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get('Content-Length', '0'))
        raw = self.rfile.read(length).decode('utf-8') if length > 0 else '{}'
        data = json.loads(raw or '{}')
        if not isinstance(data, dict):
            raise ValueError('Expected JSON object body')
        return data

    def do_GET(self) -> None:
        path = urlsplit(self.path).path
        if path == '/api/status':
            self._send_json(200, self.node.status_payload())
            return
        if path == '/api/samples.csv':
            with self.node.lock:
                samples = list(self.node.samples)
            self._send_text(200, format_probe_samples_csv_text(samples), 'text/csv; charset=utf-8')
            return
        rel = 'index.html' if path in ('', '/') else path.lstrip('/')
        rel_path = Path(rel)
        if '..' in rel_path.parts:
            self._send_json(404, {'error': f'Not found: {path}'})
            return
        file_path = self.static_dir / rel_path
        if not file_path.is_file():
            self._send_json(404, {'error': f'Not found: {path}'})
            return
        mime = {
            '.html': 'text/html; charset=utf-8',
            '.css': 'text/css; charset=utf-8',
            '.js': 'application/javascript; charset=utf-8',
        }.get(file_path.suffix, 'application/octet-stream')
        data = file_path.read_bytes()
        self.send_response(200)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self) -> None:
        path = urlsplit(self.path).path
        try:
            body = self._read_json()
            if path == '/api/probe/start':
                ns = str(body.get('namespace', '/arm_1'))
                kp_raw = body.get('kp', list(DEFAULT_PROBE_KP))
                kd_raw = body.get('kd')
                num_points = int(body.get('num_points', 9))
                pre_travel = float(body.get('pre_travel', 0.04))
                approach_speed = float(body.get('approach_speed', 0.05))
                settle_time = float(body.get('settle_time', 0.60))

                plan = body.get('joints_plan')
                configs: list[ProbeConfig] = []
                if isinstance(plan, list) and plan:
                    for item in plan:
                        configs.append(ProbeConfig(
                            namespace=ns,
                            joint=int(item['joint']),
                            q_min=float(item['q_min']),
                            q_max=float(item['q_max']),
                            num_points=num_points,
                            pre_travel=pre_travel,
                            approach_speed=approach_speed,
                            settle_time=settle_time,
                            kp=tuple(float(x) for x in kp_raw),
                            kd=None if not kd_raw else tuple(float(x) for x in kd_raw),
                        ))
                else:
                    configs.append(ProbeConfig(
                        namespace=ns,
                        joint=int(body.get('joint', 2)),
                        q_min=float(body['q_min']),
                        q_max=float(body['q_max']),
                        num_points=num_points,
                        pre_travel=pre_travel,
                        approach_speed=approach_speed,
                        settle_time=settle_time,
                        kp=tuple(float(x) for x in kp_raw),
                        kd=None if not kd_raw else tuple(float(x) for x in kd_raw),
                    ))
                payload = self.node.start_probe(
                    configs,
                    output_csv=str(body.get('output_csv', '')),
                    fit_output=str(body.get('fit_output', '')),
                )
                self._send_json(200, {'ok': True, 'status': payload})
                return

            if path == '/api/probe/stop':
                ns = str(body.get('namespace', '')).strip() or None
                payload = self.node.stop_probe(namespace=ns)
                self._send_json(200, {'ok': True, 'status': payload})
                return

            if path == '/api/arm/compensation_scales':
                ns = str(body.get('namespace', '/arm_1'))
                g_scale = float(body.get('gravity_scale', 0.0))
                f_scale = float(body.get('friction_scale', 0.0))
                sent = self.node.request_compensation_scales(ns, g_scale, f_scale)
                self._send_json(200, {
                    'ok': True,
                    'sent': sent,
                    'namespace': ns,
                    'gravity_scale': g_scale,
                    'friction_scale': f_scale,
                })
                return

            if path == '/api/fit':
                arm_id = str(body.get('arm_id', 'arm_1'))
                source = str(body.get('source', 'memory'))
                torque_source = str(body.get('torque_source', 'auto'))
                out_yaml = str(body.get('output_yaml', '')).strip() or None
                fric_yaml = str(body.get('update_friction_yaml', '')).strip() or None

                if source == 'demo_all':
                    res = self.node.run_fit_all_demo(
                        arm_id=arm_id,
                        torque_source=torque_source,
                        output_yaml=out_yaml,
                    )
                    self._send_json(200, {'ok': True, **res})
                    return
                if source == 'demo':
                    joint = int(body.get('joint', 2))
                    samples = synthesize_demo_probe_samples(
                        joint=joint, arm_model=self.node.arm_models.get(arm_id))
                    with self.node.lock:
                        self.node.samples = samples
                        self.node.joint_samples.setdefault(arm_id, {})[joint] = samples
                elif source == 'csv_text':
                    samples = parse_probe_samples_csv_text(str(body.get('csv_text', '')))
                    with self.node.lock:
                        self.node.samples = samples
                        if samples:
                            self.node.joint_samples.setdefault(arm_id, {})[samples[0].joint] = samples
                else:
                    req_joint = body.get('joint')
                    with self.node.lock:
                        if req_joint is not None and int(req_joint) in self.node.joint_samples.get(arm_id, {}):
                            samples = list(self.node.joint_samples[arm_id][int(req_joint)])
                        else:
                            samples = list(self.node.samples)

                fit_payload = self.node.run_fit(
                    samples,
                    arm_id=arm_id,
                    torque_source=torque_source,
                    output_yaml=out_yaml,
                    update_friction_yaml=fric_yaml,
                )
                self._send_json(200, {'ok': True, 'fit': fit_payload})
                return

            if path == '/api/save':
                arm_id = str(body.get('arm_id', 'arm_1'))
                out_path = Path(str(body['output_yaml']))
                fric_yaml = str(body.get('update_friction_yaml', '')).strip()
                with self.node.lock:
                    model = self.node.arm_models.get(arm_id, ArmGravityModel(arm_id=arm_id))
                    saved = save_gravity_model(model, out_path)
                    if fric_yaml:
                        _sync_static_friction_to_yaml(
                            Path(fric_yaml),
                            arm_id,
                            {j: m.static_friction for j, m in model.joints.items()},
                        )
                self.node.reload_saved_model(arm_id, out_path)
                self._send_json(200, {'ok': True, 'path': str(saved)})
                return

            if path == '/api/evaluate':
                arm_id = str(body.get('arm_id', 'arm_1'))
                positions = [float(x) for x in body.get('position', [0.0] * 6)]
                with self.node.lock:
                    model = self.node.arm_models.get(arm_id, ArmGravityModel(arm_id=arm_id))
                    arm = self.node.fleet.arm(arm_id)
                    effector = self.node.effector_kwargs[arm.namespace]
                cad = gravity_torques(tuple(positions), **effector)
                cal = model.torques(positions, **effector)
                fs_vec = [model.joint(i).static_friction for i in range(1, 7)]
                self._send_json(200, {
                    'ok': True,
                    'gravity_cad': list(cad),
                    'gravity_calibrated': list(cal),
                    'static_friction': fs_vec,
                })
                return

            self._send_json(404, {'error': f'Unknown endpoint: {path}'})
        except Exception as error:
            self._send_json(400, {'ok': False, 'error': str(error)})


def create_http_server(
    node: GravityWorkbenchNode,
    host: str = '127.0.0.1',
    port: int = 8091,
) -> ThreadingHTTPServer:
    handler = type(
        'BoundGravityWorkbenchHandler',
        (_WorkbenchHandler,),
        {'node': node, 'static_dir': web_root_path()},
    )
    return ThreadingHTTPServer((host, port), handler)


def web_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Launch the PiPER-X Gravity & Static Friction Calibration Web Workbench.'
    )
    parser.add_argument('--host', default='127.0.0.1', help='HTTP bind host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=8091, help='HTTP port (default: 8091)')
    parser.add_argument('--config', default=None, help='Optional fleet arms.yaml path')
    args = parser.parse_args(argv)

    rclpy.init()
    node = GravityWorkbenchNode(config_file=args.config)
    server = create_http_server(node, host=args.host, port=args.port)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info(
        f'PiPER-X Gravity Calibration Workbench listening on http://{args.host}:{server.server_port}/'
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
