"""HTTP + ROS 2 workbench for the 5-step joint friction calibration workflow."""

import argparse
from dataclasses import asdict, dataclass, field
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
from piper_x_driver.gravity import gravity_torques
from piper_x_interfaces.msg import ArmStatus, GripperCommand, MitCommand
from piper_x_interfaces.srv import ConfigureArm, Enable, SetMode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.signal import detrend
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml

from .collector import (
    DEFAULT_GRIPPER_EFFORT,
    DEFAULT_GRIPPER_SPEEDS,
    DEFAULT_SWEEP_KP,
    DEFAULT_SWEEP_SPEEDS,
    JOINT_NAMES,
    SweepConfig,
    build_sweep_schedule,
    is_velocity_stabilized,
    resolve_safe_anchor,
    sample_velocity_confidence,
)
from .identification import (
    CSV_COLUMNS,
    IdentificationReport,
    SweepSample,
    identify_joint_friction,
    load_samples_csv,
    save_samples_csv,
)
from .model import (
    ArmFrictionModel,
    GripperFrictionModel,
    JointFrictionModel,
    SpatialHarmonic,
    load_friction_model,
    save_friction_model,
)

DEFAULT_JOINT_RANGES: dict[int, tuple[float, float]] = {
    1: (-0.50, 0.50),
    2: (0.55, 1.20),
    3: (-1.80, -1.15),
    4: (-0.45, 0.45),
    5: (-0.40, 0.40),
    6: (-0.50, 0.50),
    7: (0.003, 0.080),
}


def web_root_path() -> Path:
    src_web = Path(__file__).resolve().parent.parent / 'web'
    if (src_web / 'index.html').is_file():
        return src_web
    try:
        share = Path(get_package_share_directory('piper_friction')) / 'web'
        if (share / 'index.html').is_file():
            return share
    except PackageNotFoundError:
        pass
    return src_web


def synthesize_demo_samples(
    joint: int = 1,
    speeds: tuple[float, ...] | None = None,
    q_min: float | None = None,
    q_max: float | None = None,
) -> list[SweepSample]:
    """Generate realistic synthetic multi-speed sweep data for J1..J6 or Gripper (J7)."""
    default_lo, default_hi = DEFAULT_JOINT_RANGES.get(joint, (-0.5, 0.5))
    lo = default_lo if q_min is None else float(q_min)
    hi = default_hi if q_max is None else float(q_max)

    if joint == 7:
        g_speeds = DEFAULT_GRIPPER_SPEEDS if speeds is None else speeds
        w_knots = tuple(float(x) for x in np.linspace(lo, hi, 25))
        span_w = max(1e-6, hi - lo)
        # Non-periodic linkage & return-spring bias + U-shaped guide rail tightness
        bias_knots = tuple(
            round(-0.18 - 0.42 * ((w - lo) / span_w) - 0.08 * math.exp(-(((w - lo) / span_w - 0.25) / 0.18) ** 2), 4)
            for w in w_knots
        )
        raw_mod = [
            0.22 * (((w - lo) / span_w - 0.48) ** 2)
            + 0.09 * math.exp(-(((w - lo) / span_w - 0.15) / 0.12) ** 2)
            - 0.06 * math.exp(-(((w - lo) / span_w - 0.55) / 0.20) ** 2)
            for w in w_knots
        ]
        mod_mean = sum(raw_mod) / len(raw_mod)
        mod_knots = tuple(round(v - mod_mean, 4) for v in raw_mod)
        demo_grip = GripperFrictionModel(
            fc=0.38,
            fs=0.72,
            vs=0.016,
            delta=2.0,
            fv=2.40,
            v0=0.002,
            vr=0.075,
            kp_equiv=85.0,
            kv_equiv=12.5,
            w_grid=w_knots,
            bias_grid=bias_knots,
            modulation_grid=mod_knots,
        )
        rng = np.random.default_rng(49)
        samples_g: list[SweepSample] = []
        t = 0.0
        for speed in g_speeds:
            n_steps = max(240, int((span_w / speed) * 80))
            dt = (span_w / speed) / n_steps
            for direction in (1.0, -1.0):
                ws = (
                    np.linspace(lo, hi, n_steps)
                    if direction > 0
                    else np.linspace(hi, lo, n_steps)
                )
                dw = direction * speed
                for w in ws:
                    f_true = demo_grip.force(float(w), float(dw))
                    noise = float(rng.normal(0.0, 0.008))
                    f_meas = f_true + noise
                    w_des = (
                        float(w)
                        + f_true / (6.0 * demo_grip.kp_equiv)
                        + float(dw) * (0.025 + 1.0 / demo_grip.kv_equiv)
                    )
                    samples_g.append(SweepSample(
                        t=t,
                        joint=7,
                        q=float(w),
                        dq=float(dw),
                        tau_meas=float(f_meas),
                        q_des=float(w_des),
                        dq_des=float(dw),
                        tau_gravity=0.0,
                    ))
                    t += dt
        return samples_g

    if speeds is None:
        speeds = (0.03, 0.06, 0.12, 0.25, 0.5, 1.0, 1.5)

    scale = (1.0, 1.25, 0.95, 0.45, 0.38, 0.25)[joint - 1]
    base_omega = (36.0, 36.0, 36.0, 28.0, 28.0, 24.0)[joint - 1]
    demo_model = JointFrictionModel(
        fc=round(0.32 * scale, 4),
        fs=round(0.76 * scale, 4),
        vs=round(0.10 + 0.01 * joint, 4),
        delta=2.0,
        fv=round(0.14 * scale, 4),
        v0=0.005,
        vr=round(0.32 + 0.02 * joint, 4),
        harmonics=(
            SpatialHarmonic(
                omega=base_omega,
                a=round(0.15 * scale, 4),
                b=round(-0.07 * scale, 4),
                c=round(0.08 * scale, 4),
                d=round(0.04 * scale, 4),
            ),
            SpatialHarmonic(
                omega=base_omega * 2.0,
                a=round(-0.04 * scale, 4),
                b=round(0.05 * scale, 4),
                c=round(0.03 * scale, 4),
                d=round(-0.02 * scale, 4),
            ),
        ),
    )
    rng = np.random.default_rng(42 + joint)
    samples: list[SweepSample] = []
    t = 0.0
    span = hi - lo
    for speed in speeds:
        n_steps = max(360, int((span / speed) * 120))
        dt = (span / speed) / n_steps
        for direction in (1.0, -1.0):
            qs = (
                np.linspace(lo, hi, n_steps)
                if direction > 0
                else np.linspace(hi, lo, n_steps)
            )
            dq = direction * speed
            for q in qs:
                tau_g = 0.0 if joint in (1, 6) else (2.2 * scale) * math.cos(float(q))
                tau_f = demo_model.torque(float(q), float(dq))
                noise = float(rng.normal(0.0, 0.003 * scale))
                samples.append(SweepSample(
                    t=t,
                    joint=joint,
                    q=float(q),
                    dq=float(dq),
                    tau_meas=float(tau_g + tau_f + noise),
                    q_des=float(q),
                    dq_des=float(dq),
                    tau_gravity=float(tau_g),
                ))
                t += dt
    return samples


def parse_samples_csv_text(text: str) -> list[SweepSample]:
    import csv

    reader = csv.DictReader(io.StringIO(text))
    rows: list[SweepSample] = []
    for row in reader:
        rows.append(SweepSample(
            t=float(row['t']),
            joint=int(row['joint']),
            q=float(row['q']),
            dq=float(row['dq']),
            tau_meas=float(row['tau_meas']),
            q_des=float(row['q_des']),
            dq_des=float(row['dq_des']),
            tau_gravity=float(row.get('tau_gravity', 0.0) or 0.0),
        ))
    return rows


def format_samples_csv_text(samples: Sequence[SweepSample]) -> str:
    import csv

    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(CSV_COLUMNS)
    for s in samples:
        writer.writerow([
            f'{s.t:.6f}',
            int(s.joint),
            f'{s.q:.8f}',
            f'{s.dq:.8f}',
            f'{s.tau_meas:.8f}',
            f'{s.q_des:.8f}',
            f'{s.dq_des:.8f}',
            f'{s.tau_gravity:.8f}',
        ])
    return buf.getvalue()


def serialize_report(
    report: IdentificationReport,
    arm_model: ArmFrictionModel,
    max_curve_points: int = 400,
) -> dict[str, Any]:
    """Convert IdentificationReport + ArmFrictionModel into JSON-serializable chart & table data."""
    m = report.model
    d = report.decomposition
    n_grid = len(d.theta_grid)
    stride = max(1, n_grid // max_curve_points)
    idx = np.arange(0, n_grid, stride)
    theta_sub = d.theta_grid[idx]

    is_gripper = isinstance(m, GripperFrictionModel)
    half_sum_0 = (
        d.half_sums[0][idx]
        if is_gripper
        else detrend(d.half_sums[0], type='linear')[idx]
    )
    half_diff_0 = d.half_diffs[0][idx]
    mean_f0 = float(np.mean(d.half_diffs[0]))
    mod_actual_0 = half_diff_0 - mean_f0

    env0 = m.envelope(float(d.speeds[0]))
    cog_fit_0 = (
        [float(m.bias_force(float(th))) for th in theta_sub]
        if isinstance(m, GripperFrictionModel)
        else [float(env0 * m.cogging_torque(float(th))) for th in theta_sub]
    )
    mod_fit_0 = [float(env0 * m.modulation_torque(float(th))) for th in theta_sub]
    tau_pos_0 = (half_sum_0 + half_diff_0).tolist()
    tau_neg_0 = (half_sum_0 - half_diff_0).tolist()

    v_max = (
        max(float(np.max(d.speeds)) * 1.2, 0.10)
        if is_gripper
        else max(float(np.max(d.speeds)) * 1.15, 1.6)
    )
    v_dense = np.linspace(0.0, v_max, 120)
    stribeck_dense = [float(m.stribeck_magnitude(float(v))) for v in v_dense]
    envelope_dense = [float(m.envelope(float(v))) for v in v_dense]

    yaml_text = yaml.safe_dump(arm_model.to_dict(), allow_unicode=True, sort_keys=False)

    harmonics_list = []
    profile_knots = []
    if isinstance(m, GripperFrictionModel):
        for w_k, b_k, mod_k in zip(m.w_grid, m.bias_grid, m.modulation_grid):
            profile_knots.append({
                'w_m': float(w_k),
                'w_mm': float(w_k * 1000.0),
                'bias_n': float(b_k),
                'modulation_n': float(mod_k),
                'local_fc_n': float(max(0.0, m.fc + mod_k)),
            })
    else:
        harmonics_list = [
            {
                **h.to_dict(),
                'cogging_amplitude': float(h.cogging_amplitude),
                'modulation_amplitude': float(h.modulation_amplitude),
                'period_deg': float(360.0 / h.omega) if h.omega > 0 else 0.0,
            }
            for h in m.harmonics
        ]

    return {
        'joint': int(report.joint),
        'is_gripper': is_gripper,
        'arm_id': arm_model.arm_id,
        'model': m.to_dict(),
        'harmonics': harmonics_list,
        'profile_knots': profile_knots,
        'metrics': {
            'rmse_total': float(report.rmse_total),
            'rmse_stribeck': float(report.rmse_stribeck),
            'rmse_cogging': float(report.rmse_cogging),
            'rmse_modulation': float(report.rmse_modulation),
            'grid_points': int(n_grid),
            'theta_min': float(d.theta_grid[0]),
            'theta_max': float(d.theta_grid[-1]),
            'speed_count': int(len(d.speeds)),
        },
        'curves': {
            'theta_grid': [float(x) for x in theta_sub],
            'tau_pos_low': [float(x) for x in tau_pos_0],
            'tau_neg_low': [float(x) for x in tau_neg_0],
            'half_sum_low': [float(x) for x in half_sum_0],
            'cogging_fit_low': cog_fit_0,
            'modulation_actual_low': [float(x) for x in mod_actual_0],
            'modulation_fit_low': mod_fit_0,
            'speeds': [float(x) for x in d.speeds],
            'mean_frictions': [float(x) for x in d.mean_frictions],
            'ripple_scales': [float(x) for x in d.ripple_scales],
            'v_dense': [float(x) for x in v_dense],
            'stribeck_dense': stribeck_dense,
            'envelope_dense': envelope_dense,
        },
        'yaml_text': yaml_text,
    }


@dataclass
class _ArmSweepRunner:
    """Independent per-arm multi-speed friction sweep state machine."""

    queue: list[SweepConfig] = field(default_factory=list)
    queue_index: int = 0
    config: SweepConfig | None = None
    phase: str = 'idle'
    error: str = ''
    initial: tuple[float, ...] | None = None
    anchor: tuple[float, ...] | None = None
    schedule: list = field(default_factory=list)
    segment_index: int = 0
    segment_started: float = 0.0
    sweep_started: float = 0.0
    current_q_des: float = 0.0
    current_dq_des: float = 0.0
    pending_future: Any = None
    segment_velocities: list[tuple[float, float]] = field(default_factory=list)
    output_csv: str = ''
    fit_output: str = ''
    saved_friction_scale: float | None = None
    samples: list[SweepSample] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        total_segs = len(self.schedule)
        cur_seg = None
        if 0 <= self.segment_index < total_segs:
            seg = self.schedule[self.segment_index]
            cur_seg = {
                'kind': seg.kind,
                'q_start': seg.q_start,
                'q_end': seg.q_end,
                'duration': seg.duration,
                'dq_des': seg.dq_des,
            }
        return {
            'phase': self.phase,
            'error': self.error,
            'config': asdict(self.config) if self.config else None,
            'queue_joints': [c.joint for c in self.queue],
            'queue_index': self.queue_index,
            'queue_total': len(self.queue),
            'segment_index': self.segment_index,
            'total_segments': total_segs,
            'current_segment': cur_seg,
            'current_q_des': self.current_q_des,
            'current_dq_des': self.current_dq_des,
            'sample_count': len(self.samples),
            'output_csv': self.output_csv,
            'fit_output': self.fit_output,
        }


class FrictionWorkbenchNode(Node):
    """ROS 2 node backing the friction calibration web workbench."""

    def __init__(self, config_file: str | Path | None = None, **kwargs):
        super().__init__('piper_friction_web', **kwargs)
        self.lock = threading.Lock()
        cfg_path = Path(config_file) if config_file else default_config_path()
        self.fleet = FleetConfig(cfg_path)
        self.arms_by_ns = {arm.namespace: arm for arm in self.fleet.arms}

        self.positions: dict[str, tuple[float, ...]] = {}
        self.velocities: dict[str, tuple[float, ...]] = {}
        self.torques: dict[str, tuple[float, ...]] = {}
        self.gripper_velocities: dict[str, float] = {}
        self.gripper_forces: dict[str, float] = {}
        self._prev_grip_samples: dict[str, tuple[float, float]] = {}
        self.joint_stamps: dict[str, float] = {}
        self.statuses: dict[str, ArmStatus] = {}
        self.status_stamps: dict[str, float] = {}
        self.control_states: dict[str, dict[str, Any]] = {}
        self.effector_kwargs: dict[str, dict[str, Any]] = {}

        self.mit_pubs: dict[str, Any] = {}
        self.gripper_pubs: dict[str, Any] = {}
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
            self.gripper_pubs[ns] = self.create_publisher(GripperCommand, f'{ns}/command/gripper', 1)
            self.config_clients[ns] = self.create_client(ConfigureArm, f'{ns}/configure_arm')
            self.enable_clients[ns] = self.create_client(Enable, f'{ns}/enable_srv')
            self.mode_clients[ns] = self.create_client(SetMode, f'{ns}/set_mode')

        self.runners: dict[str, _ArmSweepRunner] = {
            arm.namespace: _ArmSweepRunner() for arm in self.fleet.arms
        }
        self.active_namespace: str = self.fleet.arms[0].namespace
        self.samples: list[SweepSample] = []
        self.joint_samples: dict[str, dict[int, list[SweepSample]]] = {
            arm.id: {} for arm in self.fleet.arms
        }
        self.joint_fits: dict[str, dict[int, dict[str, Any]]] = {
            arm.id: {} for arm in self.fleet.arms
        }
        self.latest_fit: dict[str, Any] | None = None
        self.arm_models: dict[str, ArmFrictionModel] = {
            arm.id: (self.fleet.friction_model(arm) or ArmFrictionModel(arm_id=arm.id))
            for arm in self.fleet.arms
        }

        self.create_timer(0.01, self._tick_sweep)

    @property
    def sweep_phase(self) -> str:
        return self.runners[self.active_namespace].phase

    @property
    def sweep_error(self) -> str:
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
            if len(message.velocity) >= 7:
                self.gripper_velocities[ns] = float(message.velocity[order[6]])
            elif (
                'gripper_velocity' not in self.control_states.get(ns, {})
                and ns in self._prev_grip_samples
            ):
                dt = now - self._prev_grip_samples[ns][0]
                if 1e-4 <= dt <= 0.5:
                    self.gripper_velocities[ns] = (
                        pos[6] - self._prev_grip_samples[ns][1]
                    ) / dt
            self._prev_grip_samples[ns] = (now, pos[6])
            if len(message.effort) >= 7:
                self.gripper_forces[ns] = float(message.effort[order[6]])
            self.joint_stamps[ns] = now
            runner = self.runners.get(ns)
            if (runner is not None
                    and runner.phase == 'running'
                    and runner.config is not None
                    and abs(runner.current_dq_des) > 1e-6):
                j_idx = runner.config.joint - 1
                if runner.config.joint == 7:
                    dq_meas = self.gripper_velocities.get(ns, 0.0)
                    tau_meas = self.gripper_forces.get(ns, 0.0)
                else:
                    dq_meas = vel[j_idx]
                    tau_meas = tau[j_idx]
                runner.segment_velocities.append((now, dq_meas))
                conf = sample_velocity_confidence(runner.segment_velocities, runner.current_dq_des)
                if conf > 0.0:
                    ctrl_state = self.control_states.get(ns, {})
                    if runner.config.joint == 7:
                        g_val = 0.0
                    else:
                        g_vec = ctrl_state.get('gravity_torque')
                        if isinstance(g_vec, list) and len(g_vec) >= 6:
                            g_val = float(g_vec[j_idx])
                        else:
                            arm_obj = self.arms_by_ns.get(ns)
                            g_model = self.fleet.gravity_model(arm_obj) if arm_obj is not None else None
                            g_torques = (
                                gravity_torques(pos[:6], **self.effector_kwargs.get(ns, {}))
                                if g_model is None
                                else g_model.torques(pos[:6], **self.effector_kwargs.get(ns, {}))
                            )
                            g_scale_app = float(ctrl_state.get('gravity_scale_applied', 1.0))
                            g_val = g_scale_app * float(g_torques[j_idx])
                    sample = SweepSample(
                        t=now - runner.sweep_started,
                        joint=runner.config.joint,
                        q=pos[j_idx],
                        dq=dq_meas,
                        tau_meas=tau_meas,
                        q_des=runner.current_q_des,
                        dq_des=runner.current_dq_des,
                        tau_gravity=g_val,
                        weight=conf,
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
            if isinstance(state.get('gripper_velocity'), (int, float)):
                self.gripper_velocities[ns] = float(state['gripper_velocity'])
            if isinstance(state.get('gripper_force'), (int, float)):
                self.gripper_forces[ns] = float(state['gripper_force'])
            eff = state.get('configuration', {}).get('effector', {})
            if 'mass_kg' in eff and 'com_m' in eff:
                self.effector_kwargs[ns] = {
                    'mass_kg': float(eff['mass_kg']),
                    'com_m': tuple(float(x) for x in eff['com_m']),
                }

    def _publish_mit_locked(
        self,
        cfg: SweepConfig,
        anchor: tuple[float, ...],
        q_j: float,
        dq_j: float,
        anchor_blend: float = 1.0,
        initial: tuple[float, ...] | None = None,
    ) -> None:
        if cfg.joint == 7:
            self.gripper_pubs[cfg.namespace].publish(GripperCommand(
                width=max(0.0, min(0.1, float(q_j))),
                effort=float(cfg.gripper_effort),
            ))
            return
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
        pos[j_idx] = float(q_j)
        vel[j_idx] = float(dq_j)
        msg = MitCommand()
        msg.position = pos
        msg.velocity = vel
        msg.kp = list(cfg.kp)
        msg.kd = list(cfg.kd)
        msg.torque = [0.0] * 6
        self.mit_pubs[cfg.namespace].publish(msg)

    def start_sweep(
        self,
        configs: SweepConfig | Sequence[SweepConfig],
        output_csv: str = '',
        fit_output: str = '',
    ) -> dict[str, Any]:
        queue = [configs] if isinstance(configs, SweepConfig) else list(configs)
        if not queue:
            raise ValueError('At least one joint sweep configuration is required')
        namespaces = {c.namespace for c in queue}
        if len(namespaces) != 1:
            raise ValueError('All queued joint sweeps must target the same CAN port / arm')
        ns = next(iter(namespaces))
        with self.lock:
            if ns not in self.arms_by_ns:
                raise ValueError(f'Unknown namespace {ns}')
            runner = self.runners[ns]
            if runner.phase in (
                'waiting_feedback', 'zeroing_friction', 'enabling',
                'setting_mode', 'running', 'restoring_friction',
            ):
                raise ValueError(f'{ns} already has an active sweep; stop it first')
            runner.queue = queue
            runner.queue_index = 0
            runner.config = queue[0]
            runner.phase = 'waiting_feedback'
            runner.error = ''
            runner.initial = None
            runner.anchor = None
            runner.schedule = []
            runner.segment_index = 0
            runner.segment_velocities = []
            runner.samples = []
            runner.output_csv = output_csv.strip()
            runner.fit_output = fit_output.strip()
            self.active_namespace = ns
            self.samples = runner.samples
        return self.status_payload()

    def stop_sweep(self, namespace: str | None = None) -> dict[str, Any]:
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
                    cur_pos = self.positions[ns][:7]
                    self._publish_mit_locked(
                        runner.config, cur_pos, cur_pos[runner.config.joint - 1], 0.0,
                    )
                    if runner.config.joint == 7:
                        self.gripper_pubs[ns].publish(GripperCommand(
                            width=float(cur_pos[6]), effort=0.0,
                        ))
                runner.phase = 'stopped'
                runner.queue = []
                runner.pending_future = None
                runner.segment_velocities = []
                self._restore_friction_scale_locked(ns, runner)
        return self.status_payload()

    def _restore_friction_scale_locked(self, ns: str, runner: _ArmSweepRunner):
        if (runner.saved_friction_scale is not None
                and ns in self.config_clients
                and self.config_clients[ns].service_is_ready()):
            arm = self.arms_by_ns[ns]
            ctrl = self.control_states.get(ns, {})
            cfg_dict = ctrl.get('configuration', {})
            end_eff = str(cfg_dict.get('end_effector') or arm.end_effector)
            grav_scale = float(cfg_dict.get('gravity_scale', arm.gravity_scale))
            fut = self.config_clients[ns].call_async(ConfigureArm.Request(
                end_effector=end_eff,
                gravity_scale=grav_scale,
                friction_scale=-4.0 - runner.saved_friction_scale,
            ))
            runner.saved_friction_scale = None
            return fut
        return None

    def _fail_sweep_locked(self, runner: _ArmSweepRunner, reason: str, ns: str = '') -> None:
        if ns:
            self._restore_friction_scale_locked(ns, runner)
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

    def request_friction_scale(self, namespace: str, friction_scale: float) -> bool:
        """Asynchronously request driver to update runtime friction_scale without modifying arms.yaml."""
        scale = bounded(friction_scale, 'friction_scale', 0, 1)
        with self.lock:
            if namespace not in self.arms_by_ns:
                raise ValueError(f'Unknown namespace {namespace}')
            arm = self.arms_by_ns[namespace]
            ctrl = self.control_states.get(namespace, {})
            cfg_dict = ctrl.get('configuration', {})
            end_eff = str(cfg_dict.get('end_effector') or arm.end_effector)
            grav_scale = float(cfg_dict.get('gravity_scale', arm.gravity_scale))
            client = self.config_clients[namespace]
            if not client.service_is_ready():
                return False
            client.call_async(ConfigureArm.Request(
                end_effector=end_eff,
                gravity_scale=grav_scale,
                friction_scale=-2.0 - scale,
            ))
            return True

    def reload_saved_model(self, arm_id: str, output_yaml: str | Path) -> bool:
        """Hot-reload the saved friction YAML in the running driver without modifying arms.yaml."""
        with self.lock:
            arm = self.fleet.arm(arm_id)
            runner = self.runners[arm.namespace]
            if runner.phase in (
                'waiting_feedback', 'zeroing_friction', 'enabling', 'setting_mode', 'running',
            ):
                return False
            ctrl = self.control_states.get(arm.namespace, {})
            cfg_dict = ctrl.get('configuration', {})
            active_path = cfg_dict.get('friction_config') or arm.friction_config
            if not active_path or Path(active_path).resolve() != Path(output_yaml).resolve():
                return False
            end_eff = str(cfg_dict.get('end_effector') or arm.end_effector)
            grav_scale = float(cfg_dict.get('gravity_scale', arm.gravity_scale))
            f_scale = float(cfg_dict.get('friction_scale', arm.friction_scale))
            client = self.config_clients[arm.namespace]
            if not client.service_is_ready():
                return False
            client.call_async(ConfigureArm.Request(
                end_effector=end_eff,
                gravity_scale=grav_scale,
                friction_scale=-4.0 - f_scale,
            ))
            return True

    def _tick_sweep(self) -> None:
        completed_jobs: list[tuple[list[SweepSample], SweepConfig, str, str, bool]] = []

        with self.lock:
            for ns, runner in self.runners.items():
                if runner.phase not in (
                    'waiting_feedback', 'zeroing_friction', 'enabling',
                    'setting_mode', 'running', 'restoring_friction',
                ):
                    continue
                cfg = runner.config
                assert cfg is not None
                pos = self.positions.get(ns)
                status = self.statuses.get(ns)
                if pos is None or status is None:
                    continue
                if status.err_code or status.arm_status not in (0, 11, 12, 13):
                    self._fail_sweep_locked(
                        runner,
                        f'Arm fault: arm_status={status.arm_status}, err_code={status.err_code}',
                        ns,
                    )
                    continue

                if runner.phase == 'waiting_feedback':
                    runner.initial = pos[:7]
                    if runner.anchor is None:
                        base_anchor = (
                            resolve_safe_anchor(runner.initial[:6], cfg.joint)
                            if cfg.use_safe_anchor
                            else runner.initial[:6]
                        )
                        runner.anchor = base_anchor + (runner.initial[6],)
                    runner.current_q_des = runner.initial[cfg.joint - 1]
                    runner.current_dq_des = 0.0
                    cfg_client = self.config_clients[ns]
                    if cfg_client.service_is_ready():
                        arm = self.arms_by_ns[ns]
                        ctrl = self.control_states.get(ns, {})
                        cfg_dict = ctrl.get('configuration', {})
                        end_eff = str(cfg_dict.get('end_effector') or arm.end_effector)
                        grav_scale = float(cfg_dict.get('gravity_scale', arm.gravity_scale))
                        runner.saved_friction_scale = float(
                            cfg_dict.get('friction_scale', arm.friction_scale)
                        )
                        runner.pending_future = cfg_client.call_async(ConfigureArm.Request(
                            end_effector=end_eff,
                            gravity_scale=grav_scale,
                            friction_scale=-2.0,
                        ))
                        runner.phase = 'zeroing_friction'
                        continue
                    self._check_enable_or_mit_locked(runner, cfg, status)
                    continue

                if runner.phase == 'zeroing_friction':
                    if runner.pending_future is None or not runner.pending_future.done():
                        continue
                    res = runner.pending_future.result()
                    if res is None or not res.success:
                        self._fail_sweep_locked(
                            runner,
                            f'Zeroing friction_scale failed: {getattr(res, "message", "no response")}',
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
                        self._fail_sweep_locked(
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
                        self._fail_sweep_locked(
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
                            interp = (
                                0.5 * (1.0 - math.cos(math.pi * ratio))
                                if seg.kind in ('approach', 'return')
                                else ratio
                            )
                            runner.current_q_des = seg.q_start + interp * (seg.q_end - seg.q_start)
                            runner.current_dq_des = seg.dq_des
                            blend = (
                                interp
                                if (runner.segment_index == 0 and seg.kind == 'approach')
                                else 1.0
                            )
                            self._publish_mit_locked(
                                cfg, runner.anchor, runner.current_q_des, runner.current_dq_des,
                                anchor_blend=blend, initial=runner.initial,
                            )
                            advanced = True
                            break
                        runner.segment_index += 1
                        runner.segment_started = now
                        runner.segment_velocities = []
                    if advanced:
                        continue

                    runner.current_q_des = runner.anchor[cfg.joint - 1]
                    runner.current_dq_des = 0.0
                    runner.segment_velocities = []
                    self._publish_mit_locked(cfg, runner.anchor, runner.current_q_des, 0.0)
                    if cfg.joint == 7:
                        self.gripper_pubs[cfg.namespace].publish(GripperCommand(
                            width=float(runner.current_q_des), effort=0.0,
                        ))
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
                        runner.initial = pos[:7]
                        base_anchor = (
                            resolve_safe_anchor(runner.initial[:6], runner.config.joint)
                            if runner.config.use_safe_anchor
                            else runner.initial[:6]
                        )
                        runner.anchor = base_anchor + (runner.initial[6],)
                        runner.samples = []
                        if ns == self.active_namespace:
                            self.samples = runner.samples
                        self._start_schedule_locked(runner, runner.config)
                        has_next_joint = True
                    else:
                        has_next_joint = False
                        restore_fut = self._restore_friction_scale_locked(ns, runner)
                        if restore_fut is not None:
                            runner.pending_future = restore_fut
                            runner.phase = 'restoring_friction'
                        else:
                            runner.phase = 'completed'

                    completed_jobs.append((finished_samples, finished_cfg, out_csv, out_fit, has_next_joint))

                elif runner.phase == 'restoring_friction':
                    assert runner.anchor is not None
                    self._publish_mit_locked(cfg, runner.anchor, runner.current_q_des, 0.0)
                    if runner.pending_future is not None and not runner.pending_future.done():
                        continue
                    runner.phase = 'completed'

        for finished_samples, finished_cfg, out_csv, out_fit, has_next_joint in completed_jobs:
            try:
                if out_csv:
                    save_samples_csv(finished_samples, out_csv)
                arm_id = self.arms_by_ns[finished_cfg.namespace].id
                self.run_fit(
                    finished_samples,
                    arm_id=arm_id,
                    output_yaml=out_fit if (out_fit and not has_next_joint) else None,
                )
                if out_fit and has_next_joint:
                    with self.lock:
                        save_friction_model(self.arm_models[arm_id], out_fit)
            except Exception as error:
                with self.lock:
                    self.runners[finished_cfg.namespace].error = (
                        f'J{finished_cfg.joint} auto-fit warning: {error}'
                    )

    def _check_enable_or_mit_locked(
        self, runner: _ArmSweepRunner, cfg: SweepConfig, status: ArmStatus,
    ) -> None:
        if cfg.joint == 7:
            self._start_schedule_locked(runner, cfg)
            return
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
        self, runner: _ArmSweepRunner, cfg: SweepConfig, status: ArmStatus,
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

    def _start_schedule_locked(self, runner: _ArmSweepRunner, cfg: SweepConfig) -> None:
        assert runner.anchor is not None
        init_p = runner.initial or runner.anchor
        j_idx = cfg.joint - 1
        q_init = init_p[j_idx]
        q_ret = runner.anchor[j_idx]
        anchor_travel = (
            0.0
            if cfg.joint == 7
            else max(
                (abs(runner.anchor[i] - init_p[i]) for i in range(6) if i != j_idx),
                default=0.0,
            )
        )
        runner.schedule = build_sweep_schedule(
            q_init, cfg, anchor_travel=anchor_travel, q_return=q_ret,
        )
        runner.segment_index = 0
        runner.sweep_started = time.monotonic()
        runner.segment_started = runner.sweep_started
        runner.segment_velocities = []
        runner.phase = 'running'

    def run_fit(
        self,
        samples: Sequence[SweepSample],
        *,
        arm_id: str = 'arm_1',
        max_harmonics: int = 2,
        candidate_omegas: Sequence[float] | None = None,
        grid_step: float = 0.001,
        trim_ratio: float = 0.1,
        output_yaml: str | Path | None = None,
    ) -> dict[str, Any]:
        inertia_scale = 1.0
        if samples:
            j_idx = samples[0].joint
            g_mod = self.fleet.gravity_model(arm_id)
            if g_mod is not None and 1 <= j_idx <= 6:
                inertia_scale = float(g_mod.joint(j_idx).scale)
        report = identify_joint_friction(
            samples,
            grid_step=grid_step,
            trim_ratio=trim_ratio,
            max_harmonics=max_harmonics,
            candidate_omegas=candidate_omegas,
            inertia_scale=inertia_scale,
        )
        with self.lock:
            current_arm = self.arm_models.get(arm_id, ArmFrictionModel(arm_id=arm_id))
            if output_yaml and Path(output_yaml).is_file():
                try:
                    loaded = load_friction_model(output_yaml)
                    for j_idx, j_model in current_arm.joints.items():
                        loaded = loaded.with_joint(j_idx, j_model)
                    if current_arm.gripper is not None:
                        loaded = loaded.with_gripper(current_arm.gripper)
                    current_arm = loaded
                except Exception:
                    pass
            updated_arm = current_arm.with_joint(report.joint, report.model)
            self.arm_models[arm_id] = updated_arm
            if output_yaml:
                save_friction_model(updated_arm, output_yaml)
            payload = serialize_report(report, updated_arm)
            if output_yaml:
                payload['saved_yaml_path'] = str(output_yaml)
            self.joint_fits.setdefault(arm_id, {})[report.joint] = payload
            for j_payload in self.joint_fits[arm_id].values():
                j_payload['yaml_text'] = payload['yaml_text']
            self.latest_fit = payload
        if output_yaml:
            self.reload_saved_model(arm_id, output_yaml)
        return payload

    def status_payload(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            arms_data = []
            for arm in self.fleet.arms:
                ns = arm.namespace
                pos = self.positions.get(ns)
                vel = self.velocities.get(ns, (0.0,) * 6)
                tau = self.torques.get(ns, (0.0,) * 6)
                j_stamp = self.joint_stamps.get(ns)
                status = self.statuses.get(ns)
                ctrl = self.control_states.get(ns, {})
                cfg_dict = ctrl.get('configuration', {})
                online = j_stamp is not None and (now - j_stamp) <= 2.0
                g_torques = (
                    gravity_torques(pos[:6], **self.effector_kwargs.get(ns, {}))
                    if pos is not None else (0.0,) * 6
                )
                calibrated_joints = sorted(self.joint_fits.get(arm.id, {}).keys())
                grip_model = self.arm_models[arm.id].gripper
                arms_data.append({
                    'id': arm.id,
                    'name': arm.name,
                    'namespace': ns,
                    'can_port': arm.can_port,
                    'end_effector': arm.end_effector,
                    'gravity_scale': float(cfg_dict.get('gravity_scale', arm.gravity_scale)),
                    'friction_scale': float(cfg_dict.get('friction_scale', arm.friction_scale)),
                    'friction_scale_applied': float(
                        ctrl.get('friction_scale_applied', arm.friction_scale)
                    ),
                    'friction_config': arm.friction_config,
                    'online': online,
                    'mode': ctrl.get('mode', 'unknown'),
                    'enabled': list(status.motor_enabled) if status is not None else [False] * 6,
                    'gripper_enabled': bool(status.gripper_enabled) if status is not None else False,
                    'position': list(pos[:6]) if pos is not None else None,
                    'gripper_position': float(pos[6]) if pos is not None and len(pos) >= 7 else None,
                    'velocity': list(vel[:6]) if pos is not None else None,
                    'gripper_velocity': float(self.gripper_velocities.get(ns, 0.0)),
                    'torque': list(tau[:6]) if pos is not None else None,
                    'gripper_force': float(self.gripper_forces.get(ns, 0.0)),
                    'gripper_friction': float(ctrl.get('gripper_friction', 0.0)),
                    'gravity_torque': [float(x) for x in g_torques],
                    'sweep_phase': self.runners[ns].phase,
                    'calibrated_joints': calibrated_joints,
                    'gripper_calibrated': (
                        7 in calibrated_joints or grip_model is not None
                    ),
                    'gripper_model': (
                        grip_model.to_dict() if grip_model is not None else None
                    ),
                })

            sweeps_by_ns = {ns: runner.to_dict() for ns, runner in self.runners.items()}
            active_runner = self.runners.get(self.active_namespace, next(iter(self.runners.values())))
            return {
                'arms': arms_data,
                'sweep': active_runner.to_dict(),
                'sweeps_by_ns': sweeps_by_ns,
                'latest_fit': self.latest_fit,
                'joint_fits': {
                    arm_id: {str(j): fit for j, fit in fits.items()}
                    for arm_id, fits in self.joint_fits.items()
                },
            }


class _WorkbenchHandler(BaseHTTPRequestHandler):
    node: FrictionWorkbenchNode
    static_dir: Path

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, allow_nan=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, status: int, content: str, content_type: str) -> None:
        body = content.encode('utf-8')
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
            self._send_text(200, format_samples_csv_text(samples), 'text/csv; charset=utf-8')
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
            if path == '/api/sweep/start':
                ns = str(body.get('namespace', '/arm_1'))
                kp_raw = body.get('kp', list(DEFAULT_SWEEP_KP))
                kd_raw = body.get('kd')
                gripper_effort = float(body.get('gripper_effort', DEFAULT_GRIPPER_EFFORT))
                speeds_raw = body.get('speeds', list(DEFAULT_SWEEP_SPEEDS))
                repeats = int(body.get('repeats', 1))
                approach_speed = float(body.get('approach_speed', 0.4))
                settle_time = float(body.get('settle_time', 0.4))

                plan = body.get('joints_plan')
                configs: list[SweepConfig] = []
                if isinstance(plan, list) and plan:
                    for item in plan:
                        configs.append(SweepConfig(
                            namespace=ns,
                            joint=int(item['joint']),
                            q_min=float(item['q_min']),
                            q_max=float(item['q_max']),
                            speeds=tuple(float(x) for x in speeds_raw),
                            repeats=repeats,
                            kp=tuple(float(x) for x in kp_raw),
                            kd=None if not kd_raw else tuple(float(x) for x in kd_raw),
                            gripper_effort=gripper_effort,
                            approach_speed=approach_speed,
                            settle_time=settle_time,
                        ))
                else:
                    configs.append(SweepConfig(
                        namespace=ns,
                        joint=int(body.get('joint', 1)),
                        q_min=float(body['q_min']),
                        q_max=float(body['q_max']),
                        speeds=tuple(float(x) for x in speeds_raw),
                        repeats=repeats,
                        kp=tuple(float(x) for x in kp_raw),
                        kd=None if not kd_raw else tuple(float(x) for x in kd_raw),
                        gripper_effort=gripper_effort,
                        approach_speed=approach_speed,
                        settle_time=settle_time,
                    ))
                status = self.node.start_sweep(
                    configs,
                    output_csv=str(body.get('output_csv', '')),
                    fit_output=str(body.get('fit_output', '')),
                )
                self._send_json(200, {'ok': True, 'status': status})
                return

            if path == '/api/sweep/stop':
                ns = str(body.get('namespace', '')).strip() or None
                status = self.node.stop_sweep(namespace=ns)
                self._send_json(200, {'ok': True, 'status': status})
                return

            if path == '/api/fit':
                source = str(body.get('source', 'memory'))
                arm_id = str(body.get('arm_id', 'arm_1'))
                joint = int(body.get('joint', 1))
                max_harmonics = int(body.get('max_harmonics', 2))
                omegas_raw = body.get('omegas')
                omegas = [float(x) for x in omegas_raw] if omegas_raw else None
                grid_step = float(body.get('grid_step', 0.001))
                trim_ratio = float(body.get('trim_ratio', 0.1))
                output_yaml = body.get('output_yaml') or None

                if source == 'demo_all':
                    last_res = None
                    for j_idx in range(1, 7):
                        demo_samples = synthesize_demo_samples(joint=j_idx)
                        with self.node.lock:
                            self.node.joint_samples.setdefault(arm_id, {})[j_idx] = demo_samples
                            if j_idx == joint:
                                self.node.samples = list(demo_samples)
                        res = self.node.run_fit(
                            demo_samples,
                            arm_id=arm_id,
                            max_harmonics=max_harmonics,
                            candidate_omegas=omegas,
                            grid_step=grid_step,
                            trim_ratio=trim_ratio,
                            output_yaml=output_yaml if j_idx == 6 else None,
                        )
                        if j_idx == joint or last_res is None:
                            last_res = res
                    with self.node.lock:
                        joint_fits = {
                            str(k): v for k, v in self.node.joint_fits.get(arm_id, {}).items()
                        }
                    self._send_json(200, {'ok': True, 'fit': last_res, 'joint_fits': joint_fits})
                    return

                if source == 'demo':
                    samples = synthesize_demo_samples(
                        joint=joint,
                        q_min=float(body.get('q_min', DEFAULT_JOINT_RANGES[joint][0])),
                        q_max=float(body.get('q_max', DEFAULT_JOINT_RANGES[joint][1])),
                    )
                    with self.node.lock:
                        self.node.samples = list(samples)
                        self.node.joint_samples.setdefault(arm_id, {})[joint] = list(samples)
                elif source == 'csv_text':
                    samples = parse_samples_csv_text(str(body.get('csv_text', '')))
                    with self.node.lock:
                        self.node.samples = list(samples)
                elif source == 'csv_file':
                    samples = load_samples_csv(str(body['csv_path']))
                    with self.node.lock:
                        self.node.samples = list(samples)
                else:
                    with self.node.lock:
                        samples = list(
                            self.node.joint_samples.get(arm_id, {}).get(joint)
                            or self.node.samples
                        )
                if not samples:
                    raise ValueError('No sweep samples available; run a sweep, upload CSV, or load demo data')

                result = self.node.run_fit(
                    samples,
                    arm_id=arm_id,
                    max_harmonics=max_harmonics,
                    candidate_omegas=omegas,
                    grid_step=grid_step,
                    trim_ratio=trim_ratio,
                    output_yaml=output_yaml,
                )
                with self.node.lock:
                    joint_fits = {
                        str(k): v for k, v in self.node.joint_fits.get(arm_id, {}).items()
                    }
                self._send_json(200, {'ok': True, 'fit': result, 'joint_fits': joint_fits})
                return

            if path == '/api/arm/friction_scale':
                ns = str(body.get('namespace', '/arm_1'))
                scale = float(body.get('friction_scale', 0.0))
                sent = self.node.request_friction_scale(ns, scale)
                self._send_json(200, {'ok': True, 'sent': sent, 'friction_scale': scale})
                return

            if path == '/api/save_yaml':
                yaml_path = str(body.get('path', '')).strip()
                if not yaml_path:
                    raise ValueError('YAML output path is required')
                arm_id = str(body.get('arm_id', 'arm_1'))
                joints_payload = body.get('joints')
                gripper_payload = body.get('gripper')
                with self.node.lock:
                    model = self.node.arm_models.get(arm_id, ArmFrictionModel(arm_id=arm_id))
                    if Path(yaml_path).is_file():
                        try:
                            existing = load_friction_model(yaml_path)
                            for j_i, j_m in model.joints.items():
                                existing = existing.with_joint(j_i, j_m)
                            if model.gripper is not None:
                                existing = existing.with_gripper(model.gripper)
                            model = existing
                        except Exception:
                            pass
                    if isinstance(joints_payload, dict) and joints_payload:
                        for j_key, j_val in joints_payload.items():
                            j_idx = int(str(j_key).replace('joint', ''))
                            j_dict = j_val.get('model', j_val) if isinstance(j_val, dict) else {}
                            if j_dict:
                                if j_idx == 7:
                                    model = model.with_gripper(GripperFrictionModel.from_dict(j_dict))
                                else:
                                    model = model.with_joint(j_idx, JointFrictionModel.from_dict(j_dict))
                    if isinstance(gripper_payload, dict) and gripper_payload:
                        g_dict = gripper_payload.get('model', gripper_payload)
                        if isinstance(g_dict, dict) and g_dict:
                            model = model.with_gripper(GripperFrictionModel.from_dict(g_dict))
                    self.node.arm_models[arm_id] = model
                    if model is None or (not model.joints and model.gripper is None):
                        raise ValueError('No fitted joint or gripper model available to save')
                saved = save_friction_model(model, yaml_path).resolve()
                self.node.reload_saved_model(arm_id, saved)
                self._send_json(200, {'ok': True, 'path': str(saved)})
                return

            if path == '/api/evaluate':
                theta = float(body.get('theta', 0.0))
                velocity = float(body.get('velocity', 0.0))
                cog_scale = float(body.get('cogging_scale', 1.0))
                fric_scale = float(body.get('friction_scale', 1.0))
                model_dict = body.get('model')
                if isinstance(model_dict, dict):
                    jm: JointFrictionModel | GripperFrictionModel = (
                        GripperFrictionModel.from_dict(model_dict)
                        if 'w_grid' in model_dict
                        else JointFrictionModel.from_dict(model_dict)
                    )
                else:
                    with self.node.lock:
                        if self.node.latest_fit is None:
                            raise ValueError('Fit a model first before evaluating')
                        raw_m = self.node.latest_fit['model']
                        jm = (
                            GripperFrictionModel.from_dict(raw_m)
                            if 'w_grid' in raw_m
                            else JointFrictionModel.from_dict(raw_m)
                        )
                env = jm.envelope(velocity)
                if isinstance(jm, GripperFrictionModel):
                    if velocity != 0.0:
                        s_v = abs(velocity)
                        dry = jm.fc + (jm.fs - jm.fc) * math.exp(-((s_v / jm.vs) ** jm.delta))
                        dir_v = math.tanh(velocity / jm.v0)
                        cog = math.tanh(s_v / jm.v0) * jm.bias_force(theta)
                        mod = env * jm.modulation_force(theta) * dir_v
                        stribeck = dry * dir_v + jm.fv * velocity
                    else:
                        cog = mod = stribeck = 0.0
                    total = jm.force(
                        theta, velocity, bias_scale=cog_scale, friction_scale=fric_scale,
                    )
                else:
                    cog = env * jm.cogging_torque(theta)
                    mod = env * jm.modulation_torque(theta) * math.tanh(velocity / jm.v0)
                    stribeck = jm.stribeck_torque(velocity)
                    total = jm.torque(
                        theta, velocity, cogging_scale=cog_scale, friction_scale=fric_scale,
                    )
                self._send_json(200, {
                    'ok': True,
                    'envelope': env,
                    'cogging_nm': cog * cog_scale,
                    'modulation_nm': mod * fric_scale,
                    'stribeck_nm': stribeck * fric_scale,
                    'total_nm': total,
                })
                return

            self._send_json(404, {'error': f'Unknown endpoint: {path}'})
        except Exception as error:
            self._send_json(400, {'ok': False, 'error': str(error)})


def create_http_server(
    node: FrictionWorkbenchNode,
    host: str = '127.0.0.1',
    port: int = 8090,
    static_dir: Path | None = None,
) -> ThreadingHTTPServer:
    root = static_dir or web_root_path()
    handler = type(
        'BoundFrictionHandler',
        (_WorkbenchHandler,),
        {'node': node, 'static_dir': root},
    )
    return ThreadingHTTPServer((host, port), handler)


def web_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Start the PiPER-X Joint Friction Calibration Web Workbench.'
    )
    parser.add_argument('--host', default='127.0.0.1', help='Bind host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=8090, help='Bind port (default: 8090)')
    parser.add_argument(
        '--config', default=str(default_config_path()),
        help='Path to shared arms.yaml configuration',
    )
    args = parser.parse_args(argv)

    rclpy.init()
    node = FrictionWorkbenchNode(config_file=args.config)
    server = create_http_server(node, host=args.host, port=args.port)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()
    host_display = '127.0.0.1' if args.host == '0.0.0.0' else args.host
    node.get_logger().info(
        f'PiPER-X Friction Calibration Workbench listening on http://{host_display}:{ server.server_port }/'
    )
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
