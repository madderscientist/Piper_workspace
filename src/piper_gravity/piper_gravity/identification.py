"""Bidirectional quasi-static decomposition and regularized gravity + static friction fitting."""

import csv
from dataclasses import dataclass, replace
import math
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np
from piper_x_driver.gravity import ArmGravityModel, JointGravityModel
from scipy.optimize import least_squares


@dataclass(frozen=True)
class ProbeSample:
    """One recorded sample from a bidirectional quasi-static probe or slow sweep."""

    t: float
    joint: int
    direction: int
    q_target: float
    q: float
    dq: float
    tau_meas: float
    tau_kp: float
    tau_gravity: float
    q_full: tuple[float, ...] = ()

    def selected_torque(self, source: str = 'auto') -> float:
        if source == 'kp_error':
            return self.tau_kp if abs(self.tau_kp) > 1e-9 else self.tau_meas
        if source == 'motor_torque':
            return self.tau_meas
        return self.tau_meas if abs(self.tau_kp) <= 1e-9 else self.tau_kp


@dataclass(frozen=True)
class GravityDecompositionResult:
    """Angle-aligned bidirectional half-sum (gravity) and half-difference (static friction)."""

    theta_grid: np.ndarray
    tau_pos: np.ndarray
    tau_neg: np.ndarray
    gravity_measured: np.ndarray
    gravity_cad: np.ndarray
    gravity_fitted: np.ndarray
    stiction_profile: np.ndarray


@dataclass(frozen=True)
class GravityIdentificationReport:
    """Identified joint gravity and static friction model with fit diagnostics."""

    joint: int
    model: JointGravityModel
    decomposition: GravityDecompositionResult
    rmse_cad: float
    rmse_calibrated: float
    static_friction_mean: float
    static_friction_p85: float
    added_mass_kg: float | None = None
    total_effector_mass_kg: float | None = None
    mass_identified: bool = False


CSV_COLUMNS = (
    't', 'joint', 'direction', 'q_target', 'q', 'dq',
    'tau_meas', 'tau_kp', 'tau_gravity',
    'q1', 'q2', 'q3', 'q4', 'q5', 'q6',
)


def save_probe_samples_csv(samples: Iterable[ProbeSample], path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
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
    return target


def load_probe_samples_csv(path: str | Path) -> list[ProbeSample]:
    source = Path(path)
    with source.open(encoding='utf-8', newline='') as stream:
        return parse_probe_samples_csv_reader(csv.DictReader(stream))


def parse_probe_samples_csv_reader(reader: csv.DictReader) -> list[ProbeSample]:
    rows: list[ProbeSample] = []
    for row in reader:
        if 'direction' in row and row['direction'] not in ('', None):
            direction = int(float(row['direction']))
        else:
            dq_des = float(row.get('dq_des', 0.0) or 0.0)
            if abs(dq_des) <= 1e-6:
                continue
            direction = 1 if dq_des > 0 else -1
        if direction == 0:
            continue
        q = float(row['q'])
        q_target = float(row.get('q_target') or row.get('q_des') or q)
        tau_meas = float(row['tau_meas'])
        tau_kp = float(row.get('tau_kp', 0.0) or 0.0)
        tau_gravity = float(row.get('tau_gravity', 0.0) or 0.0)
        q_full: tuple[float, ...] = ()
        if all(f'q{i}' in row and row[f'q{i}'] not in ('', None) for i in range(1, 7)):
            q_full = tuple(float(row[f'q{i}']) for i in range(1, 7))
        rows.append(ProbeSample(
            t=float(row['t']),
            joint=int(row['joint']),
            direction=1 if direction > 0 else -1,
            q_target=q_target,
            q=q,
            dq=float(row.get('dq', 0.0) or 0.0),
            tau_meas=tau_meas,
            tau_kp=tau_kp,
            tau_gravity=tau_gravity,
            q_full=q_full,
        ))
    return rows


def _interp_sorted(qs: Sequence[float], vs: Sequence[float], grid: np.ndarray) -> np.ndarray:
    pairs = sorted(zip(qs, vs), key=lambda p: p[0])
    q_arr = np.array([p[0] for p in pairs], dtype=float)
    v_arr = np.array([p[1] for p in pairs], dtype=float)
    uq, inv = np.unique(q_arr, return_inverse=True)
    if len(uq) != len(q_arr):
        acc = np.zeros_like(uq)
        cnt = np.zeros_like(uq)
        np.add.at(acc, inv, v_arr)
        np.add.at(cnt, inv, 1.0)
        v_arr = acc / cnt
        q_arr = uq
    return np.interp(grid, q_arr, v_arr)


def _extract_quasi_static_subsets(
    samples: Sequence[ProbeSample],
    trim_ratio: float = 0.08,
) -> tuple[list[ProbeSample], list[ProbeSample]]:
    """Extract positive-approach and negative-approach quasi-static samples."""
    if not samples:
        raise ValueError('No gravity probe samples provided')
    pos_all = [s for s in samples if s.direction > 0]
    neg_all = [s for s in samples if s.direction < 0]
    if not pos_all or not neg_all:
        raise ValueError('Both positive (+1) and negative (-1) approach samples are required')

    # If data comes from multi-speed sweeps, prefer the slowest quasi-static speed pass
    pos_settled = [s for s in pos_all if abs(s.dq) <= 0.15]
    neg_settled = [s for s in neg_all if abs(s.dq) <= 0.15]
    pos_use = pos_settled if len(pos_settled) >= 6 else pos_all
    neg_use = neg_settled if len(neg_settled) >= 6 else neg_all

    is_staircase_probe = all(abs(s.dq) <= 0.03 for s in samples)

    def trim_ends(seq: list[ProbeSample]) -> list[ProbeSample]:
        if is_staircase_probe or trim_ratio <= 0.0:
            return seq
        qs = [s.q for s in seq]
        lo, hi = min(qs), max(qs)
        span = hi - lo
        if span <= 1e-4 or len(seq) <= 16:
            return seq
        return [s for s in seq if lo + trim_ratio * span <= s.q <= hi - trim_ratio * span]

    return trim_ends(pos_use), trim_ends(neg_use)


def decompose_bidirectional_gravity(
    samples: Sequence[ProbeSample],
    *,
    torque_source: str = 'auto',
    trim_ratio: float = 0.08,
    grid_step: float = 0.005,
) -> GravityDecompositionResult:
    """Interpolate bidirectional approach torques onto a common angle grid and decouple gravity & Fs."""
    if grid_step <= 0 or not math.isfinite(grid_step):
        raise ValueError('grid_step must be finite and positive')
    pos_samples, neg_samples = _extract_quasi_static_subsets(samples, trim_ratio=trim_ratio)

    lo = max(min(s.q for s in pos_samples), min(s.q for s in neg_samples))
    hi = min(max(s.q for s in pos_samples), max(s.q for s in neg_samples))
    if hi - lo <= 2.0 * grid_step:
        raise ValueError(f'Insufficient bidirectional angular overlap [{lo:.4f}, {hi:.4f}] rad')

    theta_grid = np.arange(lo, hi + 0.5 * grid_step, grid_step, dtype=float)
    tau_pos = _interp_sorted(
        [s.q for s in pos_samples],
        [s.selected_torque(torque_source) for s in pos_samples],
        theta_grid,
    )
    tau_neg = _interp_sorted(
        [s.q for s in neg_samples],
        [s.selected_torque(torque_source) for s in neg_samples],
        theta_grid,
    )
    cad_pos = _interp_sorted(
        [s.q for s in pos_samples],
        [s.tau_gravity for s in pos_samples],
        theta_grid,
    )
    cad_neg = _interp_sorted(
        [s.q for s in neg_samples],
        [s.tau_gravity for s in neg_samples],
        theta_grid,
    )
    gravity_cad = 0.5 * (cad_pos + cad_neg)
    gravity_measured = 0.5 * (tau_pos + tau_neg)
    stiction_profile = np.maximum(0.0, 0.5 * np.abs(tau_pos - tau_neg))

    return GravityDecompositionResult(
        theta_grid=theta_grid,
        tau_pos=tau_pos,
        tau_neg=tau_neg,
        gravity_measured=gravity_measured,
        gravity_cad=gravity_cad,
        gravity_fitted=gravity_cad.copy(),
        stiction_profile=stiction_profile,
    )


def identify_joint_gravity(
    samples: Sequence[ProbeSample],
    *,
    torque_source: str = 'auto',
    trim_ratio: float = 0.08,
    grid_step: float = 0.005,
) -> GravityIdentificationReport:
    """Identify per-joint gravity calibration (scale, angle_offset, bias) and static friction Fs."""
    if not samples:
        raise ValueError('Cannot identify gravity from empty sample sequence')
    joints = {s.joint for s in samples}
    if len(joints) != 1:
        raise ValueError(f'Samples must belong to a single joint, got {sorted(joints)}')
    joint_idx = next(iter(joints))

    decomp = decompose_bidirectional_gravity(
        samples,
        torque_source=torque_source,
        trim_ratio=trim_ratio,
        grid_step=grid_step,
    )
    theta_grid = decomp.theta_grid
    g_meas = decomp.gravity_measured
    g_cad = decomp.gravity_cad
    stiction = decomp.stiction_profile

    fs_mean = float(np.mean(stiction))
    fs_p85 = float(np.percentile(stiction, 85))
    rmse_cad = float(np.sqrt(np.mean((g_cad - g_meas) ** 2)))

    def eval_cad_shifted(angle_offset: float) -> np.ndarray:
        d_cad = np.gradient(g_cad, theta_grid) if len(theta_grid) >= 3 else np.zeros_like(g_cad)
        return g_cad + d_cad * angle_offset

    cad_span = float(np.max(g_cad) - np.min(g_cad))
    if joint_idx in (1, 6) or cad_span < 0.15:
        # Upright J1 and concentric J6 have zero gravity compensation (scale=0 for J6, 1 for J1, bias=0)
        scale = 0.0 if joint_idx == 6 else 1.0
        angle_offset = 0.0
        bias = 0.0
        g_fit = scale * g_cad + bias
    else:
        p0 = np.array([1.0, 0.0], dtype=float)
        lower = np.array([0.75, -0.025], dtype=float)
        upper = np.array([1.25, +0.025], dtype=float)

        def residuals(params: np.ndarray) -> np.ndarray:
            s_val, a_val = map(float, params)
            pred = s_val * eval_cad_shifted(a_val)
            data_res = (pred - g_meas) / max(1.0, math.sqrt(len(theta_grid) / 25.0))
            reg = np.array([
                (s_val - 1.0) * 0.05,
                a_val * 3.0,
            ])
            return np.concatenate([data_res, reg])

        opt = least_squares(
            residuals, p0, bounds=(lower, upper), loss='soft_l1', f_scale=0.05,
        )
        scale, angle_offset = map(float, opt.x)
        bias = 0.0
        g_fit = scale * eval_cad_shifted(angle_offset)

    rmse_cal = float(np.sqrt(np.mean((g_fit - g_meas) ** 2)))
    if rmse_cal > rmse_cad:
        scale, angle_offset, bias = 1.0, 0.0, 0.0
        g_fit = g_cad.copy()
        rmse_cal = rmse_cad

    model = JointGravityModel(
        scale=round(scale, 6),
        angle_offset=round(angle_offset, 6),
        bias=round(bias, 6),
        static_friction=round(fs_mean, 6),
    )
    updated_decomp = GravityDecompositionResult(
        theta_grid=decomp.theta_grid,
        tau_pos=decomp.tau_pos,
        tau_neg=decomp.tau_neg,
        gravity_measured=decomp.gravity_measured,
        gravity_cad=decomp.gravity_cad,
        gravity_fitted=g_fit,
        stiction_profile=decomp.stiction_profile,
    )
    return GravityIdentificationReport(
        joint=joint_idx,
        model=model,
        decomposition=updated_decomp,
        rmse_cad=rmse_cad,
        rmse_calibrated=rmse_cal,
        static_friction_mean=fs_mean,
        static_friction_p85=fs_p85,
    )


def identify_gripper_payload(
    samples_by_joint: Mapping[int, Sequence[ProbeSample]],
    arm_model: ArmGravityModel,
    *,
    torque_source: str = 'auto',
    trim_ratio: float = 0.08,
    grid_step: float = 0.005,
) -> tuple[ArmGravityModel, dict[int, GravityIdentificationReport]]:
    """Fit one added gripper mass across probes, keeping calibrated arm/known parts fixed.

    Extra mass shares the gripper-base mesh centroid and unit-mass inertia.
    Quasi-static data identifies mass; the mesh supplies its inertia shape.
    """
    if not arm_model.effector_model:
        raise ValueError('A composite effector model is required for gripper mass identification')
    nominal = arm_model.with_added_mass(0.0)
    unit_mass = nominal.with_added_mass(1.0)
    decompositions = {}
    regressors = {}
    x_rows, y_rows = [], []
    for joint, samples in sorted(samples_by_joint.items()):
        if not samples:
            continue
        if any(s.joint != joint or len(s.q_full) < 6 for s in samples):
            raise ValueError('Added-mass identification requires q1..q6 for each joint sample')
        decomp = decompose_bidirectional_gravity(
            samples, torque_source=torque_source, trim_ratio=trim_ratio, grid_step=grid_step)
        pos, neg = _extract_quasi_static_subsets(samples, trim_ratio)
        aligned = []
        for direction_samples in (pos, neg):
            values, coefficients = [], []
            for sample in direction_samples:
                q = list(sample.q_full[:6])
                q[joint - 1] = sample.q
                base = nominal.torques(q)[joint - 1]
                values.append(base)
                coefficients.append(unit_mass.torques(q)[joint - 1] - base)
            angles = [s.q for s in direction_samples]
            aligned.append((
                _interp_sorted(angles, values, decomp.theta_grid),
                _interp_sorted(angles, coefficients, decomp.theta_grid)))
        base = 0.5 * (aligned[0][0] + aligned[1][0])
        coefficient = 0.5 * (aligned[0][1] + aligned[1][1])
        decompositions[joint] = decomp
        regressors[joint] = (base, coefficient)
        x_rows.extend(coefficient)
        y_rows.extend(decomp.gravity_measured - base)
    if not decompositions:
        raise ValueError('No gravity probe samples provided')
    x, y = np.asarray(x_rows), np.asarray(y_rows)
    observable = bool(float(x @ x) > np.finfo(float).eps * len(x))
    mass = arm_model.added_mass_kg
    if observable:
        initial = max(0.0, float(x @ y) / float(x @ x))
        fitted = least_squares(
            lambda value: x * value[0] - y, [initial],
            bounds=(0.0, np.inf), loss='soft_l1', f_scale=0.05)
        mass = float(fitted.x[0])
    updated = arm_model.with_added_mass(mass)
    reports = {}
    for joint, decomp in decompositions.items():
        base, coefficient = regressors[joint]
        fitted = base + mass * coefficient
        fs_mean = float(np.mean(decomp.stiction_profile))
        joint_model = replace(updated.joint(joint), static_friction=round(fs_mean, 6))
        updated = updated.with_joint(joint, joint_model)
        reports[joint] = GravityIdentificationReport(
            joint=joint, model=joint_model,
            decomposition=replace(decomp, gravity_cad=base, gravity_fitted=fitted),
            rmse_cad=float(np.sqrt(np.mean((base - decomp.gravity_measured) ** 2))),
            rmse_calibrated=float(np.sqrt(np.mean((fitted - decomp.gravity_measured) ** 2))),
            static_friction_mean=fs_mean,
            static_friction_p85=float(np.percentile(decomp.stiction_profile, 85)),
            added_mass_kg=mass,
            total_effector_mass_kg=nominal.effector_body().mass_kg + mass,
            mass_identified=observable,
        )
    return updated, reports
