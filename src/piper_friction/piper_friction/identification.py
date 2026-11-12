"""Signal processing and parameter identification for Stribeck + periodic joint friction."""

import csv
from dataclasses import dataclass, replace
import math
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np
from piper_x_driver.gravity import joint_inertias
from scipy.optimize import least_squares, minimize_scalar
from scipy.signal import detrend

from .model import GripperFrictionModel, JointFrictionModel, SpatialHarmonic

_ANCHOR_INERTIAS = joint_inertias((0.0, 0.90, -1.50, 0.0, 0.0, 0.0))


@dataclass(frozen=True)
class SweepSample:
    """One recorded time sample during a constant-velocity joint sweep."""

    t: float
    joint: int
    q: float
    dq: float
    tau_meas: float
    q_des: float
    dq_des: float
    tau_gravity: float = 0.0
    weight: float = 1.0

    @property
    def tau_residual(self) -> float:
        return self.tau_meas - self.tau_gravity


@dataclass(frozen=True)
class DecompositionResult:
    """Grid-aligned half-sum and half-difference curves across sweep speeds."""

    theta_grid: np.ndarray
    speeds: np.ndarray
    half_sums: np.ndarray
    half_diffs: np.ndarray
    mean_frictions: np.ndarray
    ripple_scales: np.ndarray


@dataclass(frozen=True)
class IdentificationReport:
    """Identified joint or gripper friction model alongside fit diagnostics."""

    joint: int
    model: JointFrictionModel | GripperFrictionModel
    decomposition: DecompositionResult
    rmse_total: float
    rmse_stribeck: float
    rmse_cogging: float
    rmse_modulation: float


CSV_COLUMNS = (
    't', 'joint', 'q', 'dq', 'tau_meas', 'q_des', 'dq_des', 'tau_gravity',
)
DEFAULT_JOINT_V0 = (0.005, 0.005, 0.005, 0.015, 0.020, 0.030)


def save_samples_csv(samples: Iterable[SweepSample], path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.writer(stream)
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
    return target


def load_samples_csv(path: str | Path) -> list[SweepSample]:
    source = Path(path)
    rows: list[SweepSample] = []
    with source.open(encoding='utf-8', newline='') as stream:
        reader = csv.DictReader(stream)
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


def _filter_velocity_stabilized(
    seg: Sequence[SweepSample],
    trim_ratio: float = 0.1,
    vel_tol: float = 0.08,
    max_vel_tol: float = 0.25,
    inertia_scale: float = 1.0,
) -> list[SweepSample]:
    """Keep steady (w=1.0) and low-acceleration transition samples (0<w<1), rejecting large acceleration."""
    if len(seg) < 4:
        return []
    sign = 1.0 if seg[0].dq_des > 0 else -1.0
    speed = abs(seg[0].dq_des)
    qs = np.array([s.q for s in seg], dtype=float)
    q_min, q_max = float(np.min(qs)), float(np.max(qs))
    span = q_max - q_min
    if span <= 1e-4:
        return []

    low = q_min + trim_ratio * span
    high = q_max - trim_ratio * span
    pos_mask = (qs >= low) & (qs <= high)

    if speed <= 0.15:
        return [replace(s, weight=1.0) for s, keep in zip(seg, pos_mask) if keep]

    v_dir = np.array([s.dq for s in seg], dtype=float) * sign
    if np.all(np.abs(v_dir) <= 1e-6):
        return [replace(s, weight=1.0) for s, keep in zip(seg, pos_mask) if keep]

    ts = np.array([s.t for s in seg], dtype=float)
    if len(ts) >= 3:
        med_dt = float(np.median(np.diff(ts)))
        target_w = max(5, int(round(0.08 / med_dt)) | 1) if med_dt > 1e-5 else 5
    else:
        target_w = 5
    w = min(target_w, len(v_dir))
    pad_l = w // 2
    pad_r = w - 1 - pad_l
    v_padded = np.pad(v_dir, (pad_l, pad_r), mode='edge')
    v_smooth = np.convolve(v_padded, np.ones(w) / w, mode='valid')
    if len(ts) >= 3 and float(np.min(np.diff(ts))) > 1e-6:
        ddq = np.gradient(v_smooth, ts)
    else:
        ddq = np.zeros_like(v_smooth)

    # Reject initial breakaway ramp and any initial spring-windup velocity overshoot peak
    crossed_quasi = np.where(v_smooth >= (1.0 - max_vel_tol) * speed)[0]
    if len(crossed_quasi) == 0:
        return []
    quasi_start_idx = int(crossed_quasi[0])

    crossed_steady = np.where(v_smooth >= (1.0 - vel_tol) * speed)[0]
    steady_start_idx = int(crossed_steady[0]) if len(crossed_steady) > 0 else len(seg)

    overshot = np.where(v_smooth > (1.0 + vel_tol) * speed)[0]
    if len(overshot) > 0:
        peak_idx = int(overshot[int(np.argmax(v_smooth[overshot]))])
        settled_quasi = np.where(
            (np.arange(len(seg)) > peak_idx)
            & (v_smooth <= (1.0 + max_vel_tol) * speed)
        )[0]
        if len(settled_quasi) == 0:
            return []
        quasi_start_idx = int(settled_quasi[0])
        settled_steady = np.where(
            (np.arange(len(seg)) > peak_idx)
            & (v_smooth <= (1.0 + vel_tol) * speed)
        )[0]
        steady_start_idx = int(settled_steady[0]) if len(settled_steady) > 0 else len(seg)

    alpha_steady = max(0.4, 0.50 * speed)
    alpha_max = max(1.2, 1.50 * speed)

    j_idx = seg[0].joint - 1
    j_eff = inertia_scale * _ANCHOR_INERTIAS[j_idx] if 0 <= j_idx < 6 else 0.0
    out: list[SweepSample] = []
    for idx, (s, keep_pos, v_val, acc_dir) in enumerate(
        zip(seg, pos_mask, v_smooth, ddq)
    ):
        if not keep_pos or idx < quasi_start_idx:
            continue
        abs_acc = abs(float(acc_dir))
        vel_err = abs(float(v_val) - speed)
        # Hard cutoff: never accept large acceleration or large velocity error
        if abs_acc >= alpha_max or vel_err >= max_vel_tol * speed:
            continue
        if idx >= steady_start_idx and abs_acc <= alpha_steady and vel_err <= vel_tol * speed:
            conf = 1.0
        else:
            u_acc = max(0.0, abs_acc - alpha_steady) / max(1e-6, alpha_max - alpha_steady)
            u_vel = max(0.0, vel_err - vel_tol * speed) / max(1e-6, (max_vel_tol - vel_tol) * speed)
            u = math.hypot(u_acc, u_vel)
            if u >= 1.0:
                continue
            conf = 0.5 * (1.0 + math.cos(math.pi * u))
        tau_inertial = sign * j_eff * float(acc_dir)
        tau_corr = float(s.tau_meas - tau_inertial)
        out.append(replace(s, tau_meas=tau_corr, weight=float(conf * s.weight)))
    return out


def extract_steady_sweeps(
    samples: Sequence[SweepSample],
    trim_ratio: float = 0.1,
    inertia_scale: float = 1.0,
) -> dict[float, dict[str, list[list[SweepSample]]]]:
    """Group contiguous constant-rate sweeps by speed and retain velocity-stabilized segments."""
    if not 0.0 <= trim_ratio < 0.45:
        raise ValueError('trim_ratio must be in [0.0, 0.45)')
    if not samples:
        raise ValueError('No sweep samples provided')

    raw_segments: list[list[SweepSample]] = []
    current: list[SweepSample] = []
    current_key: tuple[int, float] | None = None

    for sample in samples:
        if abs(sample.dq_des) <= 1e-6:
            if len(current) >= 5:
                raw_segments.append(current)
            current = []
            current_key = None
            continue
        direction = 1 if sample.dq_des > 0 else -1
        speed_key = round(abs(sample.dq_des), 4)
        key = (direction, speed_key)
        if key != current_key:
            if len(current) >= 5:
                raw_segments.append(current)
            current = [sample]
            current_key = key
        else:
            current.append(sample)
    if len(current) >= 5:
        raw_segments.append(current)

    grouped: dict[float, dict[str, list[list[SweepSample]]]] = {}
    for seg in raw_segments:
        trimmed = _filter_velocity_stabilized(
            seg, trim_ratio=trim_ratio, inertia_scale=inertia_scale,
        )
        if len(trimmed) < 4:
            continue
        speed_key = round(abs(seg[0].dq_des), 4)
        direction_name = 'pos' if seg[0].dq_des > 0 else 'neg'
        bucket = grouped.setdefault(speed_key, {'pos': [], 'neg': []})
        bucket[direction_name].append(trimmed)

    return grouped


def _interp_segment(segment: Sequence[SweepSample], theta_grid: np.ndarray) -> np.ndarray:
    taus, _ = _interp_segment_weighted(segment, theta_grid)
    return taus


def _interp_segment_weighted(
    segment: Sequence[SweepSample],
    theta_grid: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    pairs = sorted((s.q, s.tau_residual, max(1e-6, s.weight)) for s in segment)
    qs = np.array([p[0] for p in pairs], dtype=float)
    taus = np.array([p[1] for p in pairs], dtype=float)
    ws = np.array([p[2] for p in pairs], dtype=float)
    unique_q, inverse = np.unique(qs, return_inverse=True)
    if len(unique_q) != len(qs):
        w_accum = np.zeros_like(unique_q)
        tau_accum = np.zeros_like(unique_q)
        np.add.at(w_accum, inverse, ws)
        np.add.at(tau_accum, inverse, taus * ws)
        taus = tau_accum / np.maximum(1e-9, w_accum)
        cnts = np.zeros_like(unique_q)
        np.add.at(cnts, inverse, 1.0)
        ws = w_accum / np.maximum(1.0, cnts)
        qs = unique_q
    tau_grid = np.interp(theta_grid, qs, taus)
    w_grid = np.interp(theta_grid, qs, ws, left=0.0, right=0.0)
    return tau_grid, w_grid


def decompose_half_sum_diff(
    samples: Sequence[SweepSample],
    grid_step: float = 0.001,
    trim_ratio: float = 0.1,
    inertia_scale: float = 1.0,
    grouped: dict[float, dict[str, list[list[SweepSample]]]] | None = None,
) -> DecompositionResult:
    """Interpolate forward/reverse sweeps onto a shared angle grid and decouple half-sum/diff."""
    if grid_step <= 0 or not math.isfinite(grid_step):
        raise ValueError('grid_step must be finite and positive')
    if grouped is None:
        grouped = extract_steady_sweeps(samples, trim_ratio=trim_ratio, inertia_scale=inertia_scale)
    valid_speeds = sorted(
        speed for speed, dirs in grouped.items() if dirs['pos'] and dirs['neg']
    )
    if not valid_speeds:
        raise ValueError('At least one speed with both positive and negative sweeps is required')

    pos_bounds: dict[float, tuple[float, float]] = {}
    neg_bounds: dict[float, tuple[float, float]] = {}
    overlap_bounds: dict[float, tuple[float, float]] = {}
    for speed in valid_speeds:
        p_lo = max(min(s.q for s in seg) for seg in grouped[speed]['pos'])
        p_hi = min(max(s.q for s in seg) for seg in grouped[speed]['pos'])
        n_lo = max(min(s.q for s in seg) for seg in grouped[speed]['neg'])
        n_hi = min(max(s.q for s in seg) for seg in grouped[speed]['neg'])
        pos_bounds[speed] = (p_lo, p_hi)
        neg_bounds[speed] = (n_lo, n_hi)
        lo_s, hi_s = max(p_lo, n_lo), min(p_hi, n_hi)
        if hi_s - lo_s > 4.0 * grid_step:
            overlap_bounds[speed] = (lo_s, hi_s)

    if not overlap_bounds:
        raise ValueError('No sweep speed has sufficient forward/reverse angular overlap')

    base_speed = min(overlap_bounds)
    theta_min, theta_max = overlap_bounds[base_speed]
    theta_grid = np.arange(theta_min, theta_max + 0.5 * grid_step, grid_step, dtype=float)

    base_pos = np.mean([_interp_segment(seg, theta_grid) for seg in grouped[base_speed]['pos']], axis=0)
    base_neg = np.mean([_interp_segment(seg, theta_grid) for seg in grouped[base_speed]['neg']], axis=0)
    base_half_sum = 0.5 * (base_pos + base_neg)

    speeds_out: list[float] = []
    half_sums: list[np.ndarray] = []
    half_diffs: list[np.ndarray] = []
    mean_frictions: list[float] = []

    for speed in valid_speeds:
        pos_pairs = [_interp_segment_weighted(seg, theta_grid) for seg in grouped[speed]['pos']]
        neg_pairs = [_interp_segment_weighted(seg, theta_grid) for seg in grouped[speed]['neg']]
        tau_pos = np.mean([p[0] for p in pos_pairs], axis=0)
        tau_neg = np.mean([p[0] for p in neg_pairs], axis=0)
        w_pos = np.mean([p[1] for p in pos_pairs], axis=0)
        w_neg = np.mean([p[1] for p in neg_pairs], axis=0)
        w_pair = np.minimum(w_pos, w_neg)
        half_sum = 0.5 * (tau_pos + tau_neg)
        half_diff = 0.5 * (tau_pos - tau_neg)

        # Confidence-weighted friction average across samples (steady w=1.0 fully trusted,
        # accelerating samples weighted by w^4 so steady segments dominate whenever present).
        pos_samples = [
            (s.tau_residual - float(np.interp(s.q, theta_grid, base_half_sum)), s.weight)
            for seg in grouped[speed]['pos']
            for s in seg
            if theta_min <= s.q <= theta_max
        ]
        neg_samples = [
            (s.tau_residual - float(np.interp(s.q, theta_grid, base_half_sum)), s.weight)
            for seg in grouped[speed]['neg']
            for s in seg
            if theta_min <= s.q <= theta_max
        ]
        if len(pos_samples) < 4 or len(neg_samples) < 4:
            continue
        p_vals = np.array([x[0] for x in pos_samples], dtype=float)
        p_w1 = np.array([x[1] for x in pos_samples], dtype=float)
        n_vals = np.array([x[0] for x in neg_samples], dtype=float)
        n_w1 = np.array([x[1] for x in neg_samples], dtype=float)
        min_weight_sum = 4.0 if (theta_max - theta_min) < 0.15 else 12.0
        if speed != base_speed and (
            float(np.sum(p_w1)) < min_weight_sum or float(np.sum(n_w1)) < min_weight_sum
        ):
            continue
        p_ws = p_w1 ** 4
        n_ws = n_w1 ** 4
        f_pos = float(np.sum(p_vals * p_ws) / max(1e-12, float(np.sum(p_ws))))
        f_neg = float(np.sum(n_vals * n_ws) / max(1e-12, float(np.sum(n_ws))))
        f_mean = 0.5 * (f_pos - f_neg)

        if speed in overlap_bounds:
            lo_s, hi_s = overlap_bounds[speed]
            sub_lo, sub_hi = max(theta_min, lo_s), min(theta_max, hi_s)
        else:
            sub_lo, sub_hi = 1.0, 0.0

        steady_mask = (theta_grid >= sub_lo) & (theta_grid <= sub_hi) & (w_pair >= 0.5)
        if int(np.sum(steady_mask)) >= max(10, int(0.15 * len(theta_grid))):
            w_sub = (w_pair[steady_mask] ** 4)
            f_mean = float(np.sum(half_diff[steady_mask] * w_sub) / np.sum(w_sub))
            hs_mean = float(np.sum(half_sum[steady_mask] * w_sub) / np.sum(w_sub))
            half_sum = np.where(steady_mask, half_sum, hs_mean)
            half_diff = np.where(steady_mask, half_diff, f_mean)
        else:
            half_sum = np.zeros_like(theta_grid)
            half_diff = np.full_like(theta_grid, f_mean)

        all_seg_samples = [
            s
            for seg in grouped[speed]['pos'] + grouped[speed]['neg']
            for s in seg
        ]
        if any(abs(s.dq) > 1e-5 for s in all_seg_samples):
            v_meas = float(np.mean([abs(s.dq) for s in all_seg_samples]))
        else:
            v_meas = float(speed)
        speeds_out.append(v_meas)
        half_sums.append(half_sum)
        half_diffs.append(half_diff)
        mean_frictions.append(f_mean)

    order = np.argsort(speeds_out)
    speeds_arr = np.array([speeds_out[i] for i in order], dtype=float)
    half_sums_arr = np.array([half_sums[i] for i in order], dtype=float)
    half_diffs_arr = np.array([half_diffs[i] for i in order], dtype=float)
    mean_fric_arr = np.array([mean_frictions[i] for i in order], dtype=float)

    return DecompositionResult(
        theta_grid=theta_grid,
        speeds=speeds_arr,
        half_sums=half_sums_arr,
        half_diffs=half_diffs_arr,
        mean_frictions=mean_fric_arr,
        ripple_scales=np.ones_like(speeds_arr),
    )


def _harmonic_design_matrix(theta: np.ndarray, omegas: Sequence[float]) -> np.ndarray:
    cols = []
    for omega in omegas:
        phase = omega * theta
        cols.append(np.cos(phase))
        cols.append(np.sin(phase))
    return np.column_stack(cols) if cols else np.empty((len(theta), 0))


def fit_spatial_harmonics(
    theta_grid: np.ndarray,
    half_sum: np.ndarray,
    half_diff: np.ndarray,
    *,
    max_harmonics: int = 2,
    candidate_omegas: Sequence[float] | None = None,
    min_omega: float = 2.0,
    max_omega: float = 120.0,
    min_amplitude: float = 1e-3,
    snap_integer_tolerance: float = 0.15,
    coherence_sum: np.ndarray | None = None,
    coherence_diff: np.ndarray | None = None,
) -> tuple[SpatialHarmonic, ...]:
    """Identify spatial frequencies and least-squares Fourier coefficients (a, b, c, d)."""
    if max_harmonics <= 0:
        return ()
    res_sum = detrend(np.asarray(half_sum, dtype=float), type='linear')
    res_diff = np.asarray(half_diff, dtype=float) - float(np.mean(half_diff))
    coh_sum = (
        res_sum
        if coherence_sum is None
        else detrend(np.asarray(coherence_sum, dtype=float), type='linear')
    )
    coh_diff = (
        res_diff
        if coherence_diff is None
        else np.asarray(coherence_diff, dtype=float) - float(np.mean(coherence_diff))
    )
    span = float(theta_grid[-1] - theta_grid[0])
    d_theta = float(theta_grid[1] - theta_grid[0])
    min_sep = max(1.5 * (2.0 * math.pi / span), 1.0)

    selected_omegas: list[float] = []
    if candidate_omegas is not None:
        selected_omegas = [float(w) for w in candidate_omegas if w > 0][:max_harmonics]
    else:
        n_pts = len(theta_grid)
        n_fft = max(16384, 1 << (n_pts * 16 - 1).bit_length())
        window = np.hanning(n_pts)
        freqs_cycles = np.fft.rfftfreq(n_fft, d=d_theta)
        omegas_grid = 2.0 * math.pi * freqs_cycles
        valid_mask = (omegas_grid >= min_omega) & (omegas_grid <= max_omega)

        work_sum = coh_sum.copy()
        work_diff = coh_diff.copy()

        for _ in range(max_harmonics):
            if not np.any(valid_mask):
                break
            spec_sum = np.fft.rfft(work_sum * window, n=n_fft)
            spec_diff = np.fft.rfft(work_diff * window, n=n_fft)
            power = np.abs(spec_sum) ** 2 + np.abs(spec_diff) ** 2

            masked_power = np.where(valid_mask, power, -1.0)
            for prev_w in selected_omegas:
                masked_power = np.where(
                    np.abs(omegas_grid - prev_w) < min_sep, -1.0, masked_power
                )
            best_idx = int(np.argmax(masked_power))
            if masked_power[best_idx] <= 0:
                break
            w_init = float(omegas_grid[best_idx])
            search_half = max(2.0 * math.pi / span, 0.5)
            lo = max(min_omega, w_init - search_half)
            hi = min(max_omega, w_init + search_half)

            def neg_energy(w: float) -> float:
                basis = _harmonic_design_matrix(theta_grid, [w])
                c_sum, _, _, _ = np.linalg.lstsq(basis, work_sum, rcond=None)
                c_diff, _, _, _ = np.linalg.lstsq(basis, work_diff, rcond=None)
                fit_s = basis @ c_sum
                fit_d = basis @ c_diff
                return -float(np.dot(fit_s, fit_s) + np.dot(fit_d, fit_d))

            opt: Any = minimize_scalar(neg_energy, bounds=(lo, hi), method='bounded')
            w_best = float(opt.x)
            nearest_int = round(w_best)
            if nearest_int >= 1 and abs(w_best - nearest_int) <= snap_integer_tolerance:
                w_best = float(nearest_int)

            basis_one = _harmonic_design_matrix(theta_grid, [w_best])
            coeff_s, _, _, _ = np.linalg.lstsq(basis_one, work_sum, rcond=None)
            coeff_d, _, _, _ = np.linalg.lstsq(basis_one, work_diff, rcond=None)
            amp = math.hypot(
                math.hypot(float(coeff_s[0]), float(coeff_s[1])),
                math.hypot(float(coeff_d[0]), float(coeff_d[1])),
            )
            if amp < min_amplitude:
                break
            selected_omegas.append(w_best)
            work_sum = work_sum - basis_one @ coeff_s
            work_diff = work_diff - basis_one @ coeff_d

    if not selected_omegas:
        return ()

    basis = _harmonic_design_matrix(theta_grid, selected_omegas)
    coeffs_cog, _, _, _ = np.linalg.lstsq(basis, res_sum, rcond=None)
    coeffs_mod, _, _, _ = np.linalg.lstsq(basis, res_diff, rcond=None)

    harmonics: list[SpatialHarmonic] = []
    for idx, omega in enumerate(selected_omegas):
        a = float(coeffs_cog[2 * idx])
        b = float(coeffs_cog[2 * idx + 1])
        c = float(coeffs_mod[2 * idx])
        d = float(coeffs_mod[2 * idx + 1])
        harmonics.append(SpatialHarmonic(omega=omega, a=a, b=b, c=c, d=d))
    return tuple(harmonics)


def fit_stribeck(
    speeds: np.ndarray,
    mean_frictions: np.ndarray,
    delta: float = 2.0,
) -> tuple[float, float, float, float]:
    """Fit (fc, fs, vs, fv) from steady-state friction averages across sweep speeds."""
    v = np.asarray(speeds, dtype=float)
    f = np.maximum(0.0, np.asarray(mean_frictions, dtype=float))
    if len(v) == 0:
        raise ValueError('At least one speed point is required to fit Stribeck parameters')
    if len(v) == 1:
        val = float(f[0])
        return val, val, 0.1, 0.0
    if len(v) == 2:
        slope = max(0.0, float((f[1] - f[0]) / max(1e-6, v[1] - v[0])))
        fc = max(0.0, float(f[0] - slope * v[0]))
        return fc, max(fc, float(f[0])), 0.1, slope

    f_min = float(np.min(f))
    f_max = float(np.max(f))
    v_min = float(np.min(v))
    low_mask = v <= max(0.30, 2.5 * v_min)
    f_low_min = float(np.min(f[low_mask])) if np.any(low_mask) else f_min
    f_low_max = float(np.max(f[low_mask])) if np.any(low_mask) else f_max

    fc_lower = max(0.0, 0.65 * f_low_min)
    fc_upper = max(fc_lower + 1e-3, f_low_max * 1.05)
    df_upper = max(0.02, (f_low_max - fc_lower) * 1.35)
    vs_lower = max(0.04, 1.2 * v_min)
    vs_upper = 0.25
    fv_upper = 0.35 if float(np.max(v)) <= 0.35 else 1.50

    p0 = np.array([
        0.92 * f_low_min,
        max(0.005, f_low_max - 0.92 * f_low_min),
        0.10,
        max(0.0, min(fv_upper * 0.5, (f[-1] - f_low_min) / max(0.2, float(v[-1])))),
    ])
    lower = np.array([fc_lower, 0.0, vs_lower, 0.0])
    upper = np.array([fc_upper, df_upper, vs_upper, fv_upper])
    p0 = np.clip(p0, lower + 1e-6, upper - 1e-6)

    def residuals(params: np.ndarray) -> np.ndarray:
        fc, df, vs, fv = params
        pred = fc + df * np.exp(-((v / vs) ** delta)) + fv * v
        reg = np.array([fv * (0.08 if float(np.max(v)) <= 0.35 else 0.01)])
        return np.concatenate([pred - f, reg])

    result = least_squares(
        residuals, p0, bounds=(lower, upper),
        loss='soft_l1', f_scale=max(0.02, 0.05 * f_low_min),
    )
    fc, df, vs, fv = map(float, result.x)
    return fc, fc + df, vs, fv


def fit_ripple_envelope(
    speeds: np.ndarray,
    half_sums: np.ndarray,
    half_diffs: np.ndarray,
    theta_grid: np.ndarray,
    harmonics: Sequence[SpatialHarmonic],
) -> tuple[float, tuple[SpatialHarmonic, ...], np.ndarray]:
    """Fit the high-speed decay velocity vr and rescale harmonics to true zero-speed amplitude."""
    if not harmonics:
        return 0.5, (), np.zeros_like(speeds)

    g_cog = np.sum(
        [h.a * np.cos(h.omega * theta_grid) + h.b * np.sin(h.omega * theta_grid) for h in harmonics],
        axis=0,
    )
    g_mod = np.sum(
        [h.c * np.cos(h.omega * theta_grid) + h.d * np.sin(h.omega * theta_grid) for h in harmonics],
        axis=0,
    )
    denom = float(np.dot(g_cog, g_cog) + np.dot(g_mod, g_mod))
    if denom <= 1e-12:
        return 0.5, tuple(harmonics), np.ones_like(speeds)

    raw_scales = np.empty(len(speeds), dtype=float)
    for idx in range(len(speeds)):
        active = (
            (np.abs(np.gradient(half_sums[idx])) + np.abs(np.gradient(half_diffs[idx]))) > 1e-9
        )
        if len(active) >= 3:
            active = active & np.roll(active, 1) & np.roll(active, -1)
            active[0] = False
            active[-1] = False
        if int(np.sum(active)) < 10:
            raw_scales[idx] = 0.0
            continue
        s_detrend = detrend(half_sums[idx][active], type='linear')
        d_center = half_diffs[idx][active] - float(np.mean(half_diffs[idx][active]))
        g_c = g_cog[active]
        g_m = g_mod[active]
        sub_denom = float(np.dot(g_c, g_c) + np.dot(g_m, g_m))
        if sub_denom <= 1e-12:
            raw_scales[idx] = 0.0
            continue
        proj = float(np.dot(s_detrend, g_c) + np.dot(d_center, g_m)) / sub_denom
        raw_scales[idx] = max(0.0, proj)

    if len(speeds) < 2 or float(np.max(raw_scales)) <= 1e-6:
        return 0.5, tuple(harmonics), raw_scales

    v = np.asarray(speeds, dtype=float)
    min_vr = max(0.08, float(v[0]) * 1.5)
    max_vr = 0.35 if float(np.max(v)) <= 0.30 else 0.60
    p0 = np.array([1.0, min(max_vr - 0.02, max(min_vr + 0.02, float(np.median(v))))])
    lower = np.array([0.8, min_vr])
    upper = np.array([1.25, max(min_vr + 0.05, max_vr)])
    p0 = np.clip(p0, lower + 1e-6, upper - 1e-6)

    def residuals(params: np.ndarray) -> np.ndarray:
        s0, vr = params
        return s0 * np.exp(-((v / vr) ** 2)) - raw_scales

    result = least_squares(residuals, p0, bounds=(lower, upper))
    s0, vr = float(result.x[0]), float(result.x[1])
    scaled_harmonics = tuple(
        SpatialHarmonic(
            omega=h.omega,
            a=h.a * s0,
            b=h.b * s0,
            c=h.c * s0,
            d=h.d * s0,
        )
        for h in harmonics
    )
    normalized_scales = raw_scales / s0 if s0 > 1e-9 else raw_scales
    return vr, scaled_harmonics, normalized_scales


def _smooth_1d(values: np.ndarray, window: int = 5) -> np.ndarray:
    arr = np.asarray(values, dtype=float)
    w = min(window, len(arr))
    if w < 3:
        return arr.copy()
    if w % 2 == 0:
        w -= 1
    pad = w // 2
    padded = np.pad(arr, (pad, pad), mode='edge')
    kernel = np.hanning(w + 2)[1:-1]
    kernel = kernel / np.sum(kernel)
    return np.convolve(padded, kernel, mode='valid')


def _fit_gripper_stribeck(
    speeds: np.ndarray,
    mean_frictions: np.ndarray,
    delta: float = 2.0,
) -> tuple[float, float, float, float]:
    """Fit gripper (fc, fs, vs, fv) in linear stroke units (m/s, N)."""
    v = np.asarray(speeds, dtype=float)
    f = np.maximum(0.0, np.asarray(mean_frictions, dtype=float))
    if len(v) == 0:
        raise ValueError('At least one speed point is required to fit gripper Stribeck parameters')
    if len(v) == 1:
        val = float(f[0])
        return val, val, 0.015, 0.0
    if len(v) == 2:
        slope = max(0.0, float((f[1] - f[0]) / max(1e-6, v[1] - v[0])))
        fc = max(0.0, float(f[0] - slope * v[0]))
        return fc, max(fc, float(f[0])), 0.015, slope

    f_min = float(np.min(f))
    f_max = float(np.max(f))
    v_min = float(np.min(v))
    low_mask = v <= max(0.025, 2.5 * v_min)
    f_low_min = float(np.min(f[low_mask])) if np.any(low_mask) else f_min
    f_low_max = float(np.max(f[low_mask])) if np.any(low_mask) else f_max

    fc_lower = max(0.0, 0.55 * f_low_min)
    fc_upper = max(fc_lower + 1e-3, f_low_max * 1.05)
    df_upper = max(0.02, (f_low_max - fc_lower) * 1.5)
    vs_lower = max(0.002, 0.8 * v_min)
    vs_upper = max(vs_lower + 0.005, min(0.08, float(np.max(v)) * 0.9))
    fv_upper = 20.0

    p0 = np.array([
        0.90 * f_low_min,
        max(0.01, f_low_max - 0.90 * f_low_min),
        min(vs_upper - 1e-4, max(vs_lower + 1e-4, 0.015)),
        max(0.0, min(fv_upper * 0.5, (f[-1] - f_low_min) / max(0.02, float(v[-1])))),
    ])
    lower = np.array([fc_lower, 0.0, vs_lower, 0.0])
    upper = np.array([fc_upper, df_upper, vs_upper, fv_upper])
    p0 = np.clip(p0, lower + 1e-6, upper - 1e-6)

    def residuals(params: np.ndarray) -> np.ndarray:
        fc, df, vs, fv = params
        pred = fc + df * np.exp(-((v / vs) ** delta)) + fv * v
        return pred - f

    result = least_squares(
        residuals, p0, bounds=(lower, upper),
        loss='soft_l1', f_scale=max(0.01, 0.05 * max(0.1, f_low_min)),
    )
    fc, df, vs, fv = map(float, result.x)
    return fc, fc + df, vs, fv


def identify_gripper_friction(
    samples: Sequence[SweepSample],
    *,
    grid_step: float = 0.0005,
    trim_ratio: float = 0.08,
    num_knots: int = 25,
    delta: float = 2.0,
    v0: float | None = None,
) -> IdentificationReport:
    """Identify non-periodic spatial profile + Stribeck model for the gripper (joint=7)."""
    if not samples:
        raise ValueError('Cannot identify gripper friction from empty sample sequence')
    resolved_v0 = 0.002 if v0 is None else float(v0)
    eff_grid_step = min(float(grid_step), 0.001)

    grouped = extract_steady_sweeps(samples, trim_ratio=trim_ratio, inertia_scale=0.0)
    decomp = decompose_half_sum_diff(
        samples, grid_step=eff_grid_step, trim_ratio=trim_ratio, inertia_scale=0.0, grouped=grouped,
    )
    slow_indices = [
        i for i, s in enumerate(decomp.speeds)
        if s <= max(0.025, 2.2 * float(decomp.speeds[0]))
    ] or [0]

    # 1. 非周期半和解耦（不 detrend）：完整保留复位弹簧与连杆位置偏置 F_bias(w)
    raw_bias = _smooth_1d(np.mean([decomp.half_sums[i] for i in slow_indices], axis=0), window=5)
    # 2. 非周期半差解耦：保留连杆传动比与导轨紧度随开度变化的摩擦波动 dF_c(w)
    raw_mod = _smooth_1d(
        np.mean(
            [decomp.half_diffs[i] - float(np.mean(decomp.half_diffs[i])) for i in slow_indices],
            axis=0,
        ),
        window=5,
    )

    # 3. 拟合空间摩擦波动随速度衰减包络 vr
    denom = float(np.dot(raw_mod, raw_mod))
    raw_scales = np.ones(len(decomp.speeds), dtype=float)
    vr = 0.08
    s0 = 1.0
    if denom > 1e-10 and len(decomp.speeds) >= 2:
        for idx in range(len(decomp.speeds)):
            d_center = decomp.half_diffs[idx] - float(np.mean(decomp.half_diffs[idx]))
            raw_scales[idx] = max(0.0, float(np.dot(d_center, raw_mod)) / denom)
        if float(np.max(raw_scales)) > 1e-5:
            v = np.asarray(decomp.speeds, dtype=float)
            min_vr = max(0.015, float(v[0]) * 1.4)
            max_vr = max(min_vr + 0.02, float(np.max(v)) * 1.5, 0.12)
            p0 = np.array([1.0, min(max_vr - 0.005, max(min_vr + 0.005, float(np.median(v))))])
            lower = np.array([0.75, min_vr])
            upper = np.array([1.35, max_vr])
            p0 = np.clip(p0, lower + 1e-6, upper - 1e-6)
            opt = least_squares(
                lambda p: p[0] * np.exp(-((v / p[1]) ** 2)) - raw_scales,
                p0, bounds=(lower, upper),
            )
            s0, vr = float(opt.x[0]), float(opt.x[1])
            raw_mod = raw_mod * s0
            raw_scales = raw_scales / max(1e-9, s0)

    # 4. 下采样至非周期开度节点表 w_grid
    k = max(5, min(int(num_knots), len(decomp.theta_grid)))
    w_knots = np.linspace(float(decomp.theta_grid[0]), float(decomp.theta_grid[-1]), k)
    bias_knots = np.interp(w_knots, decomp.theta_grid, raw_bias)
    mod_knots = np.interp(w_knots, decomp.theta_grid, raw_mod)

    # 5. 拟合 Stribeck 速度参数 (fc, fs, vs, fv)
    fc, fs, vs, fv = _fit_gripper_stribeck(decomp.speeds, decomp.mean_frictions, delta=delta)

    # 6. 联合估计固件串级位置-速度内环参数 kp_equiv (N/m) 与 kv_equiv (1/s)
    #    扣除 ROS + 200Hz 插值器的 25ms 传输延迟 (0.025 * |dq_des|) 后，
    #    按 |q_des - q|_fw = dw_0 + |dq_des| / kv_equiv 回归得到 kv_equiv 与 kp_equiv = median(|tau|) / dw_0
    steady_samples = [
        s
        for dirs in grouped.values()
        for seg in dirs['pos'] + dirs['neg']
        for s in seg
    ]
    spd_list: list[float] = []
    err_list: list[float] = []
    tau_list: list[float] = []
    for spd_val, dirs in sorted(grouped.items()):
        seg_pts = [s for seg in dirs['pos'] + dirs['neg'] for s in seg]
        if len(seg_pts) >= 5:
            spd_list.append(float(spd_val))
            raw_err = float(np.median([abs(s.q_des - s.q) for s in seg_pts]))
            err_list.append(max(1e-4, raw_err - 0.025 * float(spd_val)))
            tau_list.append(float(np.median([abs(s.tau_meas) for s in seg_pts])))

    kp_equiv = 45.0
    kv_equiv = 12.5
    if len(spd_list) >= 2:
        spds_arr = np.asarray(spd_list, dtype=float)
        errs_arr = np.asarray(err_list, dtype=float)
        weights = 1.0 / np.maximum(0.005, spds_arr)
        w_sqrt = np.sqrt(weights)
        design = np.column_stack([w_sqrt, w_sqrt * spds_arr])
        coeffs, _, _, _ = np.linalg.lstsq(design, w_sqrt * errs_arr, rcond=None)
        dw_0 = float(coeffs[0])
        inv_kv = float(coeffs[1])
        if inv_kv > 1e-4:
            kv_equiv = float(np.clip(1.0 / inv_kv, 2.0, 100.0))
        low_errs = [e - s / kv_equiv for s, e in zip(spd_list[:2], err_list[:2])]
        dw_0_est = max(dw_0, float(np.median(low_errs)), 2.0e-4)
        med_tau = float(np.median(tau_list[:2])) if len(tau_list) >= 2 else float(np.median(tau_list))
        if med_tau > 1e-3:
            # 稳态扫描中固件 PI 积分器使表观刚度 med_tau / dw_0 放大约 6 倍；
            # 还原瞬态比例饱和刚度 kp_equiv，使主臂每 20ms 重置起点时 f_comp / kp_equiv 达到瞬态完全饱和
            kp_equiv = float(np.clip((med_tau / dw_0_est) / 6.0, 15.0, 200.0))
    elif steady_samples:
        kp_ratios = [
            abs(s.tau_meas) / max(1e-4, abs(s.q_des - s.q) - (0.025 + 1.0 / kv_equiv) * abs(s.dq_des))
            for s in steady_samples
            if abs(s.q_des - s.q) >= 2e-4 and abs(s.tau_meas) >= 0.05
        ]
        if len(kp_ratios) >= 8:
            kp_equiv = float(np.clip(np.median(kp_ratios) / 6.0, 15.0, 200.0))

    model = GripperFrictionModel(
        fc=fc,
        fs=fs,
        vs=vs,
        delta=delta,
        fv=fv,
        v0=resolved_v0,
        vr=vr,
        kp_equiv=kp_equiv,
        kv_equiv=kv_equiv,
        w_grid=tuple(float(x) for x in w_knots),
        bias_grid=tuple(float(x) for x in bias_knots),
        modulation_grid=tuple(float(x) for x in mod_knots),
    )
    decomp = DecompositionResult(
        theta_grid=decomp.theta_grid,
        speeds=decomp.speeds,
        half_sums=decomp.half_sums,
        half_diffs=decomp.half_diffs,
        mean_frictions=decomp.mean_frictions,
        ripple_scales=raw_scales,
    )

    pred_stribeck = np.array([model.stribeck_magnitude(float(s)) for s in decomp.speeds])
    rmse_stribeck = float(np.sqrt(np.mean((pred_stribeck - decomp.mean_frictions) ** 2)))

    env0 = model.envelope(float(decomp.speeds[0]))
    bias_pred0 = np.array([model.bias_force(float(w)) for w in decomp.theta_grid])
    mod_pred0 = np.array([env0 * model.modulation_force(float(w)) for w in decomp.theta_grid])
    bias_actual0 = decomp.half_sums[0]
    mod_actual0 = decomp.half_diffs[0] - float(np.mean(decomp.half_diffs[0]))
    rmse_cogging = float(np.sqrt(np.mean((bias_pred0 - bias_actual0) ** 2)))
    rmse_modulation = float(np.sqrt(np.mean((mod_pred0 - mod_actual0) ** 2)))

    if steady_samples:
        err_sq = np.array([
            (model.force(s.q, s.dq if abs(s.dq) > 1e-6 else s.dq_des) - s.tau_residual) ** 2
            for s in steady_samples
        ], dtype=float)
        ws4 = np.array([max(1e-6, s.weight) ** 4 for s in steady_samples], dtype=float)
        rmse_total = float(np.sqrt(np.sum(err_sq * ws4) / np.sum(ws4)))
    else:
        rmse_total = 0.0

    return IdentificationReport(
        joint=7,
        model=model,
        decomposition=decomp,
        rmse_total=rmse_total,
        rmse_stribeck=rmse_stribeck,
        rmse_cogging=rmse_cogging,
        rmse_modulation=rmse_modulation,
    )


def identify_joint_friction(
    samples: Sequence[SweepSample],
    *,
    grid_step: float = 0.001,
    trim_ratio: float = 0.1,
    max_harmonics: int = 2,
    candidate_omegas: Sequence[float] | None = None,
    delta: float = 2.0,
    v0: float | None = None,
    inertia_scale: float = 1.0,
) -> IdentificationReport:
    """Run the full 4-step friction and periodic/non-periodic identification pipeline."""
    if not samples:
        raise ValueError('Cannot identify friction from empty sample sequence')
    joints = {s.joint for s in samples}
    if len(joints) != 1:
        raise ValueError(f'Samples must belong to a single joint, got {sorted(joints)}')
    joint_idx = next(iter(joints))
    if joint_idx == 7:
        return identify_gripper_friction(
            samples,
            grid_step=min(grid_step, 0.0005),
            trim_ratio=min(trim_ratio, 0.08),
            delta=delta,
            v0=v0,
        )
    resolved_v0 = (
        DEFAULT_JOINT_V0[joint_idx - 1]
        if v0 is None and 1 <= joint_idx <= 6
        else (0.005 if v0 is None else float(v0))
    )

    grouped = extract_steady_sweeps(samples, trim_ratio=trim_ratio, inertia_scale=inertia_scale)
    decomp = decompose_half_sum_diff(
        samples, grid_step=grid_step, trim_ratio=trim_ratio, inertia_scale=inertia_scale, grouped=grouped,
    )
    slow_indices = [i for i, v in enumerate(decomp.speeds) if v <= 0.16] or [0]
    coh_sum = np.mean([decomp.half_sums[i] for i in slow_indices], axis=0)
    coh_diff = np.mean([decomp.half_diffs[i] for i in slow_indices], axis=0)
    base_harmonics = fit_spatial_harmonics(
        decomp.theta_grid,
        decomp.half_sums[0],
        decomp.half_diffs[0],
        max_harmonics=max_harmonics,
        candidate_omegas=candidate_omegas,
        coherence_sum=coh_sum,
        coherence_diff=coh_diff,
    )
    vr, harmonics, ripple_scales = fit_ripple_envelope(
        decomp.speeds,
        decomp.half_sums,
        decomp.half_diffs,
        decomp.theta_grid,
        base_harmonics,
    )
    fc, fs, vs, fv = fit_stribeck(decomp.speeds, decomp.mean_frictions, delta=delta)

    model = JointFrictionModel(
        fc=fc,
        fs=fs,
        vs=vs,
        delta=delta,
        fv=fv,
        v0=resolved_v0,
        vr=vr,
        harmonics=harmonics,
    )
    decomp = DecompositionResult(
        theta_grid=decomp.theta_grid,
        speeds=decomp.speeds,
        half_sums=decomp.half_sums,
        half_diffs=decomp.half_diffs,
        mean_frictions=decomp.mean_frictions,
        ripple_scales=ripple_scales,
    )

    pred_stribeck = np.array([model.stribeck_magnitude(float(s)) for s in decomp.speeds])
    rmse_stribeck = float(np.sqrt(np.mean((pred_stribeck - decomp.mean_frictions) ** 2)))

    env0 = model.envelope(float(decomp.speeds[0]))
    cog_pred0 = np.array([env0 * model.cogging_torque(float(th)) for th in decomp.theta_grid])
    mod_pred0 = np.array([env0 * model.modulation_torque(float(th)) for th in decomp.theta_grid])
    cog_actual0 = detrend(decomp.half_sums[0], type='linear')
    mod_actual0 = decomp.half_diffs[0] - float(np.mean(decomp.half_diffs[0]))
    rmse_cogging = float(np.sqrt(np.mean((cog_pred0 - cog_actual0) ** 2)))
    rmse_modulation = float(np.sqrt(np.mean((mod_pred0 - mod_actual0) ** 2)))

    steady_samples = [
        s
        for dirs in grouped.values()
        for seg in dirs['pos'] + dirs['neg']
        for s in seg
    ]
    static_offset = float(np.mean(decomp.half_sums[0]))
    if steady_samples:
        err_sq = np.array([
            ((model.torque(s.q, s.dq_des) + static_offset) - s.tau_residual) ** 2
            for s in steady_samples
        ], dtype=float)
        ws4 = np.array([max(1e-6, s.weight) ** 4 for s in steady_samples], dtype=float)
        rmse_total = float(np.sqrt(np.sum(err_sq * ws4) / np.sum(ws4)))
    else:
        rmse_total = 0.0

    return IdentificationReport(
        joint=joint_idx,
        model=model,
        decomposition=decomp,
        rmse_total=rmse_total,
        rmse_stribeck=rmse_stribeck,
        rmse_cogging=rmse_cogging,
        rmse_modulation=rmse_modulation,
    )
