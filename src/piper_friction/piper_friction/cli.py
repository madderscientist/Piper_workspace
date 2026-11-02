"""Command-line entry points for friction data collection, parameter identification, and evaluation."""

import argparse
from pathlib import Path
import sys
from typing import Sequence

import rclpy

from .collector import (
    DEFAULT_GRIPPER_EFFORT,
    DEFAULT_SWEEP_KP,
    DEFAULT_SWEEP_SPEEDS,
    SweepCollectorNode,
    SweepConfig,
)
from .identification import (
    IdentificationReport,
    identify_joint_friction,
    load_samples_csv,
    save_samples_csv,
)
from .model import (
    ArmFrictionModel,
    GripperFrictionModel,
    load_friction_model,
    save_friction_model,
)


def format_report(report: IdentificationReport) -> str:
    m = report.model
    if isinstance(m, GripperFrictionModel):
        lines = [
            '=== Gripper (J7) Non-Periodic Friction Identification Report ===',
            'Stribeck + Viscous (Linear Stroke):',
            f'  Fc (Coulomb)   = {m.fc:.5f} N',
            f'  Fs (Static)    = {m.fs:.5f} N',
            f'  vs (Stribeck)  = {m.vs:.5f} m/s',
            f'  fv (Viscous)   = {m.fv:.5f} N/(m/s)',
            f'  v0 (Smoothing) = {m.v0:.5f} m/s',
            f'  vr (Decay)     = {m.vr:.5f} m/s',
            f'  kp_equiv       = {m.kp_equiv:.2f} N/m',
            f'  kv_equiv       = {m.kv_equiv:.2f} 1/s',
            f'Non-Periodic Spatial Profile Knots ({len(m.w_grid)} knots):',
        ]
        if m.w_grid:
            lines.append(
                f'  Stroke span    = {m.w_grid[0] * 1000:.2f} .. {m.w_grid[-1] * 1000:.2f} mm'
            )
            lines.append(
                f'  Bias range     = {min(m.bias_grid):+.4f} .. {max(m.bias_grid):+.4f} N'
            )
            lines.append(
                f'  Modulation     = {min(m.modulation_grid):+.4f} .. {max(m.modulation_grid):+.4f} N'
            )
        lines.extend([
            'Fit Residuals (RMSE):',
            f'  Total steady RMSE = {report.rmse_total:.5f} N',
            f'  Stribeck RMSE     = {report.rmse_stribeck:.5f} N',
            f'  Bias Profile RMSE = {report.rmse_cogging:.5f} N',
            f'  Mod Profile RMSE  = {report.rmse_modulation:.5f} N',
        ])
        return '\n'.join(lines)

    lines = [
        f'=== Joint J{report.joint} Friction Identification Report ===',
        f'Stribeck + Viscous:',
        f'  Fc (Coulomb)   = {m.fc:.5f} Nm',
        f'  Fs (Static)    = {m.fs:.5f} Nm',
        f'  vs (Stribeck)  = {m.vs:.5f} rad/s',
        f'  fv (Viscous)   = {m.fv:.5f} Nm/(rad/s)',
        f'  v0 (Smoothing) = {m.v0:.5f} rad/s',
        f'Periodic Gear / Cogging Ripple:',
        f'  vr (Decay)     = {m.vr:.5f} rad/s',
    ]
    if not m.harmonics:
        lines.append('  Harmonics      = (none detected above threshold)')
    else:
        for idx, h in enumerate(m.harmonics, 1):
            lines.append(
                f'  H{idx}: omega={h.omega:.3f} rad^-1 '
                f'| cogging (a={h.a:+.5f}, b={h.b:+.5f}, |A|={h.cogging_amplitude:.5f} Nm) '
                f'| modulation (c={h.c:+.5f}, d={h.d:+.5f}, |C|={h.modulation_amplitude:.5f} Nm)'
            )
    lines.extend([
        f'Fit Residuals (RMSE):',
        f'  Total steady RMSE = {report.rmse_total:.5f} Nm',
        f'  Stribeck RMSE     = {report.rmse_stribeck:.5f} Nm',
        f'  Cogging RMSE      = {report.rmse_cogging:.5f} Nm',
        f'  Modulation RMSE   = {report.rmse_modulation:.5f} Nm',
    ])
    return '\n'.join(lines)


def sweep_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Execute multi-speed forward/reverse MIT or Gripper sweeps via ROS 2 and save CSV samples.'
    )
    parser.add_argument('--namespace', default='/arm_1', help='Target arm ROS namespace (default: /arm_1)')
    parser.add_argument('--arm-id', default='arm_1', help='Arm identifier for YAML output (default: arm_1)')
    parser.add_argument('--joint', type=int, required=True, choices=range(1, 8), help='Joint index 1..6 (or 7 for gripper)')
    parser.add_argument('--q-min', type=float, required=True, help='Sweep minimum angle (rad) or width (m)')
    parser.add_argument('--q-max', type=float, required=True, help='Sweep maximum angle (rad) or width (m)')
    parser.add_argument(
        '--speeds', type=float, nargs='+', default=list(DEFAULT_SWEEP_SPEEDS),
        help='Constant sweep speeds in rad/s (or m/s for joint 7)',
    )
    parser.add_argument('--repeats', type=int, default=1, help='Passes per speed (default: 1)')
    parser.add_argument(
        '--kp', type=float, nargs=6, default=list(DEFAULT_SWEEP_KP),
        help='Six-joint MIT kp gains during sweep',
    )
    parser.add_argument('--kd', type=float, nargs=6, default=None, help='Six-joint MIT kd gains')
    parser.add_argument(
        '--gripper-effort', type=float, default=DEFAULT_GRIPPER_EFFORT,
        help='Gripper effort limit (0..5 N) when sweeping joint 7 (default: 2.0)',
    )
    parser.add_argument('--settle-time', type=float, default=0.4, help='Endpoint settle duration in s')
    parser.add_argument('--output-csv', type=Path, required=True, help='Path to write recorded CSV samples')
    parser.add_argument(
        '--fit-output', type=Path, default=None,
        help='Optional YAML path to immediately fit and update/save identified friction parameters',
    )
    args = parser.parse_args(argv)

    config = SweepConfig(
        namespace=args.namespace,
        joint=args.joint,
        q_min=args.q_min,
        q_max=args.q_max,
        speeds=tuple(args.speeds),
        repeats=args.repeats,
        kp=tuple(args.kp),
        kd=None if args.kd is None else tuple(args.kd),
        gripper_effort=args.gripper_effort,
        settle_time=args.settle_time,
    )

    rclpy.init()
    node = SweepCollectorNode(config)
    try:
        while rclpy.ok() and not node.completed and not node.error:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    if node.error:
        print(f'Sweep aborted: {node.error}', file=sys.stderr)
        return 1

    save_samples_csv(node.samples, args.output_csv)
    print(f'Saved {len(node.samples)} samples to {args.output_csv}')

    if args.fit_output is not None:
        report = identify_joint_friction(node.samples)
        print(format_report(report))
        arm_model = (
            load_friction_model(args.fit_output)
            if args.fit_output.exists()
            else ArmFrictionModel(arm_id=args.arm_id)
        )
        arm_model = arm_model.with_joint(args.joint, report.model)
        save_friction_model(arm_model, args.fit_output)
        print(f'Updated friction model saved to {args.fit_output}')
    return 0


def fit_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Fit Stribeck and periodic spatial harmonics from one or more sweep CSV files.'
    )
    parser.add_argument('csv_files', nargs='+', type=Path, help='Input sweep CSV file(s)')
    parser.add_argument('--output', '-o', type=Path, required=True, help='Output YAML friction model path')
    parser.add_argument('--arm-id', default='arm_1', help='Arm identifier (default: arm_1)')
    parser.add_argument('--max-harmonics', type=int, default=2, help='Maximum spatial harmonics per joint')
    parser.add_argument(
        '--omegas', type=float, nargs='*', default=None,
        help='Optional known spatial frequencies omega_m (cycles per 2*pi rad)',
    )
    parser.add_argument('--grid-step', type=float, default=0.001, help='Angular grid step in rad')
    parser.add_argument('--trim-ratio', type=float, default=0.1, help='Transient trim ratio at sweep ends')
    args = parser.parse_args(argv)

    by_joint: dict[int, list] = {}
    for csv_path in args.csv_files:
        for sample in load_samples_csv(csv_path):
            by_joint.setdefault(sample.joint, []).append(sample)

    arm_model = (
        load_friction_model(args.output)
        if args.output.exists()
        else ArmFrictionModel(arm_id=args.arm_id)
    )

    for joint_idx in sorted(by_joint):
        report = identify_joint_friction(
            by_joint[joint_idx],
            grid_step=args.grid_step,
            trim_ratio=args.trim_ratio,
            max_harmonics=args.max_harmonics,
            candidate_omegas=args.omegas,
        )
        print(format_report(report))
        arm_model = arm_model.with_joint(joint_idx, report.model)

    save_friction_model(arm_model, args.output)
    print(f'Saved friction model to {args.output}')
    return 0


def evaluate_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Evaluate 6-joint friction compensation torques from a calibrated YAML file.'
    )
    parser.add_argument('--model', '-m', type=Path, required=True, help='Path to calibrated YAML file')
    parser.add_argument('--position', '-q', type=float, nargs=6, required=True, help='Six joint angles (rad)')
    parser.add_argument('--velocity', '-v', type=float, nargs=6, required=True, help='Six joint velocities (rad/s)')
    parser.add_argument('--cogging-scale', type=float, default=1.0, help='Scale for cogging term')
    parser.add_argument('--friction-scale', type=float, default=1.0, help='Scale for directional friction')
    args = parser.parse_args(argv)

    arm_model = load_friction_model(args.model)
    torques = arm_model.torques(
        args.position,
        args.velocity,
        cogging_scale=args.cogging_scale,
        friction_scale=args.friction_scale,
    )
    print(' '.join(f'{t:.6f}' for t in torques))
    return 0
