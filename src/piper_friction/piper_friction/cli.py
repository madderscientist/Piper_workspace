"""Command-line entry points for friction data collection, parameter identification, and evaluation."""

import argparse
from pathlib import Path
import sys
from typing import Sequence

from piper_x_driver.configuration import FleetConfig, default_config_path
from piper_x_interfaces.srv import ConfigureArm
import rclpy

from .collector import (
    DEFAULT_GRIPPER_EFFORT,
    DEFAULT_SWEEP_KP,
    DEFAULT_SWEEP_SPEEDS,
    SweepCollectorNode,
    SweepConfig,
    V0CollectorNode,
    V0ProbeConfig,
)
from .identification import (
    DEFAULT_JOINT_V0,
    IdentificationReport,
    V0CalibrationReport,
    identify_arm_v0,
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
    parser.add_argument(
        '--skip-v0', action='store_true',
        help='Skip automatic closed-loop critical v0 search after fitting joint 1..6',
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
        if (
            1 <= args.joint <= 6
            and not args.skip_v0
            and max(report.model.fs, report.model.fc) > 0.02
        ):
            v0_rc = v0_main([
                '--namespace', args.namespace,
                '--arm-id', args.arm_id,
                '--yaml', str(args.fit_output),
                '--joints', str(args.joint),
            ])
            if v0_rc != 0:
                return v0_rc
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
        preserved_v0 = None
        if 1 <= joint_idx <= 6 and joint_idx in arm_model.joints:
            cur_v0 = arm_model.joint(joint_idx).v0
            if abs(cur_v0 - DEFAULT_JOINT_V0[joint_idx - 1]) > 1e-6:
                preserved_v0 = cur_v0
        report = identify_joint_friction(
            by_joint[joint_idx],
            grid_step=args.grid_step,
            trim_ratio=args.trim_ratio,
            max_harmonics=args.max_harmonics,
            candidate_omegas=args.omegas,
            v0=preserved_v0,
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


def format_v0_reports(reports: dict[int, V0CalibrationReport]) -> str:
    lines = ['=== Zero-Speed Smoothing v0 Calibration Report ===']
    for j, r in sorted(reports.items()):
        crit_str = f'{r.v0_critical:.4f}' if r.v0_critical > 0 else '<0.0050 (all stable)'
        lines.append(
            f'  J{j}: v0={r.v0:.5f} rad/s '
            f'(v0_crit={crit_str}, last_stable={r.v0_last_stable:.4f}, '
            f'noise_bound={r.v0_noise:.5f} | '
            f'v_noise={r.v_noise_peak:.4f} rad/s, flips={r.crit_reversals}, '
            f'vmax={r.crit_vel_peak:.4f} rad/s, ping={r.excitation_nm:.3f} Nm, '
            f'disp={r.max_disp_mrad:.3f} mrad)'
        )
    return '\n'.join(lines)


def v0_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Measure and update per-joint zero-speed v0 in place (~2.4 s) without re-running sweeps.'
    )
    parser.add_argument('--namespace', default='/arm_1', help='Target arm ROS namespace (default: /arm_1)')
    parser.add_argument('--arm-id', default='', help='Arm identifier (inferred from namespace if omitted)')
    parser.add_argument('--config-file', type=Path, default=default_config_path())
    parser.add_argument('--yaml', type=Path, default=None, help='Friction YAML file to update in place')
    parser.add_argument(
        '--joints', type=int, nargs='+', default=[1, 2, 3, 4, 5, 6], choices=range(1, 7),
        help='Joints 1..6 to calibrate v0 for (default: all 1..6)',
    )
    args = parser.parse_args(argv)

    fleet = FleetConfig(args.config_file)
    arm_cfg = (
        fleet.arm(args.arm_id)
        if args.arm_id
        else next((a for a in fleet.arms if a.namespace == args.namespace.rstrip('/')), fleet.arms[0])
    )
    yaml_path = Path(args.yaml) if args.yaml is not None else Path(arm_cfg.friction_config)
    if not yaml_path.is_file():
        print(f'Friction YAML not found: {yaml_path}', file=sys.stderr)
        return 1
    arm_model = load_friction_model(yaml_path)
    grav_model = fleet.gravity_model(arm_cfg)
    static_frictions = (
        tuple(grav_model.joint(i).static_friction for i in range(1, 7))
        if grav_model is not None else (0.0,) * 6
    )

    probe_cfg = V0ProbeConfig(namespace=arm_cfg.namespace, joints=tuple(sorted(set(args.joints))))
    rclpy.init()
    node = V0CollectorNode(
        probe_cfg, arm_model,
        static_frictions=static_frictions,
        friction_scale=arm_cfg.friction_scale,
    )
    try:
        while rclpy.ok() and not node.completed and not node.error:
            rclpy.spin_once(node, timeout_sec=0.02)
        if node.error:
            print(f'v0 calibration aborted: {node.error}', file=sys.stderr)
            return 1
        updated_model, reports = identify_arm_v0(
            arm_model,
            node.noise_samples,
            node.search_records,
            joints=probe_cfg.joints,
            friction_scale=arm_cfg.friction_scale,
            amplitudes=node.amplitudes,
        )
        save_friction_model(updated_model, yaml_path)
        print(format_v0_reports(reports))
        print(f'Updated v0 in {yaml_path}')
        if node.config_client.service_is_ready():
            fut = node.config_client.call_async(ConfigureArm.Request(
                end_effector=arm_cfg.end_effector,
                gravity_scale=arm_cfg.gravity_scale,
                friction_scale=-4.0 - arm_cfg.friction_scale,
            ))
            rclpy.spin_until_future_complete(node, fut, timeout_sec=1.5)
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
