"""Command-line entry points for bidirectional gravity & static friction probing, fitting, and evaluation."""

from dataclasses import replace
import argparse
from pathlib import Path
import sys
from typing import Sequence

from piper_x_driver.configuration import default_config_path, FleetConfig
from piper_x_driver.friction import load_friction_model, save_friction_model
import rclpy

from .collector import DEFAULT_PROBE_KP, ProbeCollectorNode, ProbeConfig
from .identification import (
    GravityIdentificationReport,
    identify_gripper_payload,
    identify_joint_gravity,
    load_probe_samples_csv,
    save_probe_samples_csv,
)
from .model import ArmGravityModel, load_gravity_model, save_gravity_model


def format_report(report: GravityIdentificationReport) -> str:
    m = report.model
    lines = [
        f'=== Joint J{report.joint} Gravity & Static Friction Report ===',
        f'Gravity Calibration (tau_g_cal = scale * tau_g_cad(q + angle_offset) + bias):',
        f'  scale            = {m.scale:.5f}',
        f'  angle_offset     = {m.angle_offset:+.5f} rad ({m.angle_offset * 180.0 / 3.141592653589793:+.3f} deg)',
        f'  bias             = {m.bias:+.5f} Nm',
        f'Static Friction (Bidirectional Half-Difference):',
        f'  Fs (mean)        = {report.static_friction_mean:.5f} Nm',
        f'  Fs (85th pct)    = {report.static_friction_p85:.5f} Nm',
        f'Gravity Fit Residuals (RMSE):',
        f'  Nominal CAD RMSE = {report.rmse_cad:.5f} Nm',
        f'  Calibrated RMSE  = {report.rmse_calibrated:.5f} Nm',
    ]
    if report.added_mass_kg is not None:
        lines += [
            f'Added gripper mass = {report.added_mass_kg:.6f} kg',
            f'Total terminal mass = {report.total_effector_mass_kg:.6f} kg',
            'Extra mass uses the uniform gripper mesh; arm calibration and known parts are retained.',
        ]
    return '\n'.join(lines)


def _fit_for_arm(by_joint, arm_model, fleet, arm_id, **kwargs):
    arm = fleet.arm(arm_id)
    arm_model = arm_model.for_effector(fleet.effectors[arm.end_effector].model)
    if arm_model.effector_model:
        return identify_gripper_payload(by_joint, arm_model, **kwargs)
    reports = {}
    for joint, samples in sorted(by_joint.items()):
        report = identify_joint_gravity(samples, **kwargs)
        reports[joint] = report
        arm_model = arm_model.with_joint(joint, report.model)
    return arm_model, reports


def _sync_static_friction_to_yaml(
    friction_yaml: Path,
    arm_id: str,
    static_frictions: dict[int, float],
) -> None:
    if not friction_yaml.exists():
        return
    fric_model = load_friction_model(friction_yaml)
    for j_idx, fs_val in static_frictions.items():
        if j_idx in fric_model.joints:
            cur = fric_model.joint(j_idx)
            fric_model = fric_model.with_joint(
                j_idx,
                replace(cur, fs=round(max(cur.fc, float(fs_val)), 6)),
            )
    save_friction_model(fric_model, friction_yaml)


def probe_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Execute bidirectional quasi-static MIT probes with compensation off and save CSV.'
    )
    parser.add_argument('--namespace', default='/arm_1', help='Target arm ROS namespace (default: /arm_1)')
    parser.add_argument('--arm-id', default='arm_1', help='Arm identifier for YAML output (default: arm_1)')
    parser.add_argument('--config-file', type=Path, default=default_config_path())
    parser.add_argument('--joint', type=int, required=True, choices=range(1, 7), help='Joint index 1..6')
    parser.add_argument('--q-min', type=float, required=True, help='Probe minimum angle in rad')
    parser.add_argument('--q-max', type=float, required=True, help='Probe maximum angle in rad')
    parser.add_argument('--num-points', type=int, default=9, help='Number of target waypoints (default: 9)')
    parser.add_argument('--pre-travel', type=float, default=0.04, help='Pre-travel distance in rad (default: 0.04)')
    parser.add_argument('--approach-speed', type=float, default=0.05, help='Approach speed in rad/s (default: 0.05)')
    parser.add_argument('--settle-time', type=float, default=0.60, help='Hold settle duration in s (default: 0.60)')
    parser.add_argument(
        '--kp', type=float, nargs=6, default=list(DEFAULT_PROBE_KP),
        help='Six-joint MIT kp gains during probe',
    )
    parser.add_argument('--kd', type=float, nargs=6, default=None, help='Six-joint MIT kd gains')
    parser.add_argument('--output-csv', type=Path, required=True, help='Path to write recorded CSV samples')
    parser.add_argument(
        '--fit-output', type=Path, default=None,
        help='Optional YAML path to immediately fit and save calibrated gravity parameters',
    )
    parser.add_argument(
        '--update-friction-yaml', type=Path, default=None,
        help='Optional friction YAML path to sync calibrated static friction Fs',
    )
    args = parser.parse_args(argv)

    config = ProbeConfig(
        namespace=args.namespace,
        joint=args.joint,
        q_min=args.q_min,
        q_max=args.q_max,
        num_points=args.num_points,
        pre_travel=args.pre_travel,
        approach_speed=args.approach_speed,
        settle_time=args.settle_time,
        kp=tuple(args.kp),
        kd=None if args.kd is None else tuple(args.kd),
    )

    rclpy.init()
    node = ProbeCollectorNode(config)
    try:
        while rclpy.ok() and not node.completed and not node.error:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    if node.error:
        print(f'Probe aborted: {node.error}', file=sys.stderr)
        return 1

    save_probe_samples_csv(node.samples, args.output_csv)
    print(f'Saved {len(node.samples)} samples to {args.output_csv}')

    if args.fit_output is not None:
        fleet = FleetConfig(args.config_file)
        arm_model = (
            load_gravity_model(args.fit_output)
            if args.fit_output.exists()
            else fleet.gravity_model(args.arm_id) or ArmGravityModel(arm_id=args.arm_id)
        )
        arm_model, reports = _fit_for_arm(
            {args.joint: node.samples}, arm_model, fleet, args.arm_id)
        report = reports[args.joint]
        print(format_report(report))
        save_gravity_model(arm_model, args.fit_output)
        print(f'Updated gravity model saved to {args.fit_output}')
        if args.update_friction_yaml is not None:
            _sync_static_friction_to_yaml(
                args.update_friction_yaml,
                args.arm_id,
                {args.joint: report.model.static_friction},
            )
    return 0


def fit_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Fit calibrated gravity and static friction from probe or sweep CSV files.'
    )
    parser.add_argument('csv_files', nargs='+', type=Path, help='Input probe or sweep CSV file(s)')
    parser.add_argument('--output', '-o', type=Path, required=True, help='Output YAML gravity model path')
    parser.add_argument('--arm-id', default='arm_1', help='Arm identifier (default: arm_1)')
    parser.add_argument('--config-file', type=Path, default=default_config_path())
    parser.add_argument(
        '--torque-source', choices=('auto', 'kp_error', 'motor_torque'), default='auto',
        help='Torque signal to use for decomposition (default: auto)',
    )
    parser.add_argument('--grid-step', type=float, default=0.005, help='Angular grid step in rad')
    parser.add_argument('--trim-ratio', type=float, default=0.08, help='Endpoint trim ratio')
    parser.add_argument(
        '--update-friction-yaml', type=Path, default=None,
        help='Optional friction YAML path to sync calibrated static friction Fs',
    )
    args = parser.parse_args(argv)

    by_joint: dict[int, list] = {}
    for csv_path in args.csv_files:
        for sample in load_probe_samples_csv(csv_path):
            by_joint.setdefault(sample.joint, []).append(sample)

    fleet = FleetConfig(args.config_file)
    arm_model = (
        load_gravity_model(args.output)
        if args.output.exists()
        else fleet.gravity_model(args.arm_id) or ArmGravityModel(arm_id=args.arm_id)
    )
    arm_model, reports = _fit_for_arm(
        by_joint, arm_model, fleet, args.arm_id,
        torque_source=args.torque_source, trim_ratio=args.trim_ratio, grid_step=args.grid_step)
    fs_updates = {}
    for joint_idx, report in sorted(reports.items()):
        print(format_report(report))
        fs_updates[joint_idx] = report.model.static_friction

    save_gravity_model(arm_model, args.output)
    print(f'Saved gravity model to {args.output}')
    if args.update_friction_yaml is not None:
        _sync_static_friction_to_yaml(args.update_friction_yaml, args.arm_id, fs_updates)
        print(f'Synced static friction Fs to {args.update_friction_yaml}')
    return 0


def evaluate_main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Evaluate 6-joint calibrated gravity compensation torques from a YAML file.'
    )
    parser.add_argument('--model', '-m', type=Path, required=True, help='Path to calibrated gravity YAML file')
    parser.add_argument('--position', '-q', type=float, nargs=6, required=True, help='Six joint angles (rad)')
    args = parser.parse_args(argv)

    arm_model = load_gravity_model(args.model)
    torques = arm_model.torques(args.position)
    print(' '.join(f'{t:.6f}' for t in torques))
    return 0
