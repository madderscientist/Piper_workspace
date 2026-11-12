"""外参标定：求腕部相机相对夹爪最高点 gripper_base_top 的固定变换 T_link<-cam。"""

from __future__ import annotations

import cv2
import numpy as np
from scipy.optimize import least_squares

from wrist_camera import transforms

MIN_HANDEYE_POSES = 3
BOARD_FIXED_TRANS_MM = 20.0
BOARD_FIXED_ANGLE_DEG = 3.0
WELL_POSED_CONDITION = 1e4
_METHODS = {
    'tsai': cv2.CALIB_HAND_EYE_TSAI,
    'park': cv2.CALIB_HAND_EYE_PARK,
    'horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def wrist_board_in_base(sample: dict, transform) -> np.ndarray:
    return (np.asarray(sample['T_base_link'], float)
            @ np.asarray(transform, float)
            @ np.asarray(sample['T_cam_board'], float))


def board_stability(samples: list[dict], transform=None) -> dict:
    if transform is not None and samples:
        boards = [wrist_board_in_base(s, transform) for s in samples]
    else:
        return {'count': 0, 'angle_rms_deg': 0.0, 'angle_max_deg': 0.0,
                'trans_rms_mm': 0.0, 'trans_max_mm': 0.0, 'fixed': True, 'per_sample': []}
    spread = transforms.spread(boards)
    spread['fixed'] = bool(spread['trans_max_mm'] <= BOARD_FIXED_TRANS_MM
                           and spread['angle_max_deg'] <= BOARD_FIXED_ANGLE_DEG)
    return spread


def solve_handeye(samples: list[dict], methods=None) -> dict:
    if len(samples) < MIN_HANDEYE_POSES:
        return {'ok': False,
                'reason': f'AX=XB 至少要 {MIN_HANDEYE_POSES} 个姿态，现在只有 {len(samples)} 个'}
    gripper = [np.asarray(s['T_base_link'], float) for s in samples]
    target = [np.asarray(s['T_cam_board'], float) for s in samples]
    if _rotation_axes_parallel(gripper):
        return {'ok': False,
                'reason': 'AX=XB 无解：各姿态之间的旋转轴几乎平行，把手腕换几个方向再采'}

    results = {}
    for name in (methods or _METHODS):
        rotation, translation = cv2.calibrateHandEye(
            [m[:3, :3] for m in gripper], [m[:3, 3] for m in gripper],
            [m[:3, :3] for m in target], [m[:3, 3] for m in target],
            method=_METHODS[name])
        matrix = transforms.rt_to_matrix(rotation, np.asarray(translation).reshape(3))
        boards = [g @ matrix @ t for g, t in zip(gripper, target)]
        board_mean = transforms.average_transforms(boards)
        results[name] = {
            'transform': transforms.matrix_to_dict(matrix),
            'matrix': matrix.tolist(),
            'board_pose': transforms.matrix_to_dict(board_mean),
            'consistency': board_consistency(samples, matrix),
        }
    best = min(results, key=lambda k: results[k]['consistency']['trans_rms_mm'])
    return {
        'ok': True, 'method': 'handeye', 'best': best,
        'samples': len(samples),
        'transform': results[best]['transform'],
        'matrix': results[best]['matrix'],
        'board_pose': results[best]['board_pose'],
        'consistency': results[best]['consistency'],
        'per_method': results,
        'method_spread': transforms.spread(
            [np.asarray(v['matrix'], float) for v in results.values()]),
    }


def solve_joint(samples: list[dict], lever: float = 0.5) -> dict:
    if not samples:
        return {'ok': False, 'reason': '一组样本都没有'}
    handeye = solve_handeye(samples)
    if not handeye.get('ok'):
        return handeye
    x_init = np.asarray(handeye['matrix'], float)
    boards_init = [wrist_board_in_base(s, x_init) for s in samples]
    b_init = transforms.average_transforms(boards_init)

    def unpack(params):
        return (transforms.perturb(x_init, params[:6]),
                transforms.perturb(b_init, params[6:12]))

    def residuals(params):
        x_mat, b_mat = unpack(params)
        rows = []
        for sample in samples:
            wrist = wrist_board_in_base(sample, x_mat)
            delta = transforms.log_delta(transforms.invert(b_mat) @ wrist)
            rows.append(np.concatenate([delta[:3] * lever, delta[3:]]))
        return np.concatenate(rows)

    params, fit = _fit(residuals, 12)
    x_opt, b_opt = unpack(params)
    return {
        'ok': True,
        'method': 'joint',
        'samples': len(samples),
        'transform': transforms.matrix_to_dict(x_opt),
        'matrix': x_opt.tolist(),
        'board_pose': transforms.matrix_to_dict(b_opt),
        'consistency': board_consistency(samples, x_opt),
        **fit,
    }


def board_consistency(samples: list[dict], transform) -> dict:
    transform = np.asarray(transform, float)
    if not samples:
        return transforms.deviation([])
    boards = [wrist_board_in_base(s, transform) for s in samples]
    return transforms.spread(boards)


def compare(a, b) -> dict:
    angle, distance = transforms.transform_delta(a, b)
    return {'angle_deg': round(angle, 4), 'trans_mm': round(distance * 1e3, 3)}


def _fit(residuals, size) -> tuple[np.ndarray, dict]:
    result = least_squares(residuals, np.zeros(size), method='lm', xtol=1e-12)
    singular = np.linalg.svd(result.jac, compute_uv=False)
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0 else float('inf')
    return result.x, {
        'residual_mm_before': _rms_mm(residuals(np.zeros(size))),
        'residual_mm': _rms_mm(result.fun),
        'condition': round(condition, 1),
        'well_posed': bool(condition < WELL_POSED_CONDITION),
    }


def _rms_mm(values) -> float:
    return round(float(np.sqrt(np.mean(np.asarray(values) ** 2))) * 1e3, 3)


def _rotation_axes_parallel(poses, threshold_deg: float = 5.0) -> bool:
    axes = []
    for index in range(1, len(poses)):
        delta = transforms.invert(poses[index - 1]) @ poses[index]
        rvec = cv2.Rodrigues(delta[:3, :3])[0].reshape(3)
        angle = float(np.linalg.norm(rvec))
        if angle > np.deg2rad(threshold_deg):
            axes.append(rvec / angle)
    if len(axes) < 2:
        return True
    axes = np.stack(axes)
    return bool(np.linalg.matrix_rank(axes, tol=0.1) < 2)
