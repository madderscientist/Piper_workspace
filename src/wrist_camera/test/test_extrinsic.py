import cv2
import numpy as np

from wrist_camera import extrinsic, transforms


def pose(rvec, translation):
    return transforms.rt_to_matrix(cv2.Rodrigues(np.asarray(rvec, float))[0],
                                   np.asarray(translation, float))


T_BASE_BOARD = pose([2.9, 0.2, 0.1], [0.55, -0.05, 0.15])
T_LINK_CAM = pose([0.10, -1.55, 0.02], [0.031, -0.008, 0.046])

ARM_POSES = [
    pose([0.20, 0.10, -0.30], [0.34, 0.19, 0.24]),
    pose([-0.35, 0.42, 0.18], [0.39, 0.13, 0.31]),
    pose([0.55, -0.24, 0.40], [0.30, 0.25, 0.19]),
    pose([-0.15, -0.50, -0.22], [0.42, 0.16, 0.28]),
    pose([0.31, 0.62, 0.11], [0.36, 0.21, 0.22]),
    pose([-0.48, 0.05, 0.35], [0.33, 0.11, 0.33]),
]


def build_single_arm(arm_poses=None, link_cam=None):
    link_cam = T_LINK_CAM if link_cam is None else link_cam
    return [{
        'T_base_link': arm,
        'T_cam_board': transforms.invert(link_cam) @ transforms.invert(arm) @ T_BASE_BOARD,
    } for arm in (arm_poses or ARM_POSES)]


def test_single_arm_handeye_and_nonlinear_refine_recover_transform():
    samples = build_single_arm()
    handeye = extrinsic.solve_handeye(samples)
    assert handeye['ok'], handeye.get('reason')
    angle, distance = transforms.transform_delta(T_LINK_CAM, np.asarray(handeye['matrix']))
    assert angle < 1e-6 and distance < 1e-9
    assert handeye['consistency']['trans_max_mm'] < 1e-6

    refined = extrinsic.solve_joint(samples)
    assert refined['ok'] and refined['well_posed']
    angle_r, dist_r = transforms.transform_delta(T_LINK_CAM, np.asarray(refined['matrix']))
    assert angle_r < 1e-6 and dist_r < 1e-9
    assert refined['residual_mm'] < 1e-6

    wrong = T_LINK_CAM @ pose([0.0, 0.0, np.deg2rad(2.0)], [0.0, 0.0, 0.0])
    assert extrinsic.board_consistency(samples, wrong)['trans_rms_mm'] > 1.0


def test_handeye_rejects_insufficient_or_parallel_poses():
    assert not extrinsic.solve_handeye(build_single_arm(ARM_POSES[:2]))['ok']
    parallel = [pose([0.0, 0.0, a], [0.35, 0.18, 0.25]) for a in (0.0, 0.3, 0.6, 0.9)]
    assert not extrinsic.solve_handeye(build_single_arm(parallel))['ok']
