import cv2
import numpy as np

from wrist_camera import intrinsic
from wrist_camera.board import Detection
from .conftest import front_pose

K = np.array([[820.0, 0, 636.0], [0, 818.0, 358.0], [0, 0, 1.0]])
D = np.array([-0.152, 0.041, 0.0011, -0.0007, 0.0])
SIZE = (1280, 720)


def synth(board, pose, size=SIZE, noise=0.0, seed=0):
    rvec = cv2.Rodrigues(pose[:3, :3])[0]
    points = cv2.projectPoints(np.ascontiguousarray(board.object_points_all), rvec,
                               pose[:3, 3], K, D)[0].reshape(-1, 2)
    if noise:
        points = points + np.random.default_rng(seed).normal(0, noise, points.shape)
    inside = ((points[:, 0] > 2) & (points[:, 0] < size[0] - 2)
              & (points[:, 1] > 2) & (points[:, 1] < size[1] - 2))
    ids = np.nonzero(inside)[0].astype(np.int32)
    return Detection(points[inside].astype(np.float32).reshape(-1, 1, 2),
                     ids.reshape(-1, 1), size, marker_count=len(ids) // 2)


def sweep(board, count=24, seed=1):
    rng = np.random.default_rng(seed)
    views = []
    for index in range(count):
        rvec = rng.uniform(-0.5, 0.5, 3)
        distance = rng.uniform(0.40, 0.90)
        offset = rng.uniform(-0.24, 0.24, 2) * distance
        pose = front_pose(board, rvec, [offset[0], offset[1], distance])
        views.append({'name': f'{index:04d}', 'detection': synth(board, pose)})
    return views


def test_recovers_intrinsics_outliers_and_profile_relations(board):
    views = sweep(board)
    result = intrinsic.calibrate(board, views)
    assert result['ok'], result.get('reason')
    matrix = np.asarray(result['camera_matrix']).reshape(3, 3)
    assert np.allclose(matrix[:2, :2], K[:2, :2], rtol=0.01, atol=1.0)
    assert np.allclose(matrix[:2, 2], K[:2, 2], atol=2.0)
    assert result['rms'] < 0.2

    bad = views[5]['detection']
    noise = np.random.default_rng(7).normal(0, 3.0, bad.corners.shape)
    bad.corners = (bad.corners + noise).astype(np.float32)
    noisy_result = intrinsic.calibrate(board, views)
    assert '0005' in intrinsic.outliers(noisy_result)

    low_scale = dict(result, width=640, height=360)
    m_scale = np.asarray(result['camera_matrix'], float).reshape(3, 3) * 0.5
    m_scale[2, 2] = 1.0
    low_scale['camera_matrix'] = m_scale.reshape(9).tolist()
    assert intrinsic.relate_profiles(low_scale, result)['kind'] == 'scale'
