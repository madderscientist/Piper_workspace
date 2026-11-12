"""内参标定：纯函数，不碰 ROS。"""

from __future__ import annotations

import math

import cv2
import numpy as np

from wrist_camera.board import Board, Detection

MIN_CORNERS = 8
MIN_VIEWS = 6


def calibrate(board: Board, views: list[dict], flags: int = 0) -> dict:
    usable = [v for v in views if v['detection'].count >= MIN_CORNERS]
    if len(usable) < MIN_VIEWS:
        return {'ok': False, 'reason': f'可用视图只有 {len(usable)} 张，至少要 {MIN_VIEWS} 张',
                'images': len(usable)}
    sizes = {v['detection'].size for v in usable}
    if len(sizes) != 1:
        return {'ok': False, 'reason': f'视图分辨率不一致：{sorted(sizes)}'}
    size = usable[0]['detection'].size

    rms, matrix, distortion, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        [v['detection'].corners for v in usable],
        [v['detection'].ids for v in usable],
        board.raw, size, np.empty((0, 0)), np.empty((0, 0)), flags=flags)

    per_image = []
    for view, rvec, tvec in zip(usable, rvecs, tvecs):
        detection = view['detection']
        expected = cv2.projectPoints(
            board.object_points(detection.ids), rvec, tvec, matrix, distortion)[0]
        error = np.linalg.norm(
            expected.reshape(-1, 2) - detection.corners.reshape(-1, 2), axis=1)
        per_image.append({
            'name': view['name'], 'corners': detection.count,
            'error_px': round(float(np.sqrt(np.mean(error ** 2))), 4),
            'max_px': round(float(error.max()), 4),
            'coverage': round(detection.coverage(), 4),
        })

    return {
        'ok': True,
        'width': size[0], 'height': size[1],
        'camera_matrix': [float(v) for v in np.asarray(matrix).reshape(9)],
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': [float(v) for v in
                                    np.asarray(distortion).reshape(-1)[:5]],
        'rms': round(float(rms), 4),
        'images': len(usable),
        'skipped': len(views) - len(usable),
        'coverage': round(coverage([v['detection'] for v in usable]), 4),
        'fov_deg': fov_deg(matrix, size),
        'per_image': per_image,
    }


def coverage(detections: list[Detection]) -> float:
    points = [d.corners.reshape(-1, 2) for d in detections if d.count]
    if not points:
        return 0.0
    stacked = np.concatenate(points).astype(np.float32)
    if len(stacked) < 3:
        return 0.0
    size = detections[0].size
    hull = cv2.convexHull(stacked)
    return float(cv2.contourArea(hull)) / float(size[0] * size[1])


def fov_deg(matrix, size) -> list[float]:
    matrix = np.asarray(matrix, float).reshape(3, 3)
    return [round(math.degrees(2 * math.atan(size[0] / (2 * matrix[0, 0]))), 2),
            round(math.degrees(2 * math.atan(size[1] / (2 * matrix[1, 1]))), 2)]


def outliers(result: dict, factor: float = 3.0) -> list[str]:
    if not result.get('ok'):
        return []
    errors = [item['error_px'] for item in result['per_image']]
    if len(errors) < 4:
        return []
    median = float(np.median(errors))
    return [item['name'] for item in result['per_image']
            if item['error_px'] > max(factor * median, median + 0.5)]


def relate_profiles(low: dict, high: dict) -> dict:
    kl = np.asarray(low['camera_matrix'], float).reshape(3, 3)
    kh = np.asarray(high['camera_matrix'], float).reshape(3, 3)
    ratio = low['width'] / high['width']
    scaled = abs(kl[0, 0] - kh[0, 0] * ratio) / (kh[0, 0] * ratio)
    cropped = abs(kl[0, 0] - kh[0, 0]) / kh[0, 0]
    kind = 'scale' if scaled < cropped else 'crop'
    return {
        'kind': kind,
        'ratio': round(float(ratio), 5),
        'fx_low': round(float(kl[0, 0]), 3),
        'fx_high': round(float(kh[0, 0]), 3),
        'fx_high_scaled': round(float(kh[0, 0] * ratio), 3),
        'scale_error_pct': round(float(scaled * 100), 3),
        'crop_error_pct': round(float(cropped * 100), 3),
        'convertible': kind == 'scale' and scaled < 0.02,
    }
