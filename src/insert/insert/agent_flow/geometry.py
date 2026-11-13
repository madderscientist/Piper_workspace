"""First-principles SE(3) calibration, dual-view ray/plane triangulation, and alignment geometry."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Any, Sequence

import cv2
import numpy as np
from piper_x_driver.gravity import ArmGravityModel, forward_kinematics
from wrist_camera import piper_fk
from wrist_camera.transforms import matrix_from_dict
import yaml

from ..policy import (
    canonical_quat,
    forward_tip_pose,
    mat_to_quat,
    normalize_tip_offset,
    quat_inv,
    quat_mul,
    quat_rotvec,
    quat_to_mat,
    quat_to_rpy_deg,
)


@dataclass(frozen=True)
class BoardSpec:
    squares_x: int = 12
    squares_y: int = 9
    square_size: float = 0.015
    marker_size: float = 0.01125
    dictionary: str = 'DICT_5X5_100'


def rt_to_mat(rotation: np.ndarray, translation: Sequence[float] | np.ndarray) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    out[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return out


def invert_se3(matrix: np.ndarray) -> np.ndarray:
    mat = np.asarray(matrix, dtype=np.float64).reshape(4, 4)
    rot = mat[:3, :3]
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rot.T
    out[:3, 3] = -rot.T @ mat[:3, 3]
    return out


def quat_xyzw_to_mat(quat_xyzw: Sequence[float] | np.ndarray) -> np.ndarray:
    x, y, z, w = (float(v) for v in np.asarray(quat_xyzw, dtype=np.float64).reshape(4))
    return quat_to_mat([w, x, y, z])


def mat_to_quat_xyzw(rotation: np.ndarray) -> list[float]:
    w, x, y, z = mat_to_quat(np.asarray(rotation, dtype=np.float64)[:3, :3])
    return [float(x), float(y), float(z), float(w)]


def mat_to_pose_dict(matrix: np.ndarray) -> dict[str, Any]:
    mat = np.asarray(matrix, dtype=np.float64).reshape(4, 4)
    q_wxyz = mat_to_quat(mat[:3, :3])
    rpy_deg = quat_to_rpy_deg(q_wxyz)
    return {
        'matrix': [[float(v) for v in row] for row in mat],
        'translation_m': [float(v) for v in mat[:3, 3]],
        'translation_mm': [round(float(v * 1000.0), 2) for v in mat[:3, 3]],
        'rotation_xyzw': mat_to_quat_xyzw(mat[:3, :3]),
        'quaternion_wxyz': [float(v) for v in q_wxyz],
        'rpy_deg': [round(float(v), 2) for v in rpy_deg],
    }


def pose_dict_to_mat(entry: dict[str, Any]) -> np.ndarray:
    if 'matrix' in entry and entry['matrix'] is not None:
        arr = np.asarray(entry['matrix'], dtype=np.float64)
        if arr.shape == (4, 4):
            return arr
    t = entry.get('translation_m') or entry.get('translation')
    if t is None:
        raise ValueError('Extrinsic entry missing translation')
    if 'quaternion_wxyz' in entry:
        rot = quat_to_mat(entry['quaternion_wxyz'])
    elif 'rotation_xyzw' in entry:
        rot = quat_xyzw_to_mat(entry['rotation_xyzw'])
    elif 'rotation' in entry:
        rot = quat_xyzw_to_mat(entry['rotation'])
    else:
        raise ValueError('Extrinsic entry missing rotation')
    return rt_to_mat(rot, t)


def average_se3(matrices: Sequence[np.ndarray]) -> np.ndarray:
    mats = [np.asarray(m, dtype=np.float64).reshape(4, 4) for m in matrices]
    if not mats:
        raise ValueError('No transforms to average')
    quats = np.stack([mat_to_quat(m[:3, :3]) for m in mats], axis=0)
    _, eigvecs = np.linalg.eigh(quats.T @ quats)
    mean_q = canonical_quat(eigvecs[:, -1])
    mean_t = np.mean([m[:3, 3] for m in mats], axis=0)
    return rt_to_mat(quat_to_mat(mean_q), mean_t)


def transform_points(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    mat = np.asarray(matrix, dtype=np.float64).reshape(4, 4)
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return pts @ mat[:3, :3].T + mat[:3, 3]


def se3_spread(matrices: Sequence[np.ndarray], reference: np.ndarray | None = None) -> dict[str, float]:
    mats = [np.asarray(m, dtype=np.float64).reshape(4, 4) for m in matrices]
    if not mats:
        return {'count': 0, 'trans_rms_mm': 0.0, 'trans_max_mm': 0.0, 'angle_rms_deg': 0.0, 'angle_max_deg': 0.0}
    ref = average_se3(mats) if reference is None else np.asarray(reference, dtype=np.float64).reshape(4, 4)
    ref_inv = invert_se3(ref)
    dists_mm: list[float] = []
    angs_deg: list[float] = []
    for m in mats:
        delta = ref_inv @ m
        dists_mm.append(float(np.linalg.norm(delta[:3, 3]) * 1000.0))
        cos_val = max(-1.0, min(1.0, (float(np.trace(delta[:3, :3])) - 1.0) * 0.5))
        angs_deg.append(math.degrees(math.acos(cos_val)))
    return {
        'count': len(mats),
        'trans_rms_mm': round(float(np.sqrt(np.mean(np.square(dists_mm)))), 2),
        'trans_max_mm': round(float(max(dists_mm)), 2),
        'angle_rms_deg': round(float(np.sqrt(np.mean(np.square(angs_deg)))), 3),
        'angle_max_deg': round(float(max(angs_deg)), 3),
    }


def _build_charuco_board(spec: BoardSpec = BoardSpec()):
    aruco: Any = cv2.aruco
    dict_id = getattr(aruco, spec.dictionary, None)
    if dict_id is None:
        raise ValueError(f'Unsupported ArUco dictionary: {spec.dictionary}')
    dictionary = (
        aruco.getPredefinedDictionary(dict_id)
        if hasattr(aruco, 'getPredefinedDictionary')
        else aruco.Dictionary_get(dict_id)
    )
    board = (
        aruco.CharucoBoard((spec.squares_x, spec.squares_y), spec.square_size, spec.marker_size, dictionary)
        if not hasattr(aruco, 'CharucoBoard_create')
        else aruco.CharucoBoard_create(
            spec.squares_x, spec.squares_y, spec.square_size, spec.marker_size, dictionary
        )
    )
    params = (
        aruco.DetectorParameters()
        if not hasattr(aruco, 'DetectorParameters_create')
        else aruco.DetectorParameters_create()
    )
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    return dictionary, board, params


def _board_chessboard_corners(board) -> np.ndarray:
    if hasattr(board, 'getChessboardCorners'):
        return np.asarray(board.getChessboardCorners(), dtype=np.float64)
    return np.asarray(board.chessboardCorners, dtype=np.float64)


def detect_charuco_pose(
    image_bgr: np.ndarray,
    camera_matrix: Sequence[float] | np.ndarray,
    distortion: Sequence[float] | np.ndarray,
    spec: BoardSpec = BoardSpec(),
    label: str = '',
) -> dict[str, Any]:
    """Detect ChArUco corners, solve T_cam_board, and render an overlay image."""
    img = np.asarray(image_bgr, dtype=np.uint8)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    K = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    dist = np.asarray(distortion, dtype=np.float64).reshape(-1, 1)
    dictionary, board, params = _build_charuco_board(spec)
    all_obj = _board_chessboard_corners(board)

    charuco_corners = None
    charuco_ids = None
    marker_corners: Sequence[Any] = ()
    marker_ids = None

    if hasattr(cv2.aruco, 'CharucoDetector'):
        c_params = cv2.aruco.CharucoParameters()
        c_params.tryRefineMarkers = True
        detector = cv2.aruco.CharucoDetector(board, c_params, params)
        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    else:
        m_corners, m_ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
        if m_ids is not None and len(m_ids) > 0:
            m_corners, m_ids, _, _ = cv2.aruco.refineDetectedMarkers(gray, board, m_corners, m_ids, rejected)
            marker_corners, marker_ids = list(m_corners), m_ids
            if m_ids is not None and len(m_ids) > 0:
                _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    m_corners, m_ids, gray, board
                )

    overlay = img.copy() if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    m_count = 0 if marker_ids is None else int(len(marker_ids))
    c_count = 0 if charuco_ids is None else int(len(charuco_ids))

    if m_count > 0:
        cv2.aruco.drawDetectedMarkers(overlay, marker_corners, marker_ids)
    if c_count > 0 and charuco_corners is not None and charuco_ids is not None:
        cv2.aruco.drawDetectedCornersCharuco(overlay, charuco_corners, charuco_ids, (0, 215, 255))

    if c_count < 6:
        _draw_banner(
            overlay,
            f'{label or "cam"} | Corners: {c_count} (<6), Markers: {m_count}',
            ok=False,
        )
        return {
            'ok': False,
            'corner_count': c_count,
            'marker_count': m_count,
            'reason': f'仅检测到 {c_count} 个 ChArUco 角点（至少需要 6 个）',
            'T_cam_board': None,
            'reproj_rms_px': None,
            'corners_2d': np.zeros((0, 2), dtype=np.float64),
            'object_3d': np.zeros((0, 3), dtype=np.float64),
            'annotated_bgr': overlay,
        }

    ids_flat = np.asarray(charuco_ids, dtype=np.int32).reshape(-1)
    corners_2d = np.asarray(charuco_corners, dtype=np.float64).reshape(-1, 2)
    object_3d = all_obj[ids_flat]

    ok, rvec, tvec = cv2.solvePnP(
        object_3d,
        corners_2d.reshape(-1, 1, 2),
        K,
        dist,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        _draw_banner(overlay, f'{label or "cam"} | solvePnP failed', ok=False)
        return {
            'ok': False,
            'corner_count': c_count,
            'marker_count': m_count,
            'reason': 'solvePnP 无法解出标定板位姿',
            'T_cam_board': None,
            'reproj_rms_px': None,
            'corners_2d': corners_2d,
            'object_3d': object_3d,
            'annotated_bgr': overlay,
        }

    proj, _ = cv2.projectPoints(object_3d, rvec, tvec, K, dist)
    err = proj.reshape(-1, 2) - corners_2d
    rms_px = float(np.sqrt(np.mean(np.sum(err * err, axis=1))))
    rot_mat, _ = cv2.Rodrigues(rvec)
    t_cam_board = rt_to_mat(rot_mat, tvec.reshape(3))
    cv2.drawFrameAxes(overlay, K, dist, rvec, tvec, 0.075, 3)
    _draw_banner(
        overlay,
        f'{label or "cam"} | Corners {c_count}/{len(all_obj)} | RMS {rms_px:.2f}px | Dist {float(np.linalg.norm(tvec))*100:.1f}cm',
        ok=True,
    )
    return {
        'ok': True,
        'corner_count': c_count,
        'marker_count': m_count,
        'reason': '',
        'T_cam_board': t_cam_board,
        'reproj_rms_px': round(rms_px, 3),
        'corners_2d': corners_2d,
        'object_3d': object_3d,
        'annotated_bgr': overlay,
    }


def wrist_camera_pose_in_base(
    joints_rad: Sequence[float],
    wrist_extrinsic_entry: dict[str, Any],
    gravity_model: ArmGravityModel | None = None,
) -> np.ndarray:
    """Compute T_base_wrist(q) = T_base_top(q) @ T_top_wrist using calibrated arm_4 FK + hand-eye."""
    if len(joints_rad) < 6:
        raise ValueError('Six joint positions are required')
    offsets = (
        None
        if gravity_model is None
        else tuple(gravity_model.joint(i + 1).angle_offset for i in range(6))
    )
    dz = float(wrist_extrinsic_entry.get('link6_to_top_z_m', 0.0928))
    return piper_fk(joints_rad, offsets, tool_offset_z_m=dz) @ matrix_from_dict(wrist_extrinsic_entry)


def calibrate_external_camera(
    samples: Sequence[dict[str, Any]],
    camera_matrix: Sequence[float] | np.ndarray,
    distortion: Sequence[float] | np.ndarray,
) -> dict[str, Any]:
    """Solve T_base_head across 1..N captured board observations via joint corner PnP."""
    if not samples:
        raise ValueError('No valid board samples provided')
    K = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    dist = np.asarray(distortion, dtype=np.float64).reshape(-1, 1)
    per_pose = [s['T_base_board'] @ invert_se3(s['T_head_board']) for s in samples]
    init_base_head = average_se3(per_pose)
    init_head_base = invert_se3(init_base_head)

    pts_base = np.concatenate(
        [transform_points(s['T_base_board'], s['object_3d']) for s in samples],
        axis=0,
    )
    uv_head = np.concatenate([s['corners_2d'] for s in samples], axis=0)

    rvec_init, _ = cv2.Rodrigues(init_head_base[:3, :3])
    tvec_init = init_head_base[:3, 3].reshape(3, 1).copy()
    ok, rvec_opt, tvec_opt = cv2.solvePnP(
        pts_base,
        uv_head.reshape(-1, 1, 2),
        K,
        dist,
        rvec_init,
        tvec_init,
        useExtrinsicGuess=True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if ok:
        rot_hb, _ = cv2.Rodrigues(rvec_opt)
        t_head_base = rt_to_mat(rot_hb, tvec_opt.reshape(3))
        t_base_head = invert_se3(t_head_base)
    else:
        t_base_head = init_base_head
        t_head_base = init_head_base
        rvec_opt, _ = cv2.Rodrigues(t_head_base[:3, :3])
        tvec_opt = t_head_base[:3, 3].reshape(3, 1)

    proj, _ = cv2.projectPoints(pts_base, rvec_opt, tvec_opt, K, dist)
    err = proj.reshape(-1, 2) - uv_head
    reproj_rms_px = float(np.sqrt(np.mean(np.sum(err * err, axis=1))))
    return {
        **mat_to_pose_dict(t_base_head),
        'T_base_cam': t_base_head,
        'samples': len(samples),
        'total_corners': int(len(uv_head)),
        'reproj_rms_px': round(reproj_rms_px, 3),
        'consistency': se3_spread(per_pose, t_base_head),
    }


def pixel_to_base_ray(
    u: float,
    v: float,
    camera_matrix: Sequence[float] | np.ndarray,
    distortion: Sequence[float] | np.ndarray,
    t_base_cam: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    K = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    dist = np.asarray(distortion, dtype=np.float64).reshape(-1, 1)
    undist = cv2.undistortPoints(np.array([[[float(u), float(v)]]], dtype=np.float64), K, dist).reshape(2)
    dir_cam = np.array([undist[0], undist[1], 1.0], dtype=np.float64)
    dir_cam /= float(np.linalg.norm(dir_cam))
    mat = np.asarray(t_base_cam, dtype=np.float64).reshape(4, 4)
    origin_b = mat[:3, 3].copy()
    dir_b = mat[:3, :3] @ dir_cam
    dir_b /= float(np.linalg.norm(dir_b))
    return origin_b, dir_b


def project_base_points(
    points_b: Sequence[Sequence[float]] | np.ndarray,
    camera_matrix: Sequence[float] | np.ndarray,
    distortion: Sequence[float] | np.ndarray,
    t_base_cam: np.ndarray,
) -> np.ndarray:
    pts = np.asarray(points_b, dtype=np.float64).reshape(-1, 3)
    K = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    dist = np.asarray(distortion, dtype=np.float64).reshape(-1, 1)
    t_cam_base = invert_se3(t_base_cam)
    rvec, _ = cv2.Rodrigues(t_cam_base[:3, :3])
    tvec = t_cam_base[:3, 3].reshape(3, 1)
    proj, _ = cv2.projectPoints(pts, rvec, tvec, K, dist)
    return proj.reshape(-1, 2)


def triangulate_two_rays(
    o1: np.ndarray, d1: np.ndarray, o2: np.ndarray, d2: np.ndarray,
) -> dict[str, Any]:
    """Exact least-squares midpoint between two 3D rays o1 + s1*d1 and o2 + s2*d2."""
    u1 = np.asarray(d1, dtype=np.float64).reshape(3)
    u1 = u1 / float(np.linalg.norm(u1))
    u2 = np.asarray(d2, dtype=np.float64).reshape(3)
    u2 = u2 / float(np.linalg.norm(u2))
    p1_0 = np.asarray(o1, dtype=np.float64).reshape(3)
    p2_0 = np.asarray(o2, dtype=np.float64).reshape(3)

    a = float(np.dot(u1, u2))
    denom = 1.0 - a * a
    w0 = p1_0 - p2_0
    b = float(np.dot(u1, w0))
    c = float(np.dot(u2, w0))
    if denom < 1e-7:
        raise ValueError('两条相机光线几乎平行，无法三角化三维坐标')
    s1 = (a * c - b) / denom
    s2 = (c - a * b) / denom
    pt1 = p1_0 + s1 * u1
    pt2 = p2_0 + s2 * u2
    mid = 0.5 * (pt1 + pt2)
    gap_m = float(np.linalg.norm(pt1 - pt2))
    ray_angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, abs(a)))))
    return {
        'point_b': mid,
        'point_cam1_b': pt1,
        'point_cam2_b': pt2,
        'depth_cam1_m': float(s1),
        'depth_cam2_m': float(s2),
        'ray_gap_m': gap_m,
        'ray_gap_mm': round(gap_m * 1000.0, 2),
        'ray_angle_deg': round(ray_angle_deg, 2),
    }


def solve_axis_from_two_views(
    hole_center_b: np.ndarray,
    cam1_hole_uv: tuple[float, float],
    cam1_axis_uv: tuple[float, float],
    cam1_K: Sequence[float] | np.ndarray,
    cam1_dist: Sequence[float] | np.ndarray,
    t_base_cam1: np.ndarray,
    cam2_hole_uv: tuple[float, float],
    cam2_axis_uv: tuple[float, float],
    cam2_K: Sequence[float] | np.ndarray,
    cam2_dist: Sequence[float] | np.ndarray,
    t_base_cam2: np.ndarray,
) -> dict[str, Any]:
    """Recover the 3D unit insertion vector in base_link from two 2D arrows via plane intersection."""
    o1, dh1 = pixel_to_base_ray(cam1_hole_uv[0], cam1_hole_uv[1], cam1_K, cam1_dist, t_base_cam1)
    _, da1 = pixel_to_base_ray(cam1_axis_uv[0], cam1_axis_uv[1], cam1_K, cam1_dist, t_base_cam1)
    o2, dh2 = pixel_to_base_ray(cam2_hole_uv[0], cam2_hole_uv[1], cam2_K, cam2_dist, t_base_cam2)
    _, da2 = pixel_to_base_ray(cam2_axis_uv[0], cam2_axis_uv[1], cam2_K, cam2_dist, t_base_cam2)

    vec2d_1 = np.array([cam1_axis_uv[0] - cam1_hole_uv[0], cam1_axis_uv[1] - cam1_hole_uv[1]], dtype=np.float64)
    vec2d_2 = np.array([cam2_axis_uv[0] - cam2_hole_uv[0], cam2_axis_uv[1] - cam2_hole_uv[1]], dtype=np.float64)
    len1_px = float(np.linalg.norm(vec2d_1))
    len2_px = float(np.linalg.norm(vec2d_2))

    n1_raw = np.cross(dh1, da1)
    n2_raw = np.cross(dh2, da2)
    n1_norm = float(np.linalg.norm(n1_raw))
    n2_norm = float(np.linalg.norm(n2_raw))

    method = 'plane_intersection'
    plane_angle_deg = 0.0
    if len1_px < 5.0 and len2_px >= 5.0:
        axis_b = dh1.copy()
        method = 'cam1_coaxial'
    elif len2_px < 5.0 and len1_px >= 5.0:
        axis_b = dh2.copy()
        method = 'cam2_coaxial'
    elif n1_norm > 1e-6 and n2_norm > 1e-6:
        n1 = n1_raw / n1_norm
        n2 = n2_raw / n2_norm
        cross_n = np.cross(n1, n2)
        cross_norm = float(np.linalg.norm(cross_n))
        plane_angle_deg = math.degrees(math.asin(max(0.0, min(1.0, cross_norm))))
        if cross_norm >= 0.12:
            axis_b = cross_n / cross_norm
        else:
            tri_end = triangulate_two_rays(o1, da1, o2, da2)
            diff = tri_end['point_b'] - np.asarray(hole_center_b, dtype=np.float64).reshape(3)
            d_norm = float(np.linalg.norm(diff))
            axis_b = diff / d_norm if d_norm > 1e-6 else ((da1 - dh1) + (da2 - dh2))
            axis_b = axis_b / float(np.linalg.norm(axis_b))
            method = 'epipolar_endpoint_triangulation'
    else:
        axis_b = dh1.copy()
        method = 'coaxial_fallback'

    p0 = np.asarray(hole_center_b, dtype=np.float64).reshape(3)
    p_plus = p0 + 0.04 * axis_b
    uv1 = project_base_points(np.stack([p0, p_plus], axis=0), cam1_K, cam1_dist, t_base_cam1)
    uv2 = project_base_points(np.stack([p0, p_plus], axis=0), cam2_K, cam2_dist, t_base_cam2)
    proj_dir_1 = uv1[1] - uv1[0]
    proj_dir_2 = uv2[1] - uv2[0]
    score = 0.0
    if len1_px >= 3.0 and float(np.linalg.norm(proj_dir_1)) > 1e-6:
        score += float(np.dot(proj_dir_1 / np.linalg.norm(proj_dir_1), vec2d_1 / len1_px))
    if len2_px >= 3.0 and float(np.linalg.norm(proj_dir_2)) > 1e-6:
        score += float(np.dot(proj_dir_2 / np.linalg.norm(proj_dir_2), vec2d_2 / len2_px))
    if score < 0.0:
        axis_b = -axis_b

    axis_b = axis_b / float(np.linalg.norm(axis_b))
    return {
        'insert_axis_b': axis_b,
        'method': method,
        'plane_angle_deg': round(plane_angle_deg, 2),
    }


def shortest_swing_rot(from_unit: np.ndarray, to_unit: np.ndarray) -> np.ndarray:
    """Return 3x3 rotation matrix rotating unit vector from_unit onto to_unit by minimum angle."""
    u = np.asarray(from_unit, dtype=np.float64).reshape(3)
    u = u / float(np.linalg.norm(u))
    v = np.asarray(to_unit, dtype=np.float64).reshape(3)
    v = v / float(np.linalg.norm(v))
    dot = max(-1.0, min(1.0, float(np.dot(u, v))))
    if dot > 1.0 - 1e-9:
        return np.eye(3, dtype=np.float64)
    if dot < -1.0 + 1e-9:
        ortho = np.array([1.0, 0.0, 0.0], dtype=np.float64) if abs(u[0]) < 0.9 else np.array([0.0, 1.0, 0.0], dtype=np.float64)
        axis = np.cross(u, ortho)
        axis /= float(np.linalg.norm(axis))
        rot, _ = cv2.Rodrigues(axis * math.pi)
        return rot
    axis = np.cross(u, v)
    axis /= float(np.linalg.norm(axis))
    angle = math.acos(dot)
    rot, _ = cv2.Rodrigues(axis * angle)
    return rot


def infer_gripper_tip_offset(
    joints_rad: Sequence[float],
    tip_point_b: Sequence[float] | np.ndarray,
    gravity_model: ArmGravityModel | None = None,
) -> tuple[float, float, float]:
    """Convert a visually triangulated 3D plug tip in base_link into insert gripper tip_offset_m."""
    q_eff = (
        tuple(float(joints_rad[i]) for i in range(6))
        if gravity_model is None
        else tuple(float(joints_rad[i]) + gravity_model.joint(i + 1).angle_offset for i in range(6))
    )
    t_link6 = forward_kinematics(q_eff)
    p_link6 = t_link6[:3, 3]
    r_link6 = t_link6[:3, :3]
    offset_link6 = r_link6.T @ (np.asarray(tip_point_b, dtype=np.float64).reshape(3) - p_link6)
    return (-float(offset_link6[0]), -float(offset_link6[1]), float(offset_link6[2]))


def compute_alignment_plan(
    *,
    hole_entrance_b: Sequence[float] | np.ndarray,
    insert_axis_b: Sequence[float] | np.ndarray,
    joints_rad: Sequence[float],
    tip_offset_m: Sequence[float] | np.ndarray,
    gravity_model: ArmGravityModel | None = None,
    standoff_m: float = 0.005,
    insert_depth_m: float = 0.020,
    hole_orient_b: Sequence[float] | np.ndarray | None = None,
    plug_orient_b: Sequence[float] | np.ndarray | None = None,
    symmetry_deg: float = 0.0,
) -> dict[str, Any]:
    """Compute target orientation q_bh, 0.5cm pre-insert pose, GoalPose p_bh, and alignment deltas."""
    p_hole = np.asarray(hole_entrance_b, dtype=np.float64).reshape(3)
    a_ins = np.asarray(insert_axis_b, dtype=np.float64).reshape(3)
    a_ins = a_ins / float(np.linalg.norm(a_ins))
    offset = normalize_tip_offset(tip_offset_m)

    p_bt_curr, q_bt_curr = forward_tip_pose(joints_rad, tip_offset_m=offset, gravity_model=gravity_model)
    r_bt_curr = quat_to_mat(q_bt_curr)
    # In insert T/H frame convention (_R_LINK6_TO_T = diag([-1, 1, -1])), -Z_H is the insertion direction.
    z_h_target = -a_ins
    r_swing = shortest_swing_rot(r_bt_curr[:, 2], z_h_target)
    r_bh = r_swing @ r_bt_curr

    twist_deg = 0.0
    hole_perp_unit: list[float] | None = None
    plug_perp_unit: list[float] | None = None
    sym = float(symmetry_deg) if math.isfinite(float(symmetry_deg)) else 0.0
    if hole_orient_b is not None and plug_orient_b is not None:
        ho = np.asarray(hole_orient_b, dtype=np.float64).reshape(3)
        po_swung = r_swing @ np.asarray(plug_orient_b, dtype=np.float64).reshape(3)
        ho_perp = ho - float(np.dot(ho, a_ins)) * a_ins
        po_perp = po_swung - float(np.dot(po_swung, a_ins)) * a_ins
        nh = float(np.linalg.norm(ho_perp))
        np_len = float(np.linalg.norm(po_perp))
        if nh > 1e-4 and np_len > 1e-4:
            eh = ho_perp / nh
            ep = po_perp / np_len
            hole_perp_unit = [round(float(v), 5) for v in eh]
            plug_orig = np.asarray(plug_orient_b, dtype=np.float64).reshape(3)
            plug_orig /= max(float(np.linalg.norm(plug_orig)), 1e-9)
            plug_perp_unit = [round(float(v), 5) for v in plug_orig]
            if sym > 1.0:
                raw_twist = math.atan2(float(np.dot(np.cross(ep, eh), a_ins)), float(np.dot(ep, eh)))
                period = math.radians(min(360.0, sym))
                wrapped = ((raw_twist + 0.5 * period) % period) - 0.5 * period
                r_twist, _ = cv2.Rodrigues(a_ins * wrapped)
                r_bh = r_twist @ r_bh
                twist_deg = round(math.degrees(wrapped), 2)

    q_bh = mat_to_quat(r_bh)

    p_pre_b = p_hole - float(standoff_m) * a_ins
    p_goal_b = p_hole + float(insert_depth_m) * a_ins

    delta_pos_m = p_pre_b - p_bt_curr
    rot_err_vec = quat_rotvec(quat_mul(q_bh, quat_inv(q_bt_curr)))
    rot_err_deg = math.degrees(float(np.linalg.norm(rot_err_vec)))
    pitch_deg = math.degrees(math.asin(max(-1.0, min(1.0, float(a_ins[2])))))
    yaw_deg = math.degrees(math.atan2(float(a_ins[1]), float(a_ins[0])))

    return {
        'hole_entrance_m': [float(v) for v in p_hole],
        'hole_entrance_mm': [round(float(v * 1000.0), 2) for v in p_hole],
        'insert_axis_unit': [round(float(v), 5) for v in a_ins],
        'insert_axis_spherical_deg': {
            'azimuth_xy_deg': round(yaw_deg, 2),
            'elevation_z_deg': round(pitch_deg, 2),
        },
        'symmetry_deg': round(sym, 1),
        'cross_section_twist_deg': twist_deg,
        'hole_orient_unit': hole_perp_unit,
        'plug_orient_unit': plug_perp_unit,
        'q_bh_wxyz': [round(float(v), 6) for v in q_bh],
        'target_rpy_deg': [round(float(v), 2) for v in quat_to_rpy_deg(q_bh)],
        'pre_insert_m': [float(v) for v in p_pre_b],
        'pre_insert_mm': [round(float(v * 1000.0), 2) for v in p_pre_b],
        'goal_inserted_m': [float(v) for v in p_goal_b],
        'goal_inserted_mm': [round(float(v * 1000.0), 2) for v in p_goal_b],
        'standoff_cm': round(float(standoff_m) * 100.0, 2),
        'insert_depth_cm': round(float(insert_depth_m) * 100.0, 2),
        'current_tip_mm': [round(float(v * 1000.0), 2) for v in p_bt_curr],
        'current_rpy_deg': [round(float(v), 2) for v in quat_to_rpy_deg(q_bt_curr)],
        'align_delta_mm': [round(float(v * 1000.0), 2) for v in delta_pos_m],
        'align_distance_mm': round(float(np.linalg.norm(delta_pos_m) * 1000.0), 2),
        'align_angle_deg': round(rot_err_deg, 2),
        'align_rotvec_deg': [round(math.degrees(float(v)), 2) for v in rot_err_vec],
        'tip_offset_m': list(offset),
        'tip_offset_cm': [round(float(v * 100.0), 2) for v in offset],
    }


def _xy_int(pt: Any) -> tuple[int, int]:
    if isinstance(pt, dict):
        return int(round(float(pt['u']))), int(round(float(pt['v'])))
    return int(round(float(pt[0]))), int(round(float(pt[1])))


def _xy_float(pt: Any) -> tuple[float, float]:
    if isinstance(pt, dict):
        return float(pt['u']), float(pt['v'])
    return float(pt[0]), float(pt[1])


_GPT_COLORS: dict[str, tuple[int, int, int]] = {
    'plug_tip': (255, 230, 0),          # Bright Cyan
    'plug_axis_point': (255, 150, 35),  # Sky Blue
    'plug_orient_point': (95, 245, 95), # Neon Green
    'hole_center': (0, 205, 255),       # Amber Gold
    'hole_insert_point': (55, 75, 255), # Coral Red
    'hole_orient_point': (245, 95, 255),# Magenta
}


def _draw_outlined_arrow(
    canvas: np.ndarray,
    p1: tuple[int, int],
    p2: tuple[int, int],
    color: tuple[int, int, int],
    *,
    thickness: int = 2,
    tip_px: float = 14.0,
) -> None:
    dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    if dist < 2.0:
        return
    tip_len = min(0.42, max(0.15, tip_px / dist))
    cv2.arrowedLine(canvas, p1, p2, (12, 16, 24), thickness + 2, cv2.LINE_AA, tipLength=tip_len)
    cv2.arrowedLine(canvas, p1, p2, color, thickness, cv2.LINE_AA, tipLength=tip_len)


def _draw_gpt_keypoint(
    canvas: np.ndarray,
    pt: tuple[int, int],
    color: tuple[int, int, int],
    *,
    primary: bool = False,
) -> None:
    if primary:
        cv2.circle(canvas, pt, 10, (12, 16, 24), 3, cv2.LINE_AA)
        cv2.circle(canvas, pt, 10, color, 2, cv2.LINE_AA)
        cv2.circle(canvas, pt, 4, (12, 16, 24), -1, cv2.LINE_AA)
        cv2.circle(canvas, pt, 3, color, -1, cv2.LINE_AA)
    else:
        cv2.circle(canvas, pt, 6, (12, 16, 24), -1, cv2.LINE_AA)
        cv2.circle(canvas, pt, 4, color, -1, cv2.LINE_AA)


def _extract_gpt_points(det_2d: dict[str, Any]) -> list[tuple[str, str, tuple[float, float], tuple[int, int, int], bool]]:
    spec = [
        ('plug_tip', 'plug_tip', True),
        ('plug_axis_point', 'plug_axis', False),
        ('plug_orient_point', 'plug_orient', False),
        ('hole_center', 'hole_center', True),
        ('hole_insert_point', 'hole_insert', False),
        ('hole_orient_point', 'hole_orient', False),
    ]
    items: list[tuple[str, str, tuple[float, float], tuple[int, int, int], bool]] = []
    for key, short_name, is_primary in spec:
        if key in det_2d and det_2d[key] is not None:
            items.append((key, short_name, _xy_float(det_2d[key]), _GPT_COLORS[key], is_primary))
    return items


def _compute_gpt_roi(
    img_w: int,
    img_h: int,
    pts_uv: Sequence[tuple[float, float]],
    *,
    aspect_wh: float = 16.0 / 9.0,
    min_w: int = 320,
    pad_scale: float = 2.25,
) -> tuple[int, int, int, int]:
    if not pts_uv:
        cw = min(img_w, max(min_w, img_w // 3))
        ch = min(img_h, max(48, int(round(cw / aspect_wh))))
        return (img_w - cw) // 2, (img_h - ch) // 2, cw, ch
    us = [p[0] for p in pts_uv]
    vs = [p[1] for p in pts_uv]
    cx = 0.5 * (min(us) + max(us))
    cy = 0.5 * (min(vs) + max(vs))
    span_u = max(40.0, max(us) - min(us))
    span_v = max(40.0, max(vs) - min(vs))
    cw_f = max(float(min_w), span_u * pad_scale + 130.0, (span_v * pad_scale + 110.0) * aspect_wh)
    cw = max(64, min(img_w, int(round(cw_f))))
    ch = max(48, min(img_h, int(round(cw / aspect_wh))))
    cw = max(64, min(img_w, int(round(ch * aspect_wh))))
    x0 = max(0, min(img_w - cw, int(round(cx - 0.5 * cw))))
    y0 = max(0, min(img_h - ch, int(round(cy - 0.5 * ch))))
    return x0, y0, cw, ch


def _draw_gpt_vectors_and_points(
    canvas: np.ndarray,
    det_2d: dict[str, Any],
    map_uv,
    *,
    thickness: int = 2,
    tip_px: float = 14.0,
) -> None:
    p_tip = map_uv(_xy_float(det_2d['plug_tip']))
    p_paxis = map_uv(_xy_float(det_2d['plug_axis_point']))
    p_hole = map_uv(_xy_float(det_2d['hole_center']))
    p_hins = map_uv(_xy_float(det_2d['hole_insert_point']))

    _draw_outlined_arrow(canvas, p_paxis, p_tip, _GPT_COLORS['plug_axis_point'], thickness=thickness, tip_px=tip_px)
    if 'plug_orient_point' in det_2d and det_2d['plug_orient_point'] is not None:
        p_pori = map_uv(_xy_float(det_2d['plug_orient_point']))
        _draw_outlined_arrow(canvas, p_tip, p_pori, _GPT_COLORS['plug_orient_point'], thickness=thickness, tip_px=tip_px)

    _draw_outlined_arrow(canvas, p_hole, p_hins, _GPT_COLORS['hole_insert_point'], thickness=thickness, tip_px=tip_px)
    if 'hole_orient_point' in det_2d and det_2d['hole_orient_point'] is not None:
        p_hori = map_uv(_xy_float(det_2d['hole_orient_point']))
        _draw_outlined_arrow(canvas, p_hole, p_hori, _GPT_COLORS['hole_orient_point'], thickness=thickness, tip_px=tip_px)

    for _key, _short, uv, color, is_primary in reversed(_extract_gpt_points(det_2d)):
        _draw_gpt_keypoint(canvas, map_uv(uv), color, primary=is_primary)


def _draw_callout_badges(
    canvas: np.ndarray,
    entries: Sequence[tuple[tuple[int, int], str, tuple[int, int, int]]],
    bounds: tuple[int, int, int, int],
    *,
    font_scale: float = 0.50,
) -> None:
    if not entries:
        return
    bx0, by0, bx1, by1 = bounds
    pts = [e[0] for e in entries]
    cx = sum(p[0] for p in pts) / float(len(pts))
    cy = sum(p[1] for p in pts) / float(len(pts))

    placed_boxes: list[tuple[int, int, int, int]] = []
    directions = [
        (1.0, -0.75), (-1.0, -0.75), (1.0, 0.75), (-1.0, 0.75),
        (1.0, -0.18), (-1.0, -0.18), (1.0, 0.22), (-1.0, 0.22),
        (0.25, -1.0), (-0.25, -1.0), (0.25, 1.0), (-0.25, 1.0),
    ]
    radii = [34.0, 58.0, 86.0, 116.0]

    for (px, py), label, color in entries:
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)
        bw = tw + 16
        bh = th + 12
        out_dx = px - cx
        out_dy = py - cy
        out_norm = math.hypot(out_dx, out_dy)
        if out_norm > 1e-3:
            out_dx /= out_norm
            out_dy /= out_norm
        else:
            out_dx, out_dy = 1.0, -0.5

        best_box = None
        best_score = -1e18
        for rad in radii:
            for dx, dy in directions:
                dn = math.hypot(dx, dy)
                ux, uy = dx / dn, dy / dn
                anchor_x = int(round(px + ux * rad))
                anchor_y = int(round(py + uy * rad))
                x0 = anchor_x if ux >= 0 else anchor_x - bw
                y0 = int(round(anchor_y - 0.5 * bh))
                x0 = max(bx0 + 6, min(bx1 - bw - 6, x0))
                y0 = max(by0 + 6, min(by1 - bh - 6, y0))
                x1, y1 = x0 + bw, y0 + bh

                score = 22.0 * (ux * out_dx + uy * out_dy) - 0.22 * rad
                for ox0, oy0, ox1, oy1 in placed_boxes:
                    ix = max(0, min(x1, ox1) - max(x0, ox0) + 6)
                    iy = max(0, min(y1, oy1) - max(y0, oy0) + 5)
                    if ix > 0 and iy > 0:
                        score -= 5000.0 + float(ix * iy)
                for qx, qy in pts:
                    if (x0 - 8) <= qx <= (x1 + 8) and (y0 - 8) <= qy <= (y1 + 8):
                        score -= 2200.0
                if score > best_score:
                    best_score = score
                    best_box = (x0, y0, x1, y1)

        assert best_box is not None
        placed_boxes.append(best_box)
        x0, y0, x1, y1 = best_box
        lx = max(x0, min(x1, px))
        ly = max(y0, min(y1, py))
        cv2.line(canvas, (px, py), (lx, ly), (15, 23, 42), 3, cv2.LINE_AA)
        cv2.line(canvas, (px, py), (lx, ly), color, 1, cv2.LINE_AA)

        sub = canvas[y0:y1, x0:x1]
        if sub.size > 0:
            bg = np.full_like(sub, (15, 23, 42))
            cv2.addWeighted(bg, 0.84, sub, 0.16, 0, sub)
        cv2.rectangle(canvas, (x0, y0), (x1, y1), color, 1, cv2.LINE_AA)
        cv2.rectangle(canvas, (x0, y0), (min(x1, x0 + 4), y1), color, -1)
        cv2.putText(
            canvas,
            label,
            (x0 + 9, y1 - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (245, 250, 255),
            1,
            cv2.LINE_AA,
        )


def _draw_gpt_legend_bar(canvas: np.ndarray) -> None:
    h, w = canvas.shape[:2]
    bar_h = 34
    y0 = max(0, h - bar_h)
    sub = canvas[y0:h, 0:w]
    bg = np.full_like(sub, (12, 18, 32))
    cv2.addWeighted(bg, 0.84, sub, 0.16, 0, sub)
    items = [
        ('plug_tip', _GPT_COLORS['plug_tip']),
        ('plug_axis->tip', _GPT_COLORS['plug_axis_point']),
        ('plug_orient', _GPT_COLORS['plug_orient_point']),
        ('hole_center', _GPT_COLORS['hole_center']),
        ('hole_insert', _GPT_COLORS['hole_insert_point']),
        ('hole_orient', _GPT_COLORS['hole_orient_point']),
    ]
    x = 14
    cy = y0 + bar_h // 2
    for text_lbl, col in items:
        if x + 135 > w:
            break
        cv2.circle(canvas, (x + 6, cy), 5, col, -1, cv2.LINE_AA)
        cv2.putText(canvas, text_lbl, (x + 16, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (232, 240, 250), 1, cv2.LINE_AA)
        (tw, _), _ = cv2.getTextSize(text_lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
        x += tw + 34


def render_gpt_crop_overlay(
    image_bgr: np.ndarray,
    *,
    cam_label: str,
    det_2d: dict[str, Any],
    symmetry_deg: float = 0.0,
    out_size: tuple[int, int] = (1280, 720),
) -> np.ndarray:
    """Render a dual-focus 16:9 split zoom (Left: Plug 3 pts, Right: Hole 3 pts) at true ~2.8x magnification."""
    raw = np.asarray(image_bgr, dtype=np.uint8)
    h, w = raw.shape[:2]
    out_w, out_h = int(out_size[0]), int(out_size[1])
    top_h = max(36, int(out_h * 0.042))
    bot_h = 34
    body_h = out_h - top_h - bot_h
    gap = 6
    half_w = (out_w - gap) // 2

    canvas = np.full((out_h, out_w, 3), (12, 18, 32), dtype=np.uint8)
    all_info = _extract_gpt_points(det_2d)
    plug_info = [it for it in all_info if it[0].startswith('plug_')]
    hole_info = [it for it in all_info if it[0].startswith('hole_')]

    def _render_subpanel(panel_w: int, panel_h: int, subset_info, title_prefix: str) -> np.ndarray:
        aspect = panel_w / float(panel_h)
        x0, y0, cw, ch = _compute_gpt_roi(w, h, [p[2] for p in subset_info], aspect_wh=aspect, min_w=220, pad_scale=2.3)
        tile = cv2.resize(raw[y0:y0 + ch, x0:x0 + cw], (panel_w, panel_h), interpolation=cv2.INTER_CUBIC)
        sx = panel_w / float(cw)
        sy = panel_h / float(ch)

        def _map_tile(uv: tuple[float, float]) -> tuple[int, int]:
            return int(round((uv[0] - x0) * sx)), int(round((uv[1] - y0) * sy))

        _draw_gpt_vectors_and_points(tile, det_2d, _map_tile, thickness=3, tip_px=18.0)
        badges = [
            (_map_tile(uv), f'{short} [{int(round(uv[0]))}, {int(round(uv[1]))}]', col)
            for _k, short, uv, col, _prim in subset_info
        ]
        _draw_callout_badges(tile, badges, (8, 34, panel_w - 8, panel_h - 8), font_scale=0.55)
        hdr = tile[0:28, 0:panel_w]
        cv2.addWeighted(np.full_like(hdr, (15, 23, 42)), 0.85, hdr, 0.15, 0, hdr)
        cv2.putText(
            tile,
            f'{title_prefix} Zoom x{sx:.1f}  crop=[{x0}:{x0+cw}, {y0}:{y0+ch}]',
            (10, 19),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (191, 219, 254),
            1,
            cv2.LINE_AA,
        )
        return tile

    left_tile = _render_subpanel(half_w, body_h, plug_info, 'PLUG (3 pts)')
    right_w = out_w - half_w - gap
    right_tile = _render_subpanel(right_w, body_h, hole_info, 'HOLE (3 pts)')
    canvas[top_h:top_h + body_h, 0:half_w] = left_tile
    canvas[top_h:top_h + body_h, half_w + gap:out_w] = right_tile
    cv2.rectangle(canvas, (0, top_h), (half_w - 1, top_h + body_h - 1), (255, 200, 40), 1, cv2.LINE_AA)
    cv2.rectangle(canvas, (half_w + gap, top_h), (out_w - 1, top_h + body_h - 1), (0, 205, 255), 1, cv2.LINE_AA)

    hud = f'{cam_label} | GPT-6.1-sol Dual Zoom (Plug + Hole) | symmetry_deg={symmetry_deg:g}'
    _draw_banner(canvas, hud, ok=True)
    _draw_gpt_legend_bar(canvas)
    return canvas


def render_gpt_output_overlay(
    image_bgr: np.ndarray,
    *,
    cam_label: str,
    det_2d: dict[str, Any],
    symmetry_deg: float = 0.0,
) -> np.ndarray:
    """Draw GPT's raw 6 [u,v] points, 4 vectors, and coordinate callouts cleanly on the full 1920x1080 frame."""
    raw = np.asarray(image_bgr, dtype=np.uint8)
    canvas = raw.copy()
    h, w = canvas.shape[:2]
    pts_info = _extract_gpt_points(det_2d)

    _draw_gpt_vectors_and_points(
        canvas, det_2d, lambda uv: (int(round(uv[0])), int(round(uv[1]))), thickness=3, tip_px=16.0,
    )
    top_y = max(46, int(h * 0.045) + 8)
    badge_entries = [
        ((int(round(uv[0])), int(round(uv[1]))), f'{short} [{int(round(uv[0]))}, {int(round(uv[1]))}]', col)
        for _k, short, uv, col, _prim in pts_info
    ]
    _draw_callout_badges(canvas, badge_entries, (10, top_y, w - 10, h - 42), font_scale=0.60)

    pt_tip = _xy_int(det_2d['plug_tip'])
    pt_hole = _xy_int(det_2d['hole_center'])
    hud = (
        f'{cam_label} | GPT-6.1-sol Output | symmetry={symmetry_deg:g} deg '
        f'| plug_tip=[{pt_tip[0]}, {pt_tip[1]}] | hole_center=[{pt_hole[0]}, {pt_hole[1]}]'
    )
    _draw_banner(canvas, hud, ok=True)
    _draw_gpt_legend_bar(canvas)
    return canvas


def render_perception_overlay(
    image_bgr: np.ndarray,
    *,
    cam_label: str,
    det_2d: dict[str, Any],
    camera_matrix: Sequence[float] | np.ndarray,
    distortion: Sequence[float] | np.ndarray,
    t_base_cam: np.ndarray,
    p_hole_b: Sequence[float] | np.ndarray,
    a_insert_b: Sequence[float] | np.ndarray,
    p_pre_b: Sequence[float] | np.ndarray,
    p_tip_vis_b: Sequence[float] | np.ndarray,
    hole_gap_mm: float,
    tip_gap_mm: float,
) -> np.ndarray:
    """Draw GPT 2D detections + 3D arm-frame reprojected hole/axis/pre-insert/tip markers."""
    raw = np.asarray(image_bgr, dtype=np.uint8)
    canvas = raw.copy()
    h, w = canvas.shape[:2]
    p_hole = np.asarray(p_hole_b, dtype=np.float64).reshape(3)
    a_ins = np.asarray(a_insert_b, dtype=np.float64).reshape(3)
    p_pre = np.asarray(p_pre_b, dtype=np.float64).reshape(3)
    p_tip = np.asarray(p_tip_vis_b, dtype=np.float64).reshape(3)

    _draw_gpt_vectors_and_points(canvas, det_2d, lambda uv: (int(round(uv[0])), int(round(uv[1]))), thickness=2, tip_px=13.0)

    pts_3d = np.stack([
        p_hole,
        p_hole + 0.045 * a_ins,
        p_pre,
        p_tip,
    ], axis=0)
    uv_reproj = project_base_points(pts_3d, camera_matrix, distortion, t_base_cam)
    rh = tuple(int(round(v)) for v in uv_reproj[0])
    ra = tuple(int(round(v)) for v in uv_reproj[1])
    rp = tuple(int(round(v)) for v in uv_reproj[2])
    rt = tuple(int(round(v)) for v in uv_reproj[3])

    _draw_outlined_arrow(canvas, rh, ra, (30, 70, 255), thickness=3, tip_px=16.0)
    cv2.drawMarker(canvas, rh, (60, 235, 90), cv2.MARKER_CROSS, 24, 3, cv2.LINE_AA)
    cv2.drawMarker(canvas, rp, (255, 80, 220), cv2.MARKER_DIAMOND, 18, 2, cv2.LINE_AA)
    cv2.drawMarker(canvas, rt, (255, 255, 80), cv2.MARKER_TILTED_CROSS, 18, 2, cv2.LINE_AA)

    pts_info = _extract_gpt_points(det_2d)
    hole_only = [p[2] for p in pts_info if p[0].startswith('hole_')] + [
        (float(uv_reproj[0, 0]), float(uv_reproj[0, 1])),
        (float(uv_reproj[1, 0]), float(uv_reproj[1, 1])),
        (float(uv_reproj[2, 0]), float(uv_reproj[2, 1])),
    ]
    inset_w = min(max(350, int(round(w * 0.35))), w - 28)
    inset_h = min(max(230, int(round(inset_w * 0.68))), h - 104)
    us_h = [p[0] for p in hole_only]
    vs_h = [p[1] for p in hole_only]
    ch_x, ch_y = 0.5 * (min(us_h) + max(us_h)), 0.5 * (min(vs_h) + max(vs_h))
    sp_u, sp_v = max(30.0, max(us_h) - min(us_h)), max(30.0, max(vs_h) - min(vs_h))
    asp_in = inset_w / float(inset_h)
    rcw = max(64, min(w, int(round(max(220.0, sp_u * 1.8 + 70.0, (sp_v * 1.8 + 60.0) * asp_in)))))
    rch = max(48, min(h, int(round(rcw / asp_in))))
    rcw = max(64, min(w, int(round(rch * asp_in))))
    rx0 = max(0, min(w - rcw, int(round(ch_x - 0.5 * rcw))))
    ry0 = max(0, min(h - rch, int(round(ch_y - 0.5 * rch))))
    rcx, rcy = rx0 + 0.5 * rcw, ry0 + 0.5 * rch
    top_y = max(46, int(h * 0.048) + 10)
    bot_y = h - 44 - inset_h
    ix0, iy0 = max(
        [(16, max(top_y, bot_y)), (w - inset_w - 16, max(top_y, bot_y)), (16, top_y), (w - inset_w - 16, top_y)],
        key=lambda c: math.hypot((c[0] + 0.5 * inset_w) - rcx, (c[1] + 0.5 * inset_h) - rcy),
    )
    ix1, iy1 = ix0 + inset_w, iy0 + inset_h
    inset = cv2.resize(raw[ry0:ry0 + rch, rx0:rx0 + rcw], (inset_w, inset_h), interpolation=cv2.INTER_CUBIC)
    sx, sy = inset_w / float(rcw), inset_h / float(rch)

    def _map_in(uv: tuple[float, float]) -> tuple[int, int]:
        return int(round((uv[0] - rx0) * sx)), int(round((uv[1] - ry0) * sy))

    _draw_gpt_vectors_and_points(inset, det_2d, _map_in, thickness=2, tip_px=14.0)
    ih = _map_in((float(uv_reproj[0, 0]), float(uv_reproj[0, 1])))
    ia = _map_in((float(uv_reproj[1, 0]), float(uv_reproj[1, 1])))
    ip = _map_in((float(uv_reproj[2, 0]), float(uv_reproj[2, 1])))
    it = _map_in((float(uv_reproj[3, 0]), float(uv_reproj[3, 1])))
    _draw_outlined_arrow(inset, ih, ia, (30, 70, 255), thickness=3, tip_px=16.0)
    cv2.drawMarker(inset, ih, (60, 235, 90), cv2.MARKER_CROSS, 22, 2, cv2.LINE_AA)
    cv2.drawMarker(inset, ip, (255, 80, 220), cv2.MARKER_DIAMOND, 16, 2, cv2.LINE_AA)
    cv2.drawMarker(inset, it, (255, 255, 80), cv2.MARKER_TILTED_CROSS, 16, 2, cv2.LINE_AA)
    _draw_callout_badges(
        inset,
        [
            (ih, '3D Hole', (60, 235, 90)),
            (ip, '-0.5cm Pre', (255, 80, 220)),
            (_map_in(_xy_float(det_2d['hole_center'])), 'GPT hole_ctr', _GPT_COLORS['hole_center']),
            (_map_in(_xy_float(det_2d['hole_insert_point'])), 'GPT hole_ins', _GPT_COLORS['hole_insert_point']),
        ],
        (6, 30, inset_w - 6, inset_h - 6),
        font_scale=0.47,
    )
    hdr = inset[0:26, 0:inset_w]
    cv2.addWeighted(np.full_like(hdr, (15, 23, 42)), 0.85, hdr, 0.15, 0, hdr)
    cv2.putText(inset, f'Hole 3D Reproj Zoom x{sx:.1f}', (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (134, 239, 172), 1, cv2.LINE_AA)
    canvas[iy0:iy1, ix0:ix1] = inset
    cv2.rectangle(canvas, (ix0, iy0), (ix1, iy1), (60, 235, 90), 2, cv2.LINE_AA)

    hud = (
        f'{cam_label} | Hole B(mm): [{p_hole[0]*1000:.1f}, {p_hole[1]*1000:.1f}, {p_hole[2]*1000:.1f}] '
        f'| Axis: [{a_ins[0]:+.2f}, {a_ins[1]:+.2f}, {a_ins[2]:+.2f}] '
        f'| Gap(Hole/Tip): {hole_gap_mm:.1f}/{tip_gap_mm:.1f}mm'
    )
    _draw_banner(canvas, hud, ok=True)
    _draw_gpt_legend_bar(canvas)
    return canvas


def _draw_banner(image: np.ndarray, text: str, *, ok: bool = True) -> None:
    h, w = image.shape[:2]
    bar_h = max(36, int(h * 0.042))
    overlay = image.copy()
    color = (22, 78, 48) if ok else (36, 36, 120)
    cv2.rectangle(overlay, (0, 0), (w, bar_h), color, -1)
    cv2.addWeighted(overlay, 0.78, image, 0.22, 0, image)
    cv2.putText(
        image,
        text,
        (14, int(bar_h * 0.70)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.68,
        (245, 250, 255),
        2,
        cv2.LINE_AA,
    )


def save_extrinsics_file(path: Path, cameras_data: dict[str, Any], arm_id: str = 'arm_4') -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    clean_cams: dict[str, Any] = {}
    for key, val in cameras_data.items():
        entry = {k: v for k, v in val.items() if k != 'T_base_cam'}
        clean_cams[key] = entry
    payload = {
        'version': 1,
        'arm': arm_id,
        'base_frame': 'base_link',
        'updated_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'cameras': clean_cams,
    }
    path.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding='utf-8')


def load_extrinsics_file(path: Path) -> dict[str, dict[str, Any]]:
    if not path.is_file():
        return {}
    raw = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    out: dict[str, dict[str, Any]] = {}
    for key, entry in (raw.get('cameras') or {}).items():
        try:
            mat = pose_dict_to_mat(entry)
        except Exception:
            continue
        item = dict(entry)
        item['T_base_cam'] = mat
        out[str(key)] = item
    return out
