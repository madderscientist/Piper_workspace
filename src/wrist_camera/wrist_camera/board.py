"""ChArUco 板：把 OpenCV 的 aruco 接口收敛在这一个文件里（兼容 OpenCV 4.5.x 与 4.7+/4.12）。"""

from __future__ import annotations

from dataclasses import dataclass, field
import re
from typing import Any

import cv2
import numpy as np

_MIN_POSE_CORNERS = 6
_CAPACITIES = (50, 100, 250, 1000)
_FAST_MARKER_MAX_WIDTH = 960


def resolve_dictionary_name(name: str, needed_markers: int = 0) -> str:
    """支持精确字典名（如 DICT_5X5_100）或家族名（如 DICT_5X5 / ArUco DICT_5X5）。"""
    raw = str(name or '').strip().upper()
    raw = re.sub(r'^ARUCO[\s_-]*', '', raw)
    raw = re.sub(r'\s+', '_', raw)
    if not raw:
        raise ValueError('字典名称不能为空')
    key = raw if raw.startswith('DICT_') else f'DICT_{raw}'
    if getattr(cv2.aruco, key, None) is not None:
        return key
    if re.fullmatch(r'DICT_[4567]X[4567]', key):
        target = max(1, int(needed_markers))
        for cap in _CAPACITIES:
            candidate = f'{key}_{cap}'
            if cap >= target and getattr(cv2.aruco, candidate, None) is not None:
                return candidate
        return f'{key}_1000'
    raise ValueError(f'OpenCV 不认识字典 {name}')


def dictionary_id(name: str, needed_markers: int = 0) -> int:
    key = resolve_dictionary_name(name, needed_markers=needed_markers)
    value = getattr(cv2.aruco, key, None)
    if value is None:
        raise ValueError(f'OpenCV 不认识字典 {name}')
    return int(value)


def _get_dictionary(dict_id: int):
    aruco: Any = cv2.aruco
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        return aruco.getPredefinedDictionary(dict_id)
    return aruco.Dictionary_get(dict_id)


def _create_charuco_board(squares_x: int, squares_y: int, square_size: float,
                          marker_size: float, dictionary):
    aruco: Any = cv2.aruco
    if hasattr(cv2.aruco, 'CharucoBoard_create'):
        return aruco.CharucoBoard_create(
            squares_x, squares_y, square_size, marker_size, dictionary)
    return aruco.CharucoBoard(
        (squares_x, squares_y), square_size, marker_size, dictionary)


def _create_detector_params():
    aruco: Any = cv2.aruco
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        params = aruco.DetectorParameters_create()
    else:
        params = aruco.DetectorParameters()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    return params


@dataclass
class Detection:
    """一张图上的检测结果。corners/ids 是 charuco 角点，marker_* 只用于画叠加图。"""

    corners: np.ndarray                       # (N, 1, 2) float32
    ids: np.ndarray                           # (N, 1) int32
    size: tuple[int, int]                     # (width, height)
    marker_count: int = 0
    marker_corners: list = field(default_factory=list)
    marker_ids: np.ndarray | None = None

    @property
    def count(self) -> int:
        return 0 if self.ids is None else int(len(self.ids))

    def coverage(self) -> float:
        if self.count < 3:
            return 0.0
        hull = cv2.convexHull(self.corners.reshape(-1, 2).astype(np.float32))
        area = float(cv2.contourArea(hull))
        return area / float(self.size[0] * self.size[1])

    def to_json(self) -> dict:
        return {
            'corners': self.corners.reshape(-1, 2).tolist(),
            'ids': self.ids.reshape(-1).tolist(),
            'width': self.size[0],
            'height': self.size[1],
            'marker_count': self.marker_count,
        }

    @staticmethod
    def from_json(data: dict) -> 'Detection':
        corners = np.asarray(data['corners'], np.float32).reshape(-1, 1, 2)
        ids = np.asarray(data['ids'], np.int32).reshape(-1, 1)
        return Detection(corners, ids, (int(data['width']), int(data['height'])),
                         int(data.get('marker_count', 0)))


class Board:
    """一块 ChArUco 板。所有尺寸来自 config/board.yaml 或页面配置。"""

    def __init__(self, squares_x: int, squares_y: int, square_size: float,
                 marker_size: float, dictionary: str) -> None:
        self.squares_x = int(squares_x)
        self.squares_y = int(squares_y)
        if self.squares_x < 3 or self.squares_y < 3:
            raise ValueError('标定板格数 squares_x 和 squares_y 至少为 3')
        self.square_size = float(square_size)
        self.marker_size = float(marker_size)
        if self.square_size <= 0 or self.marker_size <= 0:
            raise ValueError('方格边长与 marker 边长必须为正数')
        if self.marker_size >= self.square_size:
            raise ValueError('marker 边长必须小于方格边长')
        needed = self.squares_x * self.squares_y // 2
        self.dictionary_name = resolve_dictionary_name(dictionary, needed_markers=needed)
        self._dictionary = _get_dictionary(dictionary_id(self.dictionary_name, needed_markers=needed))
        capacity = int(self._dictionary.bytesList.shape[0])
        if capacity < needed:
            raise ValueError(
                f'{self.dictionary_name} 只有 {capacity} 个 marker，{self.squares_x}x{self.squares_y} '
                f'的板需要 {needed} 个')
        self._board = _create_charuco_board(
            self.squares_x, self.squares_y, self.square_size, self.marker_size,
            self._dictionary)
        self._params = _create_detector_params()
        self._charuco_detector = None
        if hasattr(cv2.aruco, 'CharucoDetector'):
            charuco_params = cv2.aruco.CharucoParameters()
            charuco_params.tryRefineMarkers = True
            self._charuco_detector = cv2.aruco.CharucoDetector(
                self._board, charuco_params, self._params)

    @classmethod
    def from_config(cls, config: dict) -> 'Board':
        section = config['board'] if 'board' in config else config
        return cls(section['squares_x'], section['squares_y'], section['square_size'],
                   section['marker_size'], section['dictionary'])

    @property
    def raw(self):
        return self._board

    @property
    def corner_count(self) -> int:
        return (self.squares_x - 1) * (self.squares_y - 1)

    @property
    def object_points_all(self) -> np.ndarray:
        board: Any = self._board
        if hasattr(self._board, 'getChessboardCorners'):
            return np.asarray(board.getChessboardCorners(), np.float32)
        return np.asarray(board.chessboardCorners, np.float32)

    def object_points(self, ids) -> np.ndarray:
        index = np.asarray(ids, np.int32).reshape(-1)
        return self.object_points_all[index]

    def describe(self) -> dict:
        return {
            'squares_x': self.squares_x,
            'squares_y': self.squares_y,
            'square_size': self.square_size,
            'marker_size': self.marker_size,
            'square_size_mm': round(self.square_size * 1000.0, 4),
            'marker_size_mm': round(self.marker_size * 1000.0, 4),
            'dictionary': self.dictionary_name,
            'corner_count': self.corner_count,
            'marker_total': self.squares_x * self.squares_y // 2,
            'width_m': round(self.squares_x * self.square_size, 6),
            'height_m': round(self.squares_y * self.square_size, 6),
        }

    def detect(self, image) -> Detection:
        gray = _gray(image)
        height, width = gray.shape[:2]
        empty = Detection(np.zeros((0, 1, 2), np.float32), np.zeros((0, 1), np.int32),
                          (width, height))
        if width > _FAST_MARKER_MAX_WIDTH:
            fast_det = self._detect_coarse_to_fine(gray, width, height)
            if fast_det is not None:
                return fast_det
        if self._charuco_detector is not None:
            charuco_corners, charuco_ids, marker_corners, marker_ids = (
                self._charuco_detector.detectBoard(gray)
            )
            if marker_ids is None or len(marker_ids) == 0:
                return empty
            if charuco_ids is None or len(charuco_ids) == 0:
                empty.marker_count = int(len(marker_ids))
                empty.marker_corners = list(marker_corners)
                empty.marker_ids = marker_ids
                return empty
            return Detection(
                np.asarray(charuco_corners, np.float32).reshape(-1, 1, 2),
                np.asarray(charuco_ids, np.int32).reshape(-1, 1),
                (width, height), int(len(marker_ids)), list(marker_corners), marker_ids)

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self._dictionary, parameters=self._params)
        if ids is None or len(ids) == 0:
            return empty
        corners, ids, _, _ = cv2.aruco.refineDetectedMarkers(
            gray, self._board, corners, ids, rejected)
        if ids is None or len(ids) == 0:
            return empty
        count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self._board)
        if not count or charuco_ids is None:
            empty.marker_count = int(len(ids))
            empty.marker_corners = list(corners)
            empty.marker_ids = ids
            return empty
        return Detection(
            np.asarray(charuco_corners, np.float32).reshape(-1, 1, 2),
            np.asarray(charuco_ids, np.int32).reshape(-1, 1),
            (width, height), int(len(ids)), list(corners), ids)

    def _detect_coarse_to_fine(self, gray: np.ndarray, width: int, height: int) -> Detection | None:
        small_w = width // 2
        small_h = height // 2
        if small_w < 320 or small_h < 240:
            return None
        small = cv2.resize(gray, (small_w, small_h), interpolation=cv2.INTER_LINEAR)
        scale = np.array([[[width / float(small_w), height / float(small_h)]]], dtype=np.float32)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            small, self._dictionary, parameters=self._params)
        if ids is None or len(ids) == 0:
            return Detection(np.zeros((0, 1, 2), np.float32), np.zeros((0, 1), np.int32),
                             (width, height))
        corners, ids, _, _ = cv2.aruco.refineDetectedMarkers(
            small, self._board, corners, ids, rejected)
        if ids is None or len(ids) == 0:
            return Detection(np.zeros((0, 1, 2), np.float32), np.zeros((0, 1), np.int32),
                             (width, height))
        full_markers = [np.asarray(c, np.float32) * scale for c in corners]
        count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            full_markers, ids, gray, self._board)
        if not count or charuco_ids is None:
            return Detection(np.zeros((0, 1, 2), np.float32), np.zeros((0, 1), np.int32),
                             (width, height), int(len(ids)), full_markers, ids)
        return Detection(
            np.asarray(charuco_corners, np.float32).reshape(-1, 1, 2),
            np.asarray(charuco_ids, np.int32).reshape(-1, 1),
            (width, height), int(len(ids)), full_markers, ids)

    def estimate_pose(self, detection: Detection, camera_matrix, distortion):
        if detection.count < _MIN_POSE_CORNERS:
            return None
        from wrist_camera import transforms
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            detection.corners, detection.ids, self._board,
            np.asarray(camera_matrix, float).reshape(3, 3),
            np.asarray(distortion, float).reshape(-1, 1), np.empty((0, 0)), np.empty((0, 0)))
        if not ok:
            return None
        return transforms.rvec_tvec_to_matrix(rvec, tvec)

    def overlay(self, image, detection: Detection, *, inplace: bool = False):
        canvas = image if inplace else image.copy()
        scale_x = canvas.shape[1] / float(detection.size[0] or canvas.shape[1])
        scale_y = canvas.shape[0] / float(detection.size[1] or canvas.shape[0])
        if detection.marker_ids is not None and len(detection.marker_corners):
            marker_corners = (
                detection.marker_corners
                if abs(scale_x - 1.0) < 1e-3 and abs(scale_y - 1.0) < 1e-3
                else [np.asarray(c, np.float32) * np.array([[[scale_x, scale_y]]], np.float32)
                      for c in detection.marker_corners]
            )
            cv2.aruco.drawDetectedMarkers(canvas, marker_corners, detection.marker_ids)
        if detection.count:
            corners = (
                detection.corners
                if abs(scale_x - 1.0) < 1e-3 and abs(scale_y - 1.0) < 1e-3
                else detection.corners * np.array([[[scale_x, scale_y]]], np.float32)
            )
            cv2.aruco.drawDetectedCornersCharuco(
                canvas, corners, detection.ids, (0, 0, 255))
        return canvas

    def draw(self, pixels_per_meter: float = 4000.0, margin_squares: float = 0.5):
        width = int(round(self.squares_x * self.square_size * pixels_per_meter))
        height = int(round(self.squares_y * self.square_size * pixels_per_meter))
        margin = int(round(margin_squares * self.square_size * pixels_per_meter))
        out_size = (width + 2 * margin, height + 2 * margin)
        board: Any = self._board
        if hasattr(self._board, 'generateImage'):
            return board.generateImage(out_size, marginSize=margin, borderBits=1)
        return board.draw(out_size, None, margin, 1)


def _gray(image) -> np.ndarray:
    image = np.asarray(image)
    if image.ndim == 2:
        return image
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def _planar_residual(board: Board, detection: Detection) -> float:
    if detection.count < 6:
        return float('inf')
    source = board.object_points(detection.ids)[:, :2].astype(np.float32)
    target = detection.corners.reshape(-1, 2).astype(np.float32)
    matrix, _ = cv2.findHomography(source, target, 0)
    if matrix is None:
        return float('inf')
    projected = cv2.perspectiveTransform(source.reshape(-1, 1, 2), matrix).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum((projected - target) ** 2, axis=1))))


def probe(image, config: dict) -> list[dict]:
    section = config.get('probe', {})
    dictionaries = section.get('dictionaries', ['DICT_5X5_100'])
    orientations = section.get('orientations', [[9, 12], [12, 9]])
    base = config['board']
    results = []
    for name in dictionaries:
        for squares_x, squares_y in orientations:
            try:
                board = Board(squares_x, squares_y, base['square_size'],
                              base['marker_size'], name)
            except ValueError as error:
                results.append({'dictionary': name, 'squares_x': squares_x,
                                'squares_y': squares_y, 'ok': False,
                                'reason': str(error)})
                continue
            detection = board.detect(image)
            residual = _planar_residual(board, detection)
            results.append({
                'dictionary': board.dictionary_name, 'squares_x': squares_x, 'squares_y': squares_y,
                'ok': True, 'markers': detection.marker_count,
                'corners': detection.count, 'max_corners': board.corner_count,
                'residual_px': None if residual == float('inf') else round(residual, 3),
            })
    results.sort(key=lambda r: (not r.get('ok'), -(r.get('corners') or 0),
                                r.get('residual_px') if r.get('residual_px') is not None
                                else float('inf')))
    return results
