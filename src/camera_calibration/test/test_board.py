import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
import pytest

from camera_calibration import transforms
from camera_calibration.board import Board, Detection, probe
from conftest import BOARD_CONFIG, front_pose, render

K = np.array([[900.0, 0, 640.0], [0, 900.0, 360.0], [0, 0, 1.0]])
D = np.zeros(5)
SIZE = (1280, 720)


def test_geometry_and_dict_family_resolution(board):
    described = board.describe()
    assert described['squares_x'] == 9 and described['squares_y'] == 12
    assert described['square_size_mm'] == pytest.approx(30.0)
    assert described['marker_size_mm'] == pytest.approx(22.5)
    assert described['dictionary'] == 'DICT_5X5_100'
    assert described['corner_count'] == 88
    assert described['width_m'] == pytest.approx(0.27)
    assert described['height_m'] == pytest.approx(0.36)

    with pytest.raises(ValueError, match='54'):
        Board(9, 12, 0.030, 0.0225, 'DICT_5X5_50')
    with pytest.raises(ValueError):
        Board(9, 12, 0.030, 0.030, 'DICT_5X5')


def test_detect_pose_and_probe(board, flat):
    truth = front_pose(board, [0.25, -0.35, 0.12], [0.0, 0.0, 0.55])
    scene, expected = render(board, flat, truth, K, D, SIZE)

    detection = board.detect(scene)
    assert detection.count == board.corner_count
    assert detection.marker_count == 54
    pose = board.estimate_pose(detection, K, D)
    angle, distance = transforms.transform_delta(truth, pose)
    assert angle < 1.0 and distance < 3e-3

    all_results = probe(scene, BOARD_CONFIG)
    best = all_results[0]
    assert (best['squares_x'], best['squares_y']) == (9, 12)
    assert best['corners'] == board.corner_count and best['residual_px'] < 2.0
    wrong = [r for r in all_results if r.get('ok') and (r['squares_x'], r['squares_y']) == (12, 9)][0]
    assert wrong['corners'] == 0 or wrong['residual_px'] > 10 * best['residual_px']
