"""Chessboard camera calibration (robot_lab_algorithms.camera_calibration).

A ray-cast chessboard seen from known poses by a camera with known
intrinsics: the calibration must recover them, half-pixel principal point
included.
"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from robot_lab_algorithms.camera_calibration import (  # noqa: E402
    ChessboardCalibrator, board_points, camera_info_yaml, parse_pattern, view_signature)

WIDTH, HEIGHT = 320, 240
FX = 160.0 / math.tan(0.625)
CX, CY = (WIDTH - 1) / 2.0, (HEIGHT - 1) / 2.0
PATTERN, SQUARE = (8, 6), 0.05


def rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def render(board_rotation, board_origin, samples=3):
    """Grey image of the board (camera frame pose of its first inner corner),
    supersampled ``samples`` x ``samples`` per pixel."""
    columns, rows = PATTERN
    offsets = (np.arange(samples) + 0.5) / samples - 0.5
    total = np.zeros((HEIGHT, WIDTH))
    u, v = np.meshgrid(np.arange(WIDTH, dtype=float), np.arange(HEIGHT, dtype=float))
    normal = board_rotation[:, 2]
    for du in offsets:
        for dv in offsets:
            rays = np.stack([(u + du - CX) / FX, (v + dv - CY) / FX, np.ones(u.shape)], axis=-1)
            s = (normal @ board_origin) / (rays @ normal)
            local = (s[..., None] * rays - board_origin) @ board_rotation   # board-frame hit points
            bx, by = local[..., 0] / SQUARE + 1.0, local[..., 1] / SQUARE + 1.0   # squares from the corner
            on_board = (bx >= 0) & (bx < columns + 1) & (by >= 0) & (by < rows + 1)
            margin = (bx >= -1) & (bx < columns + 2) & (by >= -1) & (by < rows + 2)
            dark = (np.floor(bx) + np.floor(by)) % 2 == 0
            value = np.where(on_board, np.where(dark, 20.0, 235.0), np.where(margin, 235.0, 120.0))
            total += np.where(s > 0, value, 120.0)
    return (total / samples ** 2).astype(np.uint8)


def views(count=20, seed=4):
    rng = np.random.default_rng(seed)
    centre = np.array([(PATTERN[0] - 1) * SQUARE / 2, (PATTERN[1] - 1) * SQUARE / 2, 0.0])
    for _ in range(count):
        r = rotation(*np.radians(rng.uniform([-35, -35, -20], [35, 35, 20])))
        distance = rng.uniform(0.55, 0.9)
        # Board centre somewhere in the view, at that distance.
        target = np.array([rng.uniform(-0.25, 0.25) * distance, rng.uniform(-0.2, 0.2) * distance, distance])
        yield render(r, target - r @ centre)


class CalibrationTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.calibrator = ChessboardCalibrator(PATTERN, SQUARE, min_views=12, max_views=40, min_difference=0.05)
        cls.outcomes = [cls.calibrator.add(gray) for gray in views()]

    def test_distinct_views_are_kept(self):
        self.assertGreaterEqual(self.outcomes.count('kept'), 12, self.outcomes)
        self.assertTrue(self.calibrator.ready())

    def test_intrinsics_are_recovered(self):
        result = self.calibrator.calibrate()
        k = result['camera_matrix']
        self.assertLess(result['rms_px'], 0.3)
        self.assertLess(abs(k[0, 0] / FX - 1), 0.01)
        self.assertLess(abs(k[1, 1] / FX - 1), 0.01)
        self.assertLess(abs(k[0, 2] - CX), 1.0)
        self.assertLess(abs(k[1, 2] - CY), 1.0)
        self.assertLess(np.abs(result['distortion']).max(), 0.1)
        text = camera_info_yaml('oakd', result)
        self.assertIn('image_width: 320', text)
        self.assertIn('distortion_model: plumb_bob', text)

    def test_a_blank_image_has_no_board(self):
        calibrator = ChessboardCalibrator(PATTERN, SQUARE)
        self.assertEqual(calibrator.add(np.full((HEIGHT, WIDTH), 128, np.uint8)), 'no board')


class HelperTests(unittest.TestCase):

    def test_pattern_and_board_points(self):
        self.assertEqual(parse_pattern('8x6'), (8, 6))
        with self.assertRaises(ValueError):
            parse_pattern('2x6')
        points = board_points((3, 2), 0.1)
        np.testing.assert_allclose(points[[0, 2, 3]], [[0, 0, 0], [0.2, 0, 0], [0, 0.1, 0]], atol=1e-7)

    def test_signature_of_a_centred_upright_board(self):
        columns, rows = 3, 3
        grid = np.array([[100 + 60 * c, 60 + 60 * r] for r in range(rows) for c in range(columns)], float)
        x, y, size, skew = view_signature(grid, (columns, rows), (320, 240))
        self.assertAlmostEqual(x, 0.5)
        self.assertAlmostEqual(y, 0.5)
        self.assertAlmostEqual(size, math.sqrt(120 * 120 / (320 * 240)))
        self.assertAlmostEqual(skew, 0.0)


if __name__ == '__main__':
    unittest.main()
