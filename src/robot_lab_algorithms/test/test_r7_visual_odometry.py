"""Feature-based RGB-D visual odometry (robot_lab_algorithms.visual_odometry).

A ray-cast room with a random texture on its walls and floor gives grey and
depth frames for a known camera path; every detector must recover it.
"""
import math
import os
import sys
import unittest
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from robot_lab_algorithms.visual_odometry import (  # noqa: E402
    DETECTORS, RgbdOdometry, base_motion, decode_gray, matrix_quaternion, rotation_angle, transform_matrix)

WIDTH, HEIGHT = 320, 240
FX = 160.0 / math.tan(0.625)   # the OAK-D: 1.25 rad horizontal field of view
CX, CY = (WIDTH - 1) / 2.0, (HEIGHT - 1) / 2.0

# Planes (normal, offset, texture axes) in the first camera's optical frame:
# a wall 4 m ahead, the floor 0.5 m below, a wall 2 m to the left.
PLANES = [((0, 0, 1), 4.0, (1, 0, 0), (0, 1, 0)),
          ((0, 1, 0), 0.5, (1, 0, 0), (0, 0, 1)),
          ((1, 0, 0), -2.0, (0, 0, 1), (0, 1, 0))]


def _texture():
    from scipy.ndimage import gaussian_filter
    noise = gaussian_filter(np.random.default_rng(3).random((1000, 1000)), 2.0)
    noise = (noise - noise.min()) / (noise.max() - noise.min())
    return noise * 255.0


TEXTURE = _texture()


def render(pose):
    """Grey (uint8) and depth (float32 metres) seen from a camera pose."""
    from scipy.ndimage import map_coordinates
    u, v = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
    rays = np.stack([(u - CX) / FX, (v - CY) / FX, np.ones(u.shape)], axis=-1) @ pose[:3, :3].T
    origin = pose[:3, 3]
    depth = np.full(u.shape, np.inf)
    gray = np.zeros(u.shape)
    for normal, offset, axis_a, axis_b in PLANES:
        normal = np.array(normal, dtype=float)
        with np.errstate(divide='ignore', invalid='ignore'):
            s = (offset - normal @ origin) / (rays @ normal)
        nearer = np.isfinite(s) & (s > 0) & (s < depth)
        hit = origin + s[..., None] * rays
        texel = map_coordinates(TEXTURE, [hit @ np.array(axis_a, float) * 100.0,
                                          hit @ np.array(axis_b, float) * 100.0], order=1, mode='wrap')
        depth = np.where(nearer, s, depth)
        gray = np.where(nearer, texel, gray)
    return gray.astype(np.uint8), depth.astype(np.float32)


def path(steps, forward=0.02, yaw_deg=1.0):
    """Camera poses: forward along optical z while turning about optical y (vertical)."""
    poses, pose = [], np.eye(4)
    step = np.eye(4)
    a = math.radians(yaw_deg)
    step[:3, :3] = [[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]]
    step[2, 3] = forward
    for _ in range(steps):
        poses.append(pose.copy())
        pose = pose @ step
    return poses


class VisualOdometryTests(unittest.TestCase):

    def test_every_detector_recovers_a_known_camera_path(self):
        truth = path(16)
        frames = [render(p) for p in truth]
        for detector in DETECTORS:
            with self.subTest(detector=detector):
                odometry = RgbdOdometry((FX, FX, CX, CY), detector=detector, max_features=800)
                for gray, depth in frames:
                    pose, stats = odometry.step(gray, depth)
                error = np.linalg.inv(truth[-1]) @ pose
                self.assertEqual(odometry.lost, 0)
                self.assertLess(np.linalg.norm(error[:3, 3]), 0.02)   # over 0.30 m and 15 deg
                self.assertLess(math.degrees(rotation_angle(error[:3, :3])), 0.5)

    def test_a_standstill_keeps_the_keyframe_and_the_pose(self):
        gray, depth = render(np.eye(4))
        odometry = RgbdOdometry((FX, FX, CX, CY))
        for _ in range(5):
            pose, stats = odometry.step(gray, depth)
        self.assertFalse(stats['keyframe'])
        self.assertLess(np.linalg.norm(pose[:3, 3]), 1e-3)

    def test_a_blank_view_is_lost_and_the_pose_held(self):
        gray, depth = render(np.eye(4))
        odometry = RgbdOdometry((FX, FX, CX, CY))
        odometry.step(gray, depth)
        pose, stats = odometry.step(np.full_like(gray, 128), depth)
        self.assertFalse(stats['tracked'])
        self.assertEqual(odometry.lost, 1)
        np.testing.assert_allclose(pose, np.eye(4))


class FrameTests(unittest.TestCase):

    def test_quaternion_round_trip(self):
        for q in [(0, 0, 0, 1), (0.5, -0.5, 0.5, 0.5), (0.1, 0.7, -0.2, 0.68)]:
            q = np.array(q) / np.linalg.norm(q)
            back = matrix_quaternion(transform_matrix((0, 0, 0), q)[:3, :3])
            self.assertAlmostEqual(abs(float(np.dot(back, q))), 1.0, places=9)

    def test_camera_moving_along_its_axis_moves_the_base_forward(self):
        # base <- optical frame of a camera 0.2 m up, looking forward.
        base_from_camera = transform_matrix((0.1, 0.0, 0.2), (-0.5, 0.5, -0.5, 0.5))
        camera_pose = np.eye(4)
        camera_pose[2, 3] = 1.0   # one metre along optical z
        np.testing.assert_allclose(base_motion(camera_pose, base_from_camera)[:3, 3], [1.0, 0.0, 0.0], atol=1e-12)

    def test_colour_encodings_decode_to_grey(self):
        rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        rgb[..., 0] = 200   # red
        padded = np.zeros((2, 8), dtype=np.uint8)
        padded[:, :6] = rgb.reshape(2, 6)
        msg = SimpleNamespace(encoding='rgb8', height=2, width=2, step=8, data=padded.tobytes())
        self.assertTrue(np.all(decode_gray(msg) == 60))   # 0.299 * 200
        msg = SimpleNamespace(encoding='bgr8', height=2, width=2, step=8, data=padded.tobytes())
        self.assertTrue(np.all(decode_gray(msg) == 23))   # 0.114 * 200
        with self.assertRaises(ValueError):
            decode_gray(SimpleNamespace(encoding='32FC1', height=1, width=1, step=4, data=bytes(4)))


if __name__ == '__main__':
    unittest.main()
