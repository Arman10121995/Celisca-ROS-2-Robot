"""Harmonic RGB-D messages -> ROS images and camera info (no Gazebo needed)."""
import struct
from types import SimpleNamespace

import pytest

rclpy_time = pytest.importorskip('builtin_interfaces.msg')
from robot_lab_adapter.gz_camera_bridge import ros_camera_info, ros_image  # noqa: E402

STAMP = rclpy_time.Time(sec=5, nanosec=7)


def _image(fmt, width, height, step, data):
    return SimpleNamespace(pixel_format_type=fmt, width=width, height=height, step=step, data=data)


def test_colour_and_depth_images_keep_their_bytes():
    rgb = ros_image(_image(3, 2, 1, 6, bytes(range(6))), 'cam', STAMP)
    assert (rgb.encoding, rgb.step, bytes(rgb.data)) == ('rgb8', 6, bytes(range(6)))
    depth = ros_image(_image(13, 2, 1, 0, struct.pack('<2f', 1.5, 2.0)), 'cam', STAMP)
    assert depth.encoding == '32FC1' and depth.step == 8
    assert struct.unpack('<2f', bytes(depth.data)) == (1.5, 2.0)
    assert depth.header.frame_id == 'cam' and depth.header.stamp == STAMP


def test_unknown_or_truncated_images_are_rejected():
    assert ros_image(_image(0, 2, 1, 6, bytes(6)), 'cam', STAMP) is None
    assert ros_image(_image(3, 2, 2, 6, bytes(6)), 'cam', STAMP) is None


def test_camera_info_carries_the_pinhole_model():
    k = [216.2, 0, 160, 0, 216.2, 120, 0, 0, 1]
    raw = SimpleNamespace(width=320, height=240, intrinsics=SimpleNamespace(k=k),
                          projection=SimpleNamespace(p=[]), HasField=lambda _name: False)
    info = ros_camera_info(raw, 'cam', STAMP)
    assert list(info.k) == pytest.approx(k)
    assert list(info.p) == pytest.approx([216.2, 0, 160, 0, 0, 216.2, 120, 0, 0, 0, 1, 0])
    assert info.distortion_model == 'plumb_bob' and (info.width, info.height) == (320, 240)
