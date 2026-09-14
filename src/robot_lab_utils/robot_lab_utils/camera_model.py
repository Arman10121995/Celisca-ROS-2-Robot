"""Pinhole RGB-D camera model shared by the PyBullet, MuJoCo and Isaac bridges.

Gazebo renders Bumperbot's OAK-D from the ``<sensor type="rgbd_camera">``
block in ``description_common/urdf/oakd_camera.xacro`` (320x240, horizontal
field of view 1.25 rad, clip 0.3-100 m) and bridges it to
``/oakd/rgb/image_raw``, ``/oakd/depth/image_raw`` and
``/oakd/rgb/camera_info`` in ``oakd_rgb_camera_optical_frame``.  The other
backends render the same camera from the same link, so the parts that have
to agree live here: intrinsics, the vertical field of view renderers are
configured with, depth-buffer linearisation, and the rotation from the camera
link (x forward, z up) to an OpenGL-style camera (looks along -z, y up), which
is what MuJoCo and USD cameras are.

Pure Python; ``linear_depth`` also works element-wise on NumPy arrays.
"""
import math

OAKD = {
    "link": "oakd_rgb_camera_frame",
    "optical_frame": "oakd_rgb_camera_optical_frame",
    "width": 320,
    "height": 240,
    "horizontal_fov": 1.25,
    "near": 0.3,
    "far": 100.0,
    "rgb_topic": "/oakd/rgb/image_raw",
    "depth_topic": "/oakd/depth/image_raw",
    "info_topic": "/oakd/rgb/camera_info",
}

# Scalar-first rotation from the camera link (x forward, y left, z up) to an
# OpenGL-style camera frame (x right, y up, looking along -z).
LINK_TO_OPENGL_CAMERA = (0.5, 0.5, -0.5, -0.5)


def vertical_fov(horizontal_fov, width, height):
    """Vertical field of view in radians, for square pixels."""
    return 2.0 * math.atan(math.tan(horizontal_fov / 2.0) * height / float(width))


def intrinsics(width, height, horizontal_fov):
    """``(fx, fy, cx, cy)`` of an ideal pinhole with square pixels."""
    fx = (width / 2.0) / math.tan(horizontal_fov / 2.0)
    return fx, fx, width / 2.0, height / 2.0


def camera_info_fields(width, height, horizontal_fov):
    """sensor_msgs/CameraInfo fields for an undistorted pinhole camera."""
    fx, fy, cx, cy = intrinsics(width, height, horizontal_fov)
    return {
        "width": int(width),
        "height": int(height),
        "distortion_model": "plumb_bob",
        "d": [0.0] * 5,
        "k": [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        "r": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "p": [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
    }


def linear_depth(z_buffer, near, far):
    """Depth along the optical axis from an OpenGL depth buffer in [0, 1]."""
    return far * near / (far - (far - near) * z_buffer)
