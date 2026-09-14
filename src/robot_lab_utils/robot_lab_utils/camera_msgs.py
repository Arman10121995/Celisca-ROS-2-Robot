"""sensor_msgs builders for the simulated RGB-D cameras (ROS side).

Used by the PyBullet and MuJoCo bridges so both publish the same encodings
as the Gazebo route: ``rgb8`` colour, ``32FC1`` depth in metres and an
undistorted pinhole ``CameraInfo`` from ``camera_model``.
"""
import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from . import camera_model


def image_msg(stamp, frame_id, array, encoding):
    """Image message from an HxW or HxWxC NumPy array."""
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = int(array.shape[0]), int(array.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    channels = array.shape[2] if array.ndim == 3 else 1
    msg.step = int(array.shape[1] * channels * array.itemsize)
    msg.data = np.ascontiguousarray(array).tobytes()
    return msg


def camera_info_msg(stamp, frame_id, width, height, horizontal_fov):
    fields = camera_model.camera_info_fields(width, height, horizontal_fov)
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width, msg.height = fields["width"], fields["height"]
    msg.distortion_model = fields["distortion_model"]
    msg.d, msg.k, msg.r, msg.p = fields["d"], fields["k"], fields["r"], fields["p"]
    return msg
