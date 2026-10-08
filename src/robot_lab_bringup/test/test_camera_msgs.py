"""Verify the actual ROS wire format used by the simulated RGB-D cameras."""
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image

from robot_lab_utils.camera_msgs import image_msg

pytestmark = pytest.mark.integration


@pytest.mark.parametrize('encoding', ['rgb8', '32FC1'])
@pytest.mark.parametrize('strided', [False, True])
def test_image_wire_preserves_pixels_stride_and_header(encoding, strided):
    if encoding == 'rgb8':
        pixels = np.arange(6*8*3, dtype=np.uint8).reshape(6, 8, 3)
    else:
        # Preserve NaN payload bits, infinities, signed zero and finite depth.
        bits = np.tile(np.array([0x7fc00001, 0x7f800000, 0xff800000,
                                 0x80000000, 0x3f800000, 0], dtype=np.uint32), 8)
        pixels = bits.view(np.float32).reshape(6, 8)
    if strided:
        pixels = pixels[::2, ::-2]
        assert not pixels.flags.c_contiguous
    stamp = Time(sec=42, nanosec=123456789)
    decoded = deserialize_message(serialize_message(
        image_msg(stamp, 'oakd_optical_frame', pixels, encoding)), Image)
    assert decoded.header.frame_id == 'oakd_optical_frame'
    assert decoded.header.stamp == stamp
    assert decoded.encoding == encoding
    assert (decoded.height, decoded.width) == pixels.shape[:2]
    assert decoded.is_bigendian == 0
    channels = 3 if encoding == 'rgb8' else 1
    assert decoded.step == decoded.width * channels * pixels.itemsize
    assert len(decoded.data) == decoded.height * decoded.step
    assert bytes(decoded.data) == pixels.tobytes(order='C')
