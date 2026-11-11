"""Bounded, timestamped camera observations shared by both floor detectors."""

import base64
from io import BytesIO
import math

import numpy as np


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def image_array(msg, rgb=False):
    """Decode ROS images, respecting row padding and depth byte order.

    Labels keep their first channel; an RGB request explicitly preserves all
    three channels and converts BGR. Malformed/unsupported images are refused.
    """
    channels = {'mono8': 1, '8UC1': 1, 'rgb8': 3, 'bgr8': 3, '8UC3': 3,
                'rgba8': 4, 'bgra8': 4}
    try:
        h, w, step = int(msg.height), int(msg.width), int(msg.step)
        if h <= 0 or w <= 0 or step <= 0 or len(msg.data) != h * step:
            return None
        if msg.encoding == '32FC1' and not rgb:
            if step < w * 4 or step % 4:
                return None
            dtype = np.dtype('>f4' if getattr(msg, 'is_bigendian', False) else '<f4')
            return np.frombuffer(bytes(msg.data), dtype=dtype).reshape(h, step // 4)[:, :w]
        n = channels.get(msg.encoding)
        if n is None or step < w * n:
            return None
        rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, step)
        array = rows[:, :w * n].reshape(h, w, n)
        if not rgb:
            return array[:, :, 0]
        if n < 3:
            return None
        array = array[:, :, :3]
        return array[:, :, ::-1].copy() if msg.encoding in ('bgr8', 'bgra8') else array
    except (AttributeError, TypeError, ValueError):
        return None


def aligned_pair(first, second, tolerance_s=0.1):
    """Require registered, equally sized frames at approximately the same time."""
    if first is None or second is None:
        return False
    return (first.width == second.width and first.height == second.height
            and abs(stamp_seconds(first) - stamp_seconds(second)) <= tolerance_s)


def add_snapshots(objects, rgb, stamp):
    """Attach at most 16 small JPEG crops, never a full camera stream."""
    if rgb is None or rgb.ndim != 3:
        return
    from PIL import Image
    height, width = rgb.shape[:2]
    for obj in objects[:16]:
        box = obj.get('bbox')
        if not box or len(box) != 4 or not all(math.isfinite(v) for v in box):
            continue
        x1, y1 = max(0, int(box[0]) - 8), max(0, int(box[1]) - 8)
        x2, y2 = min(width, math.ceil(box[2]) + 8), min(height, math.ceil(box[3]) + 8)
        if x2 <= x1 or y2 <= y1:
            continue
        image = Image.fromarray(rgb[y1:y2, x1:x2], 'RGB')
        image.thumbnail((160, 120))
        stream = BytesIO()
        image.save(stream, format='JPEG', quality=75)
        obj['snapshot'] = {'encoding': 'jpeg', 'width': image.width, 'height': image.height,
                           't': stamp, 'data': base64.b64encode(stream.getvalue()).decode('ascii')}
