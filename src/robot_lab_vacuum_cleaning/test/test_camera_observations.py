"""Regression checks for real ROS image layout, pairing and bounded snapshots."""
import base64
from io import BytesIO
from types import SimpleNamespace as NS

import numpy as np
from PIL import Image

from robot_lab_vacuum_cleaning.camera_observations import (
    add_snapshots, aligned_pair, image_array)


def frame(encoding, width, height, step, data, t=1, **kwargs):
    return NS(encoding=encoding, width=width, height=height, step=step, data=data,
              header=NS(stamp=NS(sec=t, nanosec=0)), **kwargs)


def test_rgb_keeps_colour_and_padding_and_labels_keep_first_channel():
    msg = frame('bgr8', 2, 1, 8, bytes([1, 2, 3, 4, 5, 6, 255, 255]))
    assert image_array(msg, rgb=True).tolist() == [[[3, 2, 1], [6, 5, 4]]]
    assert image_array(msg, rgb=True).flags.c_contiguous  # torch rejects negative strides
    assert image_array(msg).tolist() == [[1, 4]]
    assert image_array(frame('rgb8', 2, 1, 5, bytes(5)), rgb=True) is None


def test_depth_respects_byte_order_and_padding():
    msg = frame('32FC1', 2, 1, 12, np.array([1., 2., 999.], dtype='>f4').tobytes(),
                is_bigendian=True)
    np.testing.assert_equal(image_array(msg), [[1., 2.]])
    assert image_array(msg, rgb=True) is None


def test_snapshot_requires_matching_time_and_dimensions():
    rgb = frame('rgb8', 400, 300, 1200, bytes(360000))
    depth = frame('32FC1', 400, 300, 1600, bytes(480000))
    assert aligned_pair(rgb, depth)
    depth.header.stamp.sec = 2
    assert not aligned_pair(rgb, depth)
    obj = {'bbox': [0, 0, 400, 300]}
    add_snapshots([obj], image_array(rgb, rgb=True), 1.)
    snap = obj['snapshot']
    image = Image.open(BytesIO(base64.b64decode(snap['data'])))
    assert image.size == (160, 120) and snap['t'] == 1.


def test_segmentation_crops_follow_each_connected_object():
    from robot_lab_vacuum_cleaning.vacuum_objects import project_labels
    labels = np.zeros((10, 20), dtype=np.uint8)
    labels[2:5, 1:4] = labels[2:5, 15:18] = 20
    objects = project_labels(labels, np.ones(labels.shape), [10, 0, 0, 0, 10, 0, 0, 0, 1],
                             np.eye(4), {20: 'cable'}, min_pixels=4, cell=.15)
    assert sorted(o['bbox'] for o in objects) == [[1, 2, 4, 5], [15, 2, 18, 5]]


def test_capture_keeps_separate_instances_and_background_frames():
    import importlib.util
    from pathlib import Path
    path = Path(__file__).resolve().parents[3] / 'scripts/capture_vacuum_detector_data.py'
    spec = importlib.util.spec_from_file_location('camera_capture_contract', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    labels = np.zeros((20, 40), dtype=np.uint8)
    labels[2:5, 3:7] = labels[10:14, 30:34] = 22
    rows = module.annotations_for(labels, image_id=6, first_id=12)
    assert [row['bbox'] for row in rows] == [[3, 2, 4, 3], [30, 10, 4, 4]]
    assert [row['id'] for row in rows] == [12, 13]
    assert all(row['image_id'] == 6 and row['category_id'] == 3 for row in rows)
    assert module.annotations_for(np.zeros_like(labels), image_id=7) == []
