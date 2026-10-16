#!/usr/bin/env python3
"""Image quality of a recorded RGB-D stream: is the colour image noise or scene?

Reads /oakd/rgb/image_raw and /oakd/depth/image_raw from a rosbag2 (sqlite)
and reports, per stream:

- ``edge_correlation``: correlation of the grey image's Sobel magnitude with
  the depth image's (pixel-paired frames).  Scene edges are mostly depth
  edges; per-pixel render noise has none (Isaac's RGB scored 0.15 here, the
  other simulators 0.57-0.89, R7.2 feature odometry 2026-10-15);
- ``temporal_noise``: mean absolute grey difference between consecutive
  frames taken while the base stood still (truth moved < 1 mm and < 0.05°),
  which is zero for a deterministic renderer;
- ``orb_repeat``: the share of ORB keypoints in a still frame that reappear
  within 1.5 px in the next still frame;
- ``gray_std``: contrast.

  python3 scripts/r7_2_image_quality.py <bag> [<bag>...] > quality.json
"""
import glob
import json
import math
import os
import sqlite3
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'robot_lab_algorithms'))

from robot_lab_algorithms.perception import decode_depth  # noqa: E402
from robot_lab_algorithms.visual_odometry import decode_gray  # noqa: E402

RGB, DEPTH, TRUTH = '/oakd/rgb/image_raw', '/oakd/depth/image_raw', '/odom/ground_truth'


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def messages(path, wanted):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    files = glob.glob(os.path.join(path, '*.db3')) if os.path.isdir(path) else [path]
    db = sqlite3.connect(files[0])
    topics = {tid: (name, get_message(kind)) for tid, name, kind in db.execute('select id, name, type from topics')}
    ids = [tid for tid, (name, _) in topics.items() if name in wanted]
    query = 'select topic_id, data from messages where topic_id in (%s) order by timestamp' % ','.join(
        str(i) for i in ids)
    for tid, raw in (db.execute(query) if ids else []):
        name, cls = topics[tid]
        yield name, deserialize_message(raw, cls)


def sobel(image):
    import cv2
    image = image.astype(np.float32)
    return np.hypot(cv2.Sobel(image, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(image, cv2.CV_32F, 0, 1, ksize=3))


def edge_correlation(gray, depth):
    valid = np.isfinite(depth) & (depth > 0)
    if valid.mean() < 0.5:
        return None
    filled = np.where(valid, depth, np.nanmedian(depth[valid]))
    a, b = sobel(gray)[valid], sobel(filled)[valid]
    if a.std() == 0 or b.std() == 0:
        return None
    return float(np.corrcoef(a, b)[0, 1])


def orb_repeat(first, second):
    import cv2
    orb = cv2.ORB_create(500)
    a, b = orb.detect(first, None), orb.detect(second, None)
    if not a or not b:
        return None
    pts = np.array([k.pt for k in b])
    return float(np.mean([np.min(np.hypot(*(pts - k.pt).T)) < 1.5 for k in a]))


def analyse(bag):
    truth, frames, depths = [], [], {}
    for name, msg in messages(bag, {RGB, DEPTH, TRUTH}):
        if name == TRUTH:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            truth.append((stamp(msg.header), p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                                 1 - 2 * (q.y * q.y + q.z * q.z))))
        elif name == DEPTH:
            depths[round(stamp(msg.header), 3)] = decode_depth(msg)
        else:
            frames.append((stamp(msg.header), decode_gray(msg)))
    truth = np.array(truth) if truth else np.zeros((0, 4))

    def pose(t):
        if not len(truth):
            return None
        i = int(np.clip(np.searchsorted(truth[:, 0], t), 0, len(truth) - 1))
        return truth[i, 1:]

    edges, noise, repeat, contrast = [], [], [], []
    for (t0, g0), (t1, g1) in zip(frames, frames[1:]):
        depth = depths.get(round(t1, 3))
        if depth is not None and depth.shape == g1.shape:
            value = edge_correlation(g1, depth)
            if value is not None:
                edges.append(value)
        contrast.append(float(g1.std()))
        p0, p1 = pose(t0), pose(t1)
        if p0 is not None and math.hypot(*(p1[:2] - p0[:2])) < 1e-3 and abs(p1[2] - p0[2]) < math.radians(0.05):
            noise.append(float(np.mean(np.abs(g1.astype(np.int16) - g0.astype(np.int16)))))
            value = orb_repeat(g0, g1)
            if value is not None:
                repeat.append(value)

    def median(values):
        return round(float(np.median(values)), 3) if values else None
    return dict(bag=bag, frames=len(frames), depth_frames=len(depths), still_pairs=len(noise),
                edge_correlation=median(edges), temporal_noise=median(noise),
                orb_repeat=median(repeat), gray_std=median(contrast))


if __name__ == '__main__':
    print(json.dumps([analyse(path) for path in sys.argv[1:]], indent=1))
