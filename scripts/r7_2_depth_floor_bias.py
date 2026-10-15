#!/usr/bin/env python3
"""R7.2: where does each simulator's depth camera put the floor?

Replays recorded depth frames (/oakd/depth/image_raw with
/oakd/rgb/camera_info, /tf_static and /odom/ground_truth, as recorded by
``scripts/slam_mapping_check.sh`` with ``EXTRA_BAG_TOPICS``) and moves the
back-projected pixels into the world with the true pose.  For pixels that
see the floor (world z within ``--band`` of 0; the few wall pixels that
low barely move a median) it reports the median world height per depth bin, twice:

* with the camera info's principal point (cx, cy), as every consumer of
  the camera info does;
* with the principal point half a pixel up and left (cx - 0.5, cy - 0.5),
  the centre of an image whose pixel centres sit at u + 0.5, as renderers
  draw them.

A principal point that is half a pixel off lifts the floor by
0.5 z / fy at depth z (2.3 mm per metre for the 320 x 240 OAK-D); a correct
one leaves it flat.

  python3 scripts/r7_2_depth_floor_bias.py --out <file.json> <run.bag> ...
"""
import argparse
import json
import os
import sys

for _variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_variable, '1')

import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'robot_lab_algorithms'))

from r7_2_feature_odometry_replay import (  # noqa: E402
    DEPTH, INFO, TF_STATIC, TRUTH, Track, messages, pose_matrix, stamp, static_chain)
from robot_lab_algorithms.perception import decode_depth, depth_to_points  # noqa: E402

BINS = [(0.5, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.0)]


def floor_bias(bag, base_frame, every, band):
    transforms, info, truth_rows = [], None, []
    heights = {name: {b: [] for b in BINS} for name in ('camera_info', 'pixel_centres')}
    frames = used = 0
    camera = None
    for name, msg in messages(bag, {DEPTH, INFO, TRUTH, TF_STATIC}):
        if name == TF_STATIC:
            transforms.extend(msg.transforms)
        elif name == INFO and info is None:
            info = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        elif name == TRUTH:
            truth_rows.append((stamp(msg.header), pose_matrix(msg.pose.pose)))
        elif name == DEPTH:
            frames += 1
            if info is None or (frames - 1) % every:
                continue
            truth = Track(truth_rows[-200:]).at(stamp(msg.header), gap=0.05)
            if truth is None:
                continue
            if camera is None:
                camera = static_chain(transforms, base_frame, msg.header.frame_id)
            world_from_camera = truth @ camera
            depth = decode_depth(msg)
            fx, fy, cx, cy = info
            used += 1
            for label, shift in (('camera_info', 0.0), ('pixel_centres', 0.5)):
                points = depth_to_points(depth, fx, fy, cx - shift, cy - shift, 2, 0.3, 4.0)
                world = points @ world_from_camera[:3, :3].T + world_from_camera[:3, 3]
                floor = np.abs(world[:, 2]) <= band
                for low, high in BINS:
                    in_bin = floor & (points[:, 2] >= low) & (points[:, 2] < high)
                    heights[label][(low, high)].extend(world[in_bin, 2].tolist())
    out = {'depth_frames': frames, 'frames_used': used, 'every': every, 'intrinsics': info, 'band_m': band}
    for label, bins in heights.items():
        out[label] = {'%.1f-%.1f m' % b: {'median_mm': round(1e3 * float(np.median(v)), 2) if v else None,
                                         'points': len(v)} for b, v in bins.items()}
    if info:
        out['half_pixel_lift_mm_per_m'] = round(1e3 * 0.5 / info[1], 2)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bags', nargs='+')
    parser.add_argument('--out', required=True)
    parser.add_argument('--base', default='base_footprint')
    parser.add_argument('--every', type=int, default=10, help='use every N-th depth frame')
    parser.add_argument('--band', type=float, default=0.02, help='floor band around world z = 0 (m)')
    args = parser.parse_args()
    results = {os.path.basename(os.path.normpath(bag)): floor_bias(bag, args.base, args.every, args.band)
               for bag in args.bags}
    with open(args.out, 'w') as handle:
        json.dump(results, handle, indent=1)
    print(json.dumps(results, indent=1))


if __name__ == '__main__':
    main()
