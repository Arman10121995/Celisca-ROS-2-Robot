#!/usr/bin/env python3
"""Capture aligned simulator RGB/semantic frames into a scene-split COCO dataset.

This read-only ROS client never commands the robot. Run one scene per call,
using a different scene/layout for validation. Output belongs on the SSD.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image as PILImage
from scipy.ndimage import label as components, find_objects

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/robot_lab_vacuum_cleaning'))
from robot_lab_vacuum_cleaning.camera_observations import aligned_pair, image_array, stamp_seconds
from robot_lab_vacuum_cleaning.vacuum_objects import DEFAULT_LABELS

CLASSES = {name: i + 1 for i, name in enumerate(sorted(DEFAULT_LABELS.values()))}


def annotations_for(labels, image_id, first_id=1):
    """One box per connected labelled instance; retain negative images too."""
    rows = []
    for semantic_id, name in DEFAULT_LABELS.items():
        groups, _ = components(labels == semantic_id)
        for instance, slices in enumerate(find_objects(groups), 1):
            if slices is None:
                continue
            pixels = int((groups[slices] == instance).sum())
            if pixels < 6:
                continue
            yy, xx = slices
            rows.append({'id': first_id + len(rows), 'image_id': image_id,
                         'category_id': CLASSES[name],
                         'bbox': [xx.start, yy.start, xx.stop - xx.start, yy.stop - yy.start],
                         'area': pixels, 'iscrowd': 0})
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--scene', required=True, help='Unique world/layout identity, e.g. house-A-layout-1')
    parser.add_argument('--split', choices=['train', 'val'], required=True)
    parser.add_argument('--frames', type=int, default=100)
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--rgb-topic', default='/vacuum/camera/image')
    parser.add_argument('--labels-topic', default='/vacuum/camera/labels')
    args = parser.parse_args()
    if (args.frames <= 0 or not np.isfinite([args.interval, args.timeout]).all()
            or args.interval <= 0 or args.timeout <= 0 or not args.scene.strip()):
        parser.error('Positive finite frame count, interval, timeout and a scene name are required')
    args.out.mkdir(parents=True, exist_ok=True)
    lock = args.out / '.capture.lock'
    try:
        fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError:
        parser.error('Another capture owns this dataset (or its lock needs inspection): %s' % lock)
    os.close(fd)
    try:
        return capture(args)
    finally:
        lock.unlink()


def capture(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    categories = [{'id': i, 'name': name} for name, i in CLASSES.items()]
    annotation_path = args.out / 'annotations.json'
    dataset = json.loads(annotation_path.read_text()) if annotation_path.exists() else {
        'images': [], 'annotations': [], 'categories': categories}
    if dataset['categories'] != categories:
        raise ValueError('Existing dataset uses a different class map')
    if any(i.get('scene') == args.scene and i.get('split') != args.split for i in dataset['images']):
        raise ValueError('A scene cannot appear in both training and validation')
    # Do not silently merge unrelated synthetic or pre-split datasets.
    if any('scene' not in image or 'split' not in image for image in dataset['images']):
        raise ValueError('Capture requires scene-labelled data; use a fresh output directory')
    image_id = max((i['id'] for i in dataset['images']), default=0) + 1
    annotation_id = max((a['id'] for a in dataset['annotations']), default=0) + 1
    (args.out / 'images').mkdir(exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('vacuum_camera_dataset_capture')
    latest = {}
    subscriptions = [node.create_subscription(Image, topic, lambda m, k=key: latest.update({k: m}),
                                              qos_profile_sensor_data)
                     for key, topic in [('rgb', args.rgb_topic), ('labels', args.labels_topic)]]
    start, last, last_stamp, count = time.monotonic(), -float('inf'), None, 0
    try:
        while rclpy.ok() and count < args.frames and time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=.05)
            rgb_msg, label_msg = latest.get('rgb'), latest.get('labels')
            if not aligned_pair(rgb_msg, label_msg) or time.monotonic() - last < args.interval:
                continue
            stamp = stamp_seconds(rgb_msg)
            if stamp == last_stamp:
                continue
            rgb, labels = image_array(rgb_msg, rgb=True), image_array(label_msg)
            if rgb is None or labels is None:
                continue
            filename = '%06d.png' % image_id
            PILImage.fromarray(rgb, 'RGB').save(args.out / 'images' / filename)
            dataset['images'].append({'id': image_id, 'file_name': filename,
                                      'sha256': hashlib.sha256((args.out / 'images' / filename).read_bytes()).hexdigest(),
                                      'width': rgb_msg.width, 'height': rgb_msg.height,
                                      'scene': args.scene, 'split': args.split, 'stamp': stamp,
                                      'source': 'ROS simulator RGB with aligned ideal semantic annotations'})
            rows = annotations_for(labels, image_id, annotation_id)
            dataset['annotations'].extend(rows)
            annotation_id += len(rows)
            image_id += 1
            count += 1
            last, last_stamp = time.monotonic(), stamp
            # Each manifest is replaced atomically; metadata retains explicit
            # split membership, so a later training reader can check it too.
            for path, value in [(annotation_path, dataset),
                                (args.out / 'val_ids.json', [i['id'] for i in dataset['images']
                                                            if i['split'] == 'val'])]:
                temp = path.with_suffix('.json.pending')
                temp.write_text(json.dumps(value, indent=1) + '\n')
                temp.replace(path)
            print('captured %d/%d (%s, %s)' % (count, args.frames, args.scene, args.split), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    result = {'captured': count, 'requested': args.frames, 'complete': count == args.frames,
              'scene': args.scene, 'split': args.split, 'wall_s': time.monotonic() - start}
    (args.out / 'last-capture.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
    return 0 if result['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
