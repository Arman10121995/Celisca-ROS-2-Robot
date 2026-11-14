#!/usr/bin/env python3
"""Held-out evaluation of a vacuum detector checkpoint through the deployed loader (R13.3).

Runs ``LearnedDetector`` (the exact runtime code, deployed score threshold)
on the held-out images of a captured dataset and reports, per class, how many
ground-truth boxes were matched at IoU >= 0.5, how many predictions were made
and how many were false, plus the mean inference time. Each prediction may
match at most one ground-truth box of its class.

  python3 scripts/evaluate_vacuum_detector.py --model vacuum_detector.pt --data <dataset> --out result.json
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
from PIL import Image

CATEGORY = {1: 'cable', 2: 'pet_waste', 3: 'shoe', 4: 'sock'}


def iou(a, b):
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0., x2 - x1) * max(0., y2 - y1)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.


def match(predictions, truths, threshold=0.5):
    """(true positives, false positives) for one class on one image, greedy by score."""
    used, tp = set(), 0
    for prediction in sorted(predictions, key=lambda p: -p['score']):
        best, index = 0., None
        for i, truth in enumerate(truths):
            if i not in used and iou(prediction['bbox'], truth) > best:
                best, index = iou(prediction['bbox'], truth), i
        if index is not None and best >= threshold:
            used.add(index)
            tp += 1
    return tp, len(predictions) - tp


def evaluate(detector, data, threshold=0.5):
    data = Path(data)
    dataset = json.loads((data / 'annotations.json').read_text())
    held_out = set(json.loads((data / 'val_ids.json').read_text()))
    truth = {}
    for row in dataset['annotations']:
        x, y, w, h = row['bbox']
        if w > 0 and h > 0:
            truth.setdefault(row['image_id'], []).append((CATEGORY[row['category_id']], [x, y, x + w, y + h]))
    per_class = {name: dict(gt=0, matched=0, predictions=0, false_positives=0) for name in CATEGORY.values()}
    times = []
    images = [image for image in dataset['images'] if image['id'] in held_out]
    for image in images:
        rgb = np.asarray(Image.open(data / 'images' / image['file_name']).convert('RGB'))
        started = time.perf_counter()
        detections = detector.detect(rgb)
        times.append(time.perf_counter() - started)
        boxes = truth.get(image['id'], [])
        for name, row in per_class.items():
            predictions = [d for d in detections if d['class'] == name]
            truths = [box for cls, box in boxes if cls == name]
            tp, fp = match(predictions, truths, threshold)
            row['gt'] += len(truths)
            row['matched'] += tp
            row['predictions'] += len(predictions)
            row['false_positives'] += fp
    for row in per_class.values():
        row['recall'] = round(row['matched'] / row['gt'], 3) if row['gt'] else None
        row['precision'] = round(row['matched'] / row['predictions'], 3) if row['predictions'] else None
    return dict(images=len(images), per_class=per_class, iou_threshold=threshold,
                score_threshold=detector.score_thresh, inference_mean_wall_s=round(float(np.mean(times)), 4))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--model', required=True)
    parser.add_argument('--data', required=True, action='append', help='dataset directory; may repeat')
    parser.add_argument('--score', type=float, default=0.4)
    parser.add_argument('--out', default='')
    options = parser.parse_args()
    import torch
    torch.set_num_threads(4)
    from robot_lab_vacuum_cleaning.learned_detector import LearnedDetector
    detector = LearnedDetector(options.model, score_thresh=options.score)
    result = dict(model=options.model, scope='deployed-loader inference on held-out Gazebo fixture RGB; no robot mission',
                  datasets={data: evaluate(detector, data) for data in options.data})
    print(json.dumps(result, indent=1))
    if options.out:
        Path(options.out).write_text(json.dumps(result, indent=1) + '\n')


if __name__ == '__main__':
    main()
