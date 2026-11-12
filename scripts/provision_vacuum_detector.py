#!/usr/bin/env python3
"""Provision an actual trained camera checkpoint as an experimental local asset.

No training, downloads, support promotion or robot commands occur here. Retain
the training dataset on SSD; this installer verifies its annotations/split/image
hashes against the trained checkpoint before publishing the local selector.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/robot_lab_vacuum_cleaning'))
from robot_lab_vacuum_cleaning.detector_assets import CLASSES, detector_root


def digest(path):
    sha = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            sha.update(block)
    return sha.hexdigest()


def provision(checkpoint, dataset, destination):
    state = torch.load(checkpoint, map_location='cpu', weights_only=True)
    if (state.get('cat_id') != CLASSES
            or state.get('architecture') != 'fasterrcnn_mobilenet_v3_large_fpn'
            or not state.get('model') or state.get('weights_initialization') != 'coco'):
        raise ValueError('Checkpoint is not a source-bound trained camera model')
    for name, key in [('annotations.json', 'annotations_sha256'), ('val_ids.json', 'split_sha256')]:
        if digest(dataset / name) != state.get(key):
            raise ValueError('Checkpoint dataset metadata hash mismatch: ' + name)
    for filename, expected in state.get('image_sha256', {}).items():
        image = (dataset / 'images' / filename).resolve()
        if not image.is_relative_to((dataset / 'images').resolve()) or digest(image) != expected:
            raise ValueError('Checkpoint image hash mismatch: ' + filename)
    if not state.get('image_sha256'):
        raise ValueError('Checkpoint has no image provenance')
    metrics_path = checkpoint.parent / 'metrics.json'
    metrics = json.loads(metrics_path.read_text())
    if not math.isfinite(metrics.get('mAP', math.nan)) or not 0 <= metrics['mAP'] <= 1:
        raise ValueError('Actual held-out metrics are required')
    sha = digest(checkpoint)
    target = destination / sha
    target.mkdir(parents=True, exist_ok=True)
    output = target / 'vacuum_detector.pt'
    if output.exists() and digest(output) != sha:
        raise ValueError('Existing content-addressed checkpoint changed')
    if not output.exists():
        shutil.copy2(checkpoint, output)
    if digest(output) != sha:
        raise ValueError('Checkpoint copy verification failed')
    for name in ('annotations.json', 'val_ids.json', 'render-receipt.json'):
        if (dataset / name).is_file():
            shutil.copy2(dataset / name, target / name)
    shutil.copy2(metrics_path, target / 'metrics.json')
    from torchvision.models.detection import FasterRCNN_MobileNet_V3_Large_FPN_Weights
    weight_url = FasterRCNN_MobileNet_V3_Large_FPN_Weights.DEFAULT.url
    cache = Path(os.environ.get('TORCH_HOME', Path.home() / '.cache/torch')) / 'hub/checkpoints'
    pretrained = cache / weight_url.rsplit('/', 1)[1]
    manifest = {
        'schema_version': 1, 'status': 'experimental_candidate',
        'path': str(output.relative_to(destination)), 'sha256': sha, 'cat_id': CLASSES,
        'architecture': state['architecture'], 'model_kwargs': state.get('model_kwargs', {}),
        'dataset_path': str(dataset.resolve()), 'metrics': metrics,
        'seed': state['seed'], 'epoch': state['epoch'], 'freeze_features': state['freeze_features'],
        'torch_version': state['torch_version'],
        'annotations_sha256': state['annotations_sha256'], 'split_sha256': state['split_sha256'],
        'pretrained_weights': {'url': weight_url,
                               'sha256': digest(pretrained) if pretrained.is_file() else None},
        'code_license': 'BSD-3-Clause (torchvision); lab wrapper MIT',
        'source': 'https://github.com/pytorch/vision',
        'qualification': 'trained on rendered source primitives; robot mission/real objects unqualified',
    }
    (target / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    pending = destination / 'current.json.pending'
    pending.write_text(json.dumps(manifest, indent=2) + '\n')
    pending.replace(destination / 'current.json')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--out', type=Path, default=detector_root())
    args = parser.parse_args()
    print(json.dumps(provision(args.checkpoint, args.data, args.out), indent=2))


if __name__ == '__main__':
    main()
