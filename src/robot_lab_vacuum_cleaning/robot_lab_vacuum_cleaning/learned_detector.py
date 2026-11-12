"""Learned floor-object detector (torchvision) for the robot-vacuum camera.

A torchvision ``fasterrcnn_mobilenet_v3_large_fpn`` (BSD-licensed source)
loads a local checkpoint trained for cable, sock, shoe and pet_waste.
No trained checkpoint is bundled here. It turns RGB camera frames into the same
floor-object JSON that ``vacuum_objects.py`` publishes, flagged
``detector: 'learned'`` / ``ideal: false`` so the cleaning mission can tell an
ideal segmentation detector from a learned one.

The module is ROS-free: ``LearnedDetector.detect`` takes an RGB image and
returns 2D boxes; ``project_detections`` turns boxes into map-frame floor
objects using depth + TF, mirroring the geometry in ``vacuum_objects.py``.
"""
import os
from typing import Dict, List, Optional, Sequence

import numpy as np
import torch
from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_fpn
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.transforms import functional as F
from torchvision.ops import batched_nms

# Vacuum camera object classes and label ids (mirrors vacuum_objects.DEFAULT_LABELS).
LABELS = {20: 'cable', 21: 'sock', 22: 'shoe', 23: 'pet_waste'}
CLASS_TO_LABEL = {v: k for k, v in LABELS.items()}
# Nominal object radius (metres), half the floor extent of each class.
CLASS_RADIUS = {'cable': 0.4, 'sock': 0.119, 'shoe': 0.151, 'pet_waste': 0.025}


class LearnedDetector:
    """torchvision detector fine-tuned for the four vacuum floor-object classes."""

    def __init__(self, model_path: Optional[str] = None, score_thresh: float = 0.4,
                 device: str = 'cpu', labels: Dict[int, str] = LABELS):
        self.labels = dict(labels)
        self.label_to_class = {v: k for k, v in self.labels.items()}
        self.score_thresh = float(score_thresh)
        if not np.isfinite(self.score_thresh) or not 0.0 <= self.score_thresh <= 1.0:
            raise ValueError('Detection confidence threshold must be between 0 and 1')
        self.device = torch.device(device)
        # category id 1..N in class order; background is 0
        class_names = sorted(self.labels.values())
        self.cat_to_class = {i + 1: name for i, name in enumerate(class_names)}
        self.ok = False
        self.model = None
        if not model_path or not os.path.isfile(model_path):
            raise FileNotFoundError('Learned detector checkpoint is required: %s' % model_path)
        ckpt = torch.load(model_path, map_location=self.device, weights_only=True)
        if not isinstance(ckpt, dict) or ckpt.get('cat_id') != {
                name: key for key, name in self.cat_to_class.items()}:
            raise ValueError('Checkpoint class map does not match the vacuum detector')
        model_kwargs = ckpt.get('model_kwargs', {})
        if (not isinstance(model_kwargs, dict) or set(model_kwargs) - {'min_size', 'max_size'}
                or any(type(value) is not int or not 64 <= value <= 2048
                       for value in model_kwargs.values())
                or model_kwargs.get('min_size', 800) > model_kwargs.get('max_size', 1333)):
            raise ValueError('Checkpoint image transform is invalid')
        # Never download a backbone at runtime. Both weight arguments are
        # necessary: torchvision defaults weights_backbone to ImageNet.
        self.model = fasterrcnn_mobilenet_v3_large_fpn(weights=None, weights_backbone=None, **model_kwargs)
        in_features = self.model.roi_heads.box_predictor.cls_score.in_features
        self.model.roi_heads.box_predictor = FastRCNNPredictor(in_features, len(class_names) + 1)
        self.model.to(self.device)
        self.model.eval()
        self.model.load_state_dict(ckpt['model'])
        self.model.eval()
        self.ok = True

    def detect(self, rgb: np.ndarray) -> List[Dict]:
        """Run inference on an HxWx3 uint8 RGB image.

        Returns [{'class', 'bbox': [x1, y1, x2, y2], 'score'}] in pixel coords.
        """
        if not self.ok:
            return []
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError('Detector expects an H x W x 3 uint8 RGB image')
        img = F.to_tensor(rgb).to(self.device)  # C,H,W in [0,1]
        with torch.no_grad():
            out = self.model([img])[0]
        keep = batched_nms(out['boxes'], out['scores'], out['labels'], 0.45)
        boxes = out['boxes'][keep].cpu().numpy()
        scores = out['scores'][keep].cpu().numpy()
        labels = out['labels'][keep].cpu().numpy()
        detections = []
        for box, score, label in zip(boxes, scores, labels):
            if float(score) < self.score_thresh:
                continue
            cls = self.cat_to_class.get(int(label))
            if cls is None:
                continue
            detections.append({
                'class': cls,
                'label': self.label_to_class[cls],
                'bbox': [float(box[0]), float(box[1]), float(box[2]), float(box[3])],
                'score': float(score),
            })
        return detections


def project_detections(detections: Sequence[Dict], depth: np.ndarray, k: Sequence[float],
                       cam_to_map: np.ndarray, radius_override: Optional[Dict[str, float]] = None,
                       min_pixels: int = 6, max_range: float = 2.5) -> List[Dict]:
    """Project learned detections onto the floor in the map frame.

    For each detection the median depth inside its bounding box gives the
    object range; the box centre back-projects to a 3D point which is
    transformed to the map frame.  Returns objects like
    ``project_labels``: {class, x, y, radius, pixels}.
    """
    out: List[Dict] = []
    if depth is None:
        return out
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    if not np.isfinite([fx, fy, cx, cy]).all() or fx <= 0 or fy <= 0:
        return out
    h, w = depth.shape
    radius_override = radius_override or CLASS_RADIUS
    for det in detections:
        if not np.isfinite(det['bbox']).all() or det['class'] not in radius_override:
            continue
        x1, y1, x2, y2 = [int(round(v)) for v in det['bbox']]
        x1, x2 = max(0, x1), min(w, x2)
        y1, y2 = max(0, y1), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            continue
        box_depth = depth[y1:y2, x1:x2]
        box_depth = box_depth[np.isfinite(box_depth) & (box_depth > 0.05) & (box_depth < max_range)]
        if box_depth.size < min_pixels:
            continue
        z = float(np.median(box_depth))
        cx_pix = 0.5 * (x1 + x2); cy_pix = 0.5 * (y1 + y2)
        x_cam = (cx_pix - cx) * z / fx
        y_cam = (cy_pix - cy) * z / fy
        pt = cam_to_map @ np.array([x_cam, y_cam, z, 1.0])
        if not np.isfinite(pt).all() or abs(float(pt[2])) > 0.3:
            continue
        radius = float(radius_override.get(det['class'], 0.08))
        out.append({'class': det['class'], 'label': det['label'],
                    'x': round(float(pt[0]), 3), 'y': round(float(pt[1]), 3),
                    'radius': round(max(radius, 0.03), 3),
                    'pixels': int(box_depth.size), 'score': round(det['score'], 3),
                    'bbox': [x1, y1, x2, y2]})
    return out
