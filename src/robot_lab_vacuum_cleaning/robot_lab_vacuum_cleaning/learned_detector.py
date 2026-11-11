"""Learned floor-object detector (torchvision) for the robot-vacuum camera.

A torchvision ``fasterrcnn_mobilenet_v3_large_fpn`` (COCO-pretrained, BSD
licensed) fine-tuned on rendered images of the four vacuum camera classes
(cable, sock, shoe, pet_waste).  It turns RGB camera frames into the same
floor-object JSON that ``vacuum_objects.py`` publishes, flagged
``detector: 'learned'`` / ``ideal: false`` so the cleaning mission can tell an
ideal segmentation detector from a learned one.

The module is ROS-free: ``LearnedDetector.detect`` takes an RGB image and
returns 2D boxes; ``project_detections`` turns boxes into map-frame floor
objects using depth + TF, mirroring the geometry in ``vacuum_objects.py``.
"""
import json
import os
from typing import Dict, List, Optional, Sequence

import numpy as np
import torch
from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_fpn
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.transforms import functional as F
from torchvision.ops import nms

# Vacuum camera object classes and label ids (mirrors vacuum_objects.DEFAULT_LABELS).
LABELS = {20: 'cable', 21: 'sock', 22: 'shoe', 23: 'pet_waste'}
CLASS_TO_LABEL = {v: k for k, v in LABELS.items()}
# Nominal object radius (metres), half the floor extent of each class.
CLASS_RADIUS = {'cable': 0.012, 'sock': 0.11, 'shoe': 0.14, 'pet_waste': 0.025}
COCO_MEAN = [0.485, 0.456, 0.406]
COCO_STD = [0.229, 0.224, 0.225]


class LearnedDetector:
    """torchvision detector fine-tuned for the four vacuum floor-object classes."""

    def __init__(self, model_path: Optional[str] = None, score_thresh: float = 0.4,
                 device: str = 'cpu', labels: Dict[int, str] = LABELS):
        self.labels = dict(labels)
        self.label_to_class = {v: k for k, v in self.labels.items()}
        self.score_thresh = float(score_thresh)
        self.device = torch.device(device)
        # category id 1..N in class order; background is 0
        class_names = sorted(self.labels.values())
        self.cat_to_class = {i + 1: name for i, name in enumerate(class_names)}
        self.model = fasterrcnn_mobilenet_v3_large_fpn(weights=None)
        in_features = self.model.roi_heads.box_predictor.cls_score.in_features
        self.model.roi_heads.box_predictor = FastRCNNPredictor(in_features, len(class_names) + 1)
        self.model.to(self.device)
        self.model.eval()
        self.ok = False
        if model_path and os.path.exists(model_path):
            ckpt = torch.load(model_path, map_location=self.device)
            state = ckpt['model'] if isinstance(ckpt, dict) and 'model' in ckpt else ckpt
            self.model.load_state_dict(state)
            self.model.eval()
            self.ok = True

    def detect(self, rgb: np.ndarray) -> List[Dict]:
        """Run inference on an HxWx3 uint8 RGB image.

        Returns [{'class', 'bbox': [x1, y1, x2, y2], 'score'}] in pixel coords.
        """
        if not self.ok:
            return []
        img = F.to_tensor(rgb).to(self.device)  # C,H,W in [0,1]
        with torch.no_grad():
            out = self.model([img])[0]
        keep = nms(out['boxes'], out['scores'], 0.45)
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
    h, w = depth.shape
    vs, us = np.mgrid[0:h, 0:w]
    radius_override = radius_override or CLASS_RADIUS
    for det in detections:
        x1, y1, x2, y2 = [int(round(v)) for v in det['bbox']]
        x1 = max(0, min(x1, w - 1)); x2 = max(0, min(x2, w - 1))
        y1 = max(0, min(y1, h - 1)); y2 = max(0, min(y2, h - 1))
        box_depth = depth[y1:y2 + 1, x1:x2 + 1]
        box_depth = box_depth[np.isfinite(box_depth) & (box_depth > 0.05) & (box_depth < max_range)]
        if box_depth.size < min_pixels:
            continue
        z = float(np.median(box_depth))
        cx_pix = 0.5 * (x1 + x2); cy_pix = 0.5 * (y1 + y2)
        x_cam = (cx_pix - cx) * z / fx
        y_cam = (cy_pix - cy) * z / fy
        pt = cam_to_map @ np.array([x_cam, y_cam, z, 1.0])
        radius = float(radius_override.get(det['class'], 0.08))
        out.append({'class': det['class'], 'label': det['label'],
                    'x': round(float(pt[0]), 3), 'y': round(float(pt[1]), 3),
                    'radius': round(max(radius, 0.03), 3),
                    'pixels': int(box_depth.size), 'score': round(det['score'], 3)})
    return out
