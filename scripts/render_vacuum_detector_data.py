#!/usr/bin/env python3
"""Render synthetic training images for the vacuum learned floor-object detector.

Produces RGB images that mimic the oomwoo front camera (320x240, HFOV 1.2 rad,
about 0.6 m above the floor looking slightly down) with cable / sock / shoe /
pet_waste objects at randomized poses on a textured floor.  Every pixel carries
its exact class label, so COCO-style bounding-box annotations are derived
directly from the label masks -- no simulator label system required.

Output (--out):
  images/<id>.png          RGB training image
  annotations.json         COCO-style {images, annotations, categories}
  classes.json             {label_id: class_name} mirroring vacuum_objects.DEFAULT_LABELS

The images are deliberately simple (flat shaded primitives on a textured floor)
so a torchvision detector fine-tuned here learns the class shape/colour/size cues
and transfers to the real Gazebo-rendered camera frames the detector runs on.

Example:
  scripts/render_vacuum_detector_data.py --out /tmp/vacuum_detector_data --images 200 --seed 7
"""
import argparse
import json
import math
import os
import random
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

# Oomwoo front camera (see oomwoo_gazebo.xacro): 320x240, HFOV 1.2 rad, 5 Hz.
IMG_W, IMG_H = 320, 240
HFOV = 1.2
FX = FY = (IMG_W / 2.0) / math.tan(HFOV / 2.0)
CX, CY = IMG_W / 2.0, IMG_H / 2.0
CAM_HEIGHT = 0.6
CAM_PITCH = 0.35  # rad, looking down

LABELS = {20: 'cable', 21: 'sock', 22: 'shoe', 23: 'pet_waste'}
SHAPES = {
    'cable':     (0.008, 0.006, 0.80, 0.012, (70, 70, 80)),
    'sock':      (0.11, 0.045, 0.22, 0.09,  (210, 215, 220)),
    'shoe':      (0.14, 0.09, 0.28, 0.10,  (110, 70, 50)),
    'pet_waste': (0.025, 0.025, 0.05, 0.05,  (90, 60, 40)),
}
FLOOR_COLOUR = (110, 110, 105)
DISTRACTOR_COLOUR = (90, 90, 85)

rng = random.Random()


def rot_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def rot_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def project(points_cam):
    z = points_cam[:, 2]
    valid = z > 0.05
    u = np.full(len(points_cam), np.nan)
    v = np.full(len(points_cam), np.nan)
    u[valid] = FX * points_cam[valid, 0] / z[valid] + CX
    v[valid] = FY * points_cam[valid, 1] / z[valid] + CY
    return u, v, valid

def world_to_camera(Pw):
    R = rot_x(CAM_PITCH)
    t = np.array([0.0, CAM_HEIGHT, 0.0])
    return (R.T @ (Pw - t).T).T


def object_box_3d(cls, centre, yaw, size_jitter):
    sx, sy, sz, height, _ = SHAPES[cls]
    sx *= size_jitter
    sz *= size_jitter
    cx, cy, cz = centre
    half = np.array([sx / 2.0, height / 2.0, sz / 2.0])
    corners = np.array([[x, y, z] for x in (-half[0], half[0])
                        for y in (-half[1], half[1])
                        for z in (-half[2], half[2])], dtype=float)
    corners = corners @ rot_y(yaw).T
    corners[:, 1] += 0.0
    return corners + np.array([cx, cy, cz])


def _draw_poly(rgb_draw, label_draw, points_2d, colour, lab):
    pts = [(float(p[0]), float(p[1])) for p in points_2d
           if math.isfinite(p[0]) and math.isfinite(p[1])]
    if len(pts) < 3:
        return
    rgb_draw.polygon(pts, fill=colour)
    if lab:
        label_draw.polygon(pts, fill=lab)


def render_scene(objects, distractors, nrng):
    base = np.full((IMG_H, IMG_W, 3), FLOOR_COLOUR, dtype=np.uint8)
    noise = (nrng.random((IMG_H, IMG_W)) * 40 - 20).astype(np.int16)
    rgb = np.clip(base.astype(np.int16) + noise[..., None], 0, 255).astype(np.uint8)
    label = np.zeros((IMG_H, IMG_W), dtype=np.uint8)
    cam_R = rot_x(CAM_PITCH)
    cam_t = np.array([0.0, CAM_HEIGHT, 0.0])
    label_of = {v: k for k, v in LABELS.items()}
    rgb_img = Image.fromarray(rgb.copy(), 'RGB')
    label_img = Image.fromarray(label.copy(), 'L')

    for cls, centre, yaw in objects:
        box = object_box_3d(cls, centre, yaw, 1.0)
        pc = (cam_R.T @ (box - cam_t).T).T
        u, v, valid = project(pc)
        if not valid.any():
            continue
        pts = np.column_stack([u, v])
        pts[:, 0] = np.clip(pts[:, 0], 0, IMG_W - 1)
        pts[:, 1] = np.clip(pts[:, 1], 0, IMG_H - 1)
        _sx, _sy, _sz, _h, colour = SHAPES[cls]
        _draw_poly(ImageDraw.Draw(rgb_img), ImageDraw.Draw(label_img), pts, colour, label_of[cls])

    for centre, yaw, sx, sz in distractors:
        box = object_box_3d('sock', centre, yaw, 1.0)
        box[:, 0] *= sx / 0.22
        box[:, 2] *= sz / 0.22
        pc = (cam_R.T @ (box - cam_t).T).T
        u, v, valid = project(pc)
        if not valid.any():
            continue
        pts = np.column_stack([u, v])
        pts[:, 0] = np.clip(pts[:, 0], 0, IMG_W - 1)
        pts[:, 1] = np.clip(pts[:, 1], 0, IMG_H - 1)
        _draw_poly(ImageDraw.Draw(rgb_img), ImageDraw.Draw(label_img), pts, DISTRACTOR_COLOUR, 0)

    rgb = np.array(rgb_img)
    label = np.array(label_img)
    yy, xx = np.mgrid[0:IMG_H, 0:IMG_W]
    vignette = 1.0 - 0.15 * ((xx - CX) ** 2 / CX ** 2 + (yy - CY) ** 2 / CY ** 2)
    rgb = np.clip(rgb.astype(np.float32) * vignette[..., None], 0, 255).astype(np.uint8)
    return rgb, label
def boxes_from_mask(label, class_id):
    ys, xs = np.nonzero(label == class_id)
    if len(xs) == 0:
        return []
    x0, x1 = int(xs.min()), int(xs.max())
    y0, y1 = int(ys.min()), int(ys.max())
    return [[x0, y0, x1 - x0 + 1, y1 - y0 + 1]]


def place_object(min_dist, max_dist, angle_spread):
    z = rng.uniform(min_dist, max_dist)
    ang = rng.uniform(-angle_spread, angle_spread)
    x = z * math.tan(ang) + rng.uniform(-0.15, 0.15)
    return (x, 0.0, z), ang


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--images', type=int, default=150)
    ap.add_argument('--val-fraction', type=float, default=0.2)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--min-dist', type=float, default=0.9)
    ap.add_argument('--max-dist', type=float, default=3.2)
    ap.add_argument('--angle-spread', type=float, default=0.6)
    args = ap.parse_args()
    rng.seed(args.seed)
    nrng = np.random.default_rng(args.seed)

    out = args.out
    img_dir = out / 'images'
    img_dir.mkdir(parents=True, exist_ok=True)

    cat_id = {name: i + 1 for i, name in enumerate(sorted(LABELS.values()))}
    images, annotations = [], []
    ann_id = 1

    for i in range(args.images):
        n_obj = rng.randint(1, 3)
        classes = rng.sample(list(LABELS.values()), n_obj)
        objects = []
        for cls in classes:
            centre, yaw = place_object(args.min_dist, args.max_dist, args.angle_spread)
            objects.append((cls, centre, yaw))
        distractors = []
        for _ in range(rng.randint(0, 2)):
            c, _ = place_object(1.0, 3.0, 0.9)
            distractors.append((c, rng.uniform(-math.pi, math.pi),
                                rng.uniform(0.3, 0.7), rng.uniform(0.3, 0.7)))

        rgb, label = render_scene(objects, distractors, nrng)

        a = rng.uniform(0.7, 1.3)
        b = rng.uniform(-15, 15)
        rgb = np.clip(rgb.astype(np.float32) * a + b, 0, 255).astype(np.uint8)
        if rng.random() < 0.4:
            rgb = np.array(Image.fromarray(rgb).filter(ImageFilter.GaussianBlur(radius=rng.uniform(0.5, 1.5))))
        if rng.random() < 0.4:
            rgb = np.clip(rgb.astype(np.int16) + nrng.integers(-12, 12, rgb.shape), 0, 255).astype(np.uint8)

        img_id = i + 1
        name = f'{img_id:05d}.png'
        Image.fromarray(rgb).save(img_dir / name)

        images.append({'id': img_id, 'width': IMG_W, 'height': IMG_H,
                       'file_name': name, 'license': 1, 'flickr_url': '', 'coco_url': '', 'date_captured': ''})
        for cls, centre, yaw in objects:
            lab = {v: k for k, v in LABELS.items()}[cls]
            boxes = boxes_from_mask(label, lab)
            if not boxes:
                continue
            x, y, w, h = boxes[0]
            if w < 3 or h < 3:
                continue
            annotations.append({'id': ann_id, 'image_id': img_id,
                                'category_id': cat_id[cls], 'bbox': [x, y, w, h],
                                'area': w * h, 'iscrowd': 0})
            ann_id += 1

    coco = {
        'images': images,
        'annotations': annotations,
        'categories': [{'id': cid, 'name': name, 'supercategory': 'floor_object'}
                       for name, cid in cat_id.items()],
    }
    (out / 'annotations.json').write_text(json.dumps(coco))
    (out / 'classes.json').write_text(json.dumps({str(k): v for k, v in LABELS.items()}))
    n_val = max(1, int(len(images) * args.val_fraction))
    (out / 'val_ids.json').write_text(json.dumps(sorted({im['id'] for im in images[-n_val:]})))
    print(f'rendered {len(images)} images, {len(annotations)} boxes -> {out}')


if __name__ == '__main__':
    main()

