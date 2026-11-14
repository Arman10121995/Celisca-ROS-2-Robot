#!/usr/bin/env python3
"""Fine-tune a torchvision detection model for the vacuum floor-object classes.

Trains a ``fasterrcnn_mobilenet_v3_large_fpn`` (COCO-pretrained, permissive BSD
license) on images rendered by ``render_vacuum_detector_data.py`` for the four
vacuum camera classes: cable, sock, shoe, pet_waste.  No AGPL code is used.

Output (--out):
  vacuum_detector.pt       fine-tuned model state_dict + class map
  vacuum_detector_classes.json  {label_id: class_name}
  metrics.json             per-class AP@0.5 and mAP on the val split

Example:
  scripts/train_vacuum_detector.py --data <render_out> --out models --epochs 8"""
import json
import hashlib
import time
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader, Subset
from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_fpn
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
import os
from PIL import Image
from torchvision.transforms import functional as F
from torchvision.ops import box_iou

LABELS = {20: 'cable', 21: 'sock', 22: 'shoe', 23: 'pet_waste'}
CAT_ID = {name: i + 1 for i, name in enumerate(sorted(LABELS.values()))}
CLASS_NAMES = sorted(LABELS.values())  # cable, pet_waste, shoe, sock (alphabet)


class DetData(torch.utils.data.Dataset):
    """Custom dataset for rendered vacuum detector images.

    The data dir (--data) contains:
      images/<id:05d>.png   -- rendered RGB images
      annotations.json       -- COCO style {images, annotations, categories}
      val_ids.json           -- val split image ids
    Each annotation has {bbox: [x,y,w,h], category_id: 1..4}.
    """
    def __init__(self, root, ann_file, val_file=None, transform=None):
        self.root = str(root)
        self.ann = json.load(open(ann_file))
        if {c['name']: c['id'] for c in self.ann.get('categories', [])} != CAT_ID:
            raise ValueError('Dataset categories do not match the four vacuum classes')
        self.images = self.ann['images']
        # build lookups: file_name -> image record, image_id -> annotations
        self.name2img = {img['file_name']: img for img in self.images}
        self.img2anns = {}
        for ann in self.ann['annotations']:
            self.img2anns.setdefault(ann['image_id'], []).append(ann)
        # all image ids
        self.ids = [img['id'] for img in self.images]
        # val split ids
        if val_file and os.path.exists(val_file):
            self.val_ids = set(json.load(open(val_file)))
        else:
            self.val_ids = set()
        # build default transform if none provided
        self.transform = transform or self._default_transform

    @staticmethod
    def _default_transform(pil_img):
        # GeneralizedRCNNTransform normalizes internally, as during inference.
        return F.to_tensor(pil_img)

    def __len__(self):
        return len(self.ids)

    def __getitem__(self, idx):
        img_id = self.ids[idx]
        # find image record
        img_rec = next(i for i in self.images if i['id'] == img_id)
        fname = img_rec['file_name']
        pil_img = Image.open(os.path.join(self.root, 'images', fname)).convert('RGB')
        if self.transform:
            img = self.transform(pil_img)
        else:
            img = self._default_transform(pil_img)
        # build target dict
        anns = self.img2anns.get(img_id, [])
        boxes, labels = [], []
        for ann in anns:
            x, y, w, h = ann['bbox']
            if w <= 0 or h <= 0:
                continue
            boxes.append([x, y, x + w, y + h])
            labels.append(int(ann['category_id']))
        target = {}
        if len(boxes) > 0:
            target['boxes'] = torch.as_tensor(boxes, dtype=torch.float32).reshape(-1, 4)
            target['labels'] = torch.as_tensor(labels, dtype=torch.int64)
        else:
            target['boxes'] = torch.zeros((0, 4), dtype=torch.float32)
            target['labels'] = torch.zeros((0,), dtype=torch.int64)
        target['image_id'] = torch.tensor([img_id])
        return img, target

    @staticmethod
    def collate_fn(batch):
        return tuple(zip(*batch))

class HorizontalFlip(torch.utils.data.Dataset):
    """Training-only mirror augmentation: the image and its boxes flip together with probability *p*."""

    def __init__(self, subset, p=0.5, seed=7):
        self.subset, self.p = subset, p
        self.generator = torch.Generator().manual_seed(seed)

    def __len__(self):
        return len(self.subset)

    def __getitem__(self, idx):
        image, target = self.subset[idx]
        if float(torch.rand(1, generator=self.generator)) < self.p:
            width = image.shape[-1]
            image = image.flip(-1)
            boxes = target['boxes'].clone()
            if len(boxes):
                boxes[:, [0, 2]] = width - target['boxes'][:, [2, 0]]
            target = dict(target, boxes=boxes)
        return image, target


def train_one_epoch(model, optimizer, data_loader, device):
    model.train()
    total = 0.0
    n = 0
    for images, targets in data_loader:
        # images are already tensors from the dataset; no convert_image_dtype needed
        images = [im.to(device) for im in images]
        targets = [{k: v.to(device) for k, v in t.items()} for t in targets]
        loss_dict = model(images, targets)
        loss = sum(loss_dict.values())
        if not torch.isfinite(loss):
            raise RuntimeError('Non-finite detector training loss')
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        total += float(loss)
        n += 1
    return total / max(n, 1)


def evaluate(model, data_loader, device):
    """AP@0.5 per class (11-point interpolation); mAP = mean over classes."""
    model.eval()
    preds = {1: [], 2: [], 3: [], 4: []}
    gts = {1: [], 2: [], 3: [], 4: []}
    with torch.no_grad():
        for images, targets in data_loader:
            images = [im.to(device) for im in images]
            outputs = model(images)
            for output, target in zip(outputs, targets):
                boxes = output['boxes'].cpu().numpy()
                scores = output['scores'].cpu().numpy()
                labels = output['labels'].cpu().numpy()
                gt_boxes = target['boxes'].cpu().numpy()
                gt_labels = target['labels'].cpu().numpy()
                for cls in (1, 2, 3, 4):
                    pb = boxes[labels == cls]
                    ps = scores[labels == cls]
                    order = np.argsort(-ps) if len(ps) else []
                    preds[cls].append((pb[order], ps[order]))
                    gts[cls].append(gt_boxes[gt_labels == cls])

    aps = {}
    for cls in (1, 2, 3, 4):
        all_scores, all_tp, n_gt = [], [], 0
        for (pb, ps), gb in zip(preds[cls], gts[cls]):
            n_gt += len(gb)
            matched = np.zeros(len(gb), dtype=bool)
            for i in range(len(pb)):
                if len(gb) == 0:
                    all_scores.append(float(ps[i])); all_tp.append(0); continue
                ious = box_iou(torch.from_numpy(pb[i:i + 1]),
                               torch.from_numpy(gb))[0].numpy()
                j = int(ious.argmax())
                if ious[j] >= 0.5 and not matched[j]:
                    matched[j] = True
                    all_scores.append(float(ps[i])); all_tp.append(1)
                else:
                    all_scores.append(float(ps[i])); all_tp.append(0)
        if n_gt == 0:
            aps[cls] = 0.0
            continue
        order = np.argsort(-np.array(all_scores))
        tp = np.array(all_tp, dtype=float)[order]
        fp = 1 - tp
        tp_c = np.cumsum(tp); fp_c = np.cumsum(fp)
        rec = tp_c / n_gt
        prec = tp_c / np.maximum(tp_c + fp_c, 1e-9)
        ap = 0.0
        for t in np.linspace(0, 1, 11):
            p = float(prec[rec >= t].max()) if np.any(rec >= t) else 0.0
            ap += p / 11.0
        aps[cls] = float(ap)
    return aps, float(np.mean(list(aps.values())))


def split_dataset(dataset):
    """Honor the renderer's held-out IDs; reject empty or malformed splits."""
    known = set(dataset.ids)
    if len(known) != len(dataset.ids) or not dataset.val_ids <= known:
        raise ValueError('Duplicate image IDs or unknown validation IDs')
    train = [i for i, image_id in enumerate(dataset.ids) if image_id not in dataset.val_ids]
    val = [i for i, image_id in enumerate(dataset.ids) if image_id in dataset.val_ids]
    if not train or not val:
        raise ValueError('Both training and held-out validation images are required')
    if hasattr(dataset, 'img2anns'):
        for name, indices in [('training', train), ('validation', val)]:
            present = {annotation['category_id'] for i in indices
                       for annotation in dataset.img2anns.get(dataset.ids[i], [])}
            missing = set(CAT_ID.values()) - present
            if missing:
                raise ValueError('%s split lacks classes: %s' % (
                    name, ', '.join(cls for cls, key in CAT_ID.items() if key in missing)))
    if hasattr(dataset, 'images'):
        scenes = {}
        for image in dataset.images:
            if 'scene' not in image:
                continue
            split = 'val' if image['id'] in dataset.val_ids else 'train'
            if image.get('split', split) != split:
                raise ValueError('Validation IDs disagree with captured split metadata')
            scenes.setdefault(image['scene'], set()).add(split)
        if any(len(splits) > 1 for splits in scenes.values()):
            raise ValueError('The same scene appears in training and validation')
    return Subset(dataset, train), Subset(dataset, val)


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--data', type=Path, required=True, help='render_vacuum_detector_data output dir')
    ap.add_argument('--out', type=Path, required=True, help='directory for the checkpoint')
    ap.add_argument('--epochs', type=int, default=8)
    ap.add_argument('--batch', type=int, default=4)
    ap.add_argument('--lr', type=float, default=0.005)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--device', default='cpu')
    ap.add_argument('--min-size', type=int, default=320,
                    help='Resize short edge for both training and deployed inference')
    ap.add_argument('--max-size', type=int, default=640)
    ap.add_argument('--torch-threads', type=int, default=2)
    ap.add_argument('--freeze-features', action='store_true',
                    help='Fine-tune proposals/ROI heads over frozen pretrained MobileNet/FPN features')
    ap.add_argument('--lr-step', type=int, default=3, help='Epochs between 10x learning-rate drops')
    ap.add_argument('--hflip', action='store_true', help='Mirror training images (and boxes) half the time')
    ap.add_argument('--weights', choices=['coco', 'none'], default='coco',
                    help='none: offline architecture smoke test only, not a trained model claim')
    args = ap.parse_args()
    if args.epochs <= 0 or args.batch <= 0 or not np.isfinite(args.lr) or args.lr <= 0:
        ap.error('Positive epochs, batch size and finite learning rate are required')
    if not 64 <= args.min_size <= args.max_size <= 2048 or args.torch_threads <= 0:
        ap.error('Require 64 <= min-size <= max-size <= 2048 and positive torch threads')
    torch.set_num_threads(args.torch_threads)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    full = DetData(str(args.data), str(args.data / 'annotations.json'),
                   val_file=str(args.data / 'val_ids.json'))
    train_set, val_set = split_dataset(full)
    image_hashes = {}
    for image in full.images:
        digest = hashlib.sha256((args.data / 'images' / image['file_name']).read_bytes()).hexdigest()
        if image.get('sha256', digest) != digest:
            raise ValueError('Captured image hash changed: %s' % image['file_name'])
        image_hashes[image['file_name']] = digest
    train_loader = DataLoader(HorizontalFlip(train_set, seed=args.seed) if args.hflip else train_set,
                              batch_size=args.batch,
                              shuffle=True, collate_fn=DetData.collate_fn, num_workers=0)
    val_loader = DataLoader(val_set,
                            batch_size=args.batch,
                            shuffle=False, collate_fn=DetData.collate_fn, num_workers=0)
    device = torch.device(args.device)
    model_kwargs = {'min_size': args.min_size, 'max_size': args.max_size}
    model = fasterrcnn_mobilenet_v3_large_fpn(
        weights='DEFAULT' if args.weights == 'coco' else None, weights_backbone=None,
        **model_kwargs)
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, len(CAT_ID) + 1)
    if args.freeze_features:
        for parameter in model.backbone.parameters():
            parameter.requires_grad_(False)
    model.to(device)

    params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.SGD(params, lr=args.lr, momentum=0.9, weight_decay=1e-4)
    lr_scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=args.lr_step, gamma=0.1)

    best = {'mAP': -1.0}
    for epoch in range(args.epochs):
        t0 = time.time()
        loss = train_one_epoch(model, optimizer, train_loader, device)
        aps, mAP = evaluate(model, val_loader, device)
        lr_scheduler.step()
        print(f'epoch {epoch+1}/{args.epochs} loss={loss:.4f} mAP@0.5={mAP:.3f} '
              + ' '.join(f'{CLASS_NAMES[k-1]}={v:.3f}' for k, v in sorted(aps.items()))
              + f' ({time.time()-t0:.0f}s)', flush=True)
        if mAP > best['mAP']:
            best['mAP'] = mAP
            args.out.mkdir(parents=True, exist_ok=True)
            torch.save({
                'model': model.state_dict(),
                'labels': LABELS,
                'cat_id': CAT_ID,
                'epoch': epoch,
                'weights_initialization': args.weights,
                'seed': args.seed,
                'architecture': 'fasterrcnn_mobilenet_v3_large_fpn',
                'model_kwargs': model_kwargs,
                'freeze_features': args.freeze_features,
                'training': {'epochs': args.epochs, 'batch': args.batch, 'lr': args.lr, 'lr_step': args.lr_step,
                             'hflip': args.hflip},
                'torch_version': str(torch.__version__),
                'annotations_sha256': hashlib.sha256((args.data / 'annotations.json').read_bytes()).hexdigest(),
                'split_sha256': hashlib.sha256((args.data / 'val_ids.json').read_bytes()).hexdigest(),
                'image_sha256': image_hashes,
                'train_ids': [full.ids[i] for i in train_set.indices],
                'val_ids': [full.ids[i] for i in val_set.indices],
            }, args.out / 'vacuum_detector.pt')
            (args.out / 'vacuum_detector_classes.json').write_text(json.dumps({str(k): v for k, v in LABELS.items()}))
            (args.out / 'metrics.json').write_text(json.dumps({
                'mAP': mAP, 'aps': {CLASS_NAMES[k-1]: v for k, v in aps.items()}, 'epoch': epoch,
                'train_images': len(train_set), 'val_images': len(val_set),
                'scope': 'held-out synthetic images; not a physical cleaning mission'}))
    print('best mAP@0.5:', best['mAP'])


if __name__ == '__main__':
    main()

