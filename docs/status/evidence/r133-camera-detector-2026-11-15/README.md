# R13.3 learned camera detector, multi-map retrain — November 15, 2026

Status: **implemented, experimental**. The earlier single-map checkpoint
(`vacuum-camera-model-2026-11-12`) and its measured cable misses and shoe false
positives are kept unchanged.

## What changed

- `scripts/generate_vacuum_camera_dataset.py` renders random object layouts on
  several maps (`--train-maps/--val-maps`, `--train-layouts/--val-layouts`).
  It places them on free floor taken from each map's occupancy image, centring
  each layout automatically and limiting the camera distance with
  `--max-distance`. A share of views has no objects (`--background-fraction`),
  so the model sees empty floor, walls and furniture as negatives.
  `set_pose` is retried, which fixes the small_house fixture timeouts.
- `scripts/train_vacuum_detector.py`: horizontal-flip augmentation (`--hflip`),
  step LR schedule (`--lr-step`), training metadata in the checkpoint.
- `scripts/evaluate_vacuum_detector.py` (new): runs the deployed
  `LearnedDetector` on held-out frames and reports per-class matches (IoU ≥ 0.5),
  false positives and inference time.
- `simulated_room_vacuum.launch.py`: `vacuum_detector_score` (default 0.75).

## Data and training

Dataset 2 is `data2-render-receipt.json`:

- 18 training layouts and 3 held-out layouts across nav_empty, nav_obstacle and
  small_house.
- Seed 29, 50 frames per layout, 1050 images: 900 training and 150 held-out.

Model 2 was trained with
`--epochs 16 --batch 4 --lr 0.01 --lr-step 6 --hflip --min-size 480 --max-size 960 --seed 7`
on CPU (CUDA unavailable on this host).

- Best held-out mAP@0.5: 0.800, at epoch 10. Per class: cable 0.843,
  pet_waste 0.647, shoe 0.904, sock 0.808.
- Source: `model2-metrics.json`.

## Held-out comparison through the deployed loader

Each cell is matched / ground truth (false positives). Three held-out sets:

- **d1**: the November 12 dataset.
- **dA**: the November 15 single-map dataset (model 1's own held-out split).
- **dB**: the multi-map dataset 2 held-out split.

| Model, score | Set | Cable | Pet waste | Shoe | Sock |
|---|---|---|---|---|---|
| Model 1, 0.60 | d1 | 7/13 (11) | 4/5 (1) | 12/12 (16) | 4/4 (0) |
| Model 1, 0.60 | dA | 40/51 (46) | 23/25 (10) | 40/43 (46) | 28/43 (0) |
| Model 1, 0.60 | dB | 62/86 (17) | 31/52 (4) | 47/58 (68) | 54/70 (20) |
| Model 2, 0.60 | d1 | 8/13 (7) | 4/5 (3) | 12/12 (0) | 4/4 (1) |
| Model 2, 0.60 | dA | 42/51 (6) | 22/25 (10) | 40/43 (0) | 29/43 (15) |
| Model 2, 0.60 | dB | 72/86 (24) | 31/52 (8) | 54/58 (1) | 56/70 (22) |
| **Model 2, 0.75** | d1 | 8/13 (2) | 4/5 (3) | 12/12 (0) | 4/4 (1) |
| **Model 2, 0.75** | dA | 40/51 (2) | 20/25 (4) | 40/43 (0) | 27/43 (10) |
| **Model 2, 0.75** | dB | 69/86 (11) | 28/52 (4) | 53/58 (0) | 56/70 (16) |
| Model 2, 0.85 | d1 | 6/13 (1) | 4/5 (2) | 12/12 (0) | 4/4 (0) |
| Model 2, 0.85 | dA | 37/51 (1) | 17/25 (3) | 40/43 (0) | 26/43 (7) |
| Model 2, 0.85 | dB | 61/86 (4) | 25/52 (3) | 50/58 (0) | 49/70 (8) |

The deployed threshold is 0.75. At that threshold:

- Shoe false positives fall from 46–68 (model 1) to 0.
- Cable false positives fall from 46 to 2 on dA.
- Sock false positives remain the weakest point (10–16).
- Pet-waste recall on dB is 54 %.
- Mean inference time is 0.28–0.51 s per frame on this CPU.

Note that dA and dB were rendered by a single `fixture_world` camera
(dataset generation, not robot motion).

Model 2 is installed as the content-addressed `experimental_candidate`
(`installed-model.json`, sha256 `78565183…f231`), after its dataset image,
annotation and split hashes were verified.

## Robot mission

MISSION_RESULT_PLACEHOLDER

## Limits

- All frames are synthetic Gazebo primitives. The learned model has not been
  tested on real objects or real cameras.
- small_house was among the training maps, with different random layouts, so
  the mission world is not a held-out map.
