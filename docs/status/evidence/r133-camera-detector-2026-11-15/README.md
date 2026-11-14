# R13.3 learned camera detector: multi-map retrain and learned-camera mission (November 15, 2026)

**Status:** implemented, experimental. One mission run.

The November 12 checkpoint is unchanged and its receipt is kept as history:
[vacuum-camera-model-2026-11-12](../vacuum-camera-model-2026-11-12/README.md).

## What changed

**`scripts/generate_vacuum_camera_dataset.py`**
- Renders random object layouts across several maps. New options:
  `--train-maps/--val-maps`, `--train-layouts/--val-layouts` and `--scatter`.
- Places objects on free floor from each map's occupancy image, centring the
  layout automatically. `--max-distance` bounds the camera distance.
- `--background-fraction` adds a share of views with no objects. These teach
  that empty floor, walls and furniture are not objects.
- Retries `set_pose`, which fixes the small_house fixture timeouts.

**`scripts/train_vacuum_detector.py`**
- Adds horizontal-flip augmentation (`--hflip`) and a step LR schedule
  (`--lr-step`).
- Stores the training settings in the checkpoint.

**`scripts/evaluate_vacuum_detector.py`** (new)
- Runs the deployed `LearnedDetector` on held-out frames.
- Reports per-class matches at IoU ≥ 0.5, false positives and inference time.

**`simulated_room_vacuum.launch.py`**
- New argument `vacuum_detector_score`, default 0.75.

## Checkpoints

| Checkpoint | Data | Held-out mAP@0.5 |
|---|---|---|
| Nov 12 (history) | 96 views, 3 layouts | 0.564 |
| Retrain 1 (Nov 15) | 720 views; nav_empty train, nav_obstacle val | 0.781 |
| **Retrain 2 (Nov 15), installed** | 1050 views: 18 train layouts and 3 val layouts over nav_empty, nav_obstacle and small_house (`data2-render-receipt.json`) | 0.800 |

Retrain 2 per-class AP: cable 0.843, pet waste 0.647, shoe 0.904, sock 0.808.

Retrain 2 was trained with:
`--epochs 16 --batch 4 --lr 0.01 --lr-step 6 --hflip --min-size 480 --max-size 960 --seed 7`.
Training ran on CPU because the host's driver is too old for CUDA. The best
epoch was 10.

Retrain 2 is installed as the content-addressed `experimental_candidate`:
- sha256 `78565183…f231` (`installed-model.json`);
- its dataset image, annotation and split hashes were verified;
- `vacuum_detector_model:=auto` now resolves to it.

## Held-out comparison through the deployed loader

Each cell is matched / ground truth (false positives). Three held-out sets:

- **d1**: the November 12 set, 32 images.
- **dA**: retrain 1's held-out split, 120 images.
- **dB**: retrain 2's held-out split, 150 images.

| Checkpoint, score | Set | Cable | Pet waste | Shoe | Sock |
|---|---|---|---|---|---|
| Nov 12, 0.40 | d1 | 1/13 (1) | 4/5 (1) | 8/12 (22) | 3/4 (0) |
| Nov 12, 0.40 | dA | 15/51 (11) | 12/25 (8) | 31/43 (36) | 4/43 (2) |
| Nov 12, 0.40 | dB | 32/86 (12) | 16/52 (10) | 31/58 (58) | 11/70 (4) |
| Retrain 1, 0.60 | d1 | 7/13 (11) | 4/5 (1) | 12/12 (16) | 4/4 (0) |
| Retrain 1, 0.60 | dA | 40/51 (46) | 23/25 (10) | 40/43 (46) | 28/43 (0) |
| Retrain 1, 0.60 | dB | 62/86 (17) | 31/52 (4) | 47/58 (68) | 54/70 (20) |
| Retrain 2, 0.60 | d1 | 8/13 (7) | 4/5 (3) | 12/12 (0) | 4/4 (1) |
| Retrain 2, 0.60 | dA | 42/51 (6) | 22/25 (10) | 40/43 (0) | 29/43 (15) |
| Retrain 2, 0.60 | dB | 72/86 (24) | 31/52 (8) | 54/58 (1) | 56/70 (22) |
| **Retrain 2, 0.75** | d1 | 8/13 (2) | 4/5 (3) | 12/12 (0) | 4/4 (1) |
| **Retrain 2, 0.75** | dA | 40/51 (2) | 20/25 (4) | 40/43 (0) | 27/43 (10) |
| **Retrain 2, 0.75** | dB | 69/86 (11) | 28/52 (4) | 53/58 (0) | 56/70 (16) |
| Retrain 2, 0.85 | d1 | 6/13 (1) | 4/5 (2) | 12/12 (0) | 4/4 (0) |
| Retrain 2, 0.85 | dA | 37/51 (1) | 17/25 (3) | 40/43 (0) | 26/43 (7) |
| Retrain 2, 0.85 | dB | 61/86 (4) | 25/52 (3) | 50/58 (0) | 49/70 (8) |

Through this evaluator, the November 12 checkpoint reproduces its historical
result on d1 (cable 1/13 at 0.4).

The deployed setting is retrain 2 at score 0.75. Compared with the November 12
checkpoint:

- Cable matches rise from 1/13 to 8/13 on d1.
- Shoe false positives fall from 22–58 to 0.
- Sock is now the class with the most false positives (10–16 per set).
- Pet-waste recall is 54 % on dB.
- Mean inference time is 0.28–0.51 s per frame on this CPU.

All held-out frames come from a single `fixture_world` camera. That is dataset
generation, not robot motion.

## Learned-camera robot mission

**Setup**
- Gazebo, `oomwoo`, a small_house copy from `scripts/vacuum_object_world.py`
  with its default cable, sock, shoe and pet mess
  (`small_house_objects.objects.json`).
- Zone `[4.0, -3.5, 7.5, -0.5]`.
- `vacuum_detector:=learned`, `vacuum_detector_model:=auto` (retrain 2),
  score 0.75.
- Truth manifest passed through `vacuum_object_truth`.
- Run with `scripts/sim_vacuum_mission.sh` on domain 117, revision `22185690`
  plus this receipt.

**Result:** `completed`, docked and charging. `cleaning_screen` **pass**.

- Coverage 88.3 % (6.4 m²), 283.5 s, 51.4 m driven.
- 0 bumper contacts, 0 stall events.
- Final truth vs. estimate: 0.034 m.
- Report: `gazebo_zone_objects_learned.json`.

Minimum body clearance, from the truth trail with `cleaning_object_clearance`:

| Object | Clearance |
|---|---|
| Cable | 0.620 m |
| Sock | 0.281 m |
| Shoe | 0.232 m |
| Pet mess | 0.427 m |

All four objects were recognised by the learned RGB detector. Remembered
positions vs. truth:

| Object | Sightings | Position error |
|---|---|---|
| Cable | 22 | 0.341 m along the 0.8 m cable; within its extent |
| Shoe | 28 | 0.074 m |
| Pet mess | 14 | 0.029 m |
| Sock | 26 | 0.018 m |

One more cable entry (`o5`, seen once at 1.51 m) lies 0.575 m from the cable's
centre and about 0.2 m past its end. It is a duplicate or false sighting that
the robot also avoided.

Comparison with the ideal-segmentation run (November 10, `0cac1f24`):

| | Ideal segmentation | Learned camera |
|---|---|---|
| Coverage | 86.2 % | 88.3 % |
| Minimum clearance | 0.156 m | 0.232 m |
| Bumper contacts | 0 | 0 |

`artifact-sha256.txt` hashes the generated world and the launch log. Both stay
on SSD under `robot_lab_runtime/vacuum-learned-mission-20261115`.

## Limits

- **One run.** Repeats, other rooms and other robots are unmeasured.
- **Not a held-out map.** small_house was a training map, with different
  random layouts.
- **Synthetic objects only.** All objects are Gazebo primitives. Real objects,
  real cameras and lighting changes are unqualified.
- The ideal segmentation detector stays the launch default.
