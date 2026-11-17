# R13.3 vacuum camera detector: retrain 3 (November 17, 2026)

**Status:** implemented, experimental. Retrain 3 is the installed `auto`
checkpoint (`experimental_candidate`, sha256 `aa3cc339…bd4e9`,
`installed-model.json`). One mission run.

## What changed

- **Training data:** 750 new Gazebo renders (`data3-render-receipt.json`):
  - seed 101, 15 training and 3 validation layouts over nav_empty,
    nav_obstacle and small_house;
  - a quarter of the views are background-only;
  - views are weighted towards pet waste (2.0) and socks (1.5).
- **Fine-tune from retrain 2:** `train_vacuum_detector.py --init` (new);
  8 epochs at lr 0.003, step 4, with flips, on CPU (5 min per epoch). The
  checkpoint records the parent's sha256.
- **Provisioning:** `provision_vacuum_detector.py` accepts a fine-tune only
  if its parent checkpoint is installed, with its own provenance.

## Held-out comparison through the deployed loader

None of these images were trained on by retrain 2 or 3. Each cell is
matched / ground truth (false positives).

| Checkpoint, score | Set | Cable | Pet waste | Shoe | Sock |
|---|---|---|---|---|---|
| Retrain 2, 0.75 | d1 | 8/13 (2) | 4/5 (3) | 12/12 (0) | 4/4 (1) |
| Retrain 2, 0.75 | dA | 40/51 (2) | 20/25 (4) | 40/43 (0) | 27/43 (10) |
| Retrain 2, 0.75 | dB | 69/86 (11) | 28/52 (4) | 53/58 (0) | 56/70 (16) |
| **Retrain 3, 0.75** | d1 | 8/13 (0) | 4/5 (0) | 12/12 (0) | 4/4 (1) |
| **Retrain 3, 0.75** | dA | 42/51 (4) | 20/25 (2) | 40/43 (0) | 28/43 (6) |
| **Retrain 3, 0.75** | dB | 72/86 (7) | 31/52 (3) | 52/58 (0) | 55/70 (17) |
| Retrain 3, 0.85 | d1 | 8/13 (0) | 4/5 (0) | 12/12 (0) | 4/4 (1) |
| Retrain 3, 0.85 | dA | 35/51 (3) | 19/25 (2) | 40/43 (0) | 26/43 (2) |
| Retrain 3, 0.85 | dB | 71/86 (3) | 28/52 (2) | 52/58 (0) | 52/70 (11) |

At the deployed 0.75:

- **Pet waste:** false positives fall on every set (3 → 0, 4 → 2, 4 → 3),
  and dB recall rises from 54 % to 60 %.
- **Cable:** false positives fall on d1 and dB; dA rises from 2 to 4.
- **Sock:** false positives fall on dA (10 → 6) but not on dB (16 → 17).

Sock false positives and pet-waste recall remain the weak classes. Files:
`model3-s0.75.json`, `model3-s0.85.json`.

## Learned-camera mission

The setup is the same as the November 15 run: Gazebo, `oomwoo`, the
small_house object world (cable, sock, shoe, pet mess), zone
`[4.0, -3.5, 7.5, -0.5]`, `vacuum_detector:=learned`,
`vacuum_detector_model:=auto` (now retrain 3) and the truth manifest.

**Result:** `completed`, docked and charging, `cleaning_screen` **pass**.

- **Cleaning:** coverage 86.9 % (5.8 m²) in 273.4 s; 51.7 m driven.
- **Contacts:** 0 bumper contacts, 0 stall events.
- **Clearance:** minimum body clearance 0.286 m by truth (retrain 2:
  0.232 m). Per object: cable 0.69, sock 0.33, shoe 0.29, pet waste
  0.48 m.
- **Recognition:**
  - all four objects recognised, each seen 39–63 times;
  - three extra low-count sightings: a shoe seen once at a clear spot, and
    second cable (5×) and sock (1×) tracks next to the real ones.
- Report: `gazebo_zone_objects_learned3.json`; hashes in
  `artifact-sha256.txt`.

## Limits

- This is one run, on a map used in training.
- Repeats, a held-out room and real objects are validation (V1).
- Ideal segmentation stays the launch default.
