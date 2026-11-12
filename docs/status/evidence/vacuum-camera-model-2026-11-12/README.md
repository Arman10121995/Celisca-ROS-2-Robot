# R13.3 initial trained camera implementation — November 12

Initial implementation is complete; **robot mission acceptance remains partial**.
The experimental candidate is provisioned on the SSD and its path prefills the
Vacuum Setup page. Ideal segmentation remains the default. CLI learned selection
resolves `vacuum_detector_model:=auto` against a checksum-verified local manifest.

## Actual implementation run

Baseline `ecaaa133` plus the implementation hashes recorded here, Jetson/CPU:

1. `generate_vacuum_camera_dataset.py` starts owned, isolated Gazebo Fortress
   RGB-D/semantic camera fixtures. Camera FOV, dimensions, clipping and pitch
   come from `oomwoo`'s source camera. Only the dataset fixture is repositioned;
   no robot locomotion is simulated or claimed. Source `nav_empty` layouts
   29/30 are training; source `nav_obstacle` layout 31 is validation.
2. **96 actual rendered views in 74.239 wall seconds**: 64 train / 32 held-out.
   Label-connected instances become COCO boxes; PNGs and metadata are hashed.
3. **Eight actual CPU epochs**, frozen pretrained MobileNet/FPN features and
   trained proposal/ROI heads, batch 2, LR 0.005, seed 7, two torch threads,
   short edge 320 / max edge 640. The best checkpoint is epoch 7 (zero-based 6).
4. `provision_vacuum_detector.py` verifies checkpoint class/architecture metadata
   and every dataset image/annotation/split hash, copies the weights under their
   content hash and atomically publishes the local experimental selector.
5. The normal deployment loader reads the trained input-size configuration;
   actual deployed inference on all 32 held-out RGB frames is retained separately.

Checkpoint SHA-256:
`8cbe5224c8631d7a06493d70964f8a4b8263d8bfa54abc666085b62933d4ea16`.
Installed manifest: `/workspace/molar/robot_lab_runtime/models/vacuum_camera/current.json`.
Weights, raw images and logs stay under SSD storage; no GUI download is needed.

## Measured quality and limits

| Class | AP@0.5 | Deployed matches / GT | Predictions at score ≥ 0.4 |
|---|---:|---:|---:|
| Cable | 0.1810 | 1 / 13 | 2 |
| Pet mess | 0.8182 | 4 / 5 | 5 |
| Shoe | 0.5304 | 8 / 12 | 30 |
| Sock | 0.7273 | 3 / 4 | 3 |

Mean AP@0.5: **0.5642** (11-point interpolation). Mean deployed CPU inference:
**0.1863 wall seconds per image** over this named set. These are actual
source-primitive scene measurements, not general object-recognition performance.
Cable misses and shoe false positives are substantial. Training-quality work
and an actual learned-camera cleaning mission remain before support promotion;
no acceptance threshold was relaxed and no robot mission is marked done.

The trainer's phrase “synthetic images” includes these actual simulated-camera
renders. It does not mean this run used `render_vacuum_detector_data.py`'s PIL
seed renderer. Source-world/layout/split metadata distinguishes them.

## Software checks and artifacts

Four focused model-loader/training checks pass; three actual Tk launch/gallery
checks pass. The three affected ROS packages build in 7.21 seconds. Auto launch
resolution returns the installed immutable model. These are implementation
checks, separate from physical qualification. The complete required CI pipeline
at **ecaaa133** passes ([run](https://github.com/Arman10121995/Celisca-ROS-2-Robot/actions/runs/43155923238));
the model implementation is a later change and requires its own CI.

Raw run directory:
`/workspace/molar/robot_lab_runtime/r133-implementation-20261112`.
`render-receipt.json`, `held-out-metrics.json`, `deployed-inference.json` and
`installed-model.json` are copied actual outputs. `artifact-sha256.json`
identifies retained raw files, including weights and input images.
`producers/` preserves the actual initial capture/training source snapshots;
restore them to their original `scripts/` paths for exact source replay.

## Run

Source ROS/workspace and `scripts/ssd_env.sh`, then choose an unused domain and
partition. The reproducible initial recipe is:

```bash
export ROS_DOMAIN_ID=142
source scripts/gazebo_isolation.sh
robot_lab_isolate_gazebo camera-training-fresh
python3 scripts/generate_vacuum_camera_dataset.py --out "$ROBOT_LAB_RUNTIME_ROOT/new-camera-data" --views 32 --timeout 600
export TORCH_HOME=/workspace/molar/cache/torch
python3 scripts/train_vacuum_detector.py --data "$ROBOT_LAB_RUNTIME_ROOT/new-camera-data" --out "$ROBOT_LAB_RUNTIME_ROOT/new-camera-model" --epochs 8 --batch 2 --device cpu --freeze-features --min-size 320 --max-size 640 --torch-threads 2
python3 scripts/provision_vacuum_detector.py --checkpoint "$ROBOT_LAB_RUNTIME_ROOT/new-camera-model/vacuum_detector.pt" --data "$ROBOT_LAB_RUNTIME_ROOT/new-camera-data"
```

Keep new run directories immutable; do not overwrite this measured candidate.
Follow the remaining implementation queue with R14.1. Wider camera training and
robot/map/backend validation remain explicit later work.
