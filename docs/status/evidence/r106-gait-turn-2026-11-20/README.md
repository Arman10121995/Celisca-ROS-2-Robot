# R10.6: in-place turns about the robot's own frame (November 20, 2026)

**Status: implemented, experimental.** The hexapod now reaches its
saved-map Nav2 goal on PyBullet (0.125 m, 0.023 rad); on 11-20 it did not
pass (0.441 m). The cause was not PyBullet. On every backend, the source gait
turned the hexapod about its leg centre, 0.163 m from the URDF root frame
that odometry and Nav2 use. An in-place turn therefore swung the robot's
frame around a circle.

## What was wrong

`GaitModel.targets` moved each foot with v + ω × (foot − `body_center`).

- For `hexapod_ros2`, `body_center` is (0.115, 0.115) in the root frame,
  because the URDF root sits at a corner of the body.
- A pure-yaw command (base twist with v = 0) therefore moved the root.
- Measured in a 20 s turn, both backends:

| Backend | Turned | Root moved | Per radian |
|---|---|---|---|
| MuJoCo | 1.396 rad | 0.208 m (0.207 m sideways) | 0.149 m |
| PyBullet | 1.455 rad | 0.220 m | 0.151 m |

Near a goal, Nav2's rotate-to-heading therefore pushed the robot out of
the xy tolerance. The 11-20 PyBullet log reads "crept backward while
turning in place".

A second problem: each leg's stride was clipped to `max_stride` on its
own. At full yaw the far feet were clipped and the near feet were not, so
the stance feet no longer moved as one rigid body, and they slipped
against each other. This was true for all three models.

## What changed (`robot_lab_utils/gait_model.py`)

- **Root-frame twist.** Each foot moves with v + ω × r, with r measured
  from the root origin (plus the leg's `body_offset`). The twist is then
  the root frame's, as ROS and Nav2 define it. `body_center` is no longer
  used. The octopod's and centipede's centres are the origin, so their
  arms are unchanged.
- **Rigid stride limit.** One scale factor, set by the longest stride,
  applies to every leg. A pure turn stays a pure rotation, and a straight
  walk is unchanged.
- **Cost.** The hexapod's far feet are 0.40 m from its root, so the
  largest yaw it can execute drops to about 0.05 rad/s (0.07 before). The
  configured `max_yaw` (0.12, also RPP's rotate-to-heading rate) is
  unchanged. The executed rate was already below it.
- **Tests** (`test_gait_contract.py`):
  - an in-place turn is a rigid rotation about the root;
  - translation strides are equal for every leg;
  - both run for all three models. The rotation test fails on the
    previous model for all three.

## Measured: turn probe (`turn_probe.py`, `nav_empty`, 20 s at `max_yaw`, truth)

| Model / backend | Before: turned, root moved | After: turned, root moved |
|---|---|---|
| hexapod / MuJoCo | 1.396 rad, 0.208 m | 0.870 rad, **0.006 m** |
| hexapod / PyBullet | 1.455 rad, 0.220 m | 0.948 rad, **0.011 m** |
| octopod / MuJoCo | 2.387 rad, 0.000 m | 2.152 rad, 0.000 m |
| centipede / MuJoCo | 0.707 rad, 0.220 m | 0.571 rad, **0.112 m** |

- **Octopod.** Its tetrapod schedule is point-symmetric (front-left steps
  with back-right), so the horizontal forces cancel. Its root stays at the
  spawn to float precision before and after the change.
- **Centipede.** Its drift halves but remains. Its spine bends while
  turning (−0.5 ω), and the leg arms do not model the bend. The second
  change below adds the bend.

## Second change: legs on a bending spine

The centipede's root (`base_link`) is its head. Five spine joints chain
six segments 0.18 m apart, and each segment's legs have their foot targets
in that segment's frame. The model now:

- reads each leg root's planar path from the URDF root (fixed joints, and
  spine joints about z; any other moving joint is refused);
- places the root with this tick's commanded spine angles;
- computes the root-frame stride, then turns it into the segment's frame.

At a straight spine the chain gives the authored `body_offset` (tested).
On the bent spine of a full-rate turn, measured with the true segment
geometry, the previous model's stance strides were off a rigid rotation by
up to 10.4 mm per step; the new model is within 0.2 mm.

| Model / backend | Turned | Root moved |
|---|---|---|
| centipede / MuJoCo | 0.569 rad | 0.091 m (first change: 0.112 m) |
| hexapod / MuJoCo (no spine) | 0.871 rad | 0.006 m (unchanged) |

The centipede still drifts. It turns about its head, so its tail feet need
strides 0.9 m long in arm. The common scale cuts every stride to about a
quarter, and it executes 0.028 rad/s of the 0.15 commanded, which points
to foot slip rather than kinematics. That remains for per-model tuning.

Saved-map goals with both changes (MuJoCo, one run each):

| World | Model | Result | End to goal | Heading | Earlier |
|---|---|---|---|---|---|
| `nav_obstacle`, open spawn (−3, 2.5) | **centipede** | **passed** | 0.113 m | 0.125 rad | 11-20: not passed (0.129 m, 0.172 rad off heading) |
| `nav_obstacle`, open spawn | hexapod | passed | 0.123 m | 0.013 rad | 11-20: 0.128 m |
| `nav_obstacle`, open spawn | octopod | passed | 0.097 m | 0.003 rad | 11-20: 0.099 m |
| `nav_empty` | centipede | passed | 0.112 m | 0.048 rad | above: 0.110 m |

This is the centipede's first passing goal in a second world. With one
run per case, it cannot be credited to the spine change rather than to
run-to-run spread.

## Measured: saved-map Nav2 goals (`qualify_gait_saved_map.py`, `nav_empty`, goal 0.5 m ahead, 0.15 m / 0.15 rad)

| Model / backend | Result | End to goal | Heading error | Earlier |
|---|---|---|---|---|
| **hexapod / PyBullet** | **passed** | **0.125 m** | 0.023 rad | 11-20: not passed (0.441 m, 0.259 rad; 0.173 m at the 220 s budget) |
| hexapod / MuJoCo | passed | 0.098 m | 0.030 rad | 11-19: 0.090 m |
| octopod / MuJoCo | passed | 0.083 m | 0.001 rad | 11-19: 0.081 m |
| centipede / MuJoCo | passed | 0.110 m | 0.118 rad | 11-19: 0.157 m (not passed), 0.119 m |

## Limits

- One run per case. The centipede's PyBullet mapping step (too slow on
  11-20) and its `nav_obstacle` corner spawn (inside the no-go margin)
  were not re-run.
- The centipede's in-place turn still slips (0.091 m in 0.57 rad).
- Isaac and Gazebo gait turns were not probed.

## Reproduce

```bash
GAIT_CASE_DIRECTORY=<new dir> GAIT_BACKEND=pybullet GAIT_ROBOT=hexapod_ros2 GAIT_YAW=.12 GAIT_WALK_S=20 \
  python3 docs/status/evidence/r106-gait-turn-2026-11-20/turn_probe.py
python3 scripts/qualify_gait_saved_map.py --output <new dir> --simulator pybullet --timeout 600 --robots hexapod_ros2
```

`turn-*.json` are the probe results. The "before" runs are `turn-hexapod-*`
(the 11-20 HEAD model) and `turn-old-*` (the same model on `PYTHONPATH`);
`turn-fixed-*` are the first change and `turn-spine-*` the second. Traces are on
SSD under `robot_lab_runtime/gait-turn-20261120` (hashes in
`runtime-sha256.txt`).
