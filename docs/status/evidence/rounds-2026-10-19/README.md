# Rounds 51-54, 2026-10-19

The steps that were implemented on October 16 and left queued, run after the
host restart, one simulation at a time. Large outputs stay on the SSD under
`/workspace/molar/robot_lab_runtime/claude-v48` (round 51), `claude-v49g`,
`claude-v52-spin`, `claude-v53-go2` and `claude-v54`.

## Go2 in Isaac: forward arcs (P7)

**Turning on the spot** (`sim_drive_check.py --hold-spin 20`, loc mode,
round 52, [go2-spin/](go2-spin/)). A 0.8 rad/s turn held on nav_empty:

| Isaac setting | Straight (m/s) | 0-4 s | 5-9 s | 10-14 s | 15-19 s (rad/s) |
|---|---|---|---|---|---|
| Defaults | 0.388 | 0.296 | 0.000 | 0.000 | 0.000 |
| Friction-1 floor | 0.372 | 0.014 | 0.001 | -0.001 | 0.001 |
| DC motor on that floor | 0.383 | 0.008 | 0.000 | 0.000 | 0.000 |

The Go2 policy turns for a second or two and then stands. The 0.43 rad/s
that kept Isaac on diff-drive planning came from a 2 s window. The friction
floor and DC motor that fixed B2 do not help here. Round 51's first
attempt ran in display mode, without the policy, and is not counted.

**Missions at `0961445`** (forward arcs in MuJoCo and Gazebo, diff planning
in Isaac; [go2-forward-arcs/](go2-forward-arcs/)):

- Seed 1 passes: A 0.201 m, B 0.048 m.
- Seed 2 aborts goal B: "collision ahead" with B behind it, then the spin
  recovery stalls.
- Seed 3 reaches both goals (0.031 and 0.174 m) but walked only 0.33 m
  before the cancel point, so its cancel step fails.

**Change** (`1ab2b48`). `unitree_go2` lists Isaac in `no_spin_in_place`, so
Nav2 plans forward arcs there too. Seeds 1-3 again: round 53, below.

PENDING_ROUND53

## Camera calibration in four simulators (R7.2)

`scripts/r7_2_calibration_room_check.sh`: Bumperbot turns twice on the spot
in the calibration room (14 chessboards); `ChessboardCalibrator` fits a
pinhole that is compared with the published camera info.

Round 51 ([calibration/](calibration/)): pinhole fit against the camera
info, which has fx 221.77 and is centred at (159.5, 119.5), or at
(160, 119) in PyBullet:

| Backend | Views | fx | cx | cy | RMS (px) | Matrix screen (fx ±3 %, centre ±1.5 px, RMS < 1) |
|---|---|---|---|---|---|---|
| PyBullet | 52 | 222.59 (+0.4 %) | 160.25 | 118.43 | 0.83 | passes |
| Isaac | 58 | 222.31 (+0.2 %) | 158.52 | 120.22 | 1.66 | fails on RMS (the camera noise) |
| Gazebo | — | | | | | crashed: Ogre aborted loading the board mesh |
| MuJoCo | 0 of 428 frames saw a board | | | | | fails |

**Change** (`b563a31`). The board Collada had no normals and its triangles
faced away from the robot (−x). It now has a +x normal and +x winding.
Gazebo and MuJoCo run again in round 54, below.

PENDING_CAL54

## Car course (R5.5)

`scripts/sim_car_course.sh` (round 51, [car-course/](car-course/)).
Course: a cone slalom (four gates 0.5 m either side of a cone line, 1.5 m
apart) and a parking bay (0.8 m gap, nose in). The car must reach each
gate within 0.15 m of truth and the bay within 0.15 m and 10°, with its
footprint clear of every cone and box throughout:

| Backend | Ackermann | Rear-steer | Anti-Ackermann |
|---|---|---|---|
| Gazebo | **passes** (park 0.117 m, −9.7°) | gate 1 aborted | **passes** (park 0.139 m, −8.4°) |
| PyBullet | slalom; park −23.0° | slalom; park −11.7° | slalom; park −14.4° |
| Isaac | slalom; park −19.6° | slalom; park −10.9° | slalom; park aborted (0.50 m, −26.8°) |
| MuJoCo | report lost (see below) | gate 1 aborted | report lost |

- The slalom passes in 8 of 10 runs, with the footprint 0.057-0.114 m
  from the nearest cone.
- In MuJoCo and Gazebo the rear-steer car aborted gate 1: it starts on
  the cone line and its swinging tail reached cone 0's lethal cells
  ("starting point in lethal space").
- Parking reaches the position (0.10-0.14 m) but not the 10° heading. The
  car goal checker ignores heading, so Nav2 stops at the position whatever
  the yaw.
- `sim_car_course.py` could not print two MuJoCo reports (a numpy bool;
  fixed in `3f826e1`); round 55 runs them again.

PENDING_COURSE55

## Car publisher loss (R5.5)

`sim_drive_check.py --command-loss` (round 51,
[command-loss/](command-loss/)): five drive phases, then the publisher stops
and the base is watched for 3 s against simulator truth. Gazebo ran in slam
mode, Isaac in loc mode.

| Backend | Ackermann | Rear-steer | Anti-Ackermann |
|---|---|---|---|
| Gazebo | 0.157 m coast, at rest | 0.157 m, at rest | 0.155 m, at rest |
| Isaac | 0.151 m, at rest | 0.173 m, at rest | 0.153 m, at rest |

All six stop within the 0.35 m coast screen (the tight-left phase at about
0.26 m/s) and end with zero tail velocity.

## Textured small_house captures and drives (R7.2)

**Slam drives** (round 51, [small-house/](small-house/)): the round-25
small_house loop again, now on the true-scale, textured worlds. The
Bumperbot follows a routed 32.6 m loop through the house's waypoints
(-6,3), (6,3) and (6,-3) on wheel odometry while slam_toolbox maps. The
RGB-D stream is recorded too.

| Backend | Route | Driven | Slam RMSE vs truth | Map precision / recall at 0.10 m |
|---|---|---|---|---|
| PyBullet | completed in 358 s | 28.3 m | **1.9 cm** (heading 5.3°) | 0.97 / 0.79 |
| Isaac | completed in 112 s | 28.1 m | **1.3 cm** (heading 0.8°) | 0.97 / 0.78 |
| MuJoCo | 67 back-offs, 137 points skipped | 5.4 m | 1.49 m (heading 113°) | 0.12 / 0.16 |

In MuJoCo the Bumperbot crawled at about 5 mm/s. Its truth height was
−8.5 mm, so it sat below the floor mesh's top at z = 0; the October 16
still capture shows the same −8.5 mm. The house floor collides through
MuJoCo's triangle flex there. Before the cache fix the house was 100 times
too large, so this was never seen. Open.

**Camera captures** (round 51): textured small_house, 30 s standing and
30 s turning, scored with `scripts/r7_2_image_quality.py`
([quality_round51.json](camera/quality_round51.json)):

| Capture | Edge correlation with depth | Temporal noise | ORB repeat | Grey std |
|---|---|---|---|---|
| PyBullet, textured | 0.173 | 0.77 | 0.92 | 52.3 |
| Isaac, textured (default renderer) | 0.162 | 32.1 | 0.23 | 89.8 |
| Isaac, MinimalRendering mode 1 (round 49f) | 0.157 | 0.0 | 1.00 | 91.8 |
| Gazebo / MuJoCo textured (October 16) | 0.458 / 0.470 | 0.0 / 0.03 | 1.00 / 0.995 | 53.6 / 59.9 |

[Frames](camera/min1_pybullet_isaac.jpg), left to right: Isaac
MinimalRendering mode 1, PyBullet textured and Isaac default.
- MinimalRendering mode 1 is noise-free like mode 2. It draws the floor
  flat grey and the walls black, so neither mode is a usable default.
- PyBullet draws the textured floor, but its walls come out untextured
  grey, hence the low edge correlation.

## Clean host (R9.1)

`scripts/clean_host_reproduction.sh` at `90650c2` (round 50,
[clean-host/](clean-host/)). The revision is built in a fresh
`ros:humble-ros-base-jammy` container, then tested and run there:

| Step | Result |
|---|---|
| Image build (rosdep, pip, colcon) | passes in 484 s |
| Fast test tier | 1063 passed, **1 failed**, 26 skipped |
| MuJoCo nav_obstacle mission | passes (A 0.077 m, B 0.105 m, stop 0.14 s) |
| Gazebo mission (NVIDIA runtime) | passes (A 0.107 m, B 0.086 m, stop 0.44 s) |
| Isaac mission (host build mounted) | **failed**: Kit never started, no odometry in 783 s |
| Golden comparison (seeded R7.2 benchmark) | 25 figures, no differences |

**Changes** (`e73f9bd`):
- The failed test expected CELISCA floor 2's world variants without the
  R11 detailed floor; it now includes it.
- The Isaac build links its Kit kernel and extension cache from outside
  its tree (the packman cache and `~/.local/share/ov`). The container now
  mounts each link's directory at its own path.

Round 56 runs the clean host again at HEAD.

PENDING_CLEAN56
