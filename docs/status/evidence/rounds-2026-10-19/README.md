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

PENDING_CALIBRATION

## Car course (R5.5)

PENDING_COURSE

## Car publisher loss (R5.5)

PENDING_LOSS

## Textured small_house captures and drives (R7.2)

PENDING_HOUSE

## Clean host (R9.1)

PENDING_CLEAN
