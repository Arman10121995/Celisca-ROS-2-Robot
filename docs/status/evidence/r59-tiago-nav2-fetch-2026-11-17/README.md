# R5.9 TIAGo fetch with Nav2 transit legs (November 17, 2026)

**Status: implemented, experimental.**

TIAGo's fetch now drives its transit legs with Nav2 and AMCL localization in
Nav mode. The earlier fetch used only straight legs steered by simulator
truth ([receipt](../r59-tiago-fetch-2026-11-17/README.md)). It passes on
MuJoCo (twice) and PyBullet (once) in `mobile_fetch_lab`.

These runs were measured while a Go2 training job used most of the CPU (load
about 11 on 12 cores). Times are not comparable with quiet-host runs.

## How a fetch now drives

| Leg | Driven by |
|---|---|
| Start → run-up point before the pickup table | **Nav2** `navigate_to_pose` (map frame), when the leg is ≥ 0.5 m |
| Run-up point → pick standoff (0.6 m straight in) | straight docking move, steered by truth |
| Back-off from the table (0.6 m) | straight docking move |
| Back-off point → run-up point before the drop table | **Nav2**, carrying the object |
| Run-up point → place standoff | straight docking move |
| Final back-off | straight docking move |

- **Hand-off:** once AMCL puts the base within 0.15 m of the run-up point,
  the Nav2 goal is cancelled and docking starts. Earlier, DWB circled for
  over 100 s without meeting its 7 cm goal tolerance.
- **Footprint while carrying:** the costmap footprint is the convex hull of
  the base circle (0.3 m) and a strip out to 5 cm past the held object.
  Afterwards it returns to the base circle.
- **Arm planning** still uses the truth base pose, as the object poses do.
  Perception remains a separate R5.9 item.
- **Mutual inhibition:** the base and arm never move together. If the arm
  moves during a Nav2 leg, the task fails.

## What was added

- **`urdf_pick_place.py`:**
  - Nav2 transit legs, hand-off, carry footprint;
  - each Nav2 arrival is logged with its goal error and localization error
    (`/amcl_pose` against truth);
  - a request may set `"transit": "auto" | "nav2" | "straight"`;
  - the drive timeout is now 240 s per phase.
- **Launch:** the fetch node also runs in the non-Display modes.
- **TIAGo nav modes:** `scripts/provision_tiago.py` declares experimental
  Loc/SLAM/3D SLAM/Nav modes on MuJoCo, PyBullet and Isaac. The installed
  profile was re-provisioned; only those modes changed.
- **Nav2 overlay** `robot_lab_navigation/config/robots/asset_pal_tiago.yaml`:
  - footprint: round, 0.3 m radius;
  - inflation 0.55 m;
  - DWB limits: 0.2 m/s, 0.4 rad/s, 0.2 m/s².
- **`arm_pick_place_check.py --transit`.**

## Measured

Command: `run_with_launch.py --launch "robot_model:=asset_pal_tiago
simulator:=<sim> mode:=nav map_name:=mobile_fetch_lab
allow_experimental_assets:=true spawn_x:=-1.0 spawn_y:=-1.0"` with
`arm_pick_place_check.py --object fetch_block --support drop_table
--transit nav2`.

| Run | Result | Total | Nav2 to table / carry | Base error at hand-off (truth) | AMCL error | Carry drift | Place height error |
|---|---|---|---|---|---|---|---|
| MuJoCo 4 | passed | 265.1 s | 65.0 s / 74.8 s | 0.26 / 0.18 m | 0.12 / 0.07 m | 9.8 mm | −0.1 mm |
| MuJoCo 5 | passed | 262.1 s | 53.1 s / 68.4 s | 0.23 / 0.19 m | 0.12 / 0.07 m | 9.6 mm | −0.1 mm |
| PyBullet 1 | passed | 117.5 s | 15.2 s / 12.0 s | 0.20 / 0.11 m | 0.10 / 0.13 m | 26.2 mm | 0.0 mm |

**Interlock** (base motion during arm phases): at most 1.7 mm/s and
0.006 rad/s, with no drive ticks while the articulation was active.

**Observations:**

- PyBullet's carry drift (26.2 mm) is close to the 30 mm limit. Its
  straight-leg fetch had 17.1 mm.
- The MuJoCo legs are slow because TIAGo turns slowly there: its wheel force
  is limited to 6 N.

### Not passed (kept)

1. **MuJoCo 1:** spawned at the origin, the first Nav2 leg was only 0.23 m.
   DWB turned the base 0.95 rad away and aborted with "failed to make
   progress". Fix: legs under 0.5 m stay straight.
2. **MuJoCo 2:** the carry footprint was a 0.85 m circle. The grid planner
   plans for the base centre only, so its path ran near the pickup table.
   The circle then overlapped the table, and every replan failed ("Starting
   point in lethal space"). Fix: the real carry outline.
3. **MuJoCo 3:** Nav2 reached the run-up point in 12 s, then circled it for
   over 100 s without meeting the 7 cm tolerance, and the drive timed out.
   Fix: the localization hand-off.

## Limits

- Three passing runs in one open world, all measured under CPU load.
- AMCL error is 0.07–0.13 m in this sparse world (two tables). The truth
  docking moves absorb it; a fetch steered by localization alone would
  miss the standoff.
- Arm planning and object poses still come from simulator truth.
- Isaac was not run in Nav mode.
- Launch logs stay on SSD under `robot_lab_runtime/tiago-nav-fetch-20261117`;
  `launch-log-sha256.txt` has their hashes.
