# R9.3: support matrix with rounds 59 and 62 and the November 5 policies (2026-11-09)

`scripts/r9_3_index_missions.py` appended 40 records to
[runtime-evidence-index.yaml](../../runtime-evidence-index.yaml), now 687:

- [round59-batch.yaml](../rounds-2026-10-19/round59-batch.yaml):
  - the car course with the cone-clearance planner settings and the
    steering settle before each shuffle arc, in 12 cells. Gazebo was
    repeated in `course-r2`, indexed second, so the repeat controls the
    Gazebo cells.
  - Go2 `nav_obstacle` in Isaac with `go2_walk_r2`, seeds 1-3 with
    `spin_in_place:=true` and seed 1 with forward arcs.
- [round62-batch.yaml](../rounds-2026-10-19/round62-batch.yaml): the car
  course with the shuffle followed by the straight depth trim, 12 cells.
- [training-nov5-batch.yaml](../training-2026-10-19/training-nov5-batch.yaml):
  drive checks for four policies:
  - `go2_walk_r3` (actuation delay);
  - the first `go2w_walk_r1` and `g1_walk_r1` runs;
  - `go2_rough_r1` (rough-terrain training).

`scripts/r9_3_support_matrix.py` validated every record again and wrote
[support-matrix-current.md](support-matrix-current.md) and
[support-matrix-current.json](support-matrix-current.json).

| | November 5 (647 reports) | November 9 (687 reports) |
|---|---|---|
| Measured cells | 299 | **310** |
| Passed screens | 289 | **298** |
| Failed screens | 9 | 11 |
| Unverified | 1 | 1 |

## Changes (latest record per cell)

**Car course, round 62 (shuffle + trim): full pass in 10 of 12 cells.**
Parking ends 0.003-0.079 m from the bay and within 4.4 degrees (round
58: 0.09-0.18 m).
- Failed → passed: Ackermann and anti-Ackermann in Gazebo; anti-Ackermann
  and rear-steer in PyBullet.
- Passed → failed: anti-Ackermann in Isaac, an RPP "collision ahead"
  abort in the slalom. Gazebo rear-steer also aborts in the slalom.

Round 59 (no trim) passed 5 of 15 runs; most failures parked 0.15-0.18 m
short along the bay. That finding led to the trim.

**Go2 navigation with `go2_walk_r2` in Isaac (new cells):** all four
reports pass the run's own Go2 check. The matrix screen is stricter:
both goals within 0.25 m of truth and a stop within 1 s.
- Seed 2 passes (0.232 / 0.249 m).
- Seed 1 fails (0.298 m) and seed 3 fails (0.252 m).
- The forward-arc run reaches its goals (0.123 / 0.229 m) but takes
  1.33 s to stop.
- The matrix keeps the latest record per cell, so both cells read failed.
- Against round 58 (`go2_walk_r1`, Isaac 0 of 3): the goals are now
  reached; precision is the gap.

**Trained policies (new cells, drive screen, 0.4 m/s straight command):**

| Policy | MuJoCo | Gazebo | Isaac |
|---|---|---|---|
| `go2_walk_r3` (randomised, noise, 20 ms delay) | pass 0.416 m/s, spin 1.07 rad/s, strafe 0.34 | pass 0.355, 0.93, 0.25 | pass **0.319**, 0.80, 0.20 |
| `go2w_walk_r1` (first Go2W) | pass 0.32, 0.70, strafe 0.02 | – | – |
| `g1_walk_r1` (first G1) | **fail**: about 1.0 m/s for 0.4, forward for reverse | – | – |
| `go2_rough_r1`, nav_empty | pass 0.476 | – | pass 0.322 |
| `go2_rough_r1`, terrain_heightfield | **fail**: no motion | – | **fail**: 0.064 |

`go2_walk_r3` closes most of the Isaac speed gap: 0.319 m/s, against
0.261 for r2 and 0.221 for r1. It keeps the strafe outside MuJoCo, which
r2 lost. The rough-terrain policy walks on flat floors but not on the lab
heightfield.

## Scope

One run per cell. Catalog counts and software checks do not qualify a
mission, and unlisted cells are untested. Raw logs and traces are in
`/workspace/molar/robot_lab_runtime/claude-v59`, `claude-v62` and
`training/`.
