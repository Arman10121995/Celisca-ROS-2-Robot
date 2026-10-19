# R9.3: support matrix with the October 16 open items and the MOLAR family (2026-10-19)

`scripts/r9_3_index_missions.py` appended two batches to
[runtime-evidence-index.yaml](../../runtime-evidence-index.yaml), now 591
records:

- [open-items-2026-10-16/mission-batch.yaml](../open-items-2026-10-16/mission-batch.yaml),
  29 records:
  - the car settling missions;
  - the G1 goal-tolerance runs;
  - the G1 and Go2 Isaac runs;
  - B2 in Isaac (drive screen and nav_obstacle seeds 1-3);
  - Go2 with the wider inflation, and with rotate-to-heading (reverted);
  - the Bumperbot Gazebo mission.
- [molar-hri-2026-10-16/molar-batch.yaml](../molar-hri-2026-10-16/molar-batch.yaml),
  16 records: the MOLAR normal-GUI checks. They use the new
  `articulation_screen` kind, which needs drive, inhibited drive with the
  telescope out, the lift and both telescope stages at their limits with
  the stages coupled, a Stop hold and a roller stop after heartbeat loss.

`scripts/r9_3_support_matrix.py` then validated every record again: report
and manifest hashes, report identity and the per-kind screen contract. It
wrote [support-matrix-current.md](support-matrix-current.md) and
[support-matrix-current.json](support-matrix-current.json).

| | October 15 | October 19 |
|---|---|---|
| Indexed reports | 546 | 591 |
| Measured cells | 251 | **269** |
| Passed screens | 247 | **267** |
| Failed screens | 3 | 1 |
| Unverified | 1 | 1 (go2 MuJoCo smoke, no source manifest, unchanged) |

Changes, latest record per cell:

- New: 16 `gui_articulation` cells.
  - `molar_hri` in Gazebo, MuJoCo, PyBullet and Isaac on `nav_empty`,
    `celisca_floor_1_detailed` and `celisca_floor_2_detailed`.
  - `probot_l` on `nav_empty` in all four.
- New: B2 on nav_obstacle in Isaac, and Bumperbot on nav_obstacle in Gazebo.
- Failed → passed: B2's Isaac drive screen (DC-motor actuator and
  friction-1 floor), and G1 on nav_obstacle in Isaac (0.12 m goal).
- Still failed: Go2 on nav_obstacle in Isaac. Its latest record is round
  49c, rotate-to-heading, which failed all three seeds and was reverted.
  Round 49 with forward arcs had seed 1 at 0.283 m (over the 0.25 m
  screen) and seed 3 aborted. A rerun at the reverted configuration is
  queued.

All full-release gates stay blocked: R5-R9 tasks remain partial.
