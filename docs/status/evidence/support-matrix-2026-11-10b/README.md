# R9.3: support matrix with the later November 10 vacuum runs

`scripts/r9_3_index_missions.py docs/status/evidence/vacuum-2026-11/vacuum-batch.yaml`
appended 20 records to [runtime-evidence-index.yaml](../../runtime-evidence-index.yaml)
(now 717); `scripts/r9_3_support_matrix.py --out-directory <this folder>`
regenerated the matrix. Every vacuum report is judged by `cleaning_screen`.

| | November 10 (697 reports) | Later November 10 (717 reports) |
|---|---|---|
| Measured cells | 319 | **327** |
| Passed screens | 307 | **314** |
| Failed screens | 11 | **12** |
| Unverified | 1 | 1 |

New oomwoo cells:
- passed: Gazebo `vacuum_gui_room_pause_resume` and `vacuum_gui_zone_keepout` (live Tk Vacuum tab),
  `vacuum_zone_mop_carpet`, `vacuum_zone_carpet_boost`, `vacuum_zone_floor_objects`;
  PyBullet and Isaac `vacuum_zone_keepout`;
- failed: Gazebo `vacuum_rooms` on `celisca_floor_1_furniture` (the latest run completes and docks
  but cleans 73.6 % of the target; the earlier failed runs are kept in the index).

The repeats of office, keep-out zone and spot add reports to existing
cells, which stay passed. Unlisted combinations remain untested.
