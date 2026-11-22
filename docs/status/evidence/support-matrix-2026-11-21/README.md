# R9.3: support matrix with the November 21 navigation repeats

`scripts/r9_3_index_missions.py docs/status/evidence/v2-nav-matrix-2026-11-21/mission-batch.yaml`
appended 16 records to [runtime-evidence-index.yaml](../../runtime-evidence-index.yaml)
(now 733). `scripts/r9_3_support_matrix.py --out-directory <this folder>`
regenerated the matrix: **342 measured cells** (327 on November 10).

The 16 new records are Bumperbot and Labbot on MuJoCo, PyBullet, Gazebo
and Isaac, on `nav_obstacle` and `nav_maze`; all passed
([round](../v2-nav-matrix-2026-11-21/README.md)). Unlisted combinations
remain untested.

The same day `docs/status/evidence/v3-car-matrix-2026-11-21/mission-batch.yaml` added 12 car
records (index 745); the matrix was regenerated, still 342 cells (these cells were measured before).
`docs/status/evidence/v3-4ws-matrix-2026-11-21/mission-batch.yaml` then added 20 4WS/mecanum records
(index 765); still 342 cells.

November 22: `docs/status/evidence/v5-px4-worlds-2026-11-21/px4-worlds-batch.yaml` added 7 PX4
manual-flight records (index 782): **354 measured cells**. Six are new worlds for the X500
(aerial_indoor, nav_maze, nav_warehouse, celisca_floor_1, outdoor_terrain, small_house); the
flight verdict now expects the clear takeoff check's rise, which is ceiling limited in small_house
([round](../v5-px4-worlds-2026-11-21/README.md)). The V1 vacuum round of November 21 had raised the
count from 342 to 348; aerial_course was already a measured cell.
