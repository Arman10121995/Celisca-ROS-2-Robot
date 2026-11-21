# R9.3: support matrix with the November 21 navigation repeats

`scripts/r9_3_index_missions.py docs/status/evidence/v2-nav-matrix-2026-11-21/mission-batch.yaml`
appended 16 records to [runtime-evidence-index.yaml](../../runtime-evidence-index.yaml)
(now 733). `scripts/r9_3_support_matrix.py --out-directory <this folder>`
regenerated the matrix: **342 measured cells** (327 on November 10).

The 16 new records are Bumperbot and Labbot on MuJoCo, PyBullet, Gazebo
and Isaac, on `nav_obstacle` and `nav_maze`; all passed
([round](../v2-nav-matrix-2026-11-21/README.md)). Unlisted combinations
remain untested.
