# R9.3: support matrix with round 57 and the first trained Go2 policy (2026-10-29)

`scripts/r9_3_index_missions.py` appended seven records to
[runtime-evidence-index.yaml](../../runtime-evidence-index.yaml), now 626:

- [round57-batch.yaml](../rounds-2026-10-19/round57-batch.yaml): the rear-steer
  car course in four backends after the planner change `84422df`
  (`course_screen`);
- [training-batch.yaml](../training-2026-10-19/training-batch.yaml): the
  first trained Go2 walk policy, `go2_walk_r1`, driven through the normal
  launch in MuJoCo, Gazebo and Isaac (`drive_screen`). Its task type is
  `trained_policy_drive`, so these cells stay apart from the rl_sar
  `go2_himloco` cells (`legged_policy_drive`).

`scripts/r9_3_support_matrix.py` validated every record again and wrote
[support-matrix-current.md](support-matrix-current.md) and
[support-matrix-current.json](support-matrix-current.json).

| | October 19 (619 reports) | October 29 (626 reports) |
|---|---|---|
| Measured cells | 291 | **294** |
| Passed screens | 278 | **282** |
| Failed screens | 12 | 11 |
| Unverified | 1 | 1 |

Changes, latest record per cell:

- Failed → passed: the rear-steer car course in MuJoCo (gates and
  parking at 0.091 m and −3.4°).
- Still failed, now on parking heading only: the rear-steer course in
  Gazebo, PyBullet and Isaac. The slalom passes in all four; parking ends
  11.6-14.3° off.
- New, passed: `go2_walk_r1` in MuJoCo, Gazebo and Isaac. Isaac passes
  narrowly: 0.221 m/s straight, against the 0.200 m/s screen threshold.

Note on the round 51 course records: their indexed scope says the gates
must be within 0.15 m. The script's gate limit is 0.30 m
(`--max-position-error`). The screen reads the report's own pass flag, so
the verdicts are unaffected. The records are append-only and keep their
text.

All full-release gates stay blocked.
