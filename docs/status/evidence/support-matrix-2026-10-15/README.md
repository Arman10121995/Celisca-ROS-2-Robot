# R9.3: support matrix regenerated with the October 13-15 missions (2026-10-15)

`scripts/r9_3_index_missions.py mission-batch.yaml` appended 111 mission
reports to [runtime-evidence-index.yaml](../../runtime-evidence-index.yaml)
(380 records now):

- the legged nav_obstacle rounds, repeats and forward-arc runs;
- the car missions and the 4WS/mecanum mission rounds;
- the Isaac capsule-roller mecanum mission.

The batch lists the rounds in the order they ran, with the revision each
README records. For each round the script wrote a source-hash manifest
(`source-hashes-<revision>.json`) beside the reports. It holds the sha256
of each relevant file as git stores it at that revision. The manifests
were computed afterwards, so uncommitted working-tree changes at run time
are not captured; the manifests say so.

`scripts/r9_3_support_matrix.py` then validated every record again (report
and manifest hashes, report identity, the per-kind screen contract) and
wrote [support-matrix-current.md](support-matrix-current.md) and
[support-matrix-current.json](support-matrix-current.json).

| | October 12 | October 15 |
|---|---|---|
| Indexed reports | 269 | 380 |
| Measured cells | 211 | **251** |
| Passed screens | 209 | **243** |
| Failed screens | 1 | 7 |
| Unverified | 1 | 1 (go2 MuJoCo, no source manifest, unchanged) |

The 40 new cells:

- the four walking robots on nav_obstacle;
- the three cars in four backends;
- the 4WS patterns and the mecanum car in four backends.

In each cell the latest record controls the result.

**Failed screens** (latest record in their cell):

| Cell | Why |
|---|---|
| unitree_go2 / isaac | seed 3: the detour goal aborted beside the start block |
| unitree_g1_29dof / isaac | seed 3: "start in lethal space" (explained in the legged README) |
| ackermann_car / gazebo | resumed detour aborted after repeated collision-ahead stops |
| rear_steer_car / isaac, pybullet | near-obstacle goal 0.27 / 0.28 m from truth |
| rear_steer_car / mujoco | tf2 rerun: near-obstacle goal 0.30 m from truth |
| unitree_b2 / nav_empty / isaac (drive) | unchanged from October 12 |

The matrix's mission screen is stricter than `sim_nav_mission.py`. It
accepts final truth errors up to 0.25 m (the script, 0.30 m) and a stop
within 1 s of the cancel. So two rear-steer cells that the car README
counts as passes (0.27 and 0.28 m) are failed screens here. Both judgements
are kept as recorded.

All full-release gates stay blocked: R5-R9 tasks remain partial.
