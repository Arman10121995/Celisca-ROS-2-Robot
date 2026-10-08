# Hospital navigation after the occupancy-v4 repair

Eight actual normal GUI ground-floor `dataset_hospital` routes pass with the
existing 0.15 m / 5° independent physical endpoint gates. Each uses the selected
real v4 grid, fresh body truth, floor/height bounds, one simulated second of
settling and zero owned root exit. Goal: (2.55, 12.45, 0°), starting at
(0.55, 12.45). The normal estimator goal checker remains 0.07 m / 0.03 rad.

| Robot | Backend | Physical position error (m) | Heading error (°) | Nav2 action wall time (s) | Original report |
|---|---|---:|---:|---:|---|
| bumperbot | gazebo | 0.092 | +1.22 | 35.7 | [Measured result](bumperbot-dataset_hospital-gazebo/report.json) |
| bumperbot | isaac | 0.083 | -1.64 | 19.6 | [Measured result](bumperbot-dataset_hospital-isaac/report.json) |
| bumperbot | mujoco | 0.077 | -2.21 | 473.5 | [Measured result](../open-wall-map/bumperbot-dataset_hospital-mujoco/report.json) |
| bumperbot | pybullet | 0.043 | +1.84 | 31.5 | [Measured result](bumperbot-dataset_hospital-pybullet/report.json) |
| labbot | gazebo | 0.053 | +1.79 | 32.6 | [Measured result](labbot-dataset_hospital-gazebo/report.json) |
| labbot | isaac | 0.078 | +1.29 | 21.1 | [Measured result](labbot-dataset_hospital-isaac/report.json) |
| labbot | mujoco | 0.083 | -1.21 | 98.4 | [Measured result](labbot-dataset_hospital-mujoco/report.json) |
| labbot | pybullet | 0.067 | +1.32 | 28.2 | [Measured result](labbot-dataset_hospital-pybullet/report.json) |

Original pre-trial world/map/controller hashes, reports and raw trace/log hashes
remain in [the collection](../../checkpoint-collection.json). The seven serial
repeats retain their actual [batch producer](batch-producer.py) and
[original batch report](batch-report.json); Bumperbot/MuJoCo is the separate
first repaired-grid trial. The later registered hospital export is byte-identical
to that grid, as [verified separately](../../occupancy-v4-refresh/verification.json).

The earlier missing-wall, matched-motor and tighter-goal failures remain under
`navigation/`; their original outcome markers were not rewritten. Shutdown
errors in the system joystick child remain in raw logs despite zero root exits.

These are short straight static lobby routes. Two-floor-building missions,
upper-floor travel, actors, multi-turn obstacle/furniture routes, cancellation,
second goals and full mesh/contact clearance remain open. MuJoCo is still slow:
[one actual matched Labbot window](../../mujoco-hospital-performance/report.json)
measures 0.1305 real-time factor; it does not identify the cause or establish
a general speed benchmark.
