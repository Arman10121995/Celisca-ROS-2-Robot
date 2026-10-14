# Native fixed-arm reach diagnostics

Read [implementation scope and retained failures](../../native-arm-planning-2026-10-14.md)
and the [source/artifact receipt](../../native-arm-planning-receipt-2026-10-14.json).
These two actual GUI/native-actuator reaches on MuJoCo/nav_empty are one-shot
implementation checks. They do not qualify all thirteen arm candidates,
objects, maps, interruption/reset behavior or other simulation backends.

- [UR5e report](ur5e-4/report.json): +20 mm Z, 19.83 mm actual tool travel,
  0.489 mm endpoint error; independent measured-body TF and actual action.
- [SO101 report](so101-1/report.json): +10 mm X with position-only IK,
  9.90 mm actual travel, 0.147 mm endpoint error.

Both owned launches exit normally. Raw JointState/articulation traces, failed
trials, export checks and full logs remain on the SSD at
`/workspace/molar/robot_lab_runtime/native-arm-planning-2026-10-13/`.
No historical source certificate was refreshed. Current GUI result text and
action feedback timestamps have two separately disclosed metadata-only changes
after these runs; actuator targets, collision and settling calculations remain.
