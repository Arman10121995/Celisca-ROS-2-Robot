# Autonomous roadmap work: decision log (November 2026)

On 2026-11-17 the user asked Claude to work through the whole roadmap without
waiting for input, choosing the best option whenever a decision arises, and
to report the decisions at the end. Claude is the only agent on the
repository.

Each entry gives the date, the roadmap item, the decision and the reason.
Results and failures live in the linked receipts; this file records only the
choices.

## Decisions so far

| Date | Item | Decision | Why |
|---|---|---|---|
| 11-15 | R13.3 | Deployed the multi-map retrain at score 0.75, not 0.6 | At 0.75, false positives drop sharply (e.g. cable 6 → 2, shoe 1 → 0) for a small recall cost |
| 11-15 | R13.3 | Kept ideal segmentation as the default detector | The learned camera passed only one run, on a training map |
| 11-15 | R12 | Kept the user's Go2W wheel-gain edit (kd 1.0) although robot_lab trained with 0.5 | It measured better (strafe 0.206 → 0.248 m/s) |
| 11-15 | R12 | Added a `wheeled_walk` task with robot_lab's own Go2W reward weights instead of re-tuning `walk` | Vendor recipe; the quadruped air-time reward discouraged rolling |
| 11-15 | R5.3 | Trained BHL from a generated template of the vendored upstream config, not a hand-written contract | One source of truth for joints, gains and limits |
| 11-15 | R5.3 | Stopped BHL training after four failed CPU runs and recorded it as not passed | From scratch it never steps; warm starts collapse; this host has no usable GPU |
| 11-15 | R11 | Gazebo MOLAR belt: used the existing private DART plugin rather than writing a new belt system | It already existed (gz-sim#2495 workaround for the tracked vehicle) |
| 11-15 | R11 | Modelled the station as a passive roller table (μ 0.05) | A static 0.8-friction table holds a box partly on the deck; real stations have rollers |
| 11-15 | R11 | Limited the Gazebo MOLAR front scan to ±105° | A 360° scan from the front mount hit the robot's own body (216/540 beams) |
| 11-15 | R11 | Used MOLAR's own ros2_control controllers in every Gazebo mode, with the EKF owning odom → base_link | The shared wheel controller is not in its description |
| 11-17 | R5.2 | Recorded `get_up_roll` (2/10) as not passed and moved on | One shaping change; further progress needs curriculum or GPU-scale training |
| 11-17 | R5.7–R5.9 | Took over the former Codex manipulation lanes (user approved) | No Codex commits since 11-15; the lanes are still partial |
