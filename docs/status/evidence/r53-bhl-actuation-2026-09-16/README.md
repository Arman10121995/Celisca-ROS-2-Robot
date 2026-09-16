# R5.3 evidence: backend actuation probe (decisive), 2026-09-16

Controlled experiment that settles the R5.3 "command-tracking gap" root cause.
It **overturns** the paused-start probe's conclusion (recorded in
`r53-bhl-live-backend-2026-09-16/README.md`) that position commands do not
move the joints. The truth is the opposite:

- **The backend actuates and tracks position commands faithfully.**
- **The robot stands stably under zero position commands** (10 s, tilt 0.0).
- The spawn-fall SAFE_STOP seen in earlier probes is caused by the *policy
  node's pre-`cmd_vel` default-pose hold*, not by any backend defect.

## Setup

Standard dispatch path, headless, unpaused (no `paused:=`):

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=display map_name:=empty \
    robot_model:=berkeley_humanoid_lite_sim simulator:=gazebo \
    gui:=false start_rviz:=false
```

Controllers active after ~1 s (`cm_ready.txt` in the probe run; both
`bhl_standing_controller` and `joint_state_broadcaster` end active). Then
`actuation_probe.py` (see `run.sh`) runs two phases while watching tilt
(`/imu/out`) and four watched joints (`/joint_states`):

1. **Phase 1 — zero hold (10 s):** publish all-zero position targets at 50 Hz.
2. **Phase 2 — distinctive commands (10 s):** publish
   `arm_{left,right}_shoulder_pitch = 0.8`, `leg_{left,right}_knee_pitch = 0.4`
   (all other joints zero) at 50 Hz.

## Results (`actuation_report.json`)

| Check | Result |
|---|---|
| Phase 1: zero-hold max tilt | **0.0 rad** — the straight-legged robot stands perfectly stable for the whole phase |
| Phase 2: commanded max tilt | 1.71 rad — the robot topples (expected: commanding knee bend into a standing biped with no balance feedback destabilizes it) |
| `leg_left_knee_pitch_joint` tracks 0.4 | ✅ exact |
| `leg_right_knee_pitch_joint` tracks 0.4 | ✅ exact |
| `arm_right_shoulder_pitch_joint` tracks 0.8 | ✅ exact |
| `arm_left_shoulder_pitch_joint` commanded 0.8 | ✅ clamped at **0.7854** — exactly the vendored URDF `upper` limit (0.785398) for that joint; the backend respects the URDF limits |

Actuation conclusion: position commands move the joints to the commanded
values (clamped by URDF limits where applicable), through the standard
dispatch path with both controllers active, unpaused, from the first
commanded step.

## Corrected root cause of the spawn-fall SAFE_STOP

`humanoid_policy_controller` holds `PolicyConfig.hold_pose()` from its very
first cycle, before any `cmd_vel` arrives (documented in the node: "the policy
does not start driving until the first `cmd_vel` arrives; before that the node
holds the config default pose"). The `policy_humanoid` default pose bends the
legs (`hip_pitch −0.2, knee +0.4, ankle −0.3` per side) while the robot spawns
straight-legged (all joints zero, as measured). Commanding that bent-leg pose
instantly into a standing straight-legged biped is precisely the phase-2-style
destabilization this probe reproduces on demand — tilt rises, crosses the
0.70 rad fall threshold, and the policy latches SAFE_STOP. The earlier probes'
"tilt 0.75–0.79 rad then fallen" observations are fully explained by this
sequence; the paused-start probe's "commands produce no joint motion" reading
was a misdiagnosis of the same phenomenon.

## What this does NOT claim

- Not that the policy can balance or walk: phase 2 shows commanding pose
  changes without balance feedback topples the robot, as physics predicts.
- Not that `paused:=` is useless: it works as a launch feature (verified); it
  simply does not address this root cause.

## Next step (R5.3)

Give the policy node a startup transient that respects the spawn pose: hold
the *measured* pose (zero commands — proven stable here) until the first
`cmd_vel`, then ramp from measured pose toward the policy's pose over a
bounded interval instead of stepping. Validate live with the same harness
(success criterion: tilt stays below the 0.70 rad fall threshold through the
ramp).

## Artifacts

- `actuation_report.json` — phase results + 0.5 s-resolution timeline
- `actuation_probe.py` / `run.sh` — the probe (launch → wait controllers →
  two phases → report)
- `launch.log` — full dispatch-path bringup log
