# R5.3 BHL startup-settle live validation (2026-09-16) — split verdict, honestly recorded

Live validation of the `StartupSettle` startup transient (`6d25895`) through
the standard dispatch path, using the same harness as
`r53-bhl-actuation-2026-09-16`. Two runs, both with both controllers active
(`controllers_before.txt` / `controllers_after.txt`):

- run 1: default 2.0 s ramp (`settle_report_fast_ramp.json`)
- run 2: slow 12.0 s ramp (`settle_report.json`, `settle_probe.py` sets
  `settle_duration_s:=12.0`)

## Verified working

- **The measured-pose hold fixed the pre-command fall.** In both runs, tilt
  stayed at **0.0 rad** through the pre-policy window *and* through the
  policy-idle window (`pre_policy_hold`, `policy_idle_hold` phases). Under the
  previous behavior the policy commanded its bent-leg default pose from its
  first cycle and the robot latched SAFE_STOP before any `cmd_vel` ever
  arrived. That defect is gone: idle now holds the measured spawn pose
  (effectively zero commands) until the first `cmd_vel`.
- The ramp mechanics work as designed: `policy.log` shows
  `first cmd_vel: ramping from the measured pose...` then either
  `settle ramp complete: policy driving` or the tilt-abort handover, and
  joints track the ramped targets (knees move toward 0.4 rad in both runs).

## Verified NOT working — and why

- **The squat transition topples the free-standing biped at any ramp speed.**
  Run 1 (2 s ramp): max tilt 2.863 rad, robot fallen flat (max 3.141).
  Run 2 (12 s ramp): max tilt 2.106 rad, SAFE_STOP latched — and critically,
  the fall happened when the knees had reached only **~0.17–0.26 rad**, part
  way into the squat, well before the 0.4 rad default. Speed is therefore not
  the cause: bending the legs from a standing pose under pure position
  control, with no CoM/balance feedback, destabilizes the robot
  geometrically. A startup ramp cannot substitute for balance.
- The policy's default pose was trained for a PD-effort-tracked robot with
  the full observation loop; this sim setup drives position targets into a
  free-floating base with IMU-only feedback.

## Consequence for R5.3 scope

- Startup-ordering work is **done and correct**: hold measured pose → ramp on
  demand → safe abort. Idle behavior is stable and honest (no fabricated
  measurements; unmeasured joints fall back to the default pose).
- The remaining R5.3 residual is now precisely scoped: **closed-loop balance**.
  Options: (a) drive the joints through an effort interface with the training
  PD gains (kp 10–20, kd 2) so the policy's own control loop closes, or (b) a
  dedicated standing/balance controller with CoM/odometry feedback for the
  squat-to-stand transition. Both are controller work, not bringup work.

**Follow-up (2026-09-17):** option (a) is implemented — the joints are driven
through an effort interface with the training PD gains. See
`../r53-bhl-effort-interface-2026-09-16/README.md` (wired + unit-verified;
live validation of the effort path still pending).

## Artifacts

- `settle_report.json` — run 2 (12 s ramp) phase summary + timeline
- `settle_report_fast_ramp.json` — run 1 (2 s ramp), same phases
- `policy.log` — node log for run 2 (ramp start, completion, abort path)
- `probe_stdout.log` — probe console output (run 1)
- `controllers_before.txt` / `controllers_after.txt` — both controllers active
- `launch.log` — full dispatch-path bringup log
- `run.sh` / `settle_probe.py` — the reproduce recipe (any ramp via
  `settle_duration_s:=`)
