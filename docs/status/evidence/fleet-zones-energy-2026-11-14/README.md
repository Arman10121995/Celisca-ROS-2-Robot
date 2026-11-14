# Fleet shared zones, crossing conflicts and calibrated RMF energy (R10.2)

Recorded November 14, 2026, on `master` from `d4eb4d5a` plus this delivery's
working changes. Jetson AGX Orin, ROS Humble, Gazebo Fortress, the Humble
Open-RMF overlay. Named simulation cases in `fleet_facility`; R10.2 stays
partial and experimental.

## What was added

- `robot_lab_utils/fleet_zones.py`: shared-zone reservations. Single-lane
  passages are found from the occupancy map. Declared doors, docks and
  `shared_zones` become zones too. A member whose fresh Nav2 plan reaches a
  zone held by a peer is held 1 m before it, with its task suspended rather
  than canceled, and the same task is sent again when the zone frees.
- Fleet controller: zones wired into the tick and `/fleet/state`. The
  circular-wait breaker was rewritten: the oldest queued task proceeds and no
  task is canceled. The earlier version canceled the yielding task and read
  stale waits. The unused "occupied member" recovery fields were removed;
  nothing ever set them.
- Lift: **Recover held lift** can now drive the passenger out afterwards
  (`exit: true`, optional automatic parking) as an owned exit transport with
  the normal settled-body checks.
- `robot_lab_utils/fleet_energy_calibration.py`: drives one member through a
  19-phase profile and fits Open-RMF's `SimpleMotionPowerSink` parameters to
  the energy each phase used. The wheel power is sum(max(0, rate × effort)) / 0.75.
  The fit is checked on held-out halves of the drive. The RMF adapter enables
  battery-aware planning only when every member's model carries a
  calibration bound to its description hash, with a held-out error of at most
  25 %. It then sends the observation-backed modeled state of charge to RMF.

## Results

| Case | Result | Boundary |
|---|---|---|
| Native door crossing (`zone-door-crossing-result.json`) | **Passed.** Bumperbot alpha (west → east) and Labbot beta (east → west) were dispatched together with concurrent traffic. Alpha took the `door:main` zone; beta's plan reached it 0.96 m away, so beta was held and its task queued. 12.4 s later the zone was granted to beta and the same task resumed. Both tasks: Nav2 success plus measured settled arrival, 54 s in total. Closest truth separation 2.40 m; no contact stop | One crossing, two members, one door |
| Open-RMF crossing (`rmf-crossing-result.json`) | **Passed.** Both robots under RMF with calibrated energy enabled. Patrols to p66 (1.8, 2.9) and p22 (−3.0, −1.9) through the same door. RMF's schedule negotiation replanned (11 route replacements, 50 waypoint tasks succeeded, 0 failed). Closest separation 0.92 m; no zone hold was needed. Alpha settled at 107 s, beta at 228 s; modeled SOC 0.797 for both, sent to RMF | Same world and door; RMF graph lanes; SOC is the authored battery model |
| Bumperbot energy calibration (`calibration-bumperbot.json`) | 4087 samples, 19 phases. Held-out drive energy error +6.0 % / −3.3 % (two folds); whole drive +1.2 % | Effective parameters: the fitted acceleration term was ~0 (floored mass 0.001 kg, so the friction coefficient is an effective 14.9). Per-sample R² 0.10 |
| Labbot energy calibration (`calibration-labbot.json`) | 4108 samples, 19 phases. Held-out error −13.1 % / +15.2 %; whole drive +1.1 % | The RMF model has no term for turning in place at constant rate; scrub-heavy phases fit worse. Per-sample R² −0.63 |

The RMF fleet uses one vehicle model, so the adapter takes each term from its
largest member. In this session that is the Labbot.

## Retained failures

- The first Labbot calibration drive never moved: its manual request reused
  an operator identity whose lease was still stale, and the controller
  rejected it ("Connect a live operator"). The driver now waits for a fresh
  lease and an acknowledged manual mode. The failed samples are kept on the SSD
  (`energy-beta-failed-rejected-manual/`).
- The first fit fixed the mass to the description mass (1.07 kg and
  4.62 kg) and fitted per-sample power. The Labbot's whole-drive energy error
  was −22 %, and the per-sample R² was negative for both robots. RMF charges
  deceleration as well as acceleration (|a|), but these velocity-controlled
  simulated wheels draw almost no acceleration power. The fit was replaced by
  the per-phase energy fit with effective parameters above.
- `test_traffic_yield_retains_original_task_and_waits_for_older_peer` failed
  before this change: its fixture lacked `_same_floor`. The fixture is fixed.

## Reproduce

Source `scripts/ssd_env.sh`, ROS, the workspace and, for RMF,
`scripts/rmf_env.sh`. Use an isolated `ROS_DOMAIN_ID` and `IGN_PARTITION`.

```bash
R=$ROBOT_LAB_RUNTIME_ROOT/zone-conflict-<date>
python3 launch_case.py zone-session.yaml $R/launch.log            # add rmf:=true for RMF
CASE_OUTPUT=$R/run /usr/bin/python3 -s zone_conflict_probe.py      # or rmf_crossing_probe.py
python3 -m robot_lab_utils.fleet_energy_calibration --session <session.json> --member alpha --output $R/energy-alpha
python3 -m robot_lab_utils.fleet_energy_calibration --session <session.json> --member alpha --output $R/energy-alpha --from-samples
```

Stop the launch's process group afterwards (`kill -INT -<pid>`). Full traces,
samples, launch logs and session journals are on the SSD in
`/workspace/molar/robot_lab_runtime/zone-conflict-20261114/` and
`fleet_sessions/session-ypofdga9/`.

## Not covered

- Occupied-lift recovery with drive-out has unit tests only; it was not run
  in simulation.
- No three-or-more robot run, contact or loss screens, or larger fleets.
- Chargers and doors are still declared only in `fleet_facility`. Passages are
  found in every map, but other worlds have no physical docking or energy
  adapter.
- Energy is the authored battery model, with a drivetrain calibrated in
  simulation. It is not hardware.
