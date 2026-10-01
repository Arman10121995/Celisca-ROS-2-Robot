# Go2 Support-Transfer Gate (2026-10-01)

## Result

This is a bounded negative result, not a get-up qualification. The 18-element
`/go2/support_contact_forces` stream arrived throughout the 10 s trial. Before
the recovery stroke, the settled roll rest carried about 125 N on the braced
hips and no more than 6.2 N across the feet. The roll stroke produced transient
contact peaks but did not sustain foot support, so the 0.25 s in-range dwell
did not pass. The controller ended `failed:roll` rather than advancing on an
impact.

The run ended with 1.648 rad peak tilt and 0.139 m body height. It demonstrates
that the new signal and bounded gate are connected; it does not demonstrate a
successful transfer or recovery.

## Trial

- Source revision: `af77f2f` (`codex work`); working tree had 8 tracked changes at launch. Exact source hashes are in `manifest.json`.
- Backend/world: MuJoCo, `nav_empty`, ROS domain 212.
- Spawn: roll 0.9 rad, pitch 0.0 rad; recovery enabled with 1.0 s delay and 7.0 s active timeout.
- Probe: 10.0 simulated seconds, 0.05 s trace interval, zero perturbation force.
- Return codes: probe 0, launch 0.
- Support messages: 2,503.
- Recovery transitions: `idle` 0.004 s, `waiting` 0.100 s, `attempting:tuck` 0.856 s, `attempting:roll` 1.208 s, `failed:roll` 1.808 s.
- Peak hip contact: 555.79 N; peak foot contact: 398.73 N. These are transient loads, not support transfer.
- Settled pre-stroke trace: braced-hip load about 125 N and foot load at most 6.2 N.
- Final height: 0.1387 m; peak tilt: 1.6477 rad; final recovery state: `failed:roll`.

A final repeat against the trunk-aware gate is in `trunk_guard_repeat/`. It used
ROS domain 213 and returned zero probe/launch codes, delivered 2,502 support
messages with 0 N trunk contact, and ended `failed:roll` at 1.692 s (1.642 rad
peak tilt, 0.1387 m final height). It confirms the added trunk bound does not
change this no-transfer result.

## Support-Creation Experiments

Three matched 5 s trials tested an opt-in `+0.4 rad` braced-hip target from the
same placed roll (0.9 rad), with a 1 s recovery delay and no perturbation. Raw
manifests, traces, logs, and return codes are under `support_creation_ramp/`.

- `initial_scale/` used the original hip-load-based scale. It reached 1.575 rad
	peak tilt but ended `failed:roll` at 1.744 s and 0.1387 m.
- `foot_gap_scale/` kept splay active while feet were unloaded, but tapered it
	on instantaneous foot force. Foot peaks reached 409.9 N, hip-geom peaks
	reached 636.4/602.4 N, and the run ended `failed:roll` at 1.704 s and
	0.1387 m. The force trace showed the contact peaks were transient.
- `stroke_ramp/` ramped the target over the bounded roll stroke and held it
	through force spikes. Peak foot forces were 368.2/327.7 N and braced-hip
	peaks were 615.8/546.7 N; no dwell-qualified transfer occurred. It ended
	`failed:roll` at 1.716 s, with 1.605 rad peak tilt and 0.1387 m final height.

All three are negative get-up results. Earlier recorded hip-direction and
amplitude sweeps are also negative, so further scalar splay tuning is not the
next experiment. The experimental ramp remains opt-in and is not a recovery
qualification.

The gate accepts transfer only when total foot load is 75-125% of the 126.53 N
model weight, braced-hip load is at most 25%, trunk load is at most 10%,
combined load remains bounded, and the conditions persist for 0.25 s. Unit
tests inject 716 N foot and trunk impacts and verify that neither satisfies the
dwell.

## Artifacts

- `manifest.json`: launch configuration, source revision, and source hashes.
- `probe.json`: truth, joint, phase, foot, trunk, and per-geom support trace.
- `launch.log`, `probe.log`, `returncodes.json`: process and harness output.

The waiting hold is only applied when all measured joint positions are finite
and within declared limits; otherwise it fails closed with zero effort. The
support-transfer gate and the waiting hold remain experimental. Terrain,
velocity tracking, navigation, and successful fall recovery are still
unqualified.