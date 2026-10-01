# Go2 terrain stairs low-speed trial (2026-10-01)

## Result

Three valid trials used the named `terrain_stairs` map, aligned spawn
`(-7.5, -4.0, 0.305 m)`, the flat-ground-qualified `+0.25 m/s` command from
1.0 to 8.0 simulated seconds, and the same inverse policy map. All captured
truth, joint, effort, and foot-contact telemetry and returned zero probe/launch
codes.

| Domain | Command | X displacement | Peak tilt | Minimum/final height | Maximum effort |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 220 | +0.25 m/s | 0.039 m | 0.548 rad | 0.258 / 0.369 m | 35.55 N m |
| 221 | +0.25 m/s | 0.122 m | 0.137 rad | 0.302 / 0.369 m | 30.33 N m |
| 222 | +0.25 m/s | 0.149 m | 0.080 rad | 0.302 / 0.364 m | 25.80 N m |
| 223 | +0.50 m/s | 0.132 m | 0.320 rad | 0.301 / 0.366 m | 35.55 N m |

The first `+0.25 m/s` run crossed the 0.35 rad tilt-warning threshold at 7.804
s; its two repeats remained below it. All four runs advance less than 0.15 m
and do not traverse the 4.4 m-long first step. The matched `+0.50 m/s` run
adds no displacement over the slower repeats while increasing tilt and
saturating effort. This is bounded contact/approach evidence, not a terrain
pass.

The course-specific [`run_terrain_trial.sh`](run_terrain_trial.sh) sources ROS
before starting either process, pins the spawn explicitly, captures source
hashes, and rejects missing telemetry or an off-course measured start. It
prevents the map-default spawn from silently turning a terrain trial into a
flat-ground run.

An initial manual run and its longer follow-up are deliberately excluded: they
used the map default spawn `(-7, -5)`, outside the stair strip centered at
`y=-4`, and therefore drove on flat ground. Their temporary files are not
referenced as terrain evidence.

## Configuration and artifacts

- Backend/map: MuJoCo, `terrain_stairs`; ROS domain 220.
- Policy: `go2_policy_path:=auto`, `go2_reverse_command_map:=inverse`.
- Spawn: `x=-7.5`, `y=-4.0`, yaw `0`; measured initial height `0.305 m`.
- Probe: 10 s simulated duration, 0.05 s trace interval, joint tracing enabled.
- Drive: `vx=+0.25 m/s`, from 1.0 to 8.0 s; no yaw command.
- Repeats: `low_speed_repeat_a/` (domain 221), `low_speed_repeat_b/` (domain 222), and `high_speed_check/` (domain 223).
- `probe.json`: truth, joint, effort, per-foot contact, and sampled joint trace.
- `launch.log`, `probe.log`, `returncodes.json`: process records.
- `manifest.json`: exact command, outcome, revision, and source/model hashes.

The bundled velocity policy is documented as flat-ground trained in
`src/robot_lab_adapter/policies/go2_velocity_flat/SOURCE.md`. The lower-speed
trial does not justify another scalar speed adjustment; terrain-aware policy
adaptation or a different foothold/control strategy remains necessary.