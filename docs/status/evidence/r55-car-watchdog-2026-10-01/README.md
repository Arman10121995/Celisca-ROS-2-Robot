# R5.5 Car Publisher-Loss Screen (2026-10-01)

## Result

Seven runs across three backends exercised the car publisher-loss path:
Ackermann, rear-steer, and anti-Ackermann on PyBullet and Gazebo, plus
Ackermann on MuJoCo. PyBullet and MuJoCo used simulator-ground-truth odometry;
Gazebo used `/robot_lab_controller/odom`, its available controller estimate.
Each run drove through straight, left/right arcs, reverse-left, and tight-left
phases, then stopped publishing while the final moving command was active. No
explicit zero twist was sent during the following 3 s observation window.

| Backend | Profile | Odometry | Command-loss coast | Yaw after loss | Final 0.5 s vx / wz |
| --- | --- | --- | ---: | ---: | ---: |
| PyBullet | `ackermann_car` | Ground truth | 0.101 m | 0.229 rad | 0.000 / 0.000 |
| PyBullet | `rear_steer_car` | Ground truth | 0.109 m | 0.259 rad | 0.000 / 0.000 |
| PyBullet | `anti_ackermann_car` | Ground truth | 0.111 m | 0.239 rad | 0.000 / 0.000 |
| MuJoCo | `ackermann_car` | Ground truth | 0.017 m | 0.013 rad | 0.000 / 0.000 |
| Gazebo | `ackermann_car` | Controller odom | 0.155 m | 0.319 rad | 0.000 / 0.000 |
| Gazebo | `rear_steer_car` | Controller odom | 0.155 m | 0.319 rad | 0.000 / 0.000 |
| Gazebo | `anti_ackermann_car` | Controller odom | 0.155 m | 0.319 rad | 0.000 / 0.000 |

Each run completed its drive phases and captured the full 3 s silent window.
The PyBullet command watchdog is 0.5 s; these traces show bounded coast and
settled tail motion after publisher loss for the tested profiles. Gazebo's
three similar coast/yaw results use estimated rather than independent ground
truth. MuJoCo also settles, but its commanded arc response is weak (about
0.067 m/s and 0.108 rad/s on the left arc; 0.031 m/s and 0.023 rad/s on
tight-left). This is one screen per profile/backend, not repeated watchdog or
full cross-backend qualification. Final heading, longer slalom/parking
missions, and Isaac command-loss evidence remain open.

## Steering-Angle Trace

A follow-up set of PyBullet runs used the extended probe to record steering
joint positions during each phase. Straight steering was 0 rad on every
profile. During the `+0.5 rad/s` left arc, the measured left/right steering
angles were:

| Profile | Left-arc steering joint angles (rad) | Right-arc steering joint angles (rad) |
| --- | --- | --- |
| `ackermann_car` | front `+0.451/+0.328` | front `-0.328/-0.451` |
| `rear_steer_car` | rear `-0.452/-0.328` | rear `+0.328/+0.452` |
| `anti_ackermann_car` | front `+0.328/+0.452` | front `-0.452/-0.328` |

The traces show the steering variants produce distinct measured joint targets,
not just distinct registry labels. They remain one run per profile and do not
qualify heading accuracy or the four-backend matrix. Gazebo traces also
captured the matching steering patterns, but use controller odometry. The steering-angle traces
are under `steering_angles/`; their probe SHA-256 is recorded separately from
the earlier watchdog-only records in `manifest.json`.

Ackermann was repeated on domain 232. The repeated left/right angles were
`+0.4515/+0.3281` and `-0.3279/-0.4514 rad`, versus
`+0.4511/+0.3280` and `-0.3282/-0.4514 rad` in the first run. Publisher-loss
coast was 0.094 m versus 0.093 m. This supports repeatability for Ackermann on
PyBullet only; the two other steering profiles still have single angle traces.

## Reproduction

The `--command-loss` option added to `scripts/sim_drive_check.py` omits the
terminal zero-command phase after a moving arc and measures ground-truth coast
distance, yaw change, and final-window velocity. Example:

```bash
MODE=display ODOM=/odom/ground_truth TIMEOUT=60 WARMUP=2 \
  CHECK_ARGS='--command-loss --command-loss-duration 3' ROS_DOMAIN_ID=224 \
  bash scripts/sim_drive_check.sh pybullet ackermann_car nav_empty /tmp/car.json
```

The profile directories contain raw JSON summaries, launch logs, and probe
stderr. `manifest.json` pins source hashes and summarizes both trial sets.