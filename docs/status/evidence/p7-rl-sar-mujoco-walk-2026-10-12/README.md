# P7 / R5.2: model-matched rl_sar policies walking in MuJoCo, Isaac and Gazebo (2026-10-12)

Four upstream quadruped policies from [rl_sar](https://github.com/fan-ziqi/rl_sar)
at revision `376d42c9` (provisioning notes: `src/robot_lab_adapter/policies/rl_sar/SOURCE.md`):

- `go2_himloco` on `unitree_go2`
- `a1_legged_gym` on `unitree_a1`
- `b2_robot_lab` on `unitree_b2`
- `go2w_robot_lab` on `unitree_go2w`, which has 4 wheel joints in velocity mode

Each policy runs with its own contract: observation layout, default pose,
action scale and joint order. Its PD law
`tau = kp (q* - q) + kd (dq* - dq)` runs inside every MuJoCo physics step
(`robot_lab_mujoco/joint_pd.py`). The policy runs at 50 Hz (dt 0.005 x
decimation 4) in the ROS node `legged-policy-controller`.

## Command

One simulator at a time, artifacts on the workspace SSD:

```bash
MODE=loc WARMUP=6 CHECK_ARGS="--strafe" scripts/sim_drive_check.sh mujoco <robot> nav_empty <out>.json legged_policy:=<policy>
```

`sim_drive_check.py` publishes each `/cmd_vel` phase in simulated time and
measures body velocity from MuJoCo truth (`/odom/ground_truth`). The first
second of each phase is excluded.

## Measured (vx m/s, vy m/s, wz rad/s from truth)

| Phase (command vx, wz) | Go2 himloco | A1 legged_gym | B2 robot_lab | Go2W robot_lab |
|---|---|---|---|---|
| straight (0.4, 0) | 0.355, 0.071 | 0.359, 0.001 | 0.258, 0.086 | 0.292, -0.023 |
| left arc (0.4, 0.5) | 0.310, 0.435 | 0.325, 0.496 | 0.243, 0.557 | 0.219, 0.433 |
| right arc (0.4, -0.5) | 0.344, -0.335 | 0.306, -0.520 | 0.262, -0.358 | 0.203, -0.561 |
| reverse left (-0.3, 0.3) | -0.228, 0.219 | -0.175, 0.214 | -0.108, 0.252 | -0.155, 0.294 |
| tight left (0.3, 1.5; clamped to wz 1.0) | 0.227, 0.740 | 0.200, 0.971 | 0.172, 0.977 | 0.131, 0.874 |
| spin in place (0, 1.0) | **0.009, 0.014** | 0.038, 0.934 | 0.054, 0.891 | 0.061, 0.897 |
| strafe (vy 0.3) | vy 0.113 | vy 0.184 | **vy 0.001** | vy 0.290 |
| stop (0, 0) | 0.002, -0.062 | -0.007, -0.057 | 0.000, -0.016 | 0.003, 0.001 |

All four robots stood, walked every forward, arc and reverse phase in the
commanded direction, and stopped. None fell: every robot kept moving
through all eight phases and came to rest.

Limits:

- Go2 himloco does not turn on the spot (0.014 of 1.0 rad/s); it turns
  only while walking.
- B2 robot_lab does not strafe.
- Forward speed is 0.26-0.36 m/s of the commanded 0.4 m/s, and the
  straight phases drift up to 0.086 rad/s in yaw.

These are open-loop velocity tracking screens on a flat map. They do not
qualify terrain, push recovery, localization accuracy, mapping or
navigation.

## Isaac Sim 6.0.1: Go2 + go2_himloco through PhysX drives (same day)

In Isaac localization the robot's provisioned policy is selected by
default (`legged_policy:=none` turns it off). The policy's kp/kd/torque
limits become PhysX joint drives (gains per degree). Physics runs at the
policy's own 200 Hz. The robot starts in the policy's default pose,
spawned at z 0.448 m to clear its straight-legged rest pose.

Command: `MODE=loc WARMUP=10 CHECK_ARGS=--strafe scripts/sim_drive_check.sh isaac unitree_go2 nav_empty ...`
Result: [isaac/unitree_go2.json](isaac/unitree_go2.json).

| Phase (command vx, wz) | Isaac Go2 himloco | MuJoCo Go2 himloco |
|---|---|---|
| straight (0.4, 0) | 0.385, 0.035 | 0.355, 0.071 |
| left arc (0.4, 0.5) | 0.360, 0.372 | 0.310, 0.435 |
| right arc (0.4, -0.5) | 0.357, -0.376 | 0.344, -0.335 |
| reverse left (-0.3, 0.3) | -0.248, 0.201 | -0.228, 0.219 |
| tight left (0.3, 1.5; clamped to 1.0) | 0.254, 0.672 | 0.227, 0.740 |
| spin in place (0, 1.0) | 0.003, **0.429** | 0.009, 0.014 |
| strafe (vy 0.3) | vy 0.096 | vy 0.113 |
| stop | 0.006, -0.079 | 0.002, -0.062 |

Go2 walks in Isaac, closer to its training simulator. It also turns on the
spot at 0.43 rad/s, which it does not do in MuJoCo.

Isaac ran at about 0.13-0.20x real time: 3 simulated seconds took about
22 s of wall time. A1, B2 and Go2W are not yet run in Isaac.

## Second batch (same evening, revision 10f1e51): Gazebo Go2, G1 humanoid, Isaac A1/B2/Go2W

These are files in [batch2/](batch2/). Gazebo runs the policy through an
effort `joint_trajectory_controller` that carries the policy's gains
(ign_ros2_control, 500 Hz). Isaac runs it on PhysX drives at 200 Hz. The
humanoid is `g1_robomimic_locomotion`, a recurrent LSTM actor exported
with its state as explicit inputs.

| Phase (command vx, wz) | Gazebo Go2 | MuJoCo G1 | Isaac G1 | Isaac A1 | Isaac B2 | Isaac Go2W |
|---|---|---|---|---|---|---|
| straight (0.4, 0) | 0.374, 0.072 | 0.386, 0.002 | 0.376, -0.005 | 0.386, -0.032 | **0.651**, 0.051 | 0.280, -0.013 |
| left arc (0.4, 0.5) | 0.348, 0.376 | 0.326, 0.484 | 0.326, 0.443 | 0.352, 0.475 | 0.431, 0.627 | 0.229, 0.461 |
| right arc (0.4, -0.5) | 0.348, -0.341 | 0.333, -0.492 | 0.333, -0.447 | 0.299, -0.492 | 0.567, -0.383 | 0.224, -0.536 |
| reverse left (-0.3, 0.3) | -0.237, 0.179 | -0.273, 0.291 | -0.295, 0.281 | -0.224, 0.224 | -0.173, 0.285 | -0.148, 0.295 |
| tight left (0.3, 1.0 clamped) | 0.239, 0.635 | 0.183, 0.974 | 0.197, 0.914 | 0.212, 0.979 | 0.275, 1.025 | 0.138, 0.851 |
| spin in place (0, 1.0) | wz **0.020** | wz 0.979 | wz 0.927 | wz 0.908 | wz 1.042 | wz 0.877 |
| strafe (vy 0.3) | vy 0.120 | vy 0.256 | vy 0.262 | vy 0.199 | vy 0.159 | vy 0.268 |
| stop: vx, vy, wz | 0.001, -0.025, -0.062 | 0.007, 0.003, 0.005 | -0.004, -0.005, -0.006 | 0.046, 0.009, -0.032 | **0.160**, -0.028, -0.030 | 0.012, 0.013, -0.002 |

### Gazebo Go2

- It walks, turns on arcs, reverses and stops through the ros2_control
  path.
- As in MuJoCo, it does not turn on the spot (0.020 rad/s).
- The first attempt
  ([batch2/gazebo_unitree_go2_first_attempt_no_truth.json](batch2/gazebo_unitree_go2_first_attempt_no_truth.json))
  had the controllers active and the policy running. It could not
  measure anything because the Go2 description had no ground-truth
  publisher; that was fixed in `fb636cb`.

### G1 29-DoF humanoid

The G1 walks, turns, spins in place, strafes and stops in both MuJoCo
and Isaac. This is the first humanoid walking on `/cmd_vel` from a
downloaded model-matched policy here.

### Isaac quadrupeds

- **A1** and **Go2W** track all phases.
- **B2** overshoots straight commands (0.651 of 0.4 m/s) and keeps
  drifting at 0.16 m/s after the stop. It fails the drive screen in
  Isaac, but passed it in MuJoCo.

Remaining:

- Mapping/navigation for legged robots stays gated, because turning
  ability differs by robot and backend.
- BHL still runs only in MuJoCo.
- B2's Isaac overshoot needs a look at the actuator, friction or
  mass model.
