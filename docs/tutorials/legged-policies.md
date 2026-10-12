# Model-matched legged policies (rl_sar): implementation and limits

Documentation written October 12, 2026 against master. Read
[current status](../status/CURRENT_STATUS.md) first: this tutorial describes
an **implemented, experimental** operator path. No per-robot walking,
terrain or navigation qualification is claimed for these policies yet.

The lab runs selected locomotion policies from
[rl_sar](https://github.com/fan-ziqi/rl_sar)
revision `376d42c9` (Apache-2.0, license and per-policy provenance kept under
`src/robot_lab_adapter/policies/rl_sar/`) on the lab's own robot models.
`robot_lab_adapter/legged_policy.py` reproduces rl_sar's exact observation,
history, scaling, action and PD contract (`ComputeObservation`,
`ObservationBuffer`, `ComputeOutput` at the pinned revision), and
`robot_lab_mujoco/joint_pd.py` applies each policy's trained PD law inside
MuJoCo at every physics step instead of over ROS.

## Provisioned policies

| Policy | Robot model | Joints | Input | Notes |
|---|---|---|---|---|
| `go2_himloco` | `unitree_go2` | 12 | 270 (6-frame history) | Flat-ground velocity policy |
| `go2w_robot_lab` | `unitree_go2w` | 16 (4 wheel) | 57 | Legs position, wheels velocity |
| `a1_legged_gym` | `unitree_a1` | 12 | 270 | Flat-ground velocity policy |
| `b2_robot_lab` | `unitree_b2` | 12 | 45 | Flat-ground velocity policy |

Each directory keeps the upstream `base.yaml`/`config.yaml` unchanged and a
`source.json` recording the upstream revision, TorchScript hash, ONNX hash
and the exact robot profile the checkpoint was trained for. A policy is
only ever paired with that model: `legged_policy:=` for a different
`robot_model` fails at launch with the binding printed, and there is no
"closest robot" fallback. Re-export or add policies with
`scripts/provision_rl_sar_policies.py`.

## Run

MuJoCo localization mode only, simulation only:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42   # unused domain; see the Go2 tutorial
ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=loc simulator:=mujoco map_name:=nav_empty \
    robot_model:=unitree_go2 legged_policy:=go2_himloco gui:=false
```

In the GUI, select the robot, MuJoCo and the `loc` mode, then enable
"Model-matched legged policy (rl_sar, experimental)" in the Launch column;
the command preview shows that robot's exact `legged_policy:=` value.

What the launch does:

- builds the policy's `joint_pd_config` from its own contract and hands it
  to the MuJoCo bridge, which adds one PD actuator per joint
  (`<joint>_pd`, implicitfast integration) and spawns the robot at the
  policy's default pose;
- replaces the passive joint hold and, on Go2, the stance-gait effort
  controller — never both;
- starts `legged-policy-controller`, which runs the policy at its trained
  period (`dt x decimation`, 50 Hz for the provisioned policies) from
  `/joint_states` and `/imu/out`, and publishes targets on
  `/legged_policy/joint_targets`.

Commands arrive on the same topics as the other controllers
(`/robot_lab_controller/cmd_vel_unstamped`, `/cmd_vel`), so the normal GUI
Drive pad works; `(vx, vy, wz)` is clamped to the controller's limits
(default `0.8 0.4 1.0`).

## Safety contract

- Command timeout (0.5 s): commands expire to zero velocity; the policy
  stands.
- Stale joint/IMU state or trunk tilt beyond `legged_policy_fall_tilt_rad`
  (default 1.0 rad) latches **safe-stop**: the default standing pose is
  published until `ros2 service call /legged_policy/reset std_srvs/srv/Trigger '{}'`.
- Bridge side: target messages that are wrong-sized or non-finite are
  refused; command loss falls back to the default pose, never the last
  value.
- Stop/Space in the GUI still clears input state through the shared path.

## Limits (read before trusting it)

These are experimental controllers validated as *software*, not as robot
missions. Walking/turning quality, terrain, falls, recovery, localization
coupling and navigation on each robot remain unqualified; per-robot
measurement campaigns are a separate follow-up lane (see the task ledger,
R5.2 and R3.6). MuJoCo is the only backend with the in-step PD bridge;
Gazebo/Isaac/PyBullet and hardware have no actuation path for these
policies and the launch rejects them explicitly. A passing contract test
is not a walking robot.

Pure-logic contracts live in
`src/robot_lab_adapter/test/test_r5_2_legged_policy.py` (fast tier): policy
loading and bindings, observation/history math against each pinned input
width, PD targets, command clamping and the MuJoCo PD bridge.
