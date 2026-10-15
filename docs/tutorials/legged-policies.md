# Model-matched legged policies (rl_sar): implementation and limits

Documentation updated October 13, 2026 against master. Read
[current status](../status/CURRENT_STATUS.md) first. This tutorial
describes an **experimental** operator path. Flat-ground walks on
`/cmd_vel` are measured for each robot and backend listed below
([evidence](../status/evidence/p7-rl-sar-mujoco-walk-2026-10-12/README.md)).
Navigation and slam mapping are measured on one obstacle world per robot
and backend (2026-10-13). Terrain and push recovery are not qualified.

The lab runs selected locomotion policies from
[rl_sar](https://github.com/fan-ziqi/rl_sar)
revision `376d42c9` (Apache-2.0, license and per-policy provenance kept under
`src/robot_lab_adapter/policies/rl_sar/`) on the lab's own robot models.
`robot_lab_adapter/legged_policy.py` reproduces rl_sar's exact observation,
history, scaling, action and PD contract (`ComputeObservation`,
`ObservationBuffer`, `ComputeOutput` at the pinned revision). Each
backend applies the policy's trained PD law at the physics rate, not over
ROS:

- **MuJoCo:** general actuators (`robot_lab_mujoco/joint_pd.py`).
- **Isaac:** PhysX joint drives carrying the gains, at the policy's 200 Hz.
- **Gazebo:** an effort `joint_trajectory_controller` in the description's
  ros2_control (Go2).

## Provisioned policies

| Policy | Robot model | Joints | Input | Notes |
|---|---|---|---|---|
| `go2_himloco` | `unitree_go2` | 12 | 270 (6-frame history) | Flat-ground velocity policy |
| `go2w_robot_lab` | `unitree_go2w` | 16 (4 wheel) | 57 | Legs position, wheels velocity |
| `a1_legged_gym` | `unitree_a1` | 12 | 270 | Flat-ground velocity policy |
| `b2_robot_lab` | `unitree_b2` | 12 | 45 | Flat-ground velocity policy |
| `g1_robomimic_locomotion` | `unitree_g1_29dof` | 29 | 96 + LSTM state | Humanoid; recurrent actor exported with its LSTM state as explicit inputs |

## Measured walks (2026-10-12)

Each cell is the forward speed for a 0.4 m/s command, then the yaw rate
for a 1.0 rad/s turn on the spot. Simulator truth, flat map.

| Robot / policy | MuJoCo | Isaac | Gazebo |
|---|---|---|---|
| Go2 / go2_himloco | 0.355; no spin (0.014) | 0.385; 0.43 | 0.374; no spin (0.020) |
| A1 / a1_legged_gym | 0.359; 0.93 | 0.386; 0.91 | not available |
| B2 / b2_robot_lab | 0.258; 0.89 | **0.651, still drifting 0.16 m/s after stop** | not available |
| Go2W / go2w_robot_lab | 0.292; 0.90 | 0.280; 0.88 | not available |
| G1 / g1_robomimic_locomotion | 0.386; 0.98 | 0.376; 0.93 | not available |

Each directory keeps the upstream `base.yaml`/`config.yaml` unchanged and a
`source.json` recording the upstream revision, TorchScript hash, ONNX hash
and the exact robot profile the checkpoint was trained for. A policy is
only ever paired with that model: `legged_policy:=` for a different
`robot_model` fails at launch with the binding printed, and there is no
"closest robot" fallback. Re-export or add policies with
`scripts/provision_rl_sar_policies.py`.

## Run

Localization mode, simulation only. MuJoCo (opt-in):

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42   # unused domain; see the Go2 tutorial
ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=loc simulator:=mujoco map_name:=nav_empty \
    robot_model:=unitree_go2 legged_policy:=go2_himloco gui:=false
```

In Isaac (all five robots) and Gazebo (Go2), the robot's policy is
selected automatically, because nothing else keeps it upright there.
`legged_policy:=none` turns it off:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=loc simulator:=isaac map_name:=nav_empty robot_model:=unitree_g1_29dof gui:=false
```

In the GUI:

1. Select the robot and the `loc` mode.
2. In MuJoCo, enable "Model-matched legged policy (rl_sar, experimental)"
   in the Launch column. In Isaac/Gazebo the policy is always on and the
   checkbox is disabled.

The command preview shows the exact `legged_policy:=` value.

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

## Mapping and navigation (experimental, 2026-10-13)

In `slam` and `nav` modes, MuJoCo, Isaac and Gazebo select the robot's
policy automatically, as Isaac and Gazebo already do in `loc`. The GUI
fills in the policy for these modes.

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=nav simulator:=mujoco map_name:=nav_obstacle robot_model:=unitree_go2 gui:=false
```

**Odometry.** A walking robot has no wheels to count, so
`leg_odometry` estimates the trunk velocity from the stance feet:

- forward kinematics of each foot from `/joint_states` (URDF), plus the IMU
  rate, gives `v = -(J q̇ + ω × p)` for each foot on the ground;
- a foot counts as stance when it is within 3 cm of the lowest foot and
  agrees with the previous estimate (swing feet near the ground would
  otherwise make the robot look 20-40% slower);
- the result is published on `/leg_odometry`, and the EKF uses it as
  `odom0`. AMCL then corrects the drift against the map.

On a 9 m walk the leg odometry ends 1.3-7.1% of the distance from truth.

**Navigation.** Regulated Pure Pursuit drives the policy through
`/cmd_vel`. The per-robot overlays in
`src/robot_lab_navigation/config/robots/` set the footprints, a minimum
speed each gait actually walks at, and no turn ramp (a gait that ignores
0.1 rad/s would never start turning). Go2 does not rotate to the goal
heading, because himloco cannot turn on the spot in MuJoCo or Gazebo.

Where a robot cannot turn on the spot (Go2 in MuJoCo and Gazebo), or is
too long to turn beside walls (B2 in MuJoCo), its profile selects
`motion_model:=forward_arc`:

- Hybrid-A* plans forward-only curves (0.4 m radius for Go2, 0.5 m for
  B2) and checks the robot's footprint along them;
- the follower never spins or reverses;
- recovery clears the local costmap and replans instead of spinning or
  backing up.

This fixed the aborts beside the block at the nav_obstacle start.

**Measured** on the nav_obstacle mission (detour, cancel, resume, second
goal; [evidence](../status/evidence/p7-legged-navigation-2026-10-13/README.md)):

| Robot | MuJoCo | Isaac | Gazebo |
|---|---|---|---|
| Go2 | **3/3** (forward_arc) | 2/3 | **3/3** (forward_arc) |
| A1 | **3/3** | **3/3** | not available |
| B2 | 2/3 (forward_arc; the miss is 4 mm over the 0.30 m test) | not run (drifts) | not available |
| G1 | **3/3** | **3/3** (wider inflation, 2026-10-16; 2/3 before) | not available |

Three goal seeds per cell. Passing goals end 0.01-0.23 m from truth. B2 and G1
needed minimum speeds of 0.4 and 0.3 m/s: below those their gaits stand
still (the G1's leg odometry then crept on in Isaac, so Nav2 declared
goals reached 0.5 m short). The G1's Isaac seed-3 abort came from a path
along a wall: with its global inflation widened to 0.8 m at cost scaling
2.0, it passes all three seeds in both simulators.

**Mapping.** `mode:=slam` runs slam_toolbox on the robot's laser scan, with
leg odometry as the only odometry. Over a 26 m loop through nav_obstacle
the slam pose stayed within 4.5-7.1 cm RMSE of truth for every walking
robot tried (A1, B2, G1 in MuJoCo; Go2, A1, G1 in Isaac; Go2 in Gazebo),
and the MuJoCo and Isaac maps put every wall cell on a real wall
([evidence](../status/evidence/p7-legged-slam-mapping-2026-10-13/README.md)).

```bash
scripts/slam_mapping_check.sh mujoco unitree_a1 nav_obstacle /path/to/out.json
```

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

The walks above are flat-ground velocity screens; the navigation results
are one seeded mission per cell on one flat map. These remain unqualified
(task ledger R5.2 and R3.6):

- terrain, falls and recovery;
- mapping beyond the one measured loop;
- navigation beyond the cells measured above.

Known limits:

- Go2 himloco does not turn on the spot in MuJoCo or Gazebo. A
  navigation stack that needs in-place rotation would stall there.
- B2 overshoots and drifts in Isaac.
- Only Go2's description has the ros2_control block Gazebo needs.
- PyBullet and hardware have no actuation path for these policies; the
  launch rejects them.

Pure-logic contracts live in
`src/robot_lab_adapter/test/test_r5_2_legged_policy.py` (fast tier): policy
loading and bindings, observation/history math against each pinned input
width, PD targets, command clamping and the MuJoCo PD bridge.
