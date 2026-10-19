# Training legged policies (Training tab)

Documentation added October 19, 2026. This is an **experimental** training
tool. It is implemented and smoke-tested, but no policy trained with it has
qualified walking yet. Read [current status](../status/CURRENT_STATUS.md)
first. A trained policy is a candidate: it needs its own measured runs
before any claim.

The GUI's **Training** page (EXPERIMENTS → Policy training) trains a
locomotion policy for a quadruped and hands the result to Launch. The same
steps run from the command line through
`ros2 run robot_lab_adapter legged-train`.

## What it trains

| Robot | Contract of | Joints | Policy input |
|---|---|---|---|
| `unitree_go2` | `go2_himloco` | 12 | 45 |
| `unitree_a1` | `a1_legged_gym` | 12 | 45 |
| `unitree_b2` | `b2_robot_lab` | 12 | 45 |

Each robot trains in the contract of its
[provisioned rl_sar policy](legged-policies.md). The joint order, default
pose, action scale, kp/kd, torque limits, `dt` and decimation all come from
that policy. The observation is
`ang_vel, gravity_vec, commands, dof_pos, dof_vel, actions`. It has no
history and no linear velocity, because a robot does not measure its own
velocity. Go2W (wheels) and the G1 (a recurrent humanoid) are not trainable
here yet.

Tasks (`robot_lab_adapter/legged_training.py`, `TASKS`):

| Task | Commands | Main reward terms |
|---|---|---|
| `walk` | vx −0.6…1.0, vy ±0.4, wz ±1.0 m/s, rad/s; 10 % zero | velocity tracking, feet air time |
| `turn_in_place` | wz ±(0.3…1.0) rad/s, no translation | yaw tracking, hold position |
| `stand` | none | default pose, base height, hold position |
| `push_recovery` | none, a 0.6 m/s push every 4 s | as `stand` |

Every task adds the same legged_gym-style regularisers: vertical velocity,
roll/pitch rate, tilt, torque, joint acceleration, action rate, and −200
when the robot falls. A fall is a tilt past 60° or the base below 40 % of
its standing height.

## How it works

- **Simulator.** MuJoCo steps many copies of the robot in parallel C
  threads (`mujoco.rollout`). The model comes from the same URDF import,
  mesh staging and in-step PD actuators that the MuJoCo bridge uses
  (`robot_lab_mujoco`), on a flat floor. The policy therefore trains on the
  plant it will run on in MuJoCo.
- **Learning.** PPO (`robot_lab_adapter/ppo.py`, torch on the CPU):
  - generalised advantage estimation;
  - clipped surrogate and value losses;
  - an entropy bonus and the KL-adaptive learning rate of rsl_rl;
  - running observation normalisation, frozen within each rollout.
- **Export.** `export_onnx` folds the normalisation into the first layer
  and writes a plain ONNX graph (Gemm and Elu) without the `onnx` package.
- **Policy directory.** An exported policy is an ordinary policy directory
  (`base.yaml`, `config.yaml`, `policy.onnx`, `source.json`) bound to its
  robot by `source.json`'s `profile`. `legged_policy:=<directory>` runs it
  through the same `LeggedPolicy` code as the provisioned policies.

## From the GUI

```bash
cd /workspace/molar/ros_ws/bumperbot_ws
source scripts/ssd_env.sh && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run robot_lab_gui robot_lab_gui
```

1. Open **Policy training**. Choose the robot and task, and set the
   parallel environments, iterations and seed. A run folder is created
   under `$ROBOT_LAB_RUNTIME_ROOT/training/` unless you name one.
2. **Start training.** The page plots mean return and episode length and
   shows the reward terms, KL, learning rate and steps per second. The
   trainer runs as its own process. It keeps running if the window closes,
   and the page finds it again on reopening.
3. **Stop and export** writes a `STOP` file and signals the trainer. It
   saves a checkpoint and exports `policy/` before it exits. Finished
   runs export automatically.
4. **Evaluate policy** runs the exported ONNX through the deployment code
   in the training simulator. It reports the measured body velocities for
   a few commands in `evaluation.json`.
5. **Use policy in Launch** selects the robot and `loc` mode and turns the
   legged policy on. The command then carries `legged_policy:=<run>/policy`.
   The Launch column shows the trained policy, with **Clear** to go back
   to the rl_sar policy. Another robot never receives it.

Resume a run from any `checkpoints/model_NNNNN.pt` or `latest.pt` with
**Resume from**.

## From the command line

```bash
RUN=$ROBOT_LAB_RUNTIME_ROOT/training/go2_walk_1
ros2 run robot_lab_adapter legged-train train --robot unitree_go2 --task walk --envs 512 --iterations 1500 --out $RUN
ros2 run robot_lab_adapter legged-train evaluate --policy-dir $RUN/policy --task walk --commands "0.5,0,0;0,0,0.8;0,0,0"
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=loc simulator:=mujoco \
    map_name:=nav_empty robot_model:=unitree_go2 legged_policy:=$RUN/policy gui:=false
```

`train` writes into the run folder:

- `run.json`, the settings, revision and trainer PID;
- `progress.jsonl`, one line per iteration;
- `checkpoints/`;
- `trainer.log`, when started from the GUI;
- `policy/` at the end.

`ros2 run robot_lab_adapter legged-train export --checkpoint … --out …` exports any checkpoint.

## Measured so far

- **Pipeline smoke run** (Go2, `stand`, 64 environments, 4 iterations):
  it trains, writes checkpoints, exports a policy directory that loads as
  an ordinary policy, and evaluates through `LeggedPolicy` without falling.
- **Speed** on the 12-core Orin: 512 environments × 24 steps take 1.8-2.3 s
  per iteration (about 0.7 s of physics at 17-23 k steps/s, the rest the
  PPO update). That is about 20 M environment steps an hour.

## Limits

- Training is CPU-only on this host: torch's CUDA build does not run on the
  Orin. A useful walk needs hours; `stand` needs minutes.
- Flat floor only: no terrain curriculum, no domain randomisation of mass,
  friction or motor strength, and no latency model. A policy may fit the
  MuJoCo plant and fail in Gazebo or Isaac.
- The launch checks that a policy belongs to its robot, but not that it
  works. Measure a trained policy (`sim_drive_check.py`, the mission
  screens) before relying on it. The Stop, command-loss and neutral paths
  of `legged-policy-controller` apply to it as to any policy.
