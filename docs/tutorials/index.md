# Robot Lab Tutorials (P7.4)

Step-by-step guides that reproduce one comparison in each algorithm category.

## Quick Start

```bash
# 1. Bootstrap the workspace
bash scripts/bootstrap.sh

# 2. Activate environment
source install/setup.bash
source .venv/bin/activate

# 3. Run fast validation
bash scripts/test_fast.sh

# 4. Check workspace health
bash scripts/doctor.sh
```

## Simulator Backends

Tutorials run on the default **Gazebo Classic** backend. The same commands
accept `simulator:=isaac`, `simulator:=pybullet`, or `simulator:=mujoco`
(adapter backends; physics engines pending P7.8):

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=nav map_name:=small_office robot_model:=bumperbot simulator:=gazebo
```

## Tutorials by Category

| Category | Tutorial | What you'll compare |
|----------|----------|---------------------|
| Perception | [perception.md](perception.md) | Obstacle detection vs. scan clustering |
| Planning | [planning.md](planning.md) | RRT vs. Voronoi global planning |
| Localization | [localization.md](localization.md) | Dead reckoning integration |
| State Estimation | [state_estimation.md](state_estimation.md) | EKF convergence |
| Sensor Fusion | [sensor_fusion.md](sensor_fusion.md) | Complementary IMU filter |

## Running a Full Comparison

```bash
# Validate the registry
ros2 run robot_lab_registry robot-lab validate --cross-references

# List available experiments
ros2 run robot_lab_registry robot-lab list experiments

# Describe a specific experiment
ros2 run robot_lab_registry robot-lab describe bumperbot_smoke_test
```
