# R6.1 Gazebo car turning and reverse goals, 2026-09-30

Host: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress. Starting revision:
`9ff8921`; the navigation probe changes were in the working tree. Each
launch used its own ROS domain and Gazebo partition. The probe sent a
map-frame goal through `/robot_lab/goal_pose`, the same relay used by the
RViz goal tool, in the `nav_empty` map from the configured (-0.5, -0.5)
spawn. The turning goal was 1 m east and 1 m north with a 90-degree goal
heading; the reverse goal was 1 m west with a 0-degree heading. The car
goal checker permits a large final heading tolerance, so these are
position-goal tests, not final-heading qualification.

| Robot | Goal | Nav2 result | Final localized error | Wheel-odom forward / reverse | Wheel-odom heading travel |
| --- | --- | --- | ---: | ---: | ---: |
| `ackermann_car` | turn | succeeded | 0.272 m | 1.269 / 0.000 m | 1.313 rad |
| `ackermann_car` | reverse | succeeded | 0.206 m | 0.000 / 0.768 m | 0.077 rad |
| `rear_steer_car` | turn | succeeded | 0.272 m | 1.278 / 0.000 m | 1.316 rad |
| `rear_steer_car` | reverse | succeeded | 0.198 m | 0.000 / 0.785 m | 0.060 rad |
| `anti_ackermann_car` | turn | succeeded | 0.280 m | 1.255 / 0.000 m | 1.321 rad |
| `anti_ackermann_car` | reverse | succeeded | 0.211 m | 0.000 / 0.785 m | 0.042 rad |

The six JSON files here are the raw probe summaries. Forward/reverse
distance and heading travel are integrated from
`/robot_lab_controller/odom`, the Gazebo wheel-controller estimate.
Gazebo does not publish `/odom/ground_truth` on this launch path, so
those motion measurements are not independent ground-truth validation.
All six action results and final errors are from Nav2 and the
`map -> base_footprint` estimate. Results show useful turn/reverse
behavior in this one clear arena; furnished maps, other simulators,
obstacle avoidance, final orientation, and hardware remain unqualified.

Example reproduction after building and sourcing the workspace:

```bash
TIMEOUT=140 CHECK_ARGS='--via-topic --offset-x 1.0 --offset-y 1.0 --goal-yaw-deg 90 --clearance 0.5' ROS_DOMAIN_ID=105 \
  bash scripts/sim_nav_check.sh gazebo ackermann_car nav_empty /tmp/car_turn.json
TIMEOUT=140 CHECK_ARGS='--via-topic --offset-x -1.0 --offset-y 0.0 --goal-yaw-deg 0 --clearance 0.5' ROS_DOMAIN_ID=104 \
  bash scripts/sim_nav_check.sh gazebo ackermann_car nav_empty /tmp/car_reverse.json
```
