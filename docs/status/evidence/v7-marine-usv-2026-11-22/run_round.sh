#!/bin/bash
# V7: the November 20 lab_usv backend round (surge, coast, yaw, current drift, operator loss) repeated, two runs per backend.
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v7-usv-20261122
DIR=$(cd "$(dirname "$0")" && pwd); mkdir -p "$RUN"
source "$HERE/scripts/ssd_env.sh" >/dev/null; source /opt/ros/humble/setup.bash; source "$HERE/install/setup.bash"
export ROS_DOMAIN_ID=117; LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"
for i in 1 2; do for sim in gazebo mujoco pybullet isaac; do
  [ -s "$RUN/$sim-$i/result.json" ] || flock -w 1800 "$LOCK" python3 "$DIR/run_case.py" $sim "$RUN/$sim-$i"
  echo "$sim-$i exit=$?" >> "$DIR/progress.txt"; sleep 3
done; done
echo ALLDONE >> "$DIR/progress.txt"
