#!/bin/bash
# V7: the November 18 fleet_ward run 3 (door crossing, docking, modeled charge) repeated three times on Gazebo.
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v7-ward-20261122
DIR=$(cd "$(dirname "$0")" && pwd); mkdir -p "$RUN"
source "$HERE/scripts/ssd_env.sh" >/dev/null; source /opt/ros/humble/setup.bash; source "$HERE/install/setup.bash"
export ROS_DOMAIN_ID=117; LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"
for i in 1 2 3; do
  [ -s "$RUN/run-$i/run/result.json" ] || flock -w 1800 "$LOCK" python3 "$DIR/run_case.py" "$RUN/run-$i"
  echo "run-$i exit=$?" >> "$DIR/progress.txt"; sleep 5
done
echo ALLDONE >> "$DIR/progress.txt"
