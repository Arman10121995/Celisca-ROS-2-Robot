#!/bin/bash
# V7: the November 20 fleet lift transport (L0 -> L1 -> L0 through Facility transport requests) repeated:
# MOLAR twice and the Bumperbot regression once, Gazebo fleet_facility.
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v7-transport-20261122
DIR=$(cd "$(dirname "$0")" && pwd); mkdir -p "$RUN"
source "$HERE/scripts/ssd_env.sh" >/dev/null; source /opt/ros/humble/setup.bash; source "$HERE/install/setup.bash"
export ROS_DOMAIN_ID=117; LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"
case_() { [ -s "$RUN/$1/run/result.json" ] || flock -w 1800 "$LOCK" python3 "$DIR/run_case.py" "$DIR/$2" "$3" "$RUN/$1"
  echo "$1 exit=$?" >> "$DIR/progress.txt"; sleep 5; }
case_ molar-1 molar-transport-session.yaml molar
case_ molar-2 molar-transport-session.yaml molar
case_ bumperbot-1 bumperbot-transport-session.yaml alpha
echo ALLDONE >> "$DIR/progress.txt"
