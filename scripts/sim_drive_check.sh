#!/bin/bash
# Launch a robot, drive it through sim_drive_check.py's phases, stop.
#
# Usage: scripts/sim_drive_check.sh <simulator> <robot> <map> <output.json>
#          [extra launch args...]
# MODE (default display; Gazebo needs slam for its controllers) and ODOM
# (default /odom/ground_truth; Gazebo: /robot_lab_controller/odom).
set -m
SIM=$1; ROBOT=$2; MAP=$3; OUT=$4; shift 4
HERE=$(cd "$(dirname "$0")" && pwd)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-93}
export IGN_PARTITION=drive_check_$$ GZ_PARTITION=drive_check_$$
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
MODE=${MODE:-display}
ODOM=${ODOM:-/odom/ground_truth}

ros2 launch robot_lab_bringup simulated_robot.launch.py mode:="$MODE" \
  simulator:="$SIM" robot_model:="$ROBOT" map_name:="$MAP" gui:=false \
  start_rviz:=false "$@" > "$OUT.launch.log" 2>&1 &
LAUNCH=$!
python3 "$HERE/sim_drive_check.py" --odom "$ODOM" --timeout "${TIMEOUT:-300}" \
  --warmup "${WARMUP:-3}" > "$OUT" 2> "$OUT.err"
kill -INT "$LAUNCH" 2>/dev/null
for _ in $(seq 1 40); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
kill -0 "$LAUNCH" 2>/dev/null && kill -KILL -- -"$LAUNCH" 2>/dev/null
[ -s "$OUT" ] || echo '{"error": "checker produced no output"}' > "$OUT"
