#!/bin/bash
# Launch navigation mode, send one Nav2 goal with sim_nav_check.py, stop.
#
# Usage: scripts/sim_nav_check.sh <simulator> <robot> <map> <output.json>
#          [extra launch args...]
set -m
SIM=$1; ROBOT=$2; MAP=$3; OUT=$4; shift 4
HERE=$(cd "$(dirname "$0")" && pwd)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-92}
export IGN_PARTITION=nav_check_$$ GZ_PARTITION=nav_check_$$
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"

ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=nav \
  simulator:="$SIM" robot_model:="$ROBOT" map_name:="$MAP" gui:=false \
  start_rviz:=false "$@" > "$OUT.launch.log" 2>&1 &
LAUNCH=$!
python3 "$HERE/sim_nav_check.py" ${DIST:+--distance $DIST} --timeout "${TIMEOUT:-600}" ${CHECK_ARGS:-} > "$OUT" 2> "$OUT.err"
kill -INT "$LAUNCH" 2>/dev/null
for _ in $(seq 1 40); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
kill -0 "$LAUNCH" 2>/dev/null && kill -KILL -- -"$LAUNCH" 2>/dev/null
[ -s "$OUT" ] || echo '{"error": "checker produced no output"}' > "$OUT"
