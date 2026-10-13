#!/bin/bash
# Launch a walking robot in slam mode (policy + leg odometry), run
# leg_odometry_check.py, stop.
#
# Usage: scripts/leg_odometry_check.sh <simulator> <robot> <map> <output.json> [launch args...]
set -m
SIM=$1; ROBOT=$2; MAP=$3; OUT=$4; shift 4
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/ssd_env.sh" || exit 2
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-94}
source "$HERE/gazebo_isolation.sh"; robot_lab_isolate_gazebo leg_odom_$$
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:="${MODE:-slam}" \
  simulator:="$SIM" robot_model:="$ROBOT" map_name:="$MAP" gui:=false \
  start_rviz:=false "$@" > "$OUT.launch.log" 2>&1 &
LAUNCH=$!
python3 "$HERE/leg_odometry_check.py" --timeout "${TIMEOUT:-400}" --warmup "${WARMUP:-8}" > "$OUT" 2> "$OUT.err"
CHECK_RESULT=$?
kill -INT "$LAUNCH" 2>/dev/null
for _ in $(seq 1 40); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
kill -TERM -- -"$LAUNCH" 2>/dev/null || true
sleep 1
kill -KILL -- -"$LAUNCH" 2>/dev/null || true
[ -s "$OUT" ] || echo '{"error": "checker produced no output"}' > "$OUT"
exit "$CHECK_RESULT"
