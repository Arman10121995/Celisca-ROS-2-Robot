#!/bin/bash
# Launch a Gazebo robot with perception:=camera_calibration, show its camera
# a rendered chessboard at many poses (r7_2_calibration_check.py), stop.
#
# Usage: scripts/r7_2_calibration_check.sh <robot> <map> <output.json> [launch args...]
set -m
ROBOT=$1; MAP=$2; OUT=$3; shift 3
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/ssd_env.sh" || exit 2
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-95}
source "$HERE/gazebo_isolation.sh"; robot_lab_isolate_gazebo calibration_$$
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=slam simulator:=gazebo \
  robot_model:="$ROBOT" map_name:="$MAP" gui:=false start_rviz:=false use_sim_time:=true \
  perception:=camera_calibration "$@" > "$OUT.launch.log" 2>&1 &
LAUNCH=$!
python3 "$HERE/r7_2_calibration_check.py" --world "$MAP" --out "$OUT" ${CHECK_ARGS:-} > "$OUT.log" 2>&1
RESULT=$?
kill -INT "$LAUNCH" 2>/dev/null
for _ in $(seq 1 40); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
kill -TERM -- -"$LAUNCH" 2>/dev/null || true
sleep 1
kill -KILL -- -"$LAUNCH" 2>/dev/null || true
[ -s "$OUT" ] || echo '{"error": "checker produced no output"}' > "$OUT"
exit "$RESULT"
