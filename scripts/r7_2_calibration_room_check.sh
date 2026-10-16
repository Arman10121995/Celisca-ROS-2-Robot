#!/bin/bash
# Launch a robot in the calibration room (chessboards around the nav_empty
# spawn) with perception:=camera_calibration, turn it on the spot in front of
# the boards (r7_2_calibration_room_check.py), stop.  Any simulator.
#
# Usage: scripts/r7_2_calibration_room_check.sh <simulator> <robot> <output.json> [launch args...]
set -m
SIM=$1; ROBOT=$2; OUT=$3; shift 3
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/ssd_env.sh" || exit 2
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-95}
source "$HERE/gazebo_isolation.sh"; robot_lab_isolate_gazebo calibration_room_$$
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
WORLD=$(ros2 pkg prefix robot_lab_maps)/share/robot_lab_maps/maps/calibration_room/worlds/calibration_room.world
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=slam simulator:="$SIM" \
  robot_model:="$ROBOT" map_name:=nav_empty world_name:=calibration_room world_path:="$WORLD" \
  gui:=false start_rviz:=false use_sim_time:=true perception:=camera_calibration "$@" \
  > "$OUT.launch.log" 2>&1 &
LAUNCH=$!
python3 "$HERE/r7_2_calibration_room_check.py" --out "$OUT" ${CHECK_ARGS:-} > "$OUT.log" 2>&1
RESULT=$?
kill -INT "$LAUNCH" 2>/dev/null
for _ in $(seq 1 40); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
kill -TERM -- -"$LAUNCH" 2>/dev/null || true
sleep 1
kill -KILL -- -"$LAUNCH" 2>/dev/null || true
[ -s "$OUT" ] || echo '{"error": "checker produced no output"}' > "$OUT"
exit "$RESULT"
