#!/bin/bash
# Launch a robot in slam mode, drive it around the world with
# slam_mapping_check.py, score the trajectory and map, stop.
#
# Usage: scripts/slam_mapping_check.sh <simulator> <robot> <map> <output.json> [launch args...]
# CHECK_ARGS are passed to slam_mapping_check.py (e.g. "--base base --min-speed 0.3").
set -m
SIM=$1; ROBOT=$2; MAP=$3; OUT=$4; shift 4
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/ssd_env.sh" || exit 2
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-95}
source "$HERE/gazebo_isolation.sh"; robot_lab_isolate_gazebo slam_map_$$
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=slam \
  simulator:="$SIM" robot_model:="$ROBOT" map_name:="$MAP" gui:=false \
  start_rviz:=false "$@" > "$OUT.launch.log" 2>&1 &
LAUNCH=$!
rm -rf "$OUT.tf.bag"
ros2 bag record -o "$OUT.tf.bag" /tf /tf_static /odom/ground_truth > /dev/null 2>&1 &
BAG=$!
python3 "$HERE/slam_mapping_check.py" --map "$MAP" --map-out "${OUT%.json}.pgm" ${CHECK_ARGS:-} \
  > "$OUT" 2> "$OUT.err"
CHECK_RESULT=$?
kill -INT "$BAG" 2>/dev/null
kill -INT "$LAUNCH" 2>/dev/null
for _ in $(seq 1 40); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
kill -TERM -- -"$LAUNCH" 2>/dev/null || true
sleep 1
kill -KILL -- -"$LAUNCH" 2>/dev/null || true
wait "$BAG" 2>/dev/null
[ -s "$OUT" ] || echo '{"error": "checker produced no output"}' > "$OUT"
exit "$CHECK_RESULT"
