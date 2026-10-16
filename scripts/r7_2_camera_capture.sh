#!/bin/bash
# Record a robot's RGB-D stream standing still, then turning slowly, for
# scripts/r7_2_image_quality.py.
#
#   scripts/r7_2_camera_capture.sh <simulator> <robot> <map> <bag_dir> [launch args...]
# STILL_S (default 15) and TURN_S (default 15, at 0.3 rad/s) set the phases.
set -m
SIM=$1; ROBOT=$2; MAP=$3; BAG=$4; shift 4
HERE=$(cd "$(dirname "$0")" && pwd)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-94}
source "$HERE/gazebo_isolation.sh"; robot_lab_isolate_gazebo camera_capture_$$
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
rm -rf "$BAG"
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=loc \
  simulator:="$SIM" robot_model:="$ROBOT" map_name:="$MAP" gui:=false \
  start_rviz:=false "$@" > "$BAG.launch.log" 2>&1 &
LAUNCH=$!
# Wait for the first colour image (Isaac takes a minute or two to start).
# ros2 topic echo returns at once while the topic does not exist yet, so
# retry until an image arrives (round 47b's Isaac captures recorded only
# the first 37 s because of that).
deadline=$((SECONDS + ${READY_S:-400}))
until timeout 15 ros2 topic echo --once /oakd/rgb/image_raw > /dev/null 2>&1; do
  [ $SECONDS -ge $deadline ] && break
  sleep 3
done
sleep "${WARMUP:-5}"
ros2 bag record -o "$BAG" /oakd/rgb/image_raw /oakd/depth/image_raw /oakd/rgb/camera_info \
  /odom/ground_truth /tf_static > "$BAG.record.log" 2>&1 &
RECORD=$!
sleep "${STILL_S:-15}"
timeout "${TURN_S:-15}" ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{angular: {z: 0.3}}" > /dev/null 2>&1
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
sleep 2
kill -INT "$RECORD" 2>/dev/null; wait "$RECORD" 2>/dev/null
kill -INT "$LAUNCH" 2>/dev/null
for _ in $(seq 1 40); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
kill -TERM -- -"$LAUNCH" 2>/dev/null || true
sleep 1
kill -KILL -- -"$LAUNCH" 2>/dev/null || true
[ -d "$BAG" ]
