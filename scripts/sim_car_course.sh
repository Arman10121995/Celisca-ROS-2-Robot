#!/bin/bash
# Launch a car in the car course (cone slalom and parking bay, around the
# nav_empty spawn) in navigation mode, run sim_car_course.py, stop.
#
# Usage: scripts/sim_car_course.sh <simulator> <robot> <output.json> [extra launch args...]
set -m
SIM=$1; ROBOT=$2; OUT=$3; shift 3
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/ssd_env.sh" || exit 2
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-91}
source "$HERE/gazebo_isolation.sh"; robot_lab_isolate_gazebo car_course_$$
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
COURSE=$(ros2 pkg prefix robot_lab_maps)/share/robot_lab_maps/maps/car_course
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=nav \
  simulator:="$SIM" robot_model:="$ROBOT" map_name:=nav_empty world_name:=car_course \
  world_path:="$COURSE/worlds/car_course.world" map_yaml:="$COURSE/maps/map.yaml" \
  gui:=false start_rviz:=false "$@" > "$OUT.launch.log" 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT "$LAUNCH" 2>/dev/null || true
  for ((i=0;i<30;i++)); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
  kill -TERM -- -"$LAUNCH" 2>/dev/null || true
  sleep 1
  kill -KILL -- -"$LAUNCH" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 "$HERE/sim_car_course.py" --robot "$ROBOT" --course "$COURSE/course.json" --timeout "${TIMEOUT:-600}" \
  --trace-out "$OUT.trace.json" ${CHECK_ARGS:-} > "$OUT" 2> "$OUT.err"
CHECK_RESULT=$?
[ -s "$OUT" ] || echo '{"error": "checker produced no output"}' > "$OUT"
exit "$CHECK_RESULT"
