#!/bin/bash
# Launch a robot/map/mode, measure /clock real-time factor, stop.
#   scripts/sim_rtf_check.sh <simulator> <robot> <map> <mode> <output.json> [launch args...]
set -m
SIM=$1; ROBOT=$2; MAP=$3; MODE=$4; OUT=$5; shift 5
HERE=$(cd "$(dirname "$0")" && pwd)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-94}
source "$HERE/gazebo_isolation.sh"; robot_lab_isolate_gazebo rtf_check_$$
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
ros2 launch robot_lab_bringup simulated_robot.launch.py mode:="$MODE" \
  simulator:="$SIM" robot_model:="$ROBOT" map_name:="$MAP" gui:=false \
  start_rviz:=false "$@" > "$OUT.launch.log" 2>&1 &
LAUNCH=$!
python3 "$HERE/sim_rtf_probe.py" --window "${WINDOW:-20}" --warmup "${WARMUP:-10}" > "$OUT" 2> "$OUT.err"
kill -INT "$LAUNCH" 2>/dev/null
for _ in $(seq 1 40); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
kill -0 "$LAUNCH" 2>/dev/null && kill -KILL -- -"$LAUNCH" 2>/dev/null
[ -s "$OUT" ] || echo '{"error": "probe produced no output"}' > "$OUT"
