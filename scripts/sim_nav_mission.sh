#!/bin/bash
# Launch navigation mode, run sim_nav_mission.py (detour, cancel, resume, near-obstacle goal), stop.
#
# Usage: scripts/sim_nav_mission.sh <simulator> <robot> <map> <output.json>
#          [extra launch args...]
set -m
SIM=$1; ROBOT=$2; MAP=$3; OUT=$4; shift 4
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/ssd_env.sh" || exit 2
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-92}
source "$HERE/gazebo_isolation.sh"; robot_lab_isolate_gazebo nav_mission_$$
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
mode_check=()
for launch_arg in "$@"; do
  if [[ "$launch_arg" == four_wheel_steer_mode:=* ]]; then
    echo 'Use steering_mode:=PATTERN; four_wheel_steer_mode is not a launch argument.' >&2
    exit 2
  fi
  if [[ "$launch_arg" == steering_mode:=* ]]; then
    drive_node="${SIM}_spawner"
    [ "$SIM" != gazebo ] || drive_node=holonomic_controller
    mode_check=(--drive-node "$drive_node" --expected-steering-mode "${launch_arg#steering_mode:=}")
  fi
done

ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=nav \
  simulator:="$SIM" robot_model:="$ROBOT" map_name:="$MAP" gui:=false \
  start_rviz:=false "$@" > "$OUT.launch.log" 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT "$LAUNCH" 2>/dev/null || true
  for ((i=0;i<30;i++)); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
  # The launch parent may exit while a simulator descendant is still alive.
  kill -TERM -- -"$LAUNCH" 2>/dev/null || true
  sleep 1
  kill -KILL -- -"$LAUNCH" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 "$HERE/sim_nav_mission.py" --timeout "${TIMEOUT:-600}" --trace-out "$OUT.trace.json" "${mode_check[@]}" ${CHECK_ARGS:-} > "$OUT" 2> "$OUT.err"
CHECK_RESULT=$?
[ -s "$OUT" ] || echo '{"error": "checker produced no output"}' > "$OUT"
exit "$CHECK_RESULT"
