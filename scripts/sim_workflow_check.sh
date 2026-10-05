#!/usr/bin/env bash
# Serial, bounded live workflow check; all outputs belong on the SSD.
set -m
SIM=$1; ROBOT=$2; MAP=$3; MODE=$4; OUT=$5; shift 5
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$HERE/ssd_env.sh" || exit 2
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-163}
export IGN_PARTITION=workflow_$$ GZ_PARTITION=workflow_$$
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
mapping_args=()
if [ "$MODE" = '3d_slam' ]; then
  mapping_args+=("rtabmap_database_path:=${OUT%.json}.rtabmap.db")
fi
ros2 launch robot_lab_bringup simulated_robot.launch.py simulator:="$SIM" \
  robot_model:="$ROBOT" map_name:="$MAP" mode:="$MODE" gui:=false start_rviz:=false \
  "${mapping_args[@]}" "$@" > "$OUT.launch.log" 2>&1 &
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
timeout "${WALL_TIMEOUT:-600}" python3 "$HERE/sim_workflow_check.py" \
  --mode "$MODE" --out "$OUT" --timeout "${TIMEOUT:-240}" "${mode_check[@]}" ${CHECK_ARGS:-} > "$OUT.probe.log" 2>&1
exit $?
