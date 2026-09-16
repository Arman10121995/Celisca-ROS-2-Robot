#!/bin/bash
# Launch a robot in display mode on one backend, run sim_sensor_probe.py
# against it, then stop the launch.
#
# Usage: scripts/sim_sensor_probe.sh <gazebo|pybullet|mujoco|isaac> <map>
#          <output.json> [--robot ID] [--camera] [--reset] [--timeout S]
#
# Runs on its own ROS domain (ROS_DOMAIN_ID, default 93) so it does not join
# another graph.  Job control is on so the backgrounded launch receives
# SIGINT; a background job in a non-interactive shell otherwise ignores it.
set -m
SIM=$1; MAP=$2; OUT=$3; shift 3
HERE=$(cd "$(dirname "$0")" && pwd)
ROBOT=bumperbot
ARGS=("$@")
for ((i = 0; i < ${#ARGS[@]}; i++)); do
  [ "${ARGS[$i]}" = "--robot" ] && ROBOT=${ARGS[$((i + 1))]}
done
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-93}
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
set -u  # only after sourcing: the ROS setup scripts read unset variables

ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=display \
  simulator:="$SIM" robot_model:="$ROBOT" map_name:="$MAP" gui:=false \
  start_rviz:=false > "$OUT.launch.log" 2>&1 &
LAUNCH=$!
python3 "$HERE/sim_sensor_probe.py" --map "$MAP" "$@" > "$OUT" 2> "$OUT.probe.err"
STATUS=$?
kill -INT "$LAUNCH" 2>/dev/null
for _ in $(seq 1 30); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
if kill -0 "$LAUNCH" 2>/dev/null; then
  echo "launch still running 30 s after SIGINT" >&2
  STATUS=1
fi
exit $STATUS
