#!/bin/bash
# Run one cleaning job of the robot-vacuum cleaning system and record it.
#
# Usage: scripts/sim_vacuum_mission.sh <simulator> <robot> <map> <output.json>
#          <start: mode or JSON start command> [extra launch args...]
# The node writes the job record to <output.json> once the robot is back on
# the dock (or the job could not start); TIMEOUT (default 3600 s) bounds it.
# The cleaning state (layouts, history) goes to a fresh store next to the
# output so runs do not share remembered obstacles or room names.
set -m
SIM=$1; ROBOT=$2; MAP=$3; OUT=$4; START=$5; shift 5
HERE=$(cd "$(dirname "$0")" && pwd)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-94}
source "$HERE/gazebo_isolation.sh"; robot_lab_isolate_gazebo vacuum_$$
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
source "$HERE/ssd_env.sh"
rm -f "$OUT"

ros2 launch robot_lab_bringup simulated_room_vacuum.launch.py mode:="${MODE:-nav}" \
  simulator:="$SIM" robot_model:="$ROBOT" map_name:="$MAP" gui:=false start_rviz:=false \
  controller:=cleaning vacuum_auto_start:="$START" vacuum_report:="$OUT" \
  vacuum_store_dir:="${OUT%.json}_store/vacuum" \
  vacuum_battery_time_scale:="${BATTERY_SCALE:--1}" "$@" > "$OUT.launch.log" 2>&1 &
LAUNCH=$!
DEADLINE=$((SECONDS + ${TIMEOUT:-3600}))
# Record the Gazebo topics once the stack is up (bumper/cliff diagnostics).
if [ "$SIM" = gazebo ]; then
  ( sleep 45; { ign topic -l; echo '--- /bumper'; ign topic -i -t /bumper; } > "$OUT.gz_topics.txt" 2>&1 ) &
fi
while [ ! -s "$OUT" ] && [ $SECONDS -lt $DEADLINE ] && kill -0 "$LAUNCH" 2>/dev/null; do
  sleep 5
done
kill -INT "$LAUNCH" 2>/dev/null
for _ in $(seq 1 40); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
kill -TERM -- -"$LAUNCH" 2>/dev/null || true
sleep 1
kill -KILL -- -"$LAUNCH" 2>/dev/null || true
[ -s "$OUT" ] || echo '{"result": "no report", "error": "timed out or the launch exited"}' > "$OUT"
grep -Eq '"result": "(completed|map complete)' "$OUT"
