#!/usr/bin/env bash
# R5.3 effort-interface live probe (2026-09-17).
#
# Launches the dispatch path headless and checks, live:
#   1. both controllers load and activate against the effort command interfaces
#      (bhl_standing_controller as effort_controllers/JointGroupEffortController)
#   2. the topic contract is live (/joint_states, IMU, command topic)
#   3. the standing controller's zero-drive hold keeps the biped upright and the
#      command topic carries efforts (tau ~= 0 at the hold), not position targets
#
# Every ros2 CLI call is wrapped in `timeout` so a missing service can never
# hang the probe. Progress is appended to probe_progress.log.
#
# Usage: bash run.sh [output_dir]
set -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-$HERE}"
WS="$(cd "$HERE/../../.." && pwd)"
mkdir -p "$OUT"

PROGRESS="$OUT/probe_progress.log"
: > "$PROGRESS"
log() { echo "$@" | tee -a "$PROGRESS"; }

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
# Deliberately no `set -u`: the ROS 2 setup scripts read unset vars.
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash" 2>/dev/null || true

ros2t() { timeout 15 ros2 "$@"; }

LAUNCH_LOG="$OUT/launch.log"
: > "$LAUNCH_LOG"

cleanup() {
  if [ -n "${LAUNCH_PID:-}" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null
    sleep 2
    kill -KILL "$LAUNCH_PID" 2>/dev/null
  fi
  pkill -f 'simulated_robot.launch.py' 2>/dev/null
  pkill -f 'humanoid-standing-controller' 2>/dev/null
  sleep 1
  pkill -f 'ign gazebo' 2>/dev/null
  pkill -f 'ruby.*ign' 2>/dev/null
  return 0
}
trap cleanup EXIT

log "=== launching dispatch path (headless, gazebo) ==="
timeout -s INT 200 ros2 launch robot_lab_bringup simulated_robot.launch.py \
  mode:=display map_name:=empty robot_model:=berkeley_humanoid_lite_sim \
  simulator:=gazebo gui:=false start_rviz:=false \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

log "=== waiting for both controllers to activate ==="
deadline=$((SECONDS + 120))
activated=0
while [ "$SECONDS" -lt "$deadline" ]; do
  # Do not gate on the CLI's exit status: `ros2 control` can emit warnings and
  # still print the table, so inspect the captured output only.
  ros2t control list_controllers >"$OUT/controllers.txt" 2>/dev/null
  if grep -q 'bhl_standing_controller .*active' "$OUT/controllers.txt" \
     && grep -q 'joint_state_broadcaster .*active' "$OUT/controllers.txt"; then
    activated=1
    break
  fi
  sleep 3
done
log "activated=$activated (standing=$(grep -c 'bhl_standing_controller .*active' "$OUT/controllers.txt" 2>/dev/null) jsb=$(grep -c 'joint_state_broadcaster .*active' "$OUT/controllers.txt" 2>/dev/null))"

if [ "$activated" -ne 1 ]; then
  log "FAIL: controllers did not activate within 120 s"
  cat "$OUT/controllers.txt" >>"$PROGRESS" 2>/dev/null
  tail -25 "$LAUNCH_LOG" >>"$PROGRESS"
  exit 1
fi

log "=== controllers (name type state) ==="
cat "$OUT/controllers.txt" | tee -a "$PROGRESS"

log "=== topics ==="
ros2t topic list >"$OUT/topics.txt" 2>&1 || true
grep -E 'joint_states|imu|standing_controller|clock' "$OUT/topics.txt" | tee -a "$PROGRESS" || true

# The standing controller is a dispatched algorithm, not part of the robot
# description, so display mode does not start it: start it here so the command
# topic is actually driven.
IMU_TOPIC="$(grep -E '/imu' "$OUT/topics.txt" | head -1)"
IMU_TOPIC="${IMU_TOPIC:-/imu/out}"
log "=== starting humanoid-standing-controller (imu_topic:=$IMU_TOPIC) ==="
ros2 run robot_lab_adapter humanoid-standing-controller --ros-args \
  -p imu_topic:="$IMU_TOPIC" -p command_interface:=effort \
  >"$OUT/standing_node.log" 2>&1 &
NODE_PID=$!

sleep 3

log "=== measuring rates and tilt (12 s) ==="
timeout 60 python3 "$HERE/probe.py" "$OUT" "$IMU_TOPIC" >>"$PROGRESS" 2>&1
rc=$?
log "probe.py exit=$rc"

log "=== standing node log ==="
tail -12 "$OUT/standing_node.log" >>"$PROGRESS" 2>/dev/null || true

if [ -n "${NODE_PID:-}" ]; then
  kill -INT "$NODE_PID" 2>/dev/null
fi
exit $rc

