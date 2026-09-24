#!/usr/bin/env bash
# Run from the workspace root with a unique ROS_DOMAIN_ID per robot.
set -e
set -m
robot="$1"
report="$2"
simulator="${3:-mujoco}"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1 MUJOCO_GL=egl
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>99</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  mode:=loc simulator:="$simulator" robot_model:="$robot" map_name:=nav_obstacle \
  gui:=false start_rviz:=false > "$report.launch.log" 2>&1 &
leader=$!
cleanup() {
  kill -INT "$leader" 2>/dev/null || true
  for _ in $(seq 1 30); do
    kill -0 "$leader" 2>/dev/null || break
    sleep 1
  done
  kill -0 "$leader" 2>/dev/null && kill -KILL -- -"$leader" 2>/dev/null || true
}
trap cleanup EXIT
python3 docs/status/evidence/r51-two-wheel-2026-09-24/probe_drive.py > "$report"
