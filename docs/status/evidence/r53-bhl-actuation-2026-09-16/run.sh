#!/usr/bin/env bash
# R5.3 actuation probe: raw controller, no policy.
# Phase 1: all-zero position commands 0-8 s (rigid robot should stand).
# Phase 2: knees 0.4 / shoulders 0.8 commands 8-20 s (joints should move).
set -o pipefail
OUT=/tmp/r53actuation
mkdir -p "$OUT"
cd /home/molar1/bumperbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

pkill -f 'ign gazebo' 2>/dev/null; sleep 1

nohup ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=display map_name:=empty \
    robot_model:=berkeley_humanoid_lite_sim \
    simulator:=gazebo gui:=false start_rviz:=false \
    > "$OUT/launch.log" 2>&1 &
LPID=$!
echo "$LPID" > "$OUT/pid"

for i in $(seq 1 40); do
    if ros2 control list_controllers 2>/dev/null | grep -q 'bhl_standing_controller.*active'; then
        echo "controllers active after ~${i}s" | tee "$OUT/cm_ready.txt"
        break
    fi
    sleep 1
done

python3 "$OUT/actuation_probe.py" "$OUT/actuation_report.json" \
    > "$OUT/probe_stdout.log" 2>&1 &
PPID=$!
sleep 21
kill "$PPID" 2>/dev/null

kill "$LPID" 2>/dev/null; sleep 3
pkill -f 'ign gazebo' 2>/dev/null
echo "probe complete"
