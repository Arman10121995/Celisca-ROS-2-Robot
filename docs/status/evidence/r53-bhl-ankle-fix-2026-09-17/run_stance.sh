#!/usr/bin/env bash
# R5.3 live validation of the parallel-ankle balance fix (80925bf).
# Trace-before-spawn ordering (learned from run 1: starting the balance node
# after controller activation leaves the effort controller publishing nothing
# for seconds - the biped goes limp and folds before any balance can act).
# Here the balance node AND the trace attach BEFORE bringup, so the PD-effort
# loop is closed from the first controller-activation cycle.
# Success: no SAFE_STOP, tilt stays well below 0.70 rad, no divergence trend.
set -o pipefail
# Output dir overridable so repeat runs (e.g. after the 2026-09-21 joint-friction
# fix) do not overwrite earlier evidence: OUT=/tmp/<new> bash run_stance.sh
OUT=${OUT:-/tmp/r53ankle2}
mkdir -p "$OUT"
cd /home/molar1/bumperbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 1. balance node FIRST - attached before the robot exists (warns on missing
#    data, which is the designed no-fabrication behavior)
#    START_BALANCE=off is the PASSIVE control condition (2026-09-21): no node
#    publishes /bhl_standing_controller/commands, so the effort controller
#    holds its initial zero command and the plant runs on its own joint
#    dynamics alone. It answers the honesty question "does the balance loop
#    hold the stance, or is the stance passively stable from the description's
#    joint losses?" - the same question the balance-core tests ask of a fixed
#    pose. Usage: OUT=/tmp/x START_BALANCE=off bash run_stance.sh
if [ "${START_BALANCE:-on}" = "on" ]; then
    nohup ros2 run robot_lab_adapter humanoid-standing-controller \
        --ros-args -p imu_topic:=/imu/out -p command_interface:=effort \
        > "$OUT/standing.log" 2>&1 &
    STANDING_PID=$!
else
    echo "START_BALANCE=off: passive condition, no balance node started" \
        > "$OUT/standing.log"
    STANDING_PID=""
fi

# 2. trace FIRST - captures the spawn transient (OUT passed so the probe
#    writes its report next to the other run artifacts)
nohup python3 "$(dirname "$0")/balance_probe.py" 45 "$OUT" \
    > "$OUT/probe_stdout.log" 2>&1 &
PROBE_PID=$!
sleep 1

# 3. dispatch-path bringup (controllers auto-spawn by the profile)
nohup ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=display map_name:=empty robot_model:=berkeley_humanoid_lite_sim \
    simulator:=gazebo gui:=false start_rviz:=false \
    > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!

# 4. wait for controller_manager readiness
READY=no
for i in $(seq 1 90); do
    if ros2 service list 2>/dev/null | grep -q controller_manager; then
        echo "cm ready after ~${i}s" | tee "$OUT/cm_ready.txt"
        READY=yes
        break
    fi
    sleep 1
done
if [ "$READY" != yes ]; then echo "controller_manager never appeared" | tee "$OUT/cm_ready.txt"; fi
sleep 3
ros2 control list_controllers > "$OUT/controllers_before.txt" 2>&1

# 5. wait for the probe to finish its 45 s hold
wait "$PROBE_PID" 2>/dev/null
echo "probe rc: $?"

# 6. final state
ros2 control list_controllers > "$OUT/controllers_after.txt" 2>&1

[ -n "$STANDING_PID" ] && kill "$STANDING_PID" 2>/dev/null
kill "$LAUNCH_PID" 2>/dev/null
sleep 4
pkill -f 'ign gazebo' 2>/dev/null
pkill -f robot_state_publisher 2>/dev/null
pkill -f parameter_bridge 2>/dev/null
pkill -f humanoid-standing-controller 2>/dev/null
pkill -f spawner 2>/dev/null
echo "done"
