#!/usr/bin/env bash
# R5.3 live validation of the StartupSettle startup transient.
set -o pipefail
OUT=/tmp/r53settle
mkdir -p "$OUT"
cd /home/molar1/bumperbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 1. dispatch-path bringup (controllers auto-spawn)
nohup ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=display map_name:=empty robot_model:=berkeley_humanoid_lite_sim \
    simulator:=gazebo gui:=false start_rviz:=false \
    > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!

# 2. wait for controller_manager readiness
for i in $(seq 1 60); do
    if ros2 service list 2>/dev/null | grep -q controller_manager; then
        echo "cm ready after ~$((i))s" > "$OUT/cm_ready.txt"
        break
    fi
    sleep 1
done
sleep 3
ros2 control list_controllers > "$OUT/controllers_before.txt" 2>&1

# 3. the settle probe (policy started inside, ramp triggered by zero cmd_vel)
timeout 90 python3 "$OUT/settle_probe.py" > "$OUT/probe_stdout.log" 2>&1
echo "probe rc: $?" >> "$OUT/probe_stdout.log"

# 4. final state
ros2 control list_controllers > "$OUT/controllers_after.txt" 2>&1

kill "$LAUNCH_PID" 2>/dev/null
sleep 4
pkill -f 'ign gazebo' 2>/dev/null
pkill -f robot_state_publisher 2>/dev/null
pkill -f parameter_bridge 2>/dev/null
pkill -f humanoid-policy-controller 2>/dev/null
echo "done"