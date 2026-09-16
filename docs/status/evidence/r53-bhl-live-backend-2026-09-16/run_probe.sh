#!/usr/bin/env bash
# R5.3 live backend proof: headless Gazebo + BHL controllers + policy node.
# Reproduces the bringup captured in launch.log / policy.log / diag.txt.
#
# Prerequisites: ROS 2 Humble sourced, workspace built and installed,
#   Ignition Fortress 6.18, ign_ros2_control-system + gz_ros2_control-system
#   plugin libs, robot_lab_adapter installed (humanoid-policy-controller resolves).
#
# Usage:
#   bash docs/status/evidence/r53-bhl-live-backend-2026-09-16/run_probe.sh [out-dir]
set -euo pipefail

OUT_DIR="${1:-/tmp/r53-live-backend-probe}"
mkdir -p "$OUT_DIR"

echo "=== R5.3 live backend probe ==="
echo "out: $OUT_DIR"

# --- Step 1: headless Gazebo with the BHL sim xacro + nav_empty ---
echo "[1] starting headless gazebo (robot_name:=bhl, world:=nav_empty, gui:=false)"
ros2 launch robot_lab_description gazebo.launch.py \
    world_name:=nav_empty \
    robot_name:=bhl \
    model:=$(ros2 pkg prefix robot_lab_robots)/share/robot_lab_robots/berkeley_humanoid_lite/xacro/bhl_sim.xacro \
    gui:=false \
    > "$OUT_DIR/gazebo.log" 2>&1 &
GAZEBO_PID=$!

# Wait for the spawner + controller_manager services to appear.
echo "[1] waiting for /controller_manager/load_controller service (up to 30 s)"
for i in $(seq 1 60); do
    if ros2 service list 2>/dev/null | grep -q controller_manager/load_controller; then
        echo "[1] controller_manager ready after ~${i}s"
        break
    fi
    sleep 0.5
done

# --- Step 2: diagnostic snapshot (matches diag.txt) ---
echo "[2] diagnostic snapshot"
{
    echo "--- controller_manager services ---"
    ros2 service list 2>/dev/null | grep controller_manager || true
    echo "--- joint/imu topics ---"
    ros2 topic list 2>/dev/null | grep -E 'imu/out|joint_states' || true
    echo "--- controllers ---"
    ros2 control list_controllers 2>/dev/null || true
} > "$OUT_DIR/diag.txt" 2>&1

# --- Step 3: policy node (matches policy.log) ---
echo "[3] starting humanoid_policy_controller (policy_humanoid, 25 Hz)"
source install/setup.bash 2>/dev/null || true
humanoid-policy-controller \
    --imu_topic /imu/out \
    --joint_states_topic /joint_states \
    --command_topic /bhl_standing_controller/commands \
    --cmd_vel_topic /cmd_vel \
    --policy_name policy_humanoid \
    --command_rate_hz 25.0 \
    > "$OUT_DIR/policy.log" 2>&1 &
POLICY_PID=$!

# Give the policy node a moment to init and print its rate line.
sleep 2

# --- Step 4: send zero cmd_vel so the policy leaves the default-pose hold ---
echo "[4] sending zero cmd_vel (policy leaves hold, starts onboard loop)"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" --once 2>/dev/null || true

# --- Step 5: watch for the safe_stop (un-balanced spawn tilts ~0.79 rad) ---
echo "[5] waiting for policy safe_stop or 20 s timeout"
for i in $(seq 1 40); do
    if grep -q 'safe_stop' "$OUT_DIR/policy.log" 2>/dev/null; then
        echo "[5] safe_stop detected after ~${i}s"
        break
    fi
    sleep 0.5
done

# --- Step 6: second diagnostic snapshot ---
echo "[6] final diagnostic snapshot"
{
    echo "--- controllers (final) ---"
    ros2 control list_controllers 2>/dev/null || true
    echo "--- topics (final) ---"
    ros2 topic list 2>/dev/null | grep -E 'imu/out|joint_states|bhl_standing_controller/commands' || true
} >> "$OUT_DIR/diag.txt" 2>&1

# --- Step 7: stop everything ---
echo "[7] stopping"
kill "$POLICY_PID" 2>/dev/null || true
kill "$GAZEBO_PID" 2>/dev/null || true
wait "$POLICY_PID" 2>/dev/null || true
wait "$GAZEBO_PID" 2>/dev/null || true

echo "=== done: $OUT_DIR ==="
ls -la "$OUT_DIR"
