#!/bin/bash
# R14.3 (Claude takeover): Codex's private Isaac F100 fetch qualification repeated after the bounded open-jaw
# alignment change (ddeb3c85) and the IK fix (be8e3ddb). Private runtime root with the three profile opt-ins
# (isaac_rgbd_on_demand, align_live_grasp, reanchor_before_descent) copied from Codex's isaac-live-alignment run.
HERE=/workspace/molar/ros_ws/bumperbot_ws; BASE=/workspace/molar/robot_lab_runtime/pro630-gripper-20261120
RUN=$BASE/claude-isaac-dock5mm-$(date +%H%M); mkdir -p "$RUN/runtime"
cp -r "$BASE/isaac-live-alignment-moEzPn/runtime/external_assets" "$RUN/runtime/"
Y="$RUN/runtime/external_assets/installed/robots.yaml"; L=$(grep -n "        min_object_width: 0.035" "$Y" | head -1 | cut -d: -f1); sed -i "${L}a\\        ik_rotation_step: exact" "$Y"; sed -i "s/dock_position_tolerance: 0.01$/dock_position_tolerance: 0.005/" "$Y"; grep -c "dock_position_tolerance: 0.005" "$Y" > "$RUN/dock-tolerance-check.txt"
source "$HERE/scripts/ssd_env.sh" >/dev/null; source /opt/ros/humble/setup.bash; source "$HERE/install/setup.bash"
export ROBOT_LAB_RUNTIME_ROOT="$RUN/runtime" ROS_DOMAIN_ID=117 IGN_PARTITION=claude_f100 GZ_PARTITION=claude_f100
cd "$HERE"
flock -w 1800 /workspace/molar/robot_lab_runtime/physics-qualification.lock timeout --signal=INT 1500 xvfb-run -a python3 scripts/qualify_urdf_arm_gui.py \
  --robot asset_elephant_myagv_pro_pro630_f100 --simulator isaac --map mobile_fetch_lab_f100 --experimental \
  --pick-place fetch_block drop_table --output "$RUN/gui" > "$RUN/gui-run.log" 2>&1
echo "$RUN exit=$?" >> "$BASE/claude-runs.txt"
