#!/bin/bash
DIR=$1; W=$2; HERE=/workspace/molar/ros_ws/bumperbot_ws/scripts
source "$HERE/ssd_env.sh"; source /opt/ros/humble/setup.bash; source "$HERE/../install/setup.bash"
export ROS_DOMAIN_ID=117 DISPLAY=${DISPLAY:-:0}
timeout 900 flock -w 600 "$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock" python3 "$HERE/qualify_px4_drive_gui.py" --map $W --output "$DIR/$W" > "$DIR/$W.log" 2>&1
echo "$W exit=$?" >> "$DIR/progress.txt"
