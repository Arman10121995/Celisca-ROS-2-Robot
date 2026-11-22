#!/bin/bash
# V5 mapped-obstacle flight: three repeats of the 11-13 R5.11 protocol (scripts/qualify_drone_moving.py) on current code.
DIR=$1; RUNS=/workspace/molar/robot_lab_runtime/v5-drone-moving-20261122; HERE=/workspace/molar/ros_ws/bumperbot_ws/scripts
source "$HERE/ssd_env.sh"; source /opt/ros/humble/setup.bash; source "$HERE/../install/setup.bash"
export ROS_DOMAIN_ID=117
[ -n "$WAIT_FOR" ] && until grep -q ALLDONE "$WAIT_FOR" 2>/dev/null; do sleep 15; done
mkdir -p $RUNS
for n in 1 2 3; do
  timeout 1200 flock -w 900 "$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock" \
    python3 "$HERE/qualify_drone_moving.py" --output $RUNS/run-$n > $RUNS/run-$n.log 2>&1
  echo "run-$n exit=$?" >> "$DIR/progress.txt"; sleep 10
done
echo ALLDONE >> "$DIR/progress.txt"
