#!/bin/bash
# V11: Isaac in the concurrent coordinator - Isaac alone, then Isaac + MuJoCo at once (October 15 Nav2 workload).
# Each coordinator run holds the shared physics lock for its own duration only.
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v11-isaac-concurrency-20261128
DIR=$(cd "$(dirname "$0")" && pwd); mkdir -p "$RUN"
source "$HERE/scripts/ssd_env.sh" >/dev/null; source /opt/ros/humble/setup.bash; source "$HERE/install/setup.bash"
LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"
run() { # name spec parallel
  flock -w 1800 "$LOCK" timeout --signal=INT 1200 ros2 run robot_lab_benchmark robot-lab-concurrent "$DIR/$2" \
    --output "$RUN/$1" --parallel "$3" > "$RUN/$1.stdout" 2> "$RUN/$1.stderr"
  local rc=$?
  { echo "== $1"; ps -eo pid,args | grep -E "ign gazebo|gz sim|robot_lab_mujoco|robot_lab_pybullet|isaac|kit/python|nav2|component_container" | grep -v -E "grep|2170859|2177240"; } >> "$RUN/survivors.txt"
  echo "$1 exit=$rc" >> "$DIR/progress.txt"; }
if [ -z "$1" ]; then
run I0_isaac_alone isaac_alone.json 1
run I1_isaac_mujoco isaac_mujoco.json 2
run I2_isaac_mujoco isaac_mujoco.json 2
echo ALLDONE >> "$DIR/progress.txt"
fi
