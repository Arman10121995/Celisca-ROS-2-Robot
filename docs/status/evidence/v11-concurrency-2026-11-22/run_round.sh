#!/bin/bash
# V11: the October 15 R8.3 concurrency protocol repeated, plus three simulators at once.
# Each coordinator run holds the shared physics lock for its own duration only.
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v11-concurrency-20261122
DIR=$(cd "$(dirname "$0")" && pwd); mkdir -p "$RUN"
source "$HERE/scripts/ssd_env.sh" >/dev/null; source /opt/ros/humble/setup.bash; source "$HERE/install/setup.bash"
LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"
run() { # name spec parallel
  flock -w 1800 "$LOCK" timeout --signal=INT 1200 ros2 run robot_lab_benchmark robot-lab-concurrent "$DIR/$2" \
    --output "$RUN/$1" --parallel "$3" > "$RUN/$1.stdout" 2> "$RUN/$1.stderr"
  local rc=$?
  { echo "== $1"; ps -eo pid,args | grep -E "ign gazebo|gz sim|robot_lab_mujoco|robot_lab_pybullet|nav2|component_container" | grep -v -E "grep|2170859|2177240"; } >> "$RUN/survivors.txt"
  echo "$1 exit=$rc" >> "$DIR/progress.txt"; }
if [ -z "$1" ]; then
run A_serial pair.json 1
run B1_simultaneous pair.json 2
run B2_simultaneous pair.json 2
run C_overload overload.json 2
run E1_triple triple.json 3
run E2_triple triple.json 3
echo ALLDONE >> "$DIR/progress.txt"
fi
# Diagnosis after E1/E2 (added 11-22): the Gazebo job alone under the coordinator.
[ "$1" = gazebo_alone ] && { run F_gazebo_alone gazebo_alone.json 1; echo ALLDONE-F >> "$DIR/progress.txt"; }
# After the joystick-mux fix (added 11-22): Gazebo alone again, then two more triples.
[ "$1" = after_fix ] && { run F2_gazebo_alone_fixed gazebo_alone.json 1; run E3_triple_fixed triple.json 3; run E4_triple_fixed triple.json 3; echo ALLDONE-FIX >> "$DIR/progress.txt"; }
