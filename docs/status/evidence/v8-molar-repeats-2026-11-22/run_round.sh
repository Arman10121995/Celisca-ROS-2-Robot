#!/bin/bash
# V8: MOLAR protocols repeated on current code, one run per lock hold, with the receipts' own arguments:
#  - station transfer on nav_empty (11-17/11-18): gazebo/mujoco --time-scale 2, pybullet 4, isaac 3;
#  - CELISCA transfer (11-17): gazebo celisca_floor_1_detailed --time-scale 4;
#  - Nav2 route A, cancel/resume and goal B (11-18): sim_nav_mission.sh gazebo molar_hri celisca_floor_1_detailed.
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v8-molar-20261122
DIR=$(cd "$(dirname "$0")" && pwd); mkdir -p "$RUN"
source "$HERE/scripts/ssd_env.sh" >/dev/null; source /opt/ros/humble/setup.bash; source "$HERE/install/setup.bash"
export ROS_DOMAIN_ID=117; LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"; cd "$HERE"
transfer() { # name sim scale [map]
  local extra=(); [ -n "$4" ] && extra=(--map "$4")
  [ -s "$RUN/$1.json" ] || flock -w 1800 "$LOCK" timeout --signal=INT 1500 python3 scripts/molar_station_transfer_check.py \
    --simulator "$2" --time-scale "$3" "${extra[@]}" --out "$RUN/$1.json" > "$RUN/$1.log" 2>&1
  echo "$1 exit=$?" >> "$DIR/progress.txt"; }
for i in 1 2; do
  transfer gazebo-$i gazebo 2; transfer mujoco-$i mujoco 2; transfer pybullet-$i pybullet 4; transfer isaac-$i isaac 3
done
for i in 1 2; do transfer celisca-gazebo-$i gazebo 4 celisca_floor_1_detailed; done
for i in ${NAV_RUNS:-1 2}; do
  [ -s "$RUN/nav-celisca-$i.json" ] || CHECK_ARGS="--base base_link --robot-radius 0.49" flock -w 1800 "$LOCK" timeout --signal=INT 2400 \
    scripts/sim_nav_mission.sh gazebo molar_hri celisca_floor_1_detailed "$RUN/nav-celisca-$i.json" > "$RUN/nav-celisca-$i.log" 2>&1
  echo "nav-celisca-$i exit=$?" >> "$DIR/progress.txt"
done
echo ALLDONE >> "$DIR/progress.txt"
