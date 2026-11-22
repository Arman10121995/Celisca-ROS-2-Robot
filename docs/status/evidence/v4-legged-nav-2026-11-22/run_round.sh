#!/bin/bash
# V4: the October 13-15 legged Nav2 mission (P7, sim_nav_mission.sh on nav_obstacle) repeated on current code.
# Goals within 0.30 m of truth, heading unchecked, measured from the trunk (pelvis for the G1); seeds 1-3.
# Robot profiles choose their own motion model (forward_arc where the backend cannot spin in place).
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v4-legged-nav-20261122
DIR=$(cd "$(dirname "$0")" && pwd); mkdir -p "$RUN"; export ROS_DOMAIN_ID=117
LOCK=/workspace/molar/robot_lab_runtime/physics-qualification.lock
cell() { # sim robot seed base
  local out="$RUN/${1}_${2}_seed${3}_nav_obstacle.json"
  [ -s "$out" ] || CHECK_ARGS="--base $4 --max-yaw-error-deg 0 --seed $3" flock -w 1800 "$LOCK" \
    timeout --signal=INT 1800 "$HERE/scripts/sim_nav_mission.sh" "$1" "$2" nav_obstacle "$out" > "$out.run.log" 2>&1
  echo "$1 $2 seed$3 exit=$?" >> "$DIR/progress.txt"; sleep 3; }
if [ -z "$1" ]; then for seed in 1 2 3; do
  cell mujoco unitree_go2 $seed base; cell gazebo unitree_go2 $seed base; cell isaac unitree_go2 $seed base
  cell mujoco unitree_a1 $seed base; cell isaac unitree_a1 $seed base
  cell mujoco unitree_g1_29dof $seed pelvis; cell isaac unitree_g1_29dof $seed pelvis
  cell mujoco unitree_b2 $seed base
done
echo ALLDONE >> "$DIR/progress.txt"; fi
# B2 at the radius October's goals imply (added 11-22): its goal B clearances 0.65-0.89 m fit --robot-radius 0.6 only.
if [ "$1" = b2_r06 ]; then
  for seed in 1 2 3; do
    out="$RUN/mujoco_unitree_b2_r06_seed${seed}_nav_obstacle.json"
    [ -s "$out" ] || CHECK_ARGS="--base base --max-yaw-error-deg 0 --seed $seed --robot-radius 0.6" flock -w 1800 "$LOCK" \
      timeout --signal=INT 1800 "$HERE/scripts/sim_nav_mission.sh" mujoco unitree_b2 nav_obstacle "$out" > "$out.run.log" 2>&1
    echo "mujoco unitree_b2 r0.6 seed$seed exit=$?" >> "$DIR/progress.txt"; sleep 3
  done; echo ALLDONE-B2 >> "$DIR/progress.txt"
fi
# G1 leg-odometry screen (added 11-22, diagnosing the G1 drift): the October slam-mode screen, nav_obstacle.
if [ "$1" = g1_legodom ]; then
  for sim in mujoco isaac; do
    out="$RUN/${sim}_unitree_g1_29dof_leg_odometry.json"
    [ -s "$out" ] || flock -w 1800 "$LOCK" timeout --signal=INT 1200 "$HERE/scripts/leg_odometry_check.sh" $sim unitree_g1_29dof nav_obstacle "$out" > "$out.run.log" 2>&1
    echo "$sim unitree_g1_29dof leg_odometry exit=$?" >> "$DIR/progress.txt"; sleep 3
  done; echo ALLDONE-LEGODOM >> "$DIR/progress.txt"
fi
