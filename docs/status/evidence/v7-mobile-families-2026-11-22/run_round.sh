#!/bin/bash
# V7: repeats of the 11-20/11-21 multi-leg saved-map and tracked Nav2/command-loss protocols on current code.
DIR=$1; RUN=/workspace/molar/robot_lab_runtime/v7-mobile-families-20261122; HERE=/workspace/molar/ros_ws/bumperbot_ws
source "$HERE/scripts/ssd_env.sh"; source /opt/ros/humble/setup.bash; source "$HERE/install/setup.bash"
export ROS_DOMAIN_ID=117 LAB_EXTENSION_RUN=$RUN; mkdir -p $RUN
[ -n "$WAIT_FOR" ] && until grep -q ALLDONE "$WAIT_FOR" 2>/dev/null; do sleep 15; done
LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"; T=$HERE/docs/status/evidence/r107-tracks-backends-2026-11-21
step() { name=$1; shift; flock -w 1800 "$LOCK" timeout --signal=INT --kill-after=60s 2400 "$@" > "$RUN/$name.log" 2>&1; echo "$name exit=$?" >> "$DIR/progress.txt"; sleep 5; }
step gait-mujoco-nav_obstacle python3 $HERE/scripts/qualify_gait_saved_map.py --output $RUN/gait-mujoco-nav_obstacle --map nav_obstacle --spawn=-3.0,2.5,0.0
step gait-pybullet-nav_empty python3 $HERE/scripts/qualify_gait_saved_map.py --output $RUN/gait-pybullet-nav_empty --simulator pybullet --timeout 600
for n in 1 2; do for sim in mujoco pybullet; do
  step tracked-nav-$sim-$n python3 $T/tracked_nav_probe.py $sim nav_obstacle -3 -4
  step tracked-loss-$sim-$n python3 $T/tracked_loss_probe.py $sim
done; done
echo ALLDONE >> "$DIR/progress.txt"
