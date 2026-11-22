#!/bin/bash
# V6: the measured manipulation protocols repeated on current code, one lock hold per run, with the receipts' own commands:
#  - URDF Panda cube pick/place through the normal GUI (r57-urdf-arm-pick-place, qualify_urdf_arm_gui.py --pick-place);
#  - URDF Panda upright can by script (r57-cylinder-pick-place, arm_pick_place_check.py) on MuJoCo, PyBullet, Gazebo;
#  - TIAGo fetch with the head RGB-D estimate (r59-tiago-rgbd-fetch) on MuJoCo and PyBullet;
#  - Shadow hand GUI screen (r58-shadow-hand, qualify_native_hand_gui.py).
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v6-manipulation-20261122
DIR=$(cd "$(dirname "$0")" && pwd); mkdir -p "$RUN"
source "$HERE/scripts/ssd_env.sh" >/dev/null; source /opt/ros/humble/setup.bash; source "$HERE/install/setup.bash"
export ROS_DOMAIN_ID=117; LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"; cd "$HERE"
log() { echo "$1 exit=$2" >> "$DIR/progress.txt"; sleep 3; }
for i in 1 2; do for sim in mujoco pybullet; do n=panda-cube-gui-$sim-$i
  [ -s "$RUN/$n/report.json" ] || flock -w 1800 "$LOCK" timeout --signal=INT 1200 xvfb-run -a python3 scripts/qualify_urdf_arm_gui.py \
    --robot asset_franka_panda_panda --simulator $sim --map arm_pick_place_lab --pick-place pickup_cube place_table \
    --output "$RUN/$n" > "$RUN/$n.log" 2>&1; log $n $?
done; done
for sim in mujoco pybullet gazebo; do n=panda-can-$sim
  [ -s "$RUN/$n.json" ] || IGN_PARTITION=v6_$n GZ_PARTITION=v6_$n flock -w 1800 "$LOCK" python3 scripts/run_with_launch.py --timeout 900 --log "$RUN/$n.launch.log" \
    --launch "robot_model:=asset_franka_panda_panda simulator:=$sim mode:=display map_name:=arm_pick_place_can gui:=false perception:=none" \
    -- python3 scripts/arm_pick_place_check.py --robot asset_franka_panda_panda --object pickup_can --support place_table > "$RUN/$n.json" 2> "$RUN/$n.err"
  log $n $?
done
for sim in mujoco pybullet; do n=tiago-rgbd-fetch-$sim
  [ -s "$RUN/$n.json" ] || flock -w 1800 "$LOCK" python3 scripts/run_with_launch.py --timeout 1200 --log "$RUN/$n.launch.log" \
    --launch "robot_model:=asset_pal_tiago simulator:=$sim mode:=display map_name:=mobile_fetch_lab allow_experimental_assets:=true spawn_x:=-1.0 spawn_y:=-1.0" \
    -- python3 scripts/arm_pick_place_check.py --robot asset_pal_tiago --object fetch_block --support drop_table --pose-source rgbd > "$RUN/$n.json" 2> "$RUN/$n.err"
  log $n $?
done
for i in 1 2; do n=shadow-hand-gui-$i
  [ -s "$RUN/$n/report.json" ] || flock -w 1800 "$LOCK" timeout --signal=INT 900 xvfb-run -a -s "-screen 0 1600x1000x24" python3 scripts/qualify_native_hand_gui.py \
    --robot menagerie_shadow_hand --output "$RUN/$n" > "$RUN/$n.log" 2>&1; log $n $?
done
echo ALLDONE >> "$DIR/progress.txt"
