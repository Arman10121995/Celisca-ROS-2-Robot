#!/bin/bash
# V5: the PX4 X500 manual-flight GUI screen (scripts/qualify_px4_drive_gui.py) on the seven worlds not re-run since 11-05.
# The Tk GUI runs under Xvfb: on 11-22 Tk's update() blocked on the desktop display :0.
# The shared lock is taken before the run budget starts; SIGINT lets a timed-out run write its report.
DIR=$1; HERE=$(cd "$(dirname "$0")/../../../../scripts" && pwd)
source "$HERE/ssd_env.sh"; source /opt/ros/humble/setup.bash; source "$HERE/../install/setup.bash"
export ROS_DOMAIN_ID=117
[ -n "$WAIT_FOR" ] && until grep -q ALLDONE "$WAIT_FOR" 2>/dev/null; do sleep 15; done
for world in aerial_indoor celisca_floor_1 nav_maze nav_warehouse outdoor_terrain small_house terrain_heightfield; do
  [ -s "$DIR/$world/report.json" ] && continue
  flock -w 1800 "$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock" \
    timeout --signal=INT --kill-after=60s 900 xvfb-run -a \
    python3 "$HERE/qualify_px4_drive_gui.py" --map $world --output "$DIR/$world" > "$DIR/$world.log" 2>&1
  echo "$world exit=$?" >> "$DIR/progress.txt"; sleep 5
done
echo ALLDONE >> "$DIR/progress.txt"
