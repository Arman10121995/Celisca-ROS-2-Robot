#!/bin/bash
# V1 diagnostic: the MuJoCo office rooms job froze (11-21 and 11-22); run it with static flex culling off, and once more on.
DIR=$(cd "$(dirname "$0")" && pwd); HERE=/workspace/molar/ros_ws/bumperbot_ws/scripts
source "$HERE/ssd_env.sh" >/dev/null; export ROS_DOMAIN_ID=117
[ -n "$WAIT_FOR" ] && until grep -q ALLDONE "$WAIT_FOR" 2>/dev/null; do sleep 15; done
request=$(python3 -c "import json,sys;print(json.dumps(json.load(open(sys.argv[1]))['request']))" "$DIR/../../vacuum-2026-11/runs/gazebo_rooms_office.json")
for c in false true; do
  MODE=nav flock -w 1800 "$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock" timeout 4000 "$HERE/sim_vacuum_mission.sh" mujoco oomwoo small_house "$DIR/mujoco_rooms_office_culling_$c.json" "$request" world_flex_culling:=$c
  echo "culling $c exit=$?" >> "$DIR/progress.txt"; sleep 3
done
echo ALLDONE >> "$DIR/progress.txt"
