#!/bin/bash
# V1: the November 10 vacuum jobs (their recorded start requests) on MuJoCo, PyBullet and Isaac, one at a time.
DIR=$1; HERE=$(cd "$(dirname "$0")/../../../../scripts" && pwd)
source "$HERE/ssd_env.sh" >/dev/null
SRC=$(cd "$(dirname "$0")/../vacuum-2026-11/runs" && pwd)
for sim in ${SIMS:-mujoco pybullet isaac}; do
  for job in rooms_office spot zone_keepout; do
    out="$DIR/${sim}_${job}.json"; [ -s "$out" ] && continue
    ref=$SRC/gazebo_${job}.json
    request=$(python3 -c "import json,sys;print(json.dumps(json.load(open(sys.argv[1]))['request']))" "$ref")
    MODE=nav flock -w 1800 "$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock" timeout 4000 "$HERE/sim_vacuum_mission.sh" $sim oomwoo small_house "$out" "$request"
    echo "$sim $job exit=$?" >> "$DIR/progress.txt"; sleep 3
  done
done
echo ALLDONE >> "$DIR/progress.txt"
