#!/bin/bash
# V7: flocking with the peer search (2026-11-28): nav_empty triangle 2.5 m, 120 s local_flocking, twice,
# with the aggregation round case runner (as flocking-1 of 11-22).
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v7-swarm-flocking-20261128
DIR=$(cd "$(dirname "$0")" && pwd); mkdir -p "$RUN"
source "$HERE/scripts/ssd_env.sh" >/dev/null; source /opt/ros/humble/setup.bash; source "$HERE/install/setup.bash"
export ROS_DOMAIN_ID=117 SWARM_SESSION="$DIR/session-open-triangle.yaml"; LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"
CASE="$HERE/docs/status/evidence/v7-swarm-aggregation-2026-11-22/run_case.py"
for n in ${RUNS:-flocking-1:local_flocking:120 flocking-2:local_flocking:120}; do
  name=${n%%:*}; phases=${n#*:}
  [ -s "$RUN/$name/agg/result.json" ] || SWARM_PHASES=$phases flock -w 1800 "$LOCK" python3 "$CASE" "$RUN/$name"
  echo "$name exit=$?" >> "$DIR/progress.txt"; sleep 5
done
echo ALLDONE >> "$DIR/progress.txt"
