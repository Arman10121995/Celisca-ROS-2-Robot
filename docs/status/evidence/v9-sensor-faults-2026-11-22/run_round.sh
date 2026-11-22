#!/bin/bash
# V9: recovery under seeded sensor faults - the Bumperbot nav_obstacle mission (sim_nav_mission.sh) on MuJoCo,
# clean versus two scan-fault scenarios, goal seeds 1-3, one lock hold per run.
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v9-sensor-faults-20261122
DIR=$(cd "$(dirname "$0")" && pwd); mkdir -p "$RUN"; export ROS_DOMAIN_ID=117
LOCK=/workspace/molar/robot_lab_runtime/physics-qualification.lock
for seed in 1 2 3; do for cond in ${CONDITIONS:-clean noise-dropout occlusion-window}; do
  out="$RUN/${cond}_seed${seed}.json"; extra=(); [ $cond != clean ] && extra=(sensor_faults:="$DIR/$cond.yaml")
  [ -s "$out" ] || CHECK_ARGS="--seed $seed" flock -w 1800 "$LOCK" timeout --signal=INT 1500 \
    "$HERE/scripts/sim_nav_mission.sh" mujoco bumperbot nav_obstacle "$out" "${extra[@]}" > "$out.run.log" 2>&1
  echo "$cond seed$seed exit=$?" >> "$DIR/progress.txt"; sleep 3
done; done
echo "ALLDONE ${CONDITIONS:-}" >> "$DIR/progress.txt"
