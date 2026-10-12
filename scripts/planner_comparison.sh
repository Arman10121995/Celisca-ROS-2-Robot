#!/bin/bash
# Seeded planner comparison: every planner runs the same seeded goal
# sequence (sim_nav_goals.py), one launch per planner x seed, with a rosbag
# of the trial.  All other launch settings are identical.
#
#   scripts/planner_comparison.sh <dir> <sim> <robot> <map> <category> "<planners>" "<seeds>" [launch args...]
#   category: global_planning | local_planning
# Then: python3 scripts/planner_comparison_report.py <dir>
set -m
DIR=$1; SIM=$2; ROBOT=$3; MAP=$4; CATEGORY=$5; PLANNERS=$6; SEEDS=$7; shift 7
HERE=$(cd "$(dirname "$0")" && pwd)
source "$HERE/ssd_env.sh" || exit 2
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
mkdir -p "$DIR"
cat > "$DIR/manifest.json" <<JSON
{"simulator": "$SIM", "robot": "$ROBOT", "map": "$MAP", "category": "$CATEGORY",
 "planners": "$PLANNERS", "seeds": "$SEEDS", "launch_args": "$*",
 "goals_per_seed": "${GOALS:-3}", "revision": "$(git -C "$HERE/.." rev-parse HEAD)",
 "started": "$(date -Iseconds)"}
JSON
for planner in $PLANNERS; do
  for seed in $SEEDS; do
    run="$DIR/${planner}_seed${seed}"
    [ -s "$run.json" ] && continue
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-96}
    export IGN_PARTITION=planner_cmp_$$ GZ_PARTITION=planner_cmp_$$
    ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=nav \
      simulator:="$SIM" robot_model:="$ROBOT" map_name:="$MAP" gui:=false \
      start_rviz:=false "$CATEGORY:=$planner" "$@" > "$run.launch.log" 2>&1 &
    LAUNCH=$!
    rm -rf "$run.bag"
    ros2 bag record -o "$run.bag" /odom/ground_truth /cmd_vel /plan /goal_pose \
      /navigate_to_pose/_action/status /amcl_pose > "$run.bag.log" 2>&1 &
    BAG=$!
    timeout "${TRIAL_TIMEOUT:-2400}" python3 "$HERE/sim_nav_goals.py" --seed "$seed" \
      --goals "${GOALS:-3}" --robot-radius "${ROBOT_RADIUS:-0.2}" \
      --trace-out "$run.trace.json" > "$run.json" 2> "$run.err"
    kill -INT "$BAG" 2>/dev/null; wait "$BAG" 2>/dev/null
    kill -INT "$LAUNCH" 2>/dev/null
    for _ in $(seq 1 40); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
    kill -TERM -- -"$LAUNCH" 2>/dev/null; sleep 1; kill -KILL -- -"$LAUNCH" 2>/dev/null
    [ -s "$run.json" ] || echo '{"error": "trial produced no output"}' > "$run.json"
    echo "$planner $seed done" >> "$DIR/progress.txt"
    sleep 3
  done
done
echo ALLDONE >> "$DIR/progress.txt"
