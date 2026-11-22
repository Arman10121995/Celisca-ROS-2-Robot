#!/bin/bash
# V10: the offline R7.2-R7.5 and R7.8 benchmarks re-run at one revision with their October defaults.
# R7.3 reuses October's trajectories.json (the same Gazebo truth traces).
HERE=/workspace/molar/ros_ws/bumperbot_ws; DIR=$(cd "$(dirname "$0")" && pwd)
source "$HERE/scripts/ssd_env.sh" >/dev/null; cd "$HERE"
run() { local tag=$1; shift; local t0=$SECONDS; python3 "$@" > "$DIR/$tag/stdout.txt" 2> "$DIR/$tag/stderr.txt"; local rc=$?; echo "$tag exit=$rc $((SECONDS-t0)) s" >> "$DIR/progress.txt"; }
run r72 scripts/r7_2_perception_benchmark.py --out "$DIR/r72"
run r73 scripts/r7_3_localization_benchmark.py --out "$DIR/r73"
run r74 scripts/r7_4_state_estimation_benchmark.py --out "$DIR/r74"
run r75 scripts/r7_5_fusion_benchmark.py --out "$DIR/r75"
run r78 scripts/r7_8_control_benchmark.py --out "$DIR/r78"
echo ALLDONE >> "$DIR/progress.txt"
