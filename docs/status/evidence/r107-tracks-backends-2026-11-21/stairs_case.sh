#!/bin/bash
# stairs_case.sh <simulator> <robot> <out_dir> <auto|fixed> [duration]: the 11-14/11-19 step course on any backend.
set -u
HERE=$(cd "$(dirname "$0")" && pwd); STAIRS=$HERE/../flipper-stairs-2026-11-14
mkdir -p "$3"
PID=$(python3 $STAIRS/launch_tracked.py "$3/launch.log" simulator:=$1 robot_model:=$2 map_name:=terrain_stairs \
  spawn_x:=-6.65 spawn_y:=-5.9 spawn_yaw:=1.5708)
CASE_OUTPUT=$3 CASE_FLIPPERS=$4 CASE_DURATION=${5:-50} timeout 400 python3 $STAIRS/stairs_probe.py > "$3/probe.log" 2>&1
kill -INT -- -$PID 2>/dev/null; for i in $(seq 30); do kill -0 -- -$PID 2>/dev/null || break; sleep 1; done; kill -KILL -- -$PID 2>/dev/null
cat "$3/result.json" 2>/dev/null || tail -5 "$3/probe.log"
