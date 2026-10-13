#!/bin/bash
# Watch a Nav2 launch log for the stale-transform signature and, on the first
# hit, snapshot what a fresh TF listener sees plus the thread states of the
# Nav2 servers.  Read-only: it does not touch the running trial.
#
#   scripts/tf_freeze_probe.sh <launch.log> <out.txt>
LOG=$1; OUT=$2
until grep -q -E 'too old when converting|has 0 poses' "$LOG" 2>/dev/null; do sleep 2; done
{
  echo "detected $(date -Iseconds)"
  grep -m2 -E 'too old when converting|has 0 poses|Data time' "$LOG" | sed -E 's/\x1b\[[0-9;]*m//g'
  echo "--- fresh listener: map -> odom"
  timeout 6 stdbuf -oL ros2 run tf2_ros tf2_echo map odom --ros-args -p use_sim_time:=true 2>&1 \
    | grep -E 'At time|Translation|Waiting'
  echo "--- fresh listener: odom -> base_footprint"
  timeout 6 stdbuf -oL ros2 run tf2_ros tf2_echo odom base_footprint --ros-args -p use_sim_time:=true 2>&1 \
    | grep -E 'At time|Translation|Waiting'
  for node in controller_server planner_server amcl bt_navigator; do
    for pid in $(pgrep -f -- "__node:=$node( |$)"); do
      echo "--- $node pid $pid threads (tid comm state wchan)"
      for task in /proc/"$pid"/task/*; do
        echo "$(basename "$task") $(cat "$task/comm" 2>/dev/null)" \
          "$(awk '{print $3}' "$task/stat" 2>/dev/null) $(cat "$task/wchan" 2>/dev/null)"
      done
    done
  done
} > "$OUT" 2>&1
