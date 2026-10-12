#!/bin/bash
# Run sim_nav_mission.sh over simulators x robots x maps, one at a time,
# skipping cells that already have a result.  Results: <dir>/<sim>_<robot>_<map>.json
#
#   scripts/sim_nav_mission_matrix.sh <dir> "<sims>" "<robots>" "<maps>"
DIR=$1; SIMS=$2; ROBOTS=$3; MAPS=$4
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$DIR"
for sim in $SIMS; do
  for robot in $ROBOTS; do
    for map in $MAPS; do
      out="$DIR/${sim}_${robot}_${map}.json"
      [ -s "$out" ] && continue
      timeout "${CELL_TIMEOUT:-1800}" "$HERE/sim_nav_mission.sh" "$sim" "$robot" "$map" "$out"
      echo "$sim $robot $map exit=$?" >> "$DIR/progress.txt"
      # Nothing from this cell may outlive it.
      sleep 3
    done
  done
done
echo ALLDONE >> "$DIR/progress.txt"
