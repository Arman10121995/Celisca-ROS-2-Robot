#!/bin/bash
# course_case.sh <simulator> <map> <spawn_x> <spawn_y> <spawn_yaw> <out_dir> <duration> [stop_x]: a
# flipper-auto course (flight / up-and-over) with the strong-flipper vehicle on any backend, using the
# 11-20 up-and-over stairs_probe.py (it adds CASE_STOP_X).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$6"
PID=$(python3 $HERE/../flipper-stairs-2026-11-14/launch_tracked.py "$6/launch.log" simulator:=$1 robot_model:=lab_tracked_strong_flippers \
  map_name:=$2 spawn_x:=$3 spawn_y:=$4 spawn_yaw:=$5)
env CASE_OUTPUT=$6 CASE_FLIPPERS=auto CASE_DURATION=$7 ${8:+CASE_STOP_X=$8} timeout 900 python3 $HERE/../r107-tracked-over-2026-11-20/stairs_probe.py > "$6/probe.log" 2>&1
kill -INT -- -$PID 2>/dev/null; for i in $(seq 30); do kill -0 -- -$PID 2>/dev/null || break; sleep 1; done; kill -KILL -- -$PID 2>/dev/null
python3 - "$6" <<'PY'
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1] + '/trace.jsonl')]
top = max(rows, key=lambda r: r['z'])
states = []
for r in rows:
    s = (r.get('auto') or {}).get('state')
    if s and (not states or states[-1] != s): states.append(s)
out = dict(case=sys.argv[1].split('/')[-1], samples=len(rows), duration_s=round(rows[-1]['t'], 1), max_z=round(top['z'], 3),
           max_z_x=round(top['x'], 3), end=[round(rows[-1][k], 3) for k in ('x', 'y', 'z')], max_roll=round(max(abs(r['roll']) for r in rows), 3),
           max_pitch=round(max(abs(r['pitch']) for r in rows), 3), state_sequence=states[:40])
open(sys.argv[1] + '/course.json', 'w').write(json.dumps(out, indent=2) + '\n'); print(json.dumps(out))
PY
