#!/bin/bash
# Launch one display-mode run, check it with sim_display_check.py, stop it.
#
# Usage: scripts/sim_display_check.sh <simulator> <robot|none> <map|none>
#          <output.json> [extra launch args...]
#
# The JSON gains a "launch" object with spawn markers and error counts from
# the launch log.  Own ROS domain and Gazebo partition, and job control so the
# backgrounded launch receives SIGINT.
set -m
SIM=$1; ROBOT=$2; MAP=$3; OUT=$4; shift 4
HERE=$(cd "$(dirname "$0")" && pwd)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-94}
export IGN_PARTITION=display_check_$$ GZ_PARTITION=display_check_$$
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"

ros2 launch robot_lab_bringup simulated_robot.launch.py mode:=display \
  simulator:="$SIM" robot_model:="$ROBOT" map_name:="$MAP" gui:=false \
  start_rviz:=false "$@" > "$OUT.launch.log" 2>&1 &
LAUNCH=$!
if [ "$ROBOT" = "none" ]; then
  # Nothing to observe on the graph; give the world time to load.
  for _ in $(seq 1 ${WORLD_WAIT:-90}); do sleep 1; kill -0 $LAUNCH 2>/dev/null || break; done
  echo '{}' > "$OUT"
else
  python3 "$HERE/sim_display_check.py" --timeout "${TIMEOUT:-300}" > "$OUT" 2> "$OUT.err"
fi
kill -INT "$LAUNCH" 2>/dev/null
for _ in $(seq 1 30); do kill -0 "$LAUNCH" 2>/dev/null || break; sleep 1; done
kill -0 "$LAUNCH" 2>/dev/null && { kill -KILL -- -"$LAUNCH" 2>/dev/null; echo "launch killed" >&2; }

python3 - "$OUT" "$OUT.launch.log" "$SIM" <<'EOF'
import json, re, sys
out, log, sim = sys.argv[1:4]
text = open(log, errors="replace").read()
try:
    result = json.load(open(out))
except Exception:
    result = {"error": "checker produced no JSON"}
markers = {
    "pybullet": ["Loaded robot id=", "SDF world"],
    "mujoco": ["MuJoCo model loaded"],
    "gazebo": ["OK creation of entity", "Entity creation successful"],
    "isaac": ["Isaac runtime ready", "World shapes created"],
}[sim]
result["launch"] = {
    "markers": {m: (m in text) for m in markers},
    "fallback_model": "fallback" in text.lower() and "diff-drive" in text.lower(),
    "errors": len(re.findall(r"\[ERROR\]|Traceback|process has died", text)),
    "first_error": next((l.strip()[:300] for l in text.splitlines()
                         if re.search(r"\[ERROR\]|Traceback|Error:", l)), None),
    "world_lines": [l.strip()[:200] for l in text.splitlines()
                    if re.search(r"world|World|map shapes|MJCF|static shape", l)][:4],
}
json.dump(result, open(out, "w"))
EOF
