#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
PROBE="$ROOT/docs/status/evidence/r52-go2-2026-09-25/probe_stance.py"
OUT_DIR="${1:-$SCRIPT_DIR/trial_$(date -u +%Y%m%dT%H%M%SZ)}"
DOMAIN="${ROS_DOMAIN_ID:-221}"
DURATION="${DURATION_S:-10}"
DRIVE_VX="${DRIVE_VX:-0.25}"
DRIVE_START="${DRIVE_START_S:-1}"
DRIVE_END="${DRIVE_END_S:-8}"
SPAWN_X="-7.5"
SPAWN_Y="-4.0"
SPAWN_YAW="0.0"

mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ROOT/install/setup.bash"
fi
set -u

/usr/bin/python3 - "$OUT_DIR/manifest.json" "$ROOT" "$DOMAIN" \
    "$DURATION" "$DRIVE_VX" "$DRIVE_START" "$DRIVE_END" \
    "$SPAWN_X" "$SPAWN_Y" "$SPAWN_YAW" <<'PY'
import hashlib
import json
import subprocess
import sys
from pathlib import Path

out, root, domain, duration, vx, start, end, spawn_x, spawn_y, spawn_yaw = sys.argv[1:]
root_path = Path(root)
source_paths = (
    "src/robot_lab_adapter/robot_lab_adapter/go2_locomotion.py",
    "src/robot_lab_adapter/robot_lab_adapter/go2_velocity_policy.py",
    "src/robot_lab_adapter/policies/go2_velocity_flat/policy.onnx",
    "src/robot_lab_adapter/policies/go2_velocity_flat/policy.onnx.data",
    "src/robot_lab_bringup/launch/simulated_robot.launch.py",
    "src/robot_lab_maps/mjcf/terrain_stairs.xml",
)
Path(out).write_text(json.dumps({
    "tool": "run_terrain_trial.sh",
    "source_root": root,
    "source_revision": subprocess.check_output(
        ["git", "-C", root, "rev-parse", "--short", "HEAD"], text=True).strip(),
    "tracked_modified_files_at_launch": len(subprocess.check_output(
        ["git", "-C", root, "status", "--porcelain", "--untracked-files=no"],
        text=True).splitlines()),
    "ros_domain_id": int(domain),
    "backend": "MuJoCo",
    "map_name": "terrain_stairs",
    "policy_path": "auto",
    "reverse_command_map": "inverse",
    "spawn": {"x_m": float(spawn_x), "y_m": float(spawn_y),
              "yaw_rad": float(spawn_yaw)},
    "probe": {"duration_s": float(duration), "trace_interval_s": 0.05,
              "trace_joints": True},
    "command": {"vx_mps": float(vx), "wz_radps": 0.0,
                "start_s": float(start), "end_s": float(end)},
    "source_sha256": {
        name: hashlib.sha256((root_path / name).read_bytes()).hexdigest()
        for name in source_paths
    },
}, indent=2) + "\n")
PY

probe_pid=""
launch_pid=""
stop_owned() {
    local pid="$1"
    [[ -z "$pid" ]] && return 0
    if kill -0 "$pid" 2>/dev/null; then
        kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
        for _ in $(seq 1 30); do
            kill -0 "$pid" 2>/dev/null || return 0
            sleep 0.2
        done
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
        sleep 1
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
}
cleanup() {
    stop_owned "$launch_pid"
    stop_owned "$probe_pid"
}
trap cleanup EXIT INT TERM

ROS_DOMAIN_ID="$DOMAIN" /usr/bin/python3 "$PROBE" \
    --duration "$DURATION" --wall-timeout 90 \
    --drive-vx "$DRIVE_VX" --drive-wz 0 \
    --drive-start "$DRIVE_START" --drive-end "$DRIVE_END" \
    --trace-interval 0.05 --trace-joints \
    > "$OUT_DIR/probe.json" 2> "$OUT_DIR/probe.log" &
probe_pid=$!
sleep 1
ROS_DOMAIN_ID="$DOMAIN" setsid ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=loc simulator:=mujoco robot_model:=unitree_go2 \
    map_name:=terrain_stairs spawn_x:="$SPAWN_X" spawn_y:="$SPAWN_Y" \
    spawn_yaw:="$SPAWN_YAW" gui:=false start_rviz:=false \
    go2_policy_path:=auto go2_reverse_command_map:=inverse \
    > "$OUT_DIR/launch.log" 2>&1 &
launch_pid=$!

set +e
wait "$probe_pid"
probe_rc=$?
probe_pid=""
stop_owned "$launch_pid"
launch_rc=$?
launch_pid=""
set -e
printf '{"probe_returncode":%d,"launch_returncode":%d}\n' \
    "$probe_rc" "$launch_rc" > "$OUT_DIR/returncodes.json"

[[ -s "$OUT_DIR/probe.json" ]] || {
    echo "Terrain probe produced no JSON" >&2
    exit 1
}
/usr/bin/python3 - "$OUT_DIR" "$SPAWN_X" "$SPAWN_Y" <<'PY'
import json
import math
import sys
from pathlib import Path

directory = Path(sys.argv[1])
spawn_x, spawn_y = map(float, sys.argv[2:])
result = json.loads((directory / "probe.json").read_text())
codes = json.loads((directory / "returncodes.json").read_text())
trace = result.get("trace", [])
if not trace or result.get("truth_count", 0) < 20:
    raise SystemExit("Terrain probe did not capture enough truth samples")
initial = trace[0]["xyz"]
if abs(initial[0] - spawn_x) > 0.08 or abs(initial[1] - spawn_y) > 0.08:
    raise SystemExit(
        "Terrain trial started off-course: expected (%s, %s), measured (%s, %s)"
        % (spawn_x, spawn_y, initial[0], initial[1]))
if result.get("joint_messages", 0) < 20 or result.get("effort_messages", 0) < 20:
    raise SystemExit("Terrain probe is missing joint or effort telemetry")
summary = {
    "initial_xyz": initial,
    "final_xyz": trace[-1]["xyz"],
    "sim_duration_s": result.get("sim_duration_s"),
    "truth_count": result.get("truth_count"),
    "joint_messages": result.get("joint_messages"),
    "effort_messages": result.get("effort_messages"),
    "motion": result.get("motion"),
    "max_tilt_rad": result.get("max_tilt_rad"),
    "min_height_m": result.get("min_height_m"),
    "final_height_m": result.get("final_height_m"),
    "max_command_nm": result.get("max_command_nm"),
    "probe_returncode": codes["probe_returncode"],
    "launch_returncode": codes["launch_returncode"],
}
print(json.dumps(summary, indent=2))
PY
printf 'Terrain trial artifacts: %s\n' "$OUT_DIR"