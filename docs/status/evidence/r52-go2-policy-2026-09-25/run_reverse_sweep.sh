#!/usr/bin/env bash
# Run a matched sequential reverse-command sweep for the opt-in Go2 policy.
# The script owns only the processes it starts and records raw probe/launch logs.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
PROBE="$ROOT/docs/status/evidence/r52-go2-2026-09-25/probe_stance.py"

usage() {
    cat <<'EOF'
Usage: run_reverse_sweep.sh [options]

Options:
  --out-dir PATH       Artifact directory (default: evidence/reverse_sweep_<UTC>)
  --domain-base N      First ROS_DOMAIN_ID (default: 181)
  --duration S         Probe duration (default: 7)
  --drive-start S      Command start time (default: 1)
  --drive-end S        Command end time (default: 4)
  --commands LIST      Comma-separated negative commands (default: -0.15,-0.25,-0.35,-0.45,-0.55,-0.65)
  --map MAP            Reverse command map: feedforward or inverse (default: feedforward)
  --help               Show this help
EOF
}

out_dir=""
domain_base=181
duration=7
drive_start=1
drive_end=4
commands="-0.15,-0.25,-0.35,-0.45,-0.55,-0.65"
reverse_map="feedforward"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --out-dir) out_dir="$2"; shift 2 ;;
        --domain-base) domain_base="$2"; shift 2 ;;
        --duration) duration="$2"; shift 2 ;;
        --drive-start) drive_start="$2"; shift 2 ;;
        --drive-end) drive_end="$2"; shift 2 ;;
        --commands) commands="$2"; shift 2 ;;
        --commands=*) commands="${1#*=}"; shift ;;
        --map) reverse_map="$2"; shift 2 ;;
        --map=*) reverse_map="${1#*=}"; shift ;;
        --help|-h) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -z "$out_dir" ]]; then
    out_dir="$SCRIPT_DIR/reverse_sweep_$(date -u +%Y%m%dT%H%M%SZ)"
fi
if [[ "$reverse_map" != "feedforward" && "$reverse_map" != "inverse" ]]; then
    echo "--map must be 'feedforward' or 'inverse'" >&2
    exit 2
fi
mkdir -p "$out_dir"
out_dir="$(cd "$out_dir" && pwd)"

if [[ ! -f "$PROBE" ]]; then
    echo "Missing probe: $PROBE" >&2
    exit 2
fi

# ROS setup files read optional unset variables; keep strict mode for the harness.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ROOT/install/setup.bash"
fi
set -u

python3 - "$out_dir/manifest.json" "$ROOT" "$domain_base" "$duration" "$drive_start" "$drive_end" "$commands" "$reverse_map" <<'PY'
import json
import sys
from pathlib import Path
out, root, domain_base, duration, drive_start, drive_end, commands, reverse_map = sys.argv[1:]
payload = {
    "tool": "run_reverse_sweep.sh",
    "source_root": root,
    "domain_base": int(domain_base),
    "duration_s": float(duration),
    "drive_start_s": float(drive_start),
    "drive_end_s": float(drive_end),
    "commands": [float(value) for value in commands.split(",")],
    "reverse_command_map": reverse_map,
    "trials": [],
}
Path(out).write_text(json.dumps(payload, indent=2) + "\n")
PY
IFS=',' read -r -a command_array <<< "$commands"

stop_owned_process() {
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

launch_pid=""
probe_pid=""
cleanup() {
    stop_owned_process "$launch_pid"
    stop_owned_process "$probe_pid"
}
trap cleanup EXIT INT TERM

index=0
for command in "${command_array[@]}"; do
    index=$((index + 1))
    domain=$((domain_base + index - 1))
    label="reverse_${command//-/_}"
    label="${label//./p}"
    result="$out_dir/${index}_${label}.json"
    probe_log="$out_dir/${index}_${label}.probe.log"
    launch_log="$out_dir/${index}_${label}.launch.log"
    meta="$out_dir/${index}_${label}.meta.json"

    echo "[$index/${#command_array[@]}] command=$command domain=$domain"
    ROS_DOMAIN_ID="$domain" python3 "$PROBE" \
        --duration "$duration" --wall-timeout 60 \
        --drive-vx "$command" --drive-start "$drive_start" --drive-end "$drive_end" \
        > "$result" 2> "$probe_log" &
    probe_pid=$!
    sleep 1

    ROS_DOMAIN_ID="$domain" setsid ros2 launch robot_lab_bringup simulated_robot.launch.py \
        mode:=loc simulator:=mujoco robot_model:=unitree_go2 \
        map_name:=nav_empty gui:=false start_rviz:=false \
        go2_policy_path:=auto go2_reverse_command_map:="$reverse_map" \
        > "$launch_log" 2>&1 &
    launch_pid=$!

    set +e
    wait "$probe_pid"
    probe_rc=$?
    probe_pid=""
    stop_owned_process "$launch_pid"
    launch_rc=$?
    launch_pid=""
    set -e

    python3 - "$meta" "$command" "$reverse_map" "$domain" "$result" "$probe_log" "$launch_log" "$probe_rc" "$launch_rc" <<'PY'
import json
import sys
from pathlib import Path
meta, command, reverse_map, domain, result, probe_log, launch_log, probe_rc, launch_rc = sys.argv[1:]
payload = {
    "command_vx_mps": float(command),
    "reverse_command_map": reverse_map,
    "ros_domain_id": int(domain),
    "probe_result": str(Path(result).resolve()),
    "probe_log": str(Path(probe_log).resolve()),
    "launch_log": str(Path(launch_log).resolve()),
    "probe_returncode": int(probe_rc),
    "launch_returncode": int(launch_rc),
}
Path(meta).write_text(json.dumps(payload, indent=2) + "\n")
PY

    if [[ ! -s "$result" ]]; then
        echo "Probe produced no JSON for $command; see $probe_log" >&2
        exit 1
    fi
    python3 - "$result" <<'PY'
import json
import sys
payload = json.load(open(sys.argv[1]))
print("  sim_duration_s=%.3f drive_delta_x_m=%.3f max_tilt_rad=%.3f contact_messages=%s" % (
    payload.get("sim_duration_s", 0.0),
    payload.get("motion", {}).get("drive_delta_x_m", float("nan")),
    payload.get("max_tilt_rad", float("nan")),
    payload.get("contact_messages", 0)))
PY
done

python3 "$SCRIPT_DIR/analyze_reverse_sweep.py" \
    --manifest "$out_dir/manifest.json" \
    --out "$out_dir/summary.json"
printf 'Sweep artifacts: %s\n' "$out_dir"
