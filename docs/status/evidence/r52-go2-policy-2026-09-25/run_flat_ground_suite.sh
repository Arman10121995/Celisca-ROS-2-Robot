#!/usr/bin/env bash
# Run a matched sequential Go2 flat-ground suite through the opt-in policy.
# The script owns only the processes it starts and retains raw probe/launch logs.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
PROBE="$ROOT/docs/status/evidence/r52-go2-2026-09-25/probe_stance.py"

usage() {
    cat <<'EOF'
Usage: run_flat_ground_suite.sh [options]

Options:
  --out-dir PATH       Artifact directory (default: flat_ground_suite_<UTC>)
  --domain-base N      First ROS_DOMAIN_ID (default: 211)
  --duration S         Probe duration (default: 7)
  --drive-start S      Command start time (default: 1)
  --drive-end S        Command end time (default: 4)
  --map MAP            Reverse command map: feedforward or inverse (default: inverse)
  --help               Show this help
EOF
}

out_dir=""
domain_base=211
duration=7
drive_start=1
drive_end=4
reverse_map="inverse"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --out-dir) out_dir="$2"; shift 2 ;;
        --domain-base) domain_base="$2"; shift 2 ;;
        --duration) duration="$2"; shift 2 ;;
        --drive-start) drive_start="$2"; shift 2 ;;
        --drive-end) drive_end="$2"; shift 2 ;;
        --map) reverse_map="$2"; shift 2 ;;
        --map=*) reverse_map="${1#*=}"; shift ;;
        --help|-h) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done
if [[ -z "$out_dir" ]]; then
    out_dir="$SCRIPT_DIR/flat_ground_suite_$(date -u +%Y%m%dT%H%M%SZ)"
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

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ROOT/install/setup.bash"
fi
set -u

python3 - "$out_dir/manifest.json" "$ROOT" "$domain_base" "$duration" "$drive_start" "$drive_end" "$reverse_map" <<'PY'
import json
import sys
from pathlib import Path
out, root, domain_base, duration, drive_start, drive_end, reverse_map = sys.argv[1:]
cases = [
    {"case": "forward_stop", "vx": 0.25, "wz": 0.0, "drop": False},
    {"case": "reverse_stop", "vx": -0.35, "wz": 0.0, "drop": False},
    {"case": "turn_stop", "vx": 0.0, "wz": 0.5, "drop": False},
    {"case": "forward_command_loss", "vx": 0.25, "wz": 0.0, "drop": True},
    {"case": "zero_settle", "vx": 0.0, "wz": 0.0, "drop": False},
]
Path(out).write_text(json.dumps({
    "tool": "run_flat_ground_suite.sh",
    "source_root": root,
    "domain_base": int(domain_base),
    "duration_s": float(duration),
    "drive_start_s": float(drive_start),
    "drive_end_s": float(drive_end),
    "reverse_command_map": reverse_map,
    "cases": cases,
    "trials": [],
}, indent=2) + "\n")
PY

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

mapfile -t cases < <(python3 - "$out_dir/manifest.json" <<'PY'
import json
import sys
for case in json.load(open(sys.argv[1]))["cases"]:
    print(json.dumps(case, separators=(",", ":")))
PY
)
index=0
for case_json in "${cases[@]}"; do
    index=$((index + 1))
    read -r case vx wz drop < <(python3 - "$case_json" <<'PY'
import json
import sys
case = json.loads(sys.argv[1])
print(case["case"], case["vx"], case["wz"], str(case["drop"]).lower())
PY
)
    domain=$((domain_base + index - 1))
    label="${case//_/-}"
    result="$out_dir/${index}_${label}.json"
    probe_log="$out_dir/${index}_${label}.probe.log"
    launch_log="$out_dir/${index}_${label}.launch.log"
    meta="$out_dir/${index}_${label}.meta.json"
    echo "[$index/${#cases[@]}] case=$case command=($vx,$wz) drop=$drop domain=$domain"
    probe_args=(--duration "$duration" --wall-timeout 60
        --drive-vx "$vx" --drive-wz "$wz"
        --drive-start "$drive_start" --drive-end "$drive_end")
    if [[ "$drop" == "true" ]]; then
        probe_args+=(--drop-command-after-drive)
    fi
    ROS_DOMAIN_ID="$domain" python3 "$PROBE" "${probe_args[@]}" \
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
    python3 - "$meta" "$case" "$vx" "$wz" "$drop" "$reverse_map" "$domain" "$result" "$probe_log" "$launch_log" "$probe_rc" "$launch_rc" <<'PY'
import json
import sys
from pathlib import Path
meta, case, vx, wz, drop, reverse_map, domain, result, probe_log, launch_log, probe_rc, launch_rc = sys.argv[1:]
Path(meta).write_text(json.dumps({
    "case": case,
    "command_vx_mps": float(vx),
    "command_wz_radps": float(wz),
    "drop_command_after_drive": drop == "true",
    "reverse_command_map": reverse_map,
    "ros_domain_id": int(domain),
    "probe_result": str(Path(result).resolve()),
    "probe_log": str(Path(probe_log).resolve()),
    "launch_log": str(Path(launch_log).resolve()),
    "probe_returncode": int(probe_rc),
    "launch_returncode": int(launch_rc),
}, indent=2) + "\n")
PY
    [[ -s "$result" ]] || { echo "Probe produced no JSON for $case" >&2; exit 1; }
done

python3 "$SCRIPT_DIR/analyze_flat_ground_suite.py" \
    --manifest "$out_dir/manifest.json" --out "$out_dir/summary.json"
printf 'Suite artifacts: %s\n' "$out_dir"

