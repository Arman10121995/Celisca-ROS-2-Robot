#!/usr/bin/env bash
# Run one bounded Go2 perturbation/recovery trial through the opt-in policy.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
PROBE="$ROOT/docs/status/evidence/r52-go2-2026-09-25/probe_stance.py"
OUT_DIR="${1:-$SCRIPT_DIR/perturbation_trial_$(date -u +%Y%m%dT%H%M%SZ)}"
DOMAIN="${ROS_DOMAIN_ID:-231}"
DURATION="${DURATION_S:-7}"
FORCE_N="${FORCE_N:-5.0}"
START_S="${START_S:-3.0}"
PULSE_S="${PULSE_S:-0.2}"
RECOVERY_S="${RECOVERY_S:-2.0}"
# Opt-in re-stand attempt; off by default because it is not qualified.
FALL_RECOVERY="${FALL_RECOVERY:-false}"
FALL_RECOVERY_TIMEOUT_S="${FALL_RECOVERY_TIMEOUT_S:-4.0}"

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

result="$OUT_DIR/probe.json"
probe_log="$OUT_DIR/probe.log"
launch_log="$OUT_DIR/launch.log"
/usr/bin/python3 - "$OUT_DIR/manifest.json" "$ROOT" "$DOMAIN" "$DURATION" "$FORCE_N" "$START_S" "$PULSE_S" "$RECOVERY_S" <<'PY'
import json
import sys
from pathlib import Path
out, root, domain, duration, force, start, pulse, recovery = sys.argv[1:]
Path(out).write_text(json.dumps({
    "tool": "run_perturbation_trial.sh",
    "source_root": root,
    "ros_domain_id": int(domain),
    "duration_s": float(duration),
    "force_n": float(force),
    "start_s": float(start),
    "pulse_s": float(pulse),
    "recovery_window_s": float(recovery),
}, indent=2) + "\n")
PY

ROS_DOMAIN_ID="$DOMAIN" /usr/bin/python3 "$PROBE" \
    --duration "$DURATION" --wall-timeout 60 \
    --perturbation-start "$START_S" --perturbation-duration "$PULSE_S" \
    --perturbation-force-n "$FORCE_N" --recovery-window "$RECOVERY_S" \
    > "$result" 2> "$probe_log" &
probe_pid=$!
sleep 1
ROS_DOMAIN_ID="$DOMAIN" setsid ros2 launch robot_lab_bringup simulated_robot.launch.py \
    mode:=loc simulator:=mujoco robot_model:=unitree_go2 \
    map_name:=nav_empty gui:=false start_rviz:=false \
    go2_policy_path:=auto go2_reverse_command_map:=inverse \
    go2_perturbation_force_n:="$FORCE_N" \
    go2_perturbation_start_s:="$START_S" \
    go2_perturbation_duration_s:="$PULSE_S" \
    go2_perturbation_axis:=1 \
    enable_fall_recovery:="$FALL_RECOVERY" \
    fall_recovery_timeout_s:="$FALL_RECOVERY_TIMEOUT_S" \
    > "$launch_log" 2>&1 &
launch_pid=$!
set +e
wait "$probe_pid"
probe_rc=$?
probe_pid=""
stop_owned "$launch_pid"
launch_rc=$?
launch_pid=""
set -e
printf '%s\n' "{"probe_returncode": $probe_rc, "launch_returncode": $launch_rc}" > "$OUT_DIR/returncodes.json"
[[ -s "$result" ]] || { echo "Perturbation probe produced no JSON" >&2; exit 1; }
python3 - "$result" <<'PY'
import json
import sys
result = json.load(open(sys.argv[1]))
p = result.get("perturbation", {})
print(json.dumps({
    "sim_duration_s": result.get("sim_duration_s"),
    "max_tilt_rad": result.get("max_tilt_rad"),
    "final_height_m": result.get("final_height_m"),
    "safety_messages": result.get("safety_messages"),
    "final_safety_state": result.get("final_safety_state"),
    "safe_stop_seen": p.get("safe_stop_seen"),
    "recovery_screening_pass": p.get("recovery_screening_pass"),
    "first_failure": p.get("first_failure"),
}, indent=2))
PY
printf 'Perturbation artifacts: %s\n' "$OUT_DIR"
