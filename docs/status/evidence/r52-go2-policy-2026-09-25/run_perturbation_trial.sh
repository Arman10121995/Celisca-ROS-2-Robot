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
#: Body-frame axis of the pulse: 0=x (forward), 1=y (lateral), 2=z (up).
PERTURBATION_AXIS="${PERTURBATION_AXIS:-1}"
START_S="${START_S:-3.0}"
PULSE_S="${PULSE_S:-0.2}"
RECOVERY_S="${RECOVERY_S:-2.0}"
# Opt-in re-stand attempt; off by default because it is not qualified.
FALL_RECOVERY="${FALL_RECOVERY:-false}"
FALL_RECOVERY_TIMEOUT_S="${FALL_RECOVERY_TIMEOUT_S:-4.0}"
FALL_RECOVERY_DELAY_S="${FALL_RECOVERY_DELAY_S:-0.0}"
RECOVERY_POLICY_PATH="${RECOVERY_POLICY_PATH:-}"
TRACE_JOINTS="${TRACE_JOINTS:-false}"
# Optional spawn height. Empty keeps the map default. A non-zero height makes
# the trial a drop test: the robot free-falls onto its feet, which is the only
# mechanism here that can leave it down but *not* inverted (a lateral push
# always rolls it fully over), i.e. the only way to exercise the get-up ladder
# from a settled fallen pose.
SPAWN_Z="${SPAWN_Z:-}"
# Optional spawn attitude, in radians. Nonzero values place a *settled* fallen
# pose (the MuJoCo spawner composes Rz(yaw)*Ry(pitch)*Rx(roll)), which is the
# only way to exercise the ladder: a perturbed or dropped robot on these maps
# either stays upright or rolls fully over, so there is no fall amplitude that
# leaves it down but not inverted.
SPAWN_PITCH="${SPAWN_PITCH:-}"
SPAWN_ROLL="${SPAWN_ROLL:-}"
# Optional lateral hip input for the roll phase's braced pair. Left empty (and
# never passed as an empty launch argument) unless a sign is being measured.
ROLL_BRACE_HIP="${ROLL_BRACE_HIP:-}"
SOURCE_REVISION="$(git -C "$ROOT" rev-parse --short HEAD)"
SOURCE_DIRTY="$(git -C "$ROOT" status --porcelain --untracked-files=no | wc -l)"

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
/usr/bin/python3 - "$OUT_DIR/manifest.json" "$ROOT" "$DOMAIN" "$DURATION" "$FORCE_N" "$START_S" "$PULSE_S" "$RECOVERY_S" "$FALL_RECOVERY" "$FALL_RECOVERY_TIMEOUT_S" "$FALL_RECOVERY_DELAY_S" "$RECOVERY_POLICY_PATH" "$TRACE_JOINTS" "$SOURCE_REVISION" "$SOURCE_DIRTY" "$SPAWN_Z" "$PERTURBATION_AXIS" "$SPAWN_PITCH" "$SPAWN_ROLL" "$ROLL_BRACE_HIP" <<'PY'
import json
import sys
import hashlib
from pathlib import Path
out, root, domain, duration, force, start, pulse, recovery, fall_recovery, fall_timeout, fall_delay, recovery_policy_path, trace_joints, revision, dirty, spawn_z, perturbation_axis, spawn_pitch, spawn_roll, roll_brace_hip = sys.argv[1:]
Path(out).write_text(json.dumps({
    "tool": "run_perturbation_trial.sh",
    "source_root": root,
    "source_revision": revision,
    "tracked_modified_files_at_launch": int(dirty),
    "ros_domain_id": int(domain),
    "duration_s": float(duration),
    "force_n": float(force),
    "perturbation_axis": int(perturbation_axis),
    "start_s": float(start),
    "pulse_s": float(pulse),
    "recovery_window_s": float(recovery),
    "spawn_z_m": (float(spawn_z) if spawn_z else None),
    "spawn_pitch_rad": (float(spawn_pitch) if spawn_pitch else 0.0),
    "spawn_roll_rad": (float(spawn_roll) if spawn_roll else 0.0),
    "roll_brace_hip_rad": (float(roll_brace_hip) if roll_brace_hip else 0.0),
    "enable_fall_recovery": fall_recovery.strip().lower() in ("true", "1", "yes"),
    "fall_recovery_timeout_s": float(fall_timeout),
    "fall_recovery_start_delay_s": float(fall_delay),
    "recovery_policy_path": recovery_policy_path,
    "trace_joints": trace_joints.strip().lower() in ("true", "1", "yes"),
    "source_sha256": {
        name: hashlib.sha256((Path(root) / name).read_bytes()).hexdigest()
        for name in (
            "src/robot_lab_adapter/robot_lab_adapter/go2_locomotion.py",
            "src/robot_lab_adapter/robot_lab_adapter/go2_stance_gait_controller.py",
            "src/robot_lab_adapter/robot_lab_adapter/go2_recovery_policy.py",
            "src/robot_lab_adapter/policies/go2_recovery_nju/policy.onnx",
            "src/robot_lab_bringup/launch/simulated_robot.launch.py",
        ) if (Path(root) / name).exists()
    },
}, indent=2) + "\n")
PY

trace_args=()
if [[ "$TRACE_JOINTS" == "true" ]]; then
    trace_args+=(--trace-joints)
fi
# An empty path means "keep the built-in sequenced get-up ladder": passing
# ``go2_recovery_policy_path:=`` with no value is a malformed launch argument,
# so the argument is only added when a learned actor was actually requested.
recovery_policy_args=()
if [[ -n "$RECOVERY_POLICY_PATH" ]]; then
    recovery_policy_args+=(go2_recovery_policy_path:="$RECOVERY_POLICY_PATH")
fi
# An empty spawn height keeps the map's default (feet on the ground). A value
# turns the trial into a drop test, which is recorded in the manifest.
spawn_args=()
if [[ -n "$SPAWN_Z" ]]; then
    spawn_args+=(spawn_z:="$SPAWN_Z")
fi
if [[ -n "$SPAWN_PITCH" ]]; then
    spawn_args+=(spawn_pitch:="$SPAWN_PITCH")
fi
if [[ -n "$SPAWN_ROLL" ]]; then
    spawn_args+=(spawn_roll:="$SPAWN_ROLL")
fi
if [[ -n "$ROLL_BRACE_HIP" ]]; then
    spawn_args+=(fall_recovery_roll_brace_hip_rad:="$ROLL_BRACE_HIP")
fi
ROS_DOMAIN_ID="$DOMAIN" /usr/bin/python3 "$PROBE" "${trace_args[@]}" \
    --duration "$DURATION" --wall-timeout 60 \
    --perturbation-start "$START_S" --perturbation-duration "$PULSE_S" \
    --perturbation-force-n "$FORCE_N" --perturbation-axis "$PERTURBATION_AXIS" \
    --recovery-window "$RECOVERY_S" \
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
    go2_perturbation_axis:="$PERTURBATION_AXIS" \
    ${spawn_args[@]+"${spawn_args[@]}"} \
    enable_fall_recovery:="$FALL_RECOVERY" \
    ${recovery_policy_args[@]+"${recovery_policy_args[@]}"} \
    fall_recovery_timeout_s:="$FALL_RECOVERY_TIMEOUT_S" \
    fall_recovery_start_delay_s:="$FALL_RECOVERY_DELAY_S" \
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
printf '{"probe_returncode": %d, "launch_returncode": %d}\n' \
    "$probe_rc" "$launch_rc" > "$OUT_DIR/returncodes.json"
if [[ "$probe_rc" -ne 0 ]]; then
    echo "Perturbation probe exited with status $probe_rc" >&2
    exit "$probe_rc"
fi
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
