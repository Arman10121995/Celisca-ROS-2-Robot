#!/usr/bin/env bash
# Run the pinned upstream X500 and a bounded MAVLink flight probe on the SSD.
set -euo pipefail
set -m
TASK_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$TASK_DIR/scripts/ssd_env.sh"
PX4_ROOT=${PX4_ROOT:-/workspace/molar/px4/PX4-Autopilot}
PX4_BUILD="$PX4_ROOT/build/px4_sitl_default"
OUT_DIR=${1:?Usage: px4_sitl_check.sh /workspace/path/to/new-run-directory}
mkdir -p "$OUT_DIR/rootfs"
if [[ $(stat -c %d "$OUT_DIR") != $(stat -c %d /workspace) ]]; then
    echo 'PX4 trial output must be on the mounted workspace SSD' >&2
    exit 2
fi
if [[ ! -x "$PX4_BUILD/bin/px4" || ! -f "$PX4_BUILD/rootfs/gz_env.sh" ]]; then
    echo "Build PX4 SITL first under $PX4_ROOT; see docs/tutorials/px4_x500.md" >&2
    exit 2
fi
export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:-}
export GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
source "$PX4_BUILD/rootfs/gz_env.sh"
export HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500
export PX4_GZ_WORLD=${PX4_GZ_WORLD:-default}
export GZ_PARTITION=robot_lab_px4_$$
export PATH="$PX4_BUILD/bin:$PATH"
if [[ ! -e "$OUT_DIR/rootfs/etc" ]]; then
    ln -s "$PX4_BUILD/etc" "$OUT_DIR/rootfs/etc"
fi
PX4_PYTHON=${PX4_PYTHON:-$ROBOT_LAB_RUNTIME_ROOT/px4/venv/bin/python3}
"$PX4_PYTHON" -c 'from pymavlink import mavutil' || exit 2
FCU_PID='' 
cleanup() {
    if [[ -n "$FCU_PID" ]]; then
        kill -INT -- -"$FCU_PID" 2>/dev/null || true
        for ((i=0; i<10; i++)); do
            kill -0 "$FCU_PID" 2>/dev/null || break
            sleep 0.5
        done
        kill -TERM -- -"$FCU_PID" 2>/dev/null || true
        sleep 0.5
        kill -KILL -- -"$FCU_PID" 2>/dev/null || true
        wait "$FCU_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM
# -d prevents pxh prompt spam; each run has independent parameters and logs.
"$PX4_BUILD/bin/px4" -d "$PX4_BUILD/etc" -w "$OUT_DIR/rootfs" > "$OUT_DIR/px4.log" 2>&1 &
FCU_PID=$!
printf '%s\n' "$FCU_PID" > "$OUT_DIR/px4.pid"
git -C "$PX4_ROOT" rev-parse HEAD > "$OUT_DIR/px4-revision.txt"
set +e
timeout 150 "$PX4_PYTHON" "$TASK_DIR/scripts/px4_offboard_flight.py" --out "$OUT_DIR/flight.json" > "$OUT_DIR/probe.log" 2>&1
RESULT=$?
set -e
exit "$RESULT"
