#!/usr/bin/env bash
# Bootstrap script for Robot Lab workspace (R1.3).
#
# Separates project bootstrap from privileged host changes:
#   * PROJECT tier (always run): verify host prerequisites, then create
#     .venv and pip-install the pinned requirements.txt into it, then build
#     the workspace (optional).
#   * HOST tier (only with --with-host-deps): privileged operations — apt
#     package installs and `sudo rosdep init`. These need sudo and are
#     skipped on CI. They are NOT mixed into the project bootstrap.
#
# Failures are never swallowed: each step returns nonzero on error.
set -euo pipefail

cd "$(dirname "$0")/.."

WITH_HOST=0
SKIP_BUILD=0
for arg in "$@"; do
    case "$arg" in
        --with-host-deps) WITH_HOST=1 ;;
        --skip-build) SKIP_BUILD=1 ;;
        *) echo "usage: $0 [--with-host-deps] [--skip-build]" >&2; exit 2 ;;
    esac
done

echo "=== Robot Lab Bootstrap ==="
echo ""

# ── Host-tier prerequisites (verification only, no privileged install) ─────
echo "Checking host prerequisites..."

missing=0
check_cmd() {
    if ! command -v "$1" &>/dev/null; then
        echo "  MISSING: $1 ($2)"
        missing=$((missing + 1))
    else
        echo "  OK: $1"
    fi
}

check_cmd python3 "Python 3.10+"
check_cmd pip3 "pip package manager"
check_cmd colcon "ROS 2 build tool"
check_cmd rosdep "ROS dependency tool"
check_cmd git "version control"

if [ "$missing" -gt 0 ]; then
    echo ""
    echo "ERROR: $missing host prerequisite(s) missing. Install them first (they"
    echo "       require sudo):"
    echo "  Ubuntu 22.04: sudo apt install python3 python3-pip git"
    echo "  ROS 2 Humble:  sudo apt install python3-colcon-common-extensions python3-rosdep2"
    echo "  Or run: bash scripts/bootstrap.sh --with-host-deps"
    exit 1
fi

# ROS 2 must be discoverable for the rest of the bootstrap.
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo ""
    echo "ERROR: ROS 2 Humble not found at /opt/ros/humble/setup.bash"
    echo "Install ROS 2 Humble first: https://docs.ros.org/en/humble/Installation.html"
    echo "  (then re-run this script)"
    exit 1
fi
source /opt/ros/humble/setup.bash
echo "  ROS 2 Humble: OK"

# rosdep data must exist (rosdep install will fail cleanly otherwise).
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    if [ "$WITH_HOST" = "1" ]; then
        echo "  Initializing rosdep (privileged)..."
        sudo rosdep init
    else
        echo "  WARN: rosdep data not initialized; run 'sudo rosdep init' once"
        echo "        (or re-run bootstrap with --with-host-deps)"
    fi
fi
rosdep update

# ── Project tier: virtualenv with pinned deps ──────────────────────────────
echo ""
echo "Setting up Python virtual environment (.venv)..."
if [ ! -d .venv ]; then
    python3 -m venv .venv
    echo "  Created .venv/"
fi
# shellcheck disable=SC1091
# Upgrade pip first, then install pinned requirements (never '|| true').
python3 -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt

# ── Project tier: ROS dependencies (no privileged operations) ──────────────
# `rosdep install` fetches declared apt/python deps for packages in src/.
# It may need to run as root for apt-based installs; on a prepared host or
# CI this resolves from the cache and succeeds without sudo. Failures here
# are real and must not be masked.
echo ""
echo "Installing ROS package dependencies (rosdep)..."
rosdep install --from-paths src --ignore-src -r -y

# ── Project tier: build workspace (optional) ───────────────────────────────
if [ "$SKIP_BUILD" = "1" ]; then
    echo ""
    echo "Skipping colcon build (--skip-build)."
else
    echo ""
    echo "Building workspace (core packages)..."
    colcon build --symlink-install --packages-skip orbslam3 \
        --event-handlers console_direct+
fi

echo ""
echo "=== Bootstrap complete ==="
echo ""
echo "Next steps:"
echo "  source install/setup.bash"
echo "  source .venv/bin/activate"
echo "  bash scripts/test_fast.sh"
echo "  ros2 run robot_lab_registry robot-lab doctor"
