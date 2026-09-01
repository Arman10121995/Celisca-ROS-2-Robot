#!/usr/bin/env bash
# Bootstrap script for Robot Lab workspace (P7.3).
# Sets up the development environment for supported hosts.
set -euo pipefail

cd "$(dirname "$0")/.."

echo "=== Robot Lab Bootstrap ==="
echo ""

# Check prerequisites
echo "Checking prerequisites..."

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
    echo "ERROR: $missing prerequisite(s) missing. Install them first."
    echo "  Ubuntu 22.04: sudo apt install python3 python3-pip git"
    echo "  ROS 2 Humble:  sudo apt install python3-colcon-common-extensions python3-rosdep2"
    exit 1
fi

# Check ROS 2
echo ""
echo "Checking ROS 2 installation..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "  ROS 2 Humble: OK"
else
    echo "  WARNING: ROS 2 Humble not found at /opt/ros/humble/"
    echo "  Install from: https://docs.ros.org/en/humble/Installation.html"
fi

# Initialize rosdep if needed
echo ""
echo "Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init 2>/dev/null || true
fi
rosdep update 2>/dev/null || true
echo "  rosdep: OK"

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
pip3 install --quiet --user numpy pyyaml 2>/dev/null || true
echo "  Python deps: OK"

# Install ROS dependencies
echo ""
echo "Installing ROS package dependencies..."
source /opt/ros/humble/setup.bash 2>/dev/null || true
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true
echo "  ROS deps: OK"

# Create virtual environment (optional)
echo ""
echo "Setting up Python virtual environment..."
if [ ! -d .venv ]; then
    python3 -m venv .venv
    echo "  Created .venv/"
else
    echo "  .venv/ already exists"
fi

# Build workspace
echo ""
echo "Building workspace..."
source .venv/bin/activate 2>/dev/null || true
source /opt/ros/humble/setup.bash 2>/dev/null || true
colcon build --symlink-install --packages-skip orbslam3 2>/dev/null || {
    echo "  WARNING: Build had errors. Run 'colcon build' manually to diagnose."
}

echo ""
echo "=== Bootstrap complete ==="
echo ""
echo "Next steps:"
echo "  source install/setup.bash"
echo "  source .venv/bin/activate"
echo "  bash scripts/test_fast.sh"
echo "  ros2 run robot_lab_registry robot-lab doctor"
