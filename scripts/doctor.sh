#!/usr/bin/env bash
# Doctor script for Robot Lab workspace (P7.3).
# Diagnoses common issues and reports workspace health.
set -uo pipefail

cd "$(dirname "$0")/.."

PASS=0
WARN=0
FAIL=0

report() {
    local status="$1" msg="$2"
    case "$status" in
        pass) echo "  [PASS] $msg"; PASS=$((PASS + 1)) ;;
        warn) echo "  [WARN] $msg"; WARN=$((WARN + 1)) ;;
        fail) echo "  [FAIL] $msg"; FAIL=$((FAIL + 1)) ;;
    esac
}

echo "=== Robot Lab Doctor ==="
echo ""

# Check ROS 2
echo "ROS 2 Environment:"
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    report pass "ROS 2 Humble installed"
else
    report fail "ROS 2 Humble not found"
fi

if [ -n "${ROS_DISTRO:-}" ]; then
    report pass "ROS_DISTRO=$ROS_DISTRO"
else
    report warn "ROS_DISTRO not set"
fi

# Check workspace structure
echo ""
echo "Workspace Structure:"
for dir in src install build; do
    if [ -d "$dir" ]; then
        report pass "$dir/ exists"
    else
        report warn "$dir/ missing"
    fi
done

# Check key packages
echo ""
echo "Key Packages:"
for pkg in robot_lab_registry robot_lab_bringup bumperbot_algorithms robot_lab_benchmark; do
    if [ -d "src/robot_lab/$pkg" ] || [ -d "src/$pkg" ]; then
        report pass "$pkg source present"
    else
        report fail "$pkg source missing"
    fi
done

# Check Python environment
echo ""
echo "Python Environment:"
if [ -d .venv ]; then
    report pass "Virtual environment exists"
else
    report warn "No virtual environment (.venv)"
fi

python3 -c "import numpy" 2>/dev/null && report pass "numpy available" || report warn "numpy not installed"
python3 -c "import yaml" 2>/dev/null && report pass "pyyaml available" || report warn "pyyaml not installed"

# Check build artifacts
echo ""
echo "Build Status:"
if [ -f install/setup.bash ]; then
    report pass "Workspace built (install/setup.bash exists)"
    source install/setup.bash 2>/dev/null
else
    report warn "Workspace not built (run 'colcon build')"
fi

# Check registry
echo ""
echo "Registry:"
if [ -f src/robot_lab/robot_lab_registry/config/algorithms.yaml ]; then
    report pass "algorithms.yaml present"
else
    report fail "algorithms.yaml missing"
fi

if [ -f src/robot_lab/robot_lab_registry/config/robots.yaml ]; then
    report pass "robots.yaml present"
else
    report fail "robots.yaml missing"
fi

# Check launch files
echo ""
echo "Launch Files:"
for launch in src/robot_lab_bringup/launch/select_robot.launch.py; do
    if [ -f "$launch" ]; then
        report pass "$(basename $launch) present"
    else
        report fail "$(basename $launch) missing"
    fi
done

# Check licenses
echo ""
echo "Licenses:"
if [ -f LICENSE ]; then
    report pass "Top-level LICENSE present"
else
    report fail "LICENSE missing"
fi

if [ -f LICENSES/third-party-notices.md ]; then
    report pass "Third-party notices present"
else
    report warn "Third-party notices missing"
fi

# Summary
echo ""
echo "=== Summary ==="
echo "  Pass: $PASS"
echo "  Warn: $WARN"
echo "  Fail: $FAIL"
echo ""

if [ "$FAIL" -gt 0 ]; then
    echo "ERROR: $FAIL check(s) failed. Run 'bash scripts/bootstrap.sh' to fix."
    exit 1
elif [ "$WARN" -gt 0 ]; then
    echo "WARNING: $WARN non-critical issue(s) found."
    exit 0
else
    echo "All checks passed. Workspace is healthy."
    exit 0
fi
