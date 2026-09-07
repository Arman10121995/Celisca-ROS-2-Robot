#!/usr/bin/env bash
# Fast PR test suite (R1.2) — unit/numerical/config tier, < 60 s.
# No ROS graph required, no subprocesses, no real physics.
# The authoritative tier manifest lives in scripts/test_tiers.sh.
set -euo pipefail

cd "$(dirname "$0")/.."

export PYTHONPATH="src/robot_lab/robot_lab_registry:src/robot_lab_algorithms:src/robot_lab/robot_lab_benchmark"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

echo "=== Module compilation check ==="
for f in perception localization state_estimation sensor_fusion global_planning local_planning; do
    python3 -m py_compile src/robot_lab_algorithms/robot_lab_algorithms/$f.py
done
echo "All modules compile."

echo ""
echo "=== Fast unit tests (fast tier) ==="
bash scripts/test_tiers.sh fast

echo ""
echo "=== Registry validation ==="
PYTHONPATH=src/robot_lab/robot_lab_registry python3 -c "
from robot_lab_registry.catalog import Registry
from robot_lab_registry.validation import validate_cross_references
reg = Registry('src/robot_lab/robot_lab_registry/config')
reg.load('src/robot_lab/robot_lab_registry/config')
result = validate_cross_references(reg)
assert result.valid, f'Cross-reference errors: {result.errors}'
print('Registry cross-reference validation: PASS')
"

echo ""
echo "=== Fast PR test suite: PASS ==="
