#!/usr/bin/env bash
# Fast PR test suite (P7.1) — runs in < 60s, no simulation required.
# Excludes: xacro expansion tests (need built workspace + Gazebo),
#           launch orchestration tests (subprocess timeouts).
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
echo "=== Fast unit tests ==="
python3 -m unittest discover \
    -s src/robot_lab/robot_lab_registry/test \
    -p 'test_p5*' -v

echo ""
echo "=== Benchmark tests (schema + logic only) ==="
TEST_DIR="src/robot_lab/robot_lab_registry/test"
PYTHONPATH="$TEST_DIR:$PYTHONPATH" python3 -m unittest \
    test_p6_benchmarking.BenchmarkingTests \
    test_p6_benchmarking.GroundTruthAdapterTests \
    test_p6_benchmarking.MetricNormalizerTests \
    test_p6_benchmarking.OutputGeneratorTests \
    test_p6_benchmarking.RegressionThresholdTests \
    -v

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
