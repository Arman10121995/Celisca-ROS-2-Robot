#!/usr/bin/env bash
# Tiered Robot Lab test runner (R1.2).
#
# Usage:
#   scripts/test_tiers.sh [fast|integration|physics|all]   # default: fast
#   scripts/test_tiers.sh --list
#
# Tiers:
#   fast         Unit / numerical / config checks. No ROS graph, no
#                subprocesses, no real physics. Runs on plain Python.
#   integration  ROS-tooling tests: xacro expansion of every robot profile,
#                launch-contract and qualification suites. Requires a
#                sourced ROS 2 environment (skips with a note if ros2 is
#                unavailable).
#   physics      Optional physics engines (PyBullet/MuJoCo/Isaac) executed
#                headless. Engines that are not installed produce explicit
#                skips, never failures.
#   all          Everything above.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

export PYTHONPATH="src/robot_lab/robot_lab_registry:src/robot_lab_algorithms:src/robot_lab/robot_lab_benchmark:src/robot_lab_adapter:src/robot_lab_pybullet/python:src/robot_lab_mujoco/python:src/robot_lab_isaac/python:${PYTHONPATH:-}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
PYTEST=(python3 -m pytest -q -p no:anyio)

REG_TEST="src/robot_lab/robot_lab_registry/test"
ADP_TEST="src/robot_lab_adapter/test"
BRG_TEST="src/robot_lab_bringup/test"

FAST_TESTS=(
  "$REG_TEST/test_p5_algorithm_breadth.py"
  "$REG_TEST/test_p6_benchmarking.py"
  "$REG_TEST/test_registry.py"
  "$REG_TEST/test_p3_6_safety_limits.py"
  "$REG_TEST"/test_p4_*.py
  "$REG_TEST/test_bumperbot_qualification.py"
  "$REG_TEST/test_r3_2_composition_compatibility.py"
  "$ADP_TEST/test_adapter.py"
  "$ADP_TEST/test_launch_fragments.py"
  "$ADP_TEST/test_selectors.py"
  "$ADP_TEST/test_namespaces.py"
  "$BRG_TEST/test_sim_profiles.py"
)

INTEGRATION_TESTS=(
  "$BRG_TEST/test_xacro_expansion.py"
  "$BRG_TEST/test_simulation_clocks.py"
  "$REG_TEST"/test_go2_qualification.py
  "$REG_TEST"/test_labbot_qualification.py
  "$REG_TEST"/test_bhl_qualification.py
  "$REG_TEST"/test_quadrotor_sitl_qualification.py
)

PHYSICS_TESTS=(
  "$BRG_TEST/test_simulator_backends.py"
)

run_tier() {
  local name="$1"; shift
  local files=("$@")
  echo "=== Tier: $name ($((${#files[@]})) test files) ==="
  if [ "$name" = "integration" ] || [ "$name" = "all" ]; then
    if ! command -v ros2 >/dev/null 2>&1; then
      echo "!! ros2 not on PATH — integration tests need a sourced ROS 2 env; skipping."
      return 0
    fi
  fi
  "${PYTEST[@]}" "${files[@]}"
  echo "=== Tier: $name PASS ==="
}

list_tiers() {
  echo "fast:"
  printf '  %s\n' "${FAST_TESTS[@]}"
  echo "integration:"
  printf '  %s\n' "${INTEGRATION_TESTS[@]}"
  echo "physics:"
  printf '  %s\n' "${PHYSICS_TESTS[@]}"
}

case "${1:-fast}" in
  --list) list_tiers ;;
  fast)        run_tier fast        "${FAST_TESTS[@]}" ;;
  integration) run_tier integration "${INTEGRATION_TESTS[@]}" ;;
  physics)     run_tier physics     "${PHYSICS_TESTS[@]}" ;;
  all)
    run_tier fast        "${FAST_TESTS[@]}"
    run_tier physics     "${PHYSICS_TESTS[@]}"
    run_tier integration "${INTEGRATION_TESTS[@]}"
    ;;
  *) echo "usage: $0 [fast|integration|physics|all|--list]" >&2; exit 2 ;;
esac