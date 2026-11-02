#!/usr/bin/env bash
# Build a process-scoped official Humble graph serializer correction on SSD.
set -e
lab_workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$lab_workspace/scripts/ssd_env.sh"
source /opt/ros/humble/setup.bash
source "$lab_workspace/install/setup.bash"
source "$lab_workspace/scripts/rmf_env.sh"
cmake -S "$lab_workspace/src/robot_lab_utils/vendor/rmf_graph" \
  -B "$ROBOT_LAB_RUNTIME_ROOT/rmf/graph-fix-build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$lab_workspace/install/robot_lab_utils"
cmake --build "$ROBOT_LAB_RUNTIME_ROOT/rmf/graph-fix-build" --parallel 2
cmake --install "$ROBOT_LAB_RUNTIME_ROOT/rmf/graph-fix-build"
