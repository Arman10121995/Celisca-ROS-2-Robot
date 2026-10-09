#!/usr/bin/env bash
# Install shared geometry wheels on SSD, keeping existing NumPy unchanged.
set -euo pipefail
lab_geometry_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$lab_geometry_root/scripts/ssd_env.sh"
lab_geometry_target="$ROBOT_LAB_RUNTIME_ROOT/python_deps/geometry"
mkdir -p "$lab_geometry_target" "$ROBOT_LAB_RUNTIME_ROOT/cache/pip"
PIP_CACHE_DIR="$ROBOT_LAB_RUNTIME_ROOT/cache/pip" python3 -m pip install \
  --only-binary=:all: --no-deps --target "$lab_geometry_target" \
  --requirement "$lab_geometry_root/scripts/requirements-geometry.txt"
printf 'Shared geometry dependencies: %s\nSource scripts/ssd_env.sh before starting ROS or PX4.\n' "$lab_geometry_target"
