#!/bin/bash
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Always sanitize PATH to drop any .venv / venv bin directories that may
# have been left by an active virtualenv. This prevents ament/cmake from
# invoking a python3 that lacks catkin_pkg etc.
echo "Sanitizing PATH to remove any venv python directories..."
CLEAN_PATH=$(printf '%s' "$PATH" | tr ':' '\n' | grep -v -E '(/\.?venv/|/virtualenv/)' | tr '\n' ':' | sed 's/:$//')
export PATH="$CLEAN_PATH"
unset VIRTUAL_ENV 2>/dev/null || true
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH 2>/dev/null || true
echo "python3 now resolves to: $(command -v python3)"
if command -v python3 >/dev/null && python3 -c "import catkin_pkg" 2>/dev/null; then
  echo "catkin_pkg available: OK"
else
  echo "WARNING: python3 lacks catkin_pkg. Forcing /usr/bin/python3 if available."
  if [ -x /usr/bin/python3 ] && /usr/bin/python3 -c "import catkin_pkg" 2>/dev/null; then
    export PATH="/usr/bin:$PATH"
  fi
fi

source /opt/ros/humble/setup.bash

# Determine a reliable python that has catkin_pkg
GOOD_PYTHON="/usr/bin/python3"
if [ ! -x "$GOOD_PYTHON" ] || ! "$GOOD_PYTHON" -c "import catkin_pkg" 2>/dev/null; then
  GOOD_PYTHON=$(command -v python3)
fi
echo "Forcing cmake to use python: $GOOD_PYTHON"

# Run colcon forcing python + fresh configure so cached bad venvs in
# previous CMakeCache are ignored. Also clean PATH.
env -u VIRTUAL_ENV -u PYTHONHOME PATH="$PATH" colcon build \
  --symlink-install \
  --continue \
  --cmake-force-configure \
  --cmake-args \
    -DPYTHON_EXECUTABLE="$GOOD_PYTHON" \
    -DPython3_EXECUTABLE="$GOOD_PYTHON" \
    -DPython_EXECUTABLE="$GOOD_PYTHON"

source "$WS_DIR/install/setup.bash"

echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo "---------------"
echo "Done"
echo "---------------"
