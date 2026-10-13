#!/bin/bash
# Overlay tf2 and tf2_ros 0.25.24 into this workspace's install space.
#
# The Humble apt packages are 0.25.23, whose "waitForTransform race
# condition" fix (ros2/geometry2#974) introduced an ABBA deadlock between
# tf2_ros::Buffer::waitForTransform (MessageFilter::add, costmap executor)
# and tf2::BufferCore::testTransformableRequests (TransformListener thread):
# ros2/geometry2#982/#992, fixed in 0.25.24 (#990, 2026-09-15).  When it
# hits, the Nav2 server's TF listener thread blocks for good: its transforms
# freeze while every other node still receives them, and the server does not
# exit on SIGINT.  0.25.24 changes only .cpp files (headers identical to
# 0.25.23), so the apt Nav2 binaries load the overlay libraries through
# LD_LIBRARY_PATH once install/setup.bash is sourced.
#
# Remove this overlay (rm -rf build/tf2 build/tf2_ros install/tf2 install/tf2_ros)
# once apt ships ros-humble-tf2-ros >= 0.25.24.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/.." && pwd)
SRC=${TF2_FIX_SOURCE:-/workspace/molar/third_party/geometry2-0.25.24}
if [ ! -d "$SRC/tf2_ros" ]; then
  mkdir -p "$(dirname "$SRC")"
  git clone --depth 1 --branch 0.25.24 https://github.com/ros2/geometry2.git "$SRC"
fi
test "$(git -C "$SRC" describe --tags 2>/dev/null || echo unknown)" = 0.25.24 \
  || { echo "expected geometry2 0.25.24 at $SRC" >&2; exit 2; }
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --base-paths "$SRC" --packages-select tf2 tf2_ros \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
echo "tf2/tf2_ros 0.25.24 installed in $WS/install"
