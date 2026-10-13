#!/bin/bash
# Start the five attitude nodes (wall time, own domain) and replay one
# synthetic IMU stream to them.  Usage: scripts/r7_5_attitude_replay.sh <out.json>
OUT=$1
HERE=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/humble/setup.bash
source "$HERE/../install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-156}
export OPENBLAS_NUM_THREADS=1
PIDS=()
# Executables are started directly (not through `ros2 run`) so the PIDs
# below are the nodes themselves and the cleanup reaches them.
LIB=$(ros2 pkg prefix robot_lab_algorithms)/lib/robot_lab_algorithms
for exe in complementary_imu mahony_filter madgwick_filter attitude_eskf; do
  "$LIB/$exe" --ros-args -p use_sim_time:=false > "$OUT.$exe.log" 2>&1 &
  PIDS+=($!)
done
"$(ros2 pkg prefix imu_filter_madgwick)/lib/imu_filter_madgwick/imu_filter_madgwick_node" --ros-args -p use_sim_time:=false \
  -p use_mag:=false -p publish_tf:=false -p world_frame:=enu \
  -r imu/data_raw:=/imu/out -r imu/data:=/imu/madgwick_upstream > "$OUT.upstream.log" 2>&1 &
PIDS+=($!)
sleep 4
python3 "$HERE/r7_5_attitude_replay.py" > "$OUT"
STATUS=$?
for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
sleep 2
for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
exit $STATUS
