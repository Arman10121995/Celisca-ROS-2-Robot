#!/bin/bash
# V7: the November 18-19 lab_rov backend round (hold, surge, yaw, current, operator loss) repeated, two runs per backend,
# with the USV round's case runner (MARINE_MODEL=lab_rov) and the same rov_probe.py.
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v7-rov-20261122
DIR=$(cd "$(dirname "$0")" && pwd); mkdir -p "$RUN"
source "$HERE/scripts/ssd_env.sh" >/dev/null; source /opt/ros/humble/setup.bash; source "$HERE/install/setup.bash"
export ROS_DOMAIN_ID=117 MARINE_MODEL=lab_rov; LOCK="$ROBOT_LAB_RUNTIME_ROOT/physics-qualification.lock"
CASE="$HERE/docs/status/evidence/v7-marine-usv-2026-11-22/run_case.py"
for i in 1 2; do for sim in gazebo mujoco pybullet isaac; do
  [ -s "$RUN/$sim-$i/result.json" ] || flock -w 1800 "$LOCK" python3 "$CASE" $sim "$RUN/$sim-$i"
  echo "$sim-$i exit=$?" >> "$DIR/progress.txt"; sleep 3
done; done
echo ALLDONE >> "$DIR/progress.txt"
