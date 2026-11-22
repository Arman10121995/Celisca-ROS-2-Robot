#!/bin/bash
# V4 diagnosis: Go2 MuJoCo seed 2 with the global inflation of October 15 (0.45 m, default cost scaling) instead of
# 26b961e4's 0.7 m / 2.0; the config file is a symlink, restored from HEAD afterwards.
HERE=/workspace/molar/ros_ws/bumperbot_ws; RUN=/workspace/molar/robot_lab_runtime/v4-legged-nav-20261122/go2-inflation
F=src/robot_lab_navigation/config/robots/unitree_go2.yaml; mkdir -p "$RUN"; cd "$HERE"; export ROS_DOMAIN_ID=117
trap 'git checkout HEAD -- $F' EXIT
python3 - "$F" <<'PY'
import sys
p=sys.argv[1]; s=open(p).read()
a="        inflation_radius: 0.7\n        cost_scaling_factor: 2.0\n"
assert a in s; s=s.replace(a,"        inflation_radius: 0.45\n",1); open(p,'w').write(s)
PY
for seed in 2; do
  CHECK_ARGS="--base base --max-yaw-error-deg 0 --seed $seed" flock -w 1800 /workspace/molar/robot_lab_runtime/physics-qualification.lock \
    timeout --signal=INT 1800 "$HERE/scripts/sim_nav_mission.sh" mujoco unitree_go2 nav_obstacle "$RUN/mujoco_go2_infl045_seed$seed.json" > "$RUN/seed$seed.run.log" 2>&1
done
echo done > "$RUN/done"
