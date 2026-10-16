#!/bin/bash
# R9.1 clean-host reproduction: build one revision in a fresh container from
# the official ROS 2 Humble image, run the fast test tier and a headless
# MuJoCo Bumperbot Nav2 mission inside it, and keep every log.
#
# Usage: scripts/clean_host_reproduction.sh [revision]   (default: HEAD)
#
# Output: $ROBOT_LAB_RUNTIME_ROOT/clean_host/<revision>/ with
#   build.log            docker build (rosdep, pip, colcon)
#   image.txt            image id, base image digest, platform
#   rosdep-install.log, pip-freeze.txt, colcon-build.log   copied from the image
#   fast_tier.log        scripts/test_tiers.sh fast inside the container
#   mission/             scripts/sim_nav_mission.sh mujoco bumperbot nav_obstacle
#   gazebo/, isaac/      the same mission in Gazebo and Isaac (CLEAN_HOST_SIMS,
#                        default "gazebo isaac"; empty skips them)
#   golden_comparison.json  the seeded R7.2 perception benchmark, container vs host
#   result.json          exit codes and timings
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
source "$HERE/ssd_env.sh" || exit 2
REV=$(git -C "$REPO" rev-parse --short "${1:-HEAD}") || exit 2
OUT=$ROBOT_LAB_RUNTIME_ROOT/clean_host/$REV
CTX=$OUT/context
IMAGE=robot-lab-clean-host:$REV
mkdir -p "$OUT" && rm -rf "$CTX" "$OUT/lfs_errors.txt" && mkdir -p "$CTX/ws" "$CTX/manifests"
git -C "$REPO" archive "$REV" | tar -x -C "$CTX/ws"
# A clone with git-lfs gets file contents, while an archive keeps the LFS
# pointers: smudge every LFS path to its content, as a clone would.
git -C "$REPO" lfs ls-files -n "$REV" | while read -r path; do
  git -C "$REPO" show "$REV:$path" | git -C "$REPO" lfs smudge -- "$path" > "$CTX/ws/$path" || echo "lfs: $path" >> "$OUT/lfs_errors.txt"
done
(cd "$CTX/ws" && find src -name package.xml -o -name COLCON_IGNORE | cpio -pdm "$CTX/manifests" 2>/dev/null)
cp "$REPO/docker/clean_host/Dockerfile" "$REPO/docker/clean_host/requirements.txt" "$CTX/"
started=$(date +%s)
docker build --progress=plain -t "$IMAGE" "$CTX" > "$OUT/build.log" 2>&1
BUILD=$?
built=$(date +%s)
{
  echo "revision $REV"
  docker image inspect "$IMAGE" --format 'image {{.Id}} platform {{.Os}}/{{.Architecture}} created {{.Created}}' 2>&1
  docker image inspect ros:humble-ros-base-jammy --format 'base {{index .RepoDigests 0}}' 2>&1
} > "$OUT/image.txt"
TESTS=-1; MISSION=-1; GOLDEN=-1; GAZEBO=-1; ISAAC=-1
if [ $BUILD -eq 0 ]; then
  for f in rosdep-install.log pip-freeze.txt colcon-build.log; do
    docker run --rm "$IMAGE" cat "/$f" > "$OUT/$f" 2>/dev/null
  done
  # ssd_env.sh needs /workspace on its own device: a bind mount provides it.
  mkdir -p "$OUT/workspace"
  docker run --rm -v "$OUT/workspace:/workspace" "$IMAGE" bash -c \
    'cd /ws && source /opt/ros/humble/setup.bash && source install/setup.bash && scripts/test_tiers.sh fast' \
    > "$OUT/fast_tier.log" 2>&1
  TESTS=$?
  tested=$(date +%s)
  mkdir -p "$OUT/workspace/mission"
  # xvfb-run as the container's first process waits forever for Xvfb's
  # ready signal (2026-10-15: 2 h 15 min); an init process and a plain Xvfb
  # avoid it.  The mission is bounded at 30 minutes.
  docker run --rm --init -v "$OUT/workspace:/workspace" -e ROS_DOMAIN_ID=41 "$IMAGE" bash -c \
    'Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp > /dev/null 2>&1 & export DISPLAY=:99; sleep 2; cd /ws &&
     timeout 1800 scripts/sim_nav_mission.sh mujoco bumperbot nav_obstacle /workspace/mission/mujoco_bumperbot_nav_obstacle.json odometry_source:=wheels' \
    > "$OUT/mission.log" 2>&1
  MISSION=$?
  # Gazebo and Isaac render on the GPU: the container gets it through the
  # NVIDIA runtime.  Gazebo comes with the image (rosdep: ros_gz_sim ->
  # Ignition Fortress).  Isaac Sim is not a ROS package; the host's Isaac Sim
  # build is mounted read-only as an external simulator, like the GPU driver,
  # and everything else still comes from the image.
  SIMS=${CLEAN_HOST_SIMS-gazebo isaac}
  GPU="--runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all"
  if [[ " $SIMS " == *" gazebo "* ]]; then
    mkdir -p "$OUT/workspace/gazebo"
    docker run --rm --init $GPU -v "$OUT/workspace:/workspace" -e ROS_DOMAIN_ID=42 "$IMAGE" bash -c \
      'Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp > /dev/null 2>&1 & export DISPLAY=:99; sleep 2; cd /ws &&
       timeout 1800 scripts/sim_nav_mission.sh gazebo bumperbot nav_obstacle /workspace/gazebo/gazebo_bumperbot_nav_obstacle.json' \
      > "$OUT/gazebo.log" 2>&1
    GAZEBO=$?
  fi
  # The host's Isaac Sim build (its python.sh), mounted at the same path.
  ISAAC_PYTHON_HOST=${ISAAC_PYTHON_HOST:-$(PYTHONPATH="$REPO/src/robot_lab_utils" /usr/bin/python3 -c \
    'from robot_lab_utils.isaac_env import find_isaac_python; print(find_isaac_python())' 2>/dev/null)}
  ISAAC_ROOT=$(cd "$(dirname "${ISAAC_PYTHON_HOST:-/nonexistent}")/../../.." 2>/dev/null && pwd)
  if [[ " $SIMS " == *" isaac "* ]] && [ -x "${ISAAC_PYTHON_HOST:-}" ] && [ -n "$ISAAC_ROOT" ]; then
    mkdir -p "$OUT/workspace/isaac"
    docker run --rm --init $GPU -v "$OUT/workspace:/workspace" -v "$ISAAC_ROOT:$ISAAC_ROOT:ro" \
      -e ISAAC_PYTHON="$ISAAC_PYTHON_HOST" -e OMNI_KIT_ACCEPT_EULA=YES -e ACCEPT_EULA=Y \
      -e ROS_DOMAIN_ID=43 "$IMAGE" bash -c \
      'cd /ws && timeout 2400 scripts/sim_nav_mission.sh isaac bumperbot nav_obstacle /workspace/isaac/isaac_bumperbot_nav_obstacle.json' \
      > "$OUT/isaac.log" 2>&1
    ISAAC=$?
  fi
  # Golden comparison: the seeded R7.2 perception benchmark in the container
  # and on this host from the same archive; their accuracy figures must
  # agree (timings may not).
  docker run --rm -v "$OUT/workspace:/workspace" "$IMAGE" bash -c \
    'cd /ws && source /opt/ros/humble/setup.bash && python3 scripts/r7_2_perception_benchmark.py --out /workspace/golden/container' \
    > "$OUT/golden_container.log" 2>&1
  (cd "$CTX/ws" && /usr/bin/python3 scripts/r7_2_perception_benchmark.py --out "$OUT/golden_host") > "$OUT/golden_host.log" 2>&1
  # The container ran as root: hand the bind-mounted outputs back.
  docker run --rm -v "$OUT/workspace:/workspace" "$IMAGE" chown -R "$(id -u):$(id -g)" /workspace
  cp -r "$OUT/workspace/mission" "$OUT/workspace/gazebo" "$OUT/workspace/isaac" "$OUT/" 2>/dev/null
  python3 - "$OUT/workspace/golden/container/results.json" "$OUT/golden_host/results.json" "$OUT/golden_comparison.json" <<'PY'
import json, sys
def accuracy(node, path=''):
    if isinstance(node, dict):
        for key, value in node.items():
            if key in ('revision', 'host') or key.startswith('ms'):
                continue
            yield from accuracy(value, path + '/' + key)
    else:
        yield path, node
container, host = (dict(accuracy(json.load(open(p)))) for p in sys.argv[1:3])
differences = {k: [container.get(k), host.get(k)] for k in sorted(set(container) | set(host)) if container.get(k) != host.get(k)}
json.dump({'compared': len(set(container) | set(host)), 'differences': differences}, open(sys.argv[3], 'w'), indent=1)
sys.exit(1 if differences or not container else 0)
PY
  GOLDEN=$?
fi
finished=$(date +%s)
cat > "$OUT/result.json" <<EOF
{"revision": "$REV", "image": "$IMAGE", "build_exit": $BUILD, "fast_tier_exit": $TESTS,
 "mission_exit": $MISSION, "gazebo_mission_exit": $GAZEBO, "isaac_mission_exit": $ISAAC,
 "golden_comparison_exit": $GOLDEN, "build_s": $((built - started)), "tests_s": $(( ${tested:-$built} - built )),
 "total_s": $((finished - started))}
EOF
cat "$OUT/result.json"
