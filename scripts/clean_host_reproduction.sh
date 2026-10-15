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
TESTS=-1; MISSION=-1
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
  docker run --rm -v "$OUT/workspace:/workspace" -e ROS_DOMAIN_ID=41 "$IMAGE" bash -c \
    'cd /ws && xvfb-run -a scripts/sim_nav_mission.sh mujoco bumperbot nav_obstacle /workspace/mission/mujoco_bumperbot_nav_obstacle.json odometry_source:=wheels' \
    > "$OUT/mission.log" 2>&1
  MISSION=$?
  # The container ran as root: hand the bind-mounted outputs back.
  docker run --rm -v "$OUT/workspace:/workspace" "$IMAGE" chown -R "$(id -u):$(id -g)" /workspace
  cp -r "$OUT/workspace/mission" "$OUT/" 2>/dev/null
fi
finished=$(date +%s)
cat > "$OUT/result.json" <<EOF
{"revision": "$REV", "image": "$IMAGE", "build_exit": $BUILD, "fast_tier_exit": $TESTS,
 "mission_exit": $MISSION, "build_s": $((built - started)), "tests_s": $(( ${tested:-$built} - built )),
 "total_s": $((finished - started))}
EOF
cat "$OUT/result.json"
