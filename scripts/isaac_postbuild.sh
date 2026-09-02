#!/bin/bash
# Isaac Sim post-build pipeline (SSD-first).
# Waits for the source build to finish, relocates the Omniverse extension
# cache off the internal eMMC onto the SSD, then stages and builds the
# Docker image.  Run:  nohup bash /workspace/molar/isaac_postbuild.sh > /workspace/molar/build-logs/postbuild.log 2>&1 &
set -u
LOG=/workspace/molar/build-logs/isaac_build.log
ISAAC_DIR=/workspace/molar/installs/IsaacSim-6.0.1

echo "[postbuild] waiting for source build to finish..."
while pgrep -f '[r]epoman.py build' > /dev/null; do sleep 60; done
echo "[postbuild] source build process exited at $(date)"

# Success check: kit binary + release tree must exist and no fatal errors
if [ ! -x "${ISAAC_DIR}/_build/linux-aarch64/release/kit/kit" ]; then
    echo "[postbuild] ERROR: kit binary missing — source build FAILED. See ${LOG}" >&2
    exit 1
fi
if tail -100 "${LOG}" | grep -qiE 'build failed|error 1|error 2'; then
    echo "[postbuild] ERROR: failure markers in build log. See ${LOG}" >&2
    exit 1
fi
echo "[postbuild] source build OK"

# Relocate Omniverse extension cache eMMC -> SSD (11GB+)
if [ -d /home/molar1/.local/share/ov ] && [ ! -L /home/molar1/.local/share/ov ]; then
    echo "[postbuild] relocating ~/.local/share/ov to SSD..."
    mkdir -p /workspace/molar/xdg-data/share
    mv /home/molar1/.local/share/ov /workspace/molar/xdg-data/share/ov
    ln -s /workspace/molar/xdg-data/share/ov /home/molar1/.local/share/ov
    echo "[postbuild] ov cache relocated ($(du -sh /workspace/molar/xdg-data/share/ov | cut -f1))"
fi

# Stage the docker context (fresh, now that the build is complete)
cd "${ISAAC_DIR}" || exit 1
export XDG_DATA_HOME=/workspace/molar/xdg-data
export PM_PACKAGES_ROOT=/workspace/molar/packman-cache
export PIP_CACHE_DIR=/workspace/molar/pip-cache
export TMPDIR=/workspace/molar/docker-tmp
rm -rf _container_temp
echo "[postbuild] running prep_docker_build.sh --aarch64 ..."
if ! ./tools/docker/prep_docker_build.sh --aarch64 >> /workspace/molar/build-logs/prep.log 2>&1; then
    echo "[postbuild] ERROR: prep failed — see /workspace/molar/build-logs/prep.log" >&2
    exit 1
fi
echo "[postbuild] prep done; building docker image ..."
sg docker -c "cd ${ISAAC_DIR} && TMPDIR=/workspace/molar/docker-tmp ./tools/docker/build_docker.sh --aarch64" \
    >> /workspace/molar/build-logs/dockerbuild.log 2>&1
if [ $? -ne 0 ]; then
    echo "[postbuild] ERROR: docker build failed — see /workspace/molar/build-logs/dockerbuild.log" >&2
    exit 1
fi
echo "[postbuild] SUCCESS — isaac-sim-docker image built at $(date)"
sg docker -c 'docker images isaac-sim-docker'
