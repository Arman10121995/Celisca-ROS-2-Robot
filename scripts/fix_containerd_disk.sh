#!/bin/bash
# EMERGENCY: relocate containerd image store off the full eMMC onto the SSD.
# Docker 29 uses the containerd snapshotter whose image store lives in
# /var/lib/containerd — separate from docker's data-root (already on SSD).
# The nvcr.io base image pull filled the 57GB eMMC.  Run with sudo.
set -e
echo "=== stopping docker + containerd ==="
systemctl stop docker docker.socket containerd

echo "=== moving /var/lib/containerd -> /workspace/molar/containerd-data ==="
mv /var/lib/containerd /workspace/molar/containerd-data
ln -s /workspace/molar/containerd-data /var/lib/containerd

echo "=== restarting ==="
systemctl start containerd docker
sleep 2
df -h / | tail -1
echo "DONE — eMMC freed. Re-run the docker build:"
echo "  cd /workspace/molar/installs/IsaacSim-6.0.1"
echo "  TMPDIR=/workspace/molar/docker-tmp ./tools/docker/build_docker.sh --aarch64"
