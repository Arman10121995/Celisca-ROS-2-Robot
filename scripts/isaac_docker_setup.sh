#!/bin/bash
# Isaac Sim Docker setup for bumperbot_ws (Jetson/ARM64)
# Keeps ALL Docker storage on the 1TB SSD (/workspace), not the 64GB eMMC.
# Run with:  sudo bash /workspace/molar/isaac_docker_setup.sh
set -e

SSD_DOCKER_DATA=/workspace/molar/docker-data

echo "=== 1. Docker data-root -> SSD (${SSD_DOCKER_DATA}) ==="
mkdir -p "${SSD_DOCKER_DATA}"
cat > /etc/docker/daemon.json <<EOF
{
  "data-root": "${SSD_DOCKER_DATA}",
  "default-address-pools": [{"base": "10.200.0.0/16", "size": 24}]
}
EOF

echo "=== 2. Restart Docker to apply new data-root ==="
systemctl restart docker

echo "=== 3. Add user molar1 to docker group ==="
usermod -aG docker molar1

echo "=== 4. Install Isaac Sim build prerequisites ==="
apt-get update
apt-get install -y libx11-dev xorg-dev

echo "=== 5. NVIDIA Container Toolkit (for GPU inside containers) ==="
if ! command -v nvidia-ctk >/dev/null 2>&1; then
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    > /etc/apt/sources.list.d/nvidia-container-toolkit.list
  apt-get update
  apt-get install -y nvidia-container-toolkit
  nvidia-ctk runtime configure --runtime=docker
  systemctl restart docker
else
  echo "nvidia-container-toolkit already installed"
fi

echo "=== Verify ==="
docker info 2>/dev/null | grep -i 'docker root dir'
echo "DONE. User molar1 must log out & back in (or run: newgrp docker) for group membership."
