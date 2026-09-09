#!/usr/bin/env bash
# One-time host setup for Drone_Sim. Run with: bash scripts/host_setup.sh
# Requires sudo. Does exactly two things:
#   1. adds you to the docker group   (removes the need for `sudo docker`)
#   2. installs nvidia-container-toolkit (gives containers the RTX 2080 SUPER)
set -euo pipefail

echo "==> 1/2  docker group"
if id -nG "$USER" | tr ' ' '\n' | grep -qx docker; then
  echo "    already a member, skipping"
else
  sudo usermod -aG docker "$USER"
  echo "    added. NOTE: log out and back in (or run 'newgrp docker') to take effect."
fi

echo "==> 2/2  nvidia-container-toolkit"
if command -v nvidia-ctk >/dev/null 2>&1; then
  echo "    already installed: $(nvidia-ctk --version | head -1)"
else
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y nvidia-container-toolkit
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker
  echo "    installed and docker restarted."
fi

echo
echo "==> verifying GPU passthrough into a container"
if docker run --rm --gpus all ubuntu:24.04 nvidia-smi -L 2>/dev/null; then
  echo "    GPU passthrough OK"
else
  echo "    !! GPU test failed. If you have not re-logged in yet, try: sudo docker run --rm --gpus all ubuntu:24.04 nvidia-smi -L"
fi
