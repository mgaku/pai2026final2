#!/usr/bin/env bash
set -euo pipefail
cd /content
mkdir -p /content/pai_mamba_bin
if [ ! -x /content/pai_mamba_bin/bin/micromamba ]; then
  curl --fail --location --retry 3 --connect-timeout 30 https://micro.mamba.pm/api/micromamba/linux-64/latest -o /content/pai_micromamba.tar.bz2
  tar -xjf /content/pai_micromamba.tar.bz2 -C /content/pai_mamba_bin bin/micromamba
fi
export MAMBA_ROOT_PREFIX=/content/pai_mamba_root
# Use RoboStack's Humble channel. Do NOT add Jammy apt repos to Colab's OS.
# Let ROS package dependencies choose a compatible Python, separate from Colab.
/content/pai_mamba_bin/bin/micromamba create -y -p /content/pai_humble_env --override-channels -c conda-forge -c robostack-humble ros-humble-ros-base ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs ros-humble-std-msgs numpy pillow imageio imageio-ffmpeg pybullet ffmpeg
/content/pai_mamba_bin/bin/micromamba list -p /content/pai_humble_env --explicit > /content/ros2_environment_explicit.txt