#!/usr/bin/env bash
set -euo pipefail

# Supported/tested path for v0.13: Ubuntu 26.04 (Resolute) + ROS 2 Lyrical.
# This script intentionally does not alter Windows or install a container engine.
if [[ "$(. /etc/os-release && echo "$VERSION_CODENAME")" != "resolute" ]]; then
  echo "Expected Ubuntu 26.04 (resolute). See docs/v0.13-ros2-bridge.md for alternatives." >&2
  exit 2
fi

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y curl locales software-properties-common
"${SUDO[@]}" add-apt-repository -y universe
ros_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p')"
curl -fsSL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.resolute_all.deb"
"${SUDO[@]}" dpkg -i /tmp/ros2-apt-source.deb
"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y python3-venv ros-dev-tools ros-lyrical-ros-base

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m venv --system-site-packages "${repo_root}/.venv-ros2"
"${repo_root}/.venv-ros2/bin/python" -m pip install -e "${repo_root}"
echo "ROS 2 Lyrical ready. Source /opt/ros/lyrical/setup.bash and .venv-ros2/bin/activate."
