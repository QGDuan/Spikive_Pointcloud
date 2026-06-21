#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

docker build \
  --build-arg BASE_IMAGE="${BASE_IMAGE:-swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/library/ubuntu:20.04}" \
  --build-arg UBUNTU_APT_MIRROR="${UBUNTU_APT_MIRROR:-http://mirrors.tuna.tsinghua.edu.cn/ubuntu}" \
  --build-arg ROS_APT_MIRROR="${ROS_APT_MIRROR:-http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu}" \
  --build-arg ROS_KEY_URL="${ROS_KEY_URL:-https://cdn.jsdelivr.net/gh/ros/rosdistro@master/ros.asc}" \
  --build-arg ROSDEP_SOURCE_URL="${ROSDEP_SOURCE_URL:-https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/sources.list.d/20-default.list}" \
  --build-arg ROSDISTRO_MIRROR="${ROSDISTRO_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn/rosdistro}" \
  --build-arg INSTALL_GTSAM_FROM_SOURCE="${INSTALL_GTSAM_FROM_SOURCE:-0}" \
  --build-arg GTSAM_VERSION="${GTSAM_VERSION:-4.1.1}" \
  --build-arg USER_UID="$(id -u)" \
  --build-arg USER_GID="$(id -g)" \
  --build-arg USERNAME=ros \
  -t pointcloud-pgoba:ros-noetic \
  -f "${PROJECT_DIR}/docker/Dockerfile" \
  "${PROJECT_DIR}"
