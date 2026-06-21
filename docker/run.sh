#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_VOLUME="${WORKSPACE_VOLUME:-pointcloud-pgoba-catkin-ws}"
CONTAINER_NAME="${CONTAINER_NAME:-pointcloud-pgoba-ros}"
IMAGE_NAME="${IMAGE_NAME:-pointcloud-pgoba:ros-noetic}"

CMD=("$@")
if [ "${#CMD[@]}" -eq 0 ]; then
  CMD=(bash)
fi

TTY_ARGS=()
if [ -t 0 ]; then
  TTY_ARGS=(-it)
fi

PRIVILEGED_ARGS=()
if [ "${DOCKER_PRIVILEGED:-0}" = "1" ]; then
  PRIVILEGED_ARGS=(--privileged)
fi

if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  docker exec "${TTY_ARGS[@]}" "${CONTAINER_NAME}" \
    bash -lc 'source /opt/ros/${ROS_DISTRO}/setup.bash; if [ -f "${CATKIN_WS}/devel/setup.bash" ]; then source "${CATKIN_WS}/devel/setup.bash"; fi; THIRD_PARTY_LIB="${CATKIN_WS}/third_party/install/lib"; if [ -d "${THIRD_PARTY_LIB}" ]; then export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${THIRD_PARTY_LIB}"; fi; exec "$@"' \
    bash "${CMD[@]}"
  exit 0
fi

X11_ARGS=()
if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
  X11_ARGS=(-e DISPLAY="${DISPLAY}" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

docker volume create "${WORKSPACE_VOLUME}" >/dev/null

docker run --rm "${TTY_ARGS[@]}" \
  --name "${CONTAINER_NAME}" \
  --net=host \
  --ipc=host \
  "${PRIVILEGED_ARGS[@]}" \
  -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}" \
  -e ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}" \
  -e LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:/workspace/catkin_ws/third_party/install/lib" \
  -e POINTCLOUD_PGOBA_ROOT="${PROJECT_DIR}" \
  -v "${PROJECT_DIR}:/workspace/project:rw" \
  -v "${PROJECT_DIR}:${PROJECT_DIR}:rw" \
  -v "${WORKSPACE_VOLUME}:/workspace/catkin_ws" \
  "${X11_ARGS[@]}" \
  -w /workspace/catkin_ws \
  "${IMAGE_NAME}" \
  "${CMD[@]}"
