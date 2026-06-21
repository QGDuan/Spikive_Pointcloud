#!/usr/bin/env bash
set -e

ENTRYPOINT_ARGS=("$@")
set --

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f "${CATKIN_WS}/devel/setup.bash" ]; then
  source "${CATKIN_WS}/devel/setup.bash"
fi

THIRD_PARTY_LIB="${CATKIN_WS}/third_party/install/lib"
if [ -d "${THIRD_PARTY_LIB}" ]; then
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${THIRD_PARTY_LIB}"
fi

exec "${ENTRYPOINT_ARGS[@]}"
