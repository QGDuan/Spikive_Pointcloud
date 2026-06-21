#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="${IMAGE_NAME:-pointcloud-pgoba:ros-noetic}"
CONTAINER_NAME="${CONTAINER_NAME:-pointcloud-pgoba-pipeline}"
WORKSPACE_VOLUME="${WORKSPACE_VOLUME:-pointcloud-pgoba-catkin-ws}"
LAUNCH_FILE="${LAUNCH_FILE:-drone_2_guangzhou_pipeline.launch}"

DOCKER_RUN=(
  "${PROJECT_DIR}/docker/run.sh"
  bash
  -lc
)

LAUNCH_ARGS=()

if [ -n "${CONFIG:-}" ]; then
  LAUNCH_ARGS+=("config:=${CONFIG}")
fi

if [ -n "${BAG_PATH:-}" ]; then
  LAUNCH_ARGS+=("bag_path:=${BAG_PATH}")
fi

if [ -n "${OUTPUT_ROOT:-}" ]; then
  LAUNCH_ARGS+=("output_root:=${OUTPUT_ROOT}")
fi

if [ -n "${WORLD_FRAME_ID:-}" ]; then
  LAUNCH_ARGS+=("world_frame_id:=${WORLD_FRAME_ID}")
fi

if [ -n "${STITCH_PCD:-}" ]; then
  LAUNCH_ARGS+=("stitch_pcd:=${STITCH_PCD}")
fi

if [ -n "${MAP_RESOLUTION:-}" ]; then
  LAUNCH_ARGS+=("map_resolution:=${MAP_RESOLUTION}")
fi

if [ -n "${PGO_USE_KEY_FRAME:-}" ]; then
  LAUNCH_ARGS+=("pgo_use_key_frame:=${PGO_USE_KEY_FRAME}")
fi

if [ -n "${PGO_KEY_FRAME_LEN_THRE:-}" ]; then
  LAUNCH_ARGS+=("pgo_key_frame_len_thre:=${PGO_KEY_FRAME_LEN_THRE}")
fi

if [ -n "${PGO_KEY_FRAME_ANG_THRE:-}" ]; then
  LAUNCH_ARGS+=("pgo_key_frame_ang_thre:=${PGO_KEY_FRAME_ANG_THRE}")
fi

if [ "$#" -gt 0 ]; then
  LAUNCH_ARGS+=("$@")
fi

export IMAGE_NAME CONTAINER_NAME WORKSPACE_VOLUME

printf -v LAUNCH_FILE_Q "%q" "${LAUNCH_FILE}"
LAUNCH_ARGS_Q=()
for arg in "${LAUNCH_ARGS[@]}"; do
  printf -v arg_q "%q" "${arg}"
  LAUNCH_ARGS_Q+=("${arg_q}")
done

cd "${PROJECT_DIR}"

"${DOCKER_RUN[@]}" "
set -euo pipefail
export LD_LIBRARY_PATH=\"\${LD_LIBRARY_PATH:-}:/workspace/catkin_ws/third_party/install/lib\"
source /opt/ros/\${ROS_DISTRO}/setup.bash
source /workspace/catkin_ws/devel/setup.bash
roslaunch spikive_pipeline ${LAUNCH_FILE_Q} ${LAUNCH_ARGS_Q[*]}
"
