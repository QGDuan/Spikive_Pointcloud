#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="${IMAGE_NAME:-pointcloud-pgoba:ros-noetic}"
CONTAINER_NAME="${CONTAINER_NAME:-pointcloud-pgoba-driver-build}"
WORKSPACE_VOLUME="${WORKSPACE_VOLUME:-pointcloud-pgoba-catkin-ws}"

CMD=(
  bash
  -lc
  '
set -euo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"

PROJECT_DIR=/workspace/project
SRC_ROOT="${PROJECT_DIR}/driver_ws/src"
LIB_ROOT="${PROJECT_DIR}/lib"
THIRD_PARTY="${CATKIN_WS}/third_party"
PREFIX="${THIRD_PARTY}/install"
PGO_PREFIX="${THIRD_PARTY}/gtsam-4.0.2"
HBA_PREFIX="${THIRD_PARTY}/gtsam-4.1"
BUILD_ROOT="${THIRD_PARTY}/build"

mkdir -p "${CATKIN_WS}/src" "${PREFIX}" "${PGO_PREFIX}" "${HBA_PREFIX}" "${BUILD_ROOT}"

# Mixed GTSAM mode keeps GTSAM out of the shared dependency prefix.  Older
# builds installed 4.1 under ${PREFIX}; if those headers remain there, Ceres
# include path can make PGO compile against 4.1 headers while linking 4.0.2.
rm -rf "${PREFIX}/include/gtsam" \
       "${PREFIX}/lib/cmake/GTSAM" \
       "${PREFIX}/lib/cmake/GTSAMCMakeTools" \
       "${PREFIX}/lib/libgtsam"* \
       "${PREFIX}/lib/libmetis-gtsam"*

if [ -d "${SRC_ROOT}/livox_ros_driver/livox_ros_driver/Livox-SDK" ]; then
  echo "错误：源码目录里存在 livox_ros_driver/Livox-SDK。请先清理它，避免污染 driver_ws。" >&2
  exit 1
fi

has_gtsam_config() {
  local prefix="$1"
  [ -f "${prefix}/lib/cmake/GTSAM/GTSAMConfig.cmake" ] || \
    [ -f "${prefix}/lib/cmake/GTSAM/gtsam-config.cmake" ]
}

install_gtsam_4_1() {
  if has_gtsam_config "${HBA_PREFIX}"; then
    return
  fi

  echo "[driver_ws] building GTSAM 4.1.0 into ${HBA_PREFIX}"
  rm -rf "${BUILD_ROOT}/gtsam-4.1.0" "${BUILD_ROOT}/gtsam-4.1-build"
  unzip -q "${LIB_ROOT}/gtsam-4.1.0.zip" -d "${BUILD_ROOT}"
  cmake -S "${BUILD_ROOT}/gtsam-4.1.0" -B "${BUILD_ROOT}/gtsam-4.1-build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${HBA_PREFIX}" \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_PYTHON=OFF
  cmake --build "${BUILD_ROOT}/gtsam-4.1-build" --parallel "${BUILD_JOBS:-$(nproc)}"
  cmake --install "${BUILD_ROOT}/gtsam-4.1-build"
}

install_gtsam_4_0_2() {
  if has_gtsam_config "${PGO_PREFIX}"; then
    return
  fi

  echo "[driver_ws] building GTSAM 4.0.2 into ${PGO_PREFIX}"
  rm -rf "${BUILD_ROOT}/gtsam-4.0.2-build"
  cmake -S "${LIB_ROOT}/gtsam-4.0.2" -B "${BUILD_ROOT}/gtsam-4.0.2-build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${PGO_PREFIX}" \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF \
    -DGTSAM_BUILD_WRAP=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_PYTHON=OFF
  cmake --build "${BUILD_ROOT}/gtsam-4.0.2-build" --parallel "${BUILD_JOBS:-$(nproc)}"
  cmake --install "${BUILD_ROOT}/gtsam-4.0.2-build"
}

install_ceres_if_needed() {
  local current_version
  current_version="$(pkg-config --modversion ceres 2>/dev/null || true)"
  if [ -n "${current_version}" ]; then
    case "${current_version}" in
      2.*|3.*) return ;;
    esac
  fi

  if [ -f "${PREFIX}/lib/cmake/Ceres/CeresConfig.cmake" ]; then
    return
  fi

  echo "[driver_ws] building Ceres 2.1.0 into ${PREFIX}"
  rm -rf "${BUILD_ROOT}/ceres-solver-2.1.0" "${BUILD_ROOT}/ceres-build"
  unzip -q "${LIB_ROOT}/ceres-solver-2.1.0.zip" -d "${BUILD_ROOT}"
  cmake -S "${BUILD_ROOT}/ceres-solver-2.1.0" -B "${BUILD_ROOT}/ceres-build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_BENCHMARKS=OFF \
    -DBUILD_DOCUMENTATION=OFF \
    -DMINIGLOG=OFF
  cmake --build "${BUILD_ROOT}/ceres-build" --parallel "${BUILD_JOBS:-$(nproc)}"
  cmake --install "${BUILD_ROOT}/ceres-build"
}

install_livox_sdk() {
  if [ -f "${PREFIX}/lib/liblivox_sdk_static.a" ] && [ -f "${PREFIX}/include/livox_sdk.h" ]; then
    sudo install -d /usr/local/lib /usr/local/include
    sudo ln -sfn "${PREFIX}/lib/liblivox_sdk_static.a" /usr/local/lib/liblivox_sdk_static.a
    sudo ln -sfn "${PREFIX}/include/livox_sdk.h" /usr/local/include/livox_sdk.h
    return
  fi

  echo "[driver_ws] building Livox-SDK into ${PREFIX}"
  rm -rf "${BUILD_ROOT}/Livox-SDK" "${BUILD_ROOT}/Livox-SDK-build"
  git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK.git "${BUILD_ROOT}/Livox-SDK"
  cmake -S "${BUILD_ROOT}/Livox-SDK" -B "${BUILD_ROOT}/Livox-SDK-build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}"
  cmake --build "${BUILD_ROOT}/Livox-SDK-build" --parallel "${BUILD_JOBS:-$(nproc)}"
  cmake --install "${BUILD_ROOT}/Livox-SDK-build"
  sudo install -d /usr/local/lib /usr/local/include
  sudo ln -sfn "${PREFIX}/lib/liblivox_sdk_static.a" /usr/local/lib/liblivox_sdk_static.a
  sudo ln -sfn "${PREFIX}/include/livox_sdk.h" /usr/local/include/livox_sdk.h
}

install_gtsam_4_1
install_gtsam_4_0_2
install_ceres_if_needed
install_livox_sdk

ln -sfn "${SRC_ROOT}/HBA" "${CATKIN_WS}/src/HBA"
ln -sfn "${SRC_ROOT}/Spikive-PGO" "${CATKIN_WS}/src/Spikive-PGO"
ln -sfn "${SRC_ROOT}/Spikive-SLAM" "${CATKIN_WS}/src/Spikive-SLAM"
ln -sfn "${SRC_ROOT}/livox_ros_driver/livox_ros_driver" "${CATKIN_WS}/src/livox_ros_driver"
ln -sfn "${SRC_ROOT}/spikive_pipeline" "${CATKIN_WS}/src/spikive_pipeline"

export CMAKE_PREFIX_PATH="${PGO_PREFIX}:${HBA_PREFIX}:${PREFIX}:${CMAKE_PREFIX_PATH:-}"
export CMAKE_LIBRARY_PATH="${PGO_PREFIX}/lib:${HBA_PREFIX}/lib:${PREFIX}/lib:${CMAKE_LIBRARY_PATH:-}"
export CMAKE_INCLUDE_PATH="${PGO_PREFIX}/include:${HBA_PREFIX}/include:${PREFIX}/include:${CMAKE_INCLUDE_PATH:-}"
export LD_LIBRARY_PATH="${PGO_PREFIX}/lib:${HBA_PREFIX}/lib:${PREFIX}/lib:${LD_LIBRARY_PATH:-}"
export LIBRARY_PATH="${PGO_PREFIX}/lib:${HBA_PREFIX}/lib:${PREFIX}/lib:${LIBRARY_PATH:-}"

catkin config --extend /opt/ros/noetic \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_MODULE_PATH=/usr/share/cmake/geographiclib \
    -DGTSAM_4_0_2_DIR="${PGO_PREFIX}/lib/cmake/GTSAM" \
    -DGTSAM_4_1_DIR="${HBA_PREFIX}/lib/cmake/GTSAM" \
    -DCeres_DIR="${PREFIX}/lib/cmake/Ceres"

catkin build --summarize "$@"
'
  bash
)

if [ "$#" -gt 0 ]; then
  CMD+=("$@")
fi

TTY_ARGS=()
if [ -t 0 ]; then
  TTY_ARGS=(-it)
fi

PRIVILEGED_ARGS=()
if [ "${DOCKER_PRIVILEGED:-0}" = "1" ]; then
  PRIVILEGED_ARGS=(--privileged)
fi

if docker ps --format "{{.Names}}" | grep -qx "${CONTAINER_NAME}"; then
  docker exec "${TTY_ARGS[@]}" "${CONTAINER_NAME}" "${CMD[@]}"
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
  -v "${PROJECT_DIR}:/workspace/project:ro" \
  -v "${WORKSPACE_VOLUME}:/workspace/catkin_ws" \
  "${X11_ARGS[@]}" \
  -w /workspace/catkin_ws \
  "${IMAGE_NAME}" \
  "${CMD[@]}"
