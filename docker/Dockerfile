ARG BASE_IMAGE=swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/library/ubuntu:20.04
FROM ${BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
ARG UBUNTU_APT_MIRROR=http://mirrors.tuna.tsinghua.edu.cn/ubuntu
ARG ROS_APT_MIRROR=http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu
ARG ROS_KEY_URL=https://cdn.jsdelivr.net/gh/ros/rosdistro@master/ros.asc
ARG ROSDEP_SOURCE_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/rosdep/sources.list.d/20-default.list
ARG ROSDISTRO_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/rosdistro
ARG INSTALL_GTSAM_FROM_SOURCE=0
ARG GTSAM_VERSION=4.1.1

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    TZ=Asia/Hong_Kong \
    ROS_DISTRO=noetic \
    CATKIN_WS=/workspace/catkin_ws

SHELL ["/bin/bash", "-c"]

RUN sed -i "s|http://archive.ubuntu.com/ubuntu/|${UBUNTU_APT_MIRROR}/|g; s|http://security.ubuntu.com/ubuntu/|${UBUNTU_APT_MIRROR}/|g" /etc/apt/sources.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    bash-completion \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    gnupg2 \
    lsb-release \
    locales \
    python3-pip \
    python3-setuptools \
    sudo \
    tzdata \
    unzip \
    wget \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c "echo \"deb ${ROS_APT_MIRROR} $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros1-latest.list" \
    && curl -sSL "${ROS_KEY_URL}" | apt-key add - \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstool \
    python3-wstool \
    && mkdir -p /etc/ros/rosdep/sources.list.d \
    && curl -sSL "${ROSDEP_SOURCE_URL}" -o /etc/ros/rosdep/sources.list.d/20-default.list \
    && sed -i "s|https://raw.githubusercontent.com/ros/rosdistro/master|${ROSDISTRO_MIRROR}|g" /etc/ros/rosdep/sources.list.d/20-default.list \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    gdb \
    libatlas-base-dev \
    libapr1-dev \
    libboost-all-dev \
    libceres-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libopencv-dev \
    libpcl-dev \
    libsuitesparse-dev \
    libtbb-dev \
    libyaml-cpp-dev \
    nano \
    net-tools \
    openssh-client \
    pcl-tools \
    pkg-config \
    python3-dev \
    python3-numpy \
    python3-opencv \
    python3-rospkg \
    python3-scipy \
    ros-noetic-cv-bridge \
    ros-noetic-eigen-conversions \
    ros-noetic-geodesy \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-mavros \
    ros-noetic-mavros-msgs \
    ros-noetic-navigation \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-robot-localization \
    ros-noetic-tf \
    ros-noetic-tf-conversions \
    ros-noetic-tf2 \
    ros-noetic-tf2-eigen \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-velodyne \
    tmux \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "${INSTALL_GTSAM_FROM_SOURCE}" = "1" ]; then \
      apt-get update && apt-get install -y --no-install-recommends unzip \
      && curl -fsSL "https://github.com/borglab/gtsam/archive/refs/tags/${GTSAM_VERSION}.zip" -o /tmp/gtsam.zip \
      && unzip /tmp/gtsam.zip -d /tmp \
      && cmake -S "/tmp/gtsam-${GTSAM_VERSION}" -B /tmp/gtsam-build \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF \
      && cmake --build /tmp/gtsam-build --parallel "$(nproc)" \
      && cmake --install /tmp/gtsam-build \
      && ldconfig \
      && rm -rf /tmp/gtsam.zip "/tmp/gtsam-${GTSAM_VERSION}" /tmp/gtsam-build /var/lib/apt/lists/*; \
    fi

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -s /bin/bash \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN mkdir -p ${CATKIN_WS}/src \
    && chown -R ${USERNAME}:${USERNAME} /workspace

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

USER ${USERNAME}
WORKDIR ${CATKIN_WS}

RUN ROSDISTRO_INDEX_URL="${ROSDISTRO_MIRROR}/index-v4.yaml" rosdep update \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "[ -f ${CATKIN_WS}/devel/setup.bash ] && source ${CATKIN_WS}/devel/setup.bash" >> ~/.bashrc \
    && echo "cd ${CATKIN_WS}" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
