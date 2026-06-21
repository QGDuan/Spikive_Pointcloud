# PointCloud-PGOBA Docker 环境

这里是一套基础隔离的 Ubuntu 20.04 + ROS Noetic Docker 环境，适合 ROS、点云和 SLAM 开发。

## 构建

```bash
cd /home/colman/Project/PointCloud-PGOBA
./docker/build.sh
```

默认已经替换为国内源：

- 基础镜像：华为云 Docker Hub 同步镜像 `ubuntu:20.04`
- Ubuntu apt：清华 TUNA，默认使用 `http`，避免基础镜像首次安装 CA 证书前无法访问 `https`
- ROS apt：清华 TUNA，默认使用 `http`
- rosdep：清华 TUNA rosdistro，构建时会把 rosdep YAML 和 index 都指向国内镜像

ROS 安装步骤仍按官方 Noetic Ubuntu 安装流程：添加 ROS apt 源、导入 ROS key、安装 `ros-noetic-*`、配置 rosdep，只是 URL 换成了国内镜像。

如果某个镜像源不可用，可以复制 `.env.example` 后修改：

```bash
cp .env.example .env
source .env
./docker/build.sh
```

例如切回官方源：

```bash
BASE_IMAGE=ubuntu:20.04 \
UBUNTU_APT_MIRROR=http://archive.ubuntu.com/ubuntu \
ROS_APT_MIRROR=http://packages.ros.org/ros/ubuntu \
ROS_KEY_URL=https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
ROSDEP_SOURCE_URL=https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list \
ROSDISTRO_MIRROR=https://raw.githubusercontent.com/ros/rosdistro/master \
./docker/build.sh
```

Ubuntu 20.04 官方 apt 源默认没有 `libgtsam-dev`，所以当前基础镜像默认不安装 GTSAM。需要 GTSAM 时可以源码编译进镜像：

```bash
INSTALL_GTSAM_FROM_SOURCE=1 ./docker/build.sh
```

## 启动

```bash
cd /home/colman/Project/PointCloud-PGOBA
./docker/run.sh
```

容器内启动 ROS master：

```bash
roscore
```

另开一个终端进入同一个容器：

```bash
cd /home/colman/Project/PointCloud-PGOBA
./docker/run.sh
```

如果容器已经在运行，`docker/run.sh` 会自动进入已有容器，不会再创建第二个同名容器。

默认不启用 `--privileged`。如果需要直接访问雷达、USB、CAN 等宿主机硬件，可以按需开启：

```bash
DOCKER_PRIVILEGED=1 ./docker/run.sh
```

## 工作区

- 宿主机项目目录：`/home/colman/Project/PointCloud-PGOBA`
- 容器内项目目录：`/workspace/project`
- 容器内宿主机同名目录：`/home/colman/Project/PointCloud-PGOBA`
- 容器内 catkin 工作区：`/workspace/catkin_ws`
- catkin 工作区保存在 Docker volume：`pointcloud-pgoba-catkin-ws`

编译 ROS 包时，把包放到或链接到 `/workspace/catkin_ws/src`，然后运行：

```bash
catkin config --extend /opt/ros/noetic
catkin build
source devel/setup.bash
```

如果源码就在 `/workspace/project`，可以在容器内建立软链接：

```bash
ln -s /workspace/project /workspace/catkin_ws/src/pointcloud_pgoba
catkin build
```

`docker/run.sh` 会同时把项目挂载到 `/workspace/project` 和宿主机同名路径。
因此 pipeline 的输入/输出可以直接写宿主机路径，例如：

```yaml
input:
  bag_path: /home/colman/Project/PointCloud-PGOBA/input/shouchiliaochang.bag

output:
  root: /home/colman/Project/PointCloud-PGOBA/output/shouchiliaochang
```

如果配置里写相对路径，例如 `input/shouchiliaochang.bag` 或 `output/shouchiliaochang`，
现在会按项目根目录 `/home/colman/Project/PointCloud-PGOBA` 解析，而不是按
`/workspace/catkin_ws/src/spikive_pipeline` 解析。

## 编译 driver_ws

`driver_ws` 里的包可以直接用外部脚本编译，不需要改源码目录：

```bash
cd /home/colman/Project/PointCloud-PGOBA
./docker/build_driver_ws.sh
```

脚本会把项目目录以只读方式挂载到容器内，并把 catkin 工作区、构建产物和额外第三方库放在 Docker volume `pointcloud-pgoba-catkin-ws` 中。它会在容器工作区里软链接 `driver_ws/src` 下的包，并为当前 driver 预装匹配的 GTSAM、Ceres 和 Livox-SDK，避免向 `driver_ws` 写入 `build`、`devel` 或依赖源码。

如需清理这套独立编译环境：

```bash
docker rm -f pointcloud-pgoba-driver-build 2>/dev/null || true
docker volume rm pointcloud-pgoba-catkin-ws
```

## RViz / GUI

宿主机允许 Docker 访问 X11：

```bash
xhost +local:docker
```

进入容器后运行：

```bash
rviz
```

## 隔离说明

这套配置不会在宿主机安装 ROS、Python 包或 apt 包。只有当前项目目录会以读写方式挂载到容器内，catkin 编译产物保存在 Docker volume 中。

默认不会挂载宿主机的 `.ssh`、Python 环境、ROS 环境或系统目录。

删除容器和 catkin 编译 volume：

```bash
docker rm -f pointcloud-pgoba-ros 2>/dev/null || true
docker volume rm pointcloud-pgoba-catkin-ws
```

## 可选 Docker Compose

如果系统安装了 Docker Compose 插件，也可以使用 `docker-compose.yml`：

```bash
UID=$(id -u) GID=$(id -g) docker compose build
UID=$(id -u) GID=$(id -g) docker compose run --rm ros-noetic
```
