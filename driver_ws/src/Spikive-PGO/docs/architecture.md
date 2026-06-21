# LSDC PGO / Loop 系统架构

本文档记录 `lsdc_loop` 当前代码的回环检测、实时图优化和离线 PGO 架构。当前目录名是 `lsdc_loop`，ROS package 名是 `lsdc_pgo`。线上仓库记录为 `git@github.com:QGDuan/Spikive-PGO.git`；本地 git remote 可能仍指向旧内网地址，未在本文档工作中修改。

## 总览

`lsdc_loop` 基于 STD (Stable Triangle Descriptor) 做 3D place recognition，并用 GTSAM/ISAM2 做 pose graph optimization。当前 package 主要有两个可执行文件：

| 可执行文件 | 源文件 | 说明 |
| --- | --- | --- |
| `std_loop` | `src/std_loop.cpp` | 当前主入口，支持实时回环、从文件预处理、离线 PGO 三类流程 |
| `std_pgo` | `src/std_pgo.cpp` | 较早/独立的 PGO 实现，launch 中当前主要使用 `std_loop` |

配套架构图见 [pgo_loop.drawio](pgo_loop.drawio)。

## 主要模块

| 模块 | 位置 | 职责 |
| --- | --- | --- |
| STD descriptor | `include/STDesc.*` | 从点云中提取平面/角点并生成稳定三角描述子 |
| online pair sync | `std_loop.cpp` | 按时间戳配对 cloud、odom、GPS |
| keyframe selection | `std_loop.cpp` | 根据平移/角度阈值选关键帧 |
| loop detection | `std_loop.cpp` + `STDescManager` | 搜索候选回环并生成 loop factor |
| graph optimization | `std_loop.cpp` | GTSAM BetweenFactor、GPS factor、ISAM2 更新 |
| file preprocess | `src/file_preprocess.hpp` | 离线读取 bag/pose/GPS 输入 |
| visualization/output | `std_loop.cpp` | 发布校正点云、PGO map、路径、GPS 点 |

## 运行模式

### 实时回环模式

`std_loop.launch` 默认启动 `std_loop`，并设置：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `run_pgo` | 未在 `std_loop.launch` 设置，代码默认 `false` | `false` 时进入实时 loop |
| `use_loop` | `true` | 是否使用回环优化 |
| `use_gps` | `false` | 是否加入 GPS factor |
| `use_file` | `false` | 是否先从文件预处理历史数据 |
| `cloud_topic` | `/cloud_registered_body` | 当前帧 body 系点云 |
| `odom_topic` | `/Odometry` in launch，代码默认 `/Odometry_enu` | pose 输入 |
| `gps_topic` | `/rtk_odom` in launch，代码默认 `/rtk_odom_enu` | RTK/GPS 输入 |

实时模式会订阅 cloud、odom、GPS，将同步后的帧进入关键帧筛选、STD 描述子构建、回环检测与图优化。

### 离线 PGO 模式

`std_pgo.launch` 当前实际启动的是 `std_loop`，并设置：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `run_pgo` | `true` | 进入离线 PGO 分支 |
| `use_gps` | `true` | 使用 GPS factor |
| `cloud_topic` | `/cloud_registered_body` | 离线 bag 内 cloud topic |
| `gps_topic` | `/rtk_odom_enu` | 离线 GPS topic |
| `input_bag_names` | 多 bag 名用 `-` 串联 | 读取 `${name}_cloud.bag` |
| `input_pose_names` | `pgo` | 读取 `${name}_pose.txt` |
| `input_path` | 数据目录 | bag/pose 所在目录 |

离线模式会读取文件构建因子图，优化完成后发布结果，并保存 PCD map。

### 文件预处理模式

当 `use_file=true` 时，`std_loop` 的 `preprocessStd()` 会：

1. 从 `input_path` 读取 `${input_pose_name}_pose.txt`。
2. 从 `${input_bag_name}_cloud.bag` 读取 `/cloud_registered_body`。
3. 从 bag 中读取 `/rtk_odom` 并进入 GPS 预处理。
4. 按时间戳配对 cloud 与 pose。
5. 生成 STD 描述子、loop 文件、pose 文件和优化结果。

注意：`preprocessStd()` 内部读取 bag topic 时仍硬编码 `/cloud_registered_body` 和 `/rtk_odom`，不完全受 launch 中 `cloud_topic`、`gps_topic` 参数控制。

## 与 SLAM 的关系

PGO 不是直接替代 LIO，而是消费 LIO/RTK 结果：

```text
lsdc_slam
  -> /cloud_registered_body
  -> /Odometry_enu or /Odometry
  -> /rtk_odom_enu or /rtk_odom
  -> lsdc_pgo/std_loop
  -> /pgo_map + /pgo_path + optimized pose/map files
```

当前默认更偏离线建图/回环工作流：

- `std_loop.cpp` 代码默认 odom 为 `/Odometry_enu`，GPS 为 `/rtk_odom_enu`。
- `std_loop.launch` 中显式设置 odom 为 `/Odometry`、GPS 为 `/rtk_odom`。
- `std_pgo.launch` 中 GPS 为 `/rtk_odom_enu`，但没有显式设置 `odom_topic`。
- 无人机在线主链路输出的是 `/drone_{id}_visual_slam/odom` 与 `/drone_{id}_cloud_registered`，还没有直接适配 PGO 默认输入。

因此在接入无人机在线 PGO 前，需要先明确使用哪条链路：

| 方案 | 说明 |
| --- | --- |
| 直接消费 LIO 原始输出 | 使用 `/cloud_registered_body` + `/Odometry`，RTK 可选 `/rtk_odom` |
| 消费 ENU 化输出 | 先运行 `lsdc_save_result` 或等价转换，使用 `/Odometry_enu` + `/rtk_odom_enu` |
| 消费 Spikive remap 输出 | 需要新增适配或 remap，因为地面站 topic 是 `world` 系，不是 PGO 默认的 `camera_init/body/ENU` 语义 |

## 输出

| Topic / 文件 | 说明 |
| --- | --- |
| `/aft_mapped_to_init` | 当前校正后 odom |
| `/cloud_current` | 当前源点云 |
| `/cloud_key_points` | 当前关键点 |
| `/cloud_matched` | 匹配目标点云 |
| `/cloud_matched_key_points` | 匹配关键点 |
| `descriptor_line` | STD 匹配可视化 marker |
| `/cloud_correct` | 校正后点云 |
| `/odom_corrected` | 校正后 odom |
| `/pgo_map` | 优化后地图 |
| `/curr_cloud_correct` | 当前校正点云 |
| `/lio_path` | LIO 原始轨迹 |
| `/pgo_path` | PGO 优化轨迹 |
| `/gps_path` | GPS 轨迹 |
| `/gps_points` | GPS 点集合 |
| `/gps_use_points` | 实际用于优化的 GPS 点 |
| `${save_name}_pose.txt` | 优化/保存 pose |
| `${save_name}_loop.txt` | 回环约束 |
| `${save_name}_map.pcd` | 优化后地图 |

## 主要风险

- package 目录名和 ROS package 名不同：部署命令使用 `lsdc_pgo`。
- `std_loop.launch` 和 `std_loop.cpp` 的默认 odom/GPS topic 不完全一致，排查时以实际 rosparam 为准。
- `preprocessStd()` 有硬编码 bag topic，离线数据命名必须符合现状。
- PGO 输出 frame 多处写死为 `camera_init`，接入 `world` 或 Spikive topic 前必须先做坐标系设计。
- 当前 PGO 尚未形成与无人机在线 topic 完全一致的数据流，文档中应持续标注适配边界。

