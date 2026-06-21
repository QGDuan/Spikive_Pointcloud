# LSDC SLAM 系统架构

本文档记录 `lsdc_eeds_slam` 当前代码的系统边界、节点职责、部署链路和主要风险。当前目录名是 `lsdc_eeds_slam`，ROS package 名是 `lsdc_slam`。线上仓库记录为 `git@github.com:QGDuan/Spikive-SLAM.git`；本地 git remote 可能仍指向旧内网地址，未在本文档工作中修改。

## 总览

这套 SLAM 是基于 FastLIO2 的无人机部署版本，保留 LIO 前端主体，同时增加了 Localization、RTK、外部 INS570G/GNSS 预处理和无人机/地面站适配。当前架构整体可分为五层：

| 层级 | 主要节点 | 主要职责 |
| --- | --- | --- |
| 传感器预处理 | `lsdc_ins_preprocess`, `rs_velodyne` | 同步/重发布 INS GPS+IMU，或把 Robosense 点云转换到 Velodyne/Livox 兼容链路 |
| LIO 建图 | `lsdc_mapping` | 读取多雷达/IMU，输出 FastLIO2 风格里程计、`camera_init` 下点云和 body 系点云 |
| RTK 初始化 | `lsdc_rtk2pose`, `lsdc_repub_gnss` | 将 WGS84/RTK 姿态转换到本地里程计，保留为 RTK/实验链路 |
| Motion Control 标定适配 | `lsdc_flight_controller` | 把 LIO 输出按标定 R/T 转成运动中心的 fallback startup world，发布 `/Odometry_trans`、`/cloud_registered_trans` |
| Localization / world 收口 | `lsdc_global_match`, `lsdc_fusion_repub` | 用 WayPoint 地图和前端 initialpose 做 ICP；成功前输出 startup world，成功后输出 WayPoint 地图 world |

配套架构图见 [slam_system.drawio](slam_system.drawio)。

## Package 与可执行文件

| 可执行文件 | 源文件 | 说明 |
| --- | --- | --- |
| `lsdc_mapping` | `src/LIO/laserMapping.cpp` | LIO 主节点，FastLIO2 派生逻辑 |
| `lsdc_ins_preprocess` | `src/preprocess/ins_preprocess.cpp` | GPS/IMU 时间同步，发布 `/lsdc_rtk` |
| `rs_velodyne` | `src/preprocess/rs_to_velodyne.cpp` | Robosense 点云转 `/velodyne_points` |
| `lsdc_rtk2pose` | `src/RTK/rtk2pose.cpp` | RTK LLA/姿态转本地 pose 和初始位姿 |
| `lsdc_repub_gnss` | `src/RTK/repub_gnss.cpp` | Localization 成功后把本地定位反算成经纬度 |
| `lsdc_global_match` | `src/localization/global_match.cpp` | WayPoint 地图 ICP 匹配，输出 `/drone_{id}_diff_odom`、`/drone_{id}_init_match_success` 和匹配状态 |
| `lsdc_fusion_repub` | `src/localization/fusion_repub.cpp` | 统一发布 `/drone_*`、MAVROS pose、companion status；根据匹配状态选择 fallback 或 map world |
| `lsdc_flight_controller` | `src/localization/flight_controller.cpp` | Motion Control / calibration adapter，输出 `/Odometry_trans`、`/cloud_registered_trans` |
| `lsdc_save_result` | `src/LIO/save_result.cpp` | 保存/重发布 ENU 化 odom、RTK、bag 和 map |

## Launch 模式

| launch | 当前用途 | 关键节点 |
| --- | --- | --- |
| `launch/mapping_livox.launch` | 无人机在线建图/运行主入口 | `lsdc_mapping`, `lsdc_flight_controller`, `lsdc_fusion_repub` |
| `launch/localization.launch` | Spikive Localization 匹配入口 | `lsdc_global_match` |
| `launch/uav.launch` | 仅启动 UAV transform 适配 | `lsdc_flight_controller` |
| `launch/mapping_velodyne.launch` | Velodyne/标准点云建图入口 | `lsdc_mapping` |

`mapping_livox.launch` 中的 Spikive stable topic 由 `fusion_repub` 发布：

| topic | 发布者 |
| --- | --- |
| `/drone_{drone_id}_visual_slam/odom` | `fusion_repub` |
| `/drone_{drone_id}_cloud_registered` | `fusion_repub` |
| `/mavros/vision_pose/pose` | `fusion_repub` |
| `/mavros/companion_process/status` | `fusion_repub` |

Spikive Manager 当前配置中 LIO 启动命令是 `roslaunch lsdc_slam mapping_livox.launch`，并检查 `/laserMapping` 和 `/slam_to_uav_transform` 是否就绪。

## 核心数据链路

### 建图链路

1. `lsdc_mapping` 通过 `common/imu_topic` 读取 IMU，默认配置为 `/livox/imu/`。
2. 点云输入由 `pcl_preprocess.hpp` 根据启动参数如 `mid360` 读取对应前缀配置，例如 `mid360_common/lid_topic`。
3. 点云先按每个雷达外参变换到主雷达/IMU 相关坐标，再进入 FastLIO2 IKF 和 ikd-tree 地图更新。
4. 输出：
   - `/Odometry`: `camera_init -> body`
   - `/cloud_registered`: `camera_init` 下的当前帧点云
   - `/cloud_registered_body`: `body` 下的去畸变点云
   - `/path`: `camera_init` 下轨迹

### Localization / world 收口链路

1. WayPoint 后端加载路线后发布 latched `/drone_{id}_localization_pcl`，该点云定义最终地图 `world`。
2. 前端原生 Pose Estimate 发布 `/drone_{id}_initialpose`，表示当前无人机在 WayPoint 地图 `world` 下的初始猜测。
3. `lsdc_global_match` 订阅 `/cloud_registered_trans`、`/Odometry_trans`、地图点云和 initialpose，将 initialpose 换算成 `T_diff_guess = T_initial_world * inverse(T_calibrated_current)`。
4. ICP 成功连续达到确认次数后，`lsdc_global_match` 发布 `/drone_{id}_diff_odom` 与 `/drone_{id}_init_match_success=true`，并通过 `/drone_{id}_localization_match_status` 发布 `localized`。
5. `lsdc_fusion_repub` 在未 localized 时直接转发 calibrated fallback world；localized 后发布 `diff_odom * calibrated` 的 WayPoint 地图 `world`。

匹配状态：

| state | 含义 |
| --- | --- |
| `idle` | 未加载有效地图或收到空地图 |
| `map_loaded` / `waiting_initialpose` | 地图已加载，等待前端 initialpose |
| `matching` | 正在基于当前 initialpose 做 ICP，状态 topic 按配置周期发布 attempt/elapsed/fitness |
| `localized` | 匹配成功，stable topic 已对齐 WayPoint 地图 world |
| `failed` | 本次 initialpose 超时或失败，停止 ICP，等待重新发布 initialpose |

### RTK/INS 链路

1. `lsdc_ins_preprocess` 订阅 `ins/gps_topic` 和 `ins/imu_topic`，默认 `/rtk_gps`、`/rtk_imu`。
2. GPS 与 IMU 按时间戳近邻同步后发布 `/lsdc_rtk`，消息类型为 `nav_msgs/Odometry`，position 存 LLA，orientation 存 INS 姿态，`child_frame_id` 表示 RTK 状态。
3. `lsdc_rtk2pose` 订阅 `/lsdc_rtk`，用 `lsdc_geo.hpp` 中的原点和外参转换到本地 pose。
4. 初始化成功后发布：
   - `/initial_pose`
   - `/rtk_odom`
   - `/rtk_path`

### Motion Control 与地面站链路

1. `lsdc_flight_controller` 订阅 `/Odometry` 和 `/cloud_registered`。
2. 通过全局参数 `R`、`T` 和私有参数 `init_x/y/z` 输出运动中心 fallback startup world。
3. 发布：
   - `/Odometry_trans`
   - `/cloud_registered_trans`
4. `fusion_repub` 根据 `/drone_{id}_init_match_success` 统一输出 Spikive-Lichtblick 当前约定：
   - `/drone_{id}_visual_slam/odom`
   - `/drone_{id}_cloud_registered`
   - `/mavros/vision_pose/pose`
   - `/mavros/companion_process/status`

## 与 Spikive 系统的接口

| 系统 | 约定 |
| --- | --- |
| Spikive Manager | 每架无人机本地启动，topic 形如 `/drone_{id}_auto_manager_status` 和 `/drone_{id}_command_topic` |
| Spikive-Lichtblick | 地面站订阅 `/drone_{id}_cloud_registered`、`/drone_{id}_visual_slam/odom`，跟随 frame 通常为 `base{id}`；前端不推导是否 localized |
| Planner / odom_visualization | 通常提供 `/drone_{id}_odom_visualization/robot`、`/drone_{id}_odom_visualization/path` 和 `world -> base{id}` |
| WayPoint | 发布 `/drone_{id}_localization_pcl`，其地图 world 是最终权威坐标 |
| SLAM | 负责提供视觉里程计、当前点云和 Localization 状态；不直接管理 planner 的 TF frame |

## 已知架构风险

- `flight_controller.cpp` 中 odom 使用 `D * lio * D^-1` 的 SE3 共轭变换，点云使用 `R*p + T + init_translation`。`D` 的 `R/T` 明确定义为雷达系到运动中心外参，非法 R/T 会让节点退出。
- `world` 在未 localized 时是 fallback startup world，localized 后才是 WayPoint 地图 world。EGO、MAVROS 和前端 topic 不变，但上层如果需要判断是否对齐地图，应读取 `/drone_{id}_localization_match_status` 或 `/drone_{id}_init_match_success`。
- `/drone_{id}_odom_visualization/path` 不由 SLAM 发布；path 属于 EGO/odom_visualization，基于 stable odom 生成。
- `localization/origin_Q` 缺省时使用 `kRot90.inverse()`，与 ENU/航向定义相关，必须和 RTK 文档一起理解。
- `package.xml` 的 ROS package 名是 `lsdc_slam`，而目录名是 `lsdc_eeds_slam`，部署命令应使用 package 名。
- 本文档不改变 git remote；线上 GitHub 仓库只作为后续同步目标记录。
