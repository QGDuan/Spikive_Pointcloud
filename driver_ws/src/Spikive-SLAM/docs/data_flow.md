# LSDC SLAM 数据链路与运行模式

本文档按 launch 和 node/topic 描述当前 `lsdc_slam` 的数据流。坐标系细节见 [coordinate_frames.md](coordinate_frames.md)，总架构见 [architecture.md](architecture.md)。

## 运行入口

### `mapping_livox.launch`

主要用于无人机在线运行：

```text
config.yaml + config_avia.yaml + config_mid360.yaml + config_mid70.yaml
  -> lsdc_mapping mid360
  -> lsdc_flight_controller calibration adapter
  -> lsdc_fusion_repub stable output switch
```

关键参数：

| 参数 | 当前值/来源 | 说明 |
| --- | --- | --- |
| `drone_id` | launch arg/value | 用于 Spikive topic remap |
| `odometry_topic` | `visual_slam/odom` | remap 目标后缀 |
| `R`, `T` | `config.yaml` | `flight_controller` 标定到运动中心的变换 |
| `init_x/y/z` | node private params | `flight_controller` 输出平移偏置 |

输入输出：

| 节点 | 输入 | 输出 |
| --- | --- | --- |
| `laserMapping` | LiDAR topic, `/livox/imu/` | `/Odometry`, `/cloud_registered`, `/cloud_registered_body`, `/path` |
| `slam_to_uav_transform` | `/Odometry`, `/cloud_registered` | `/Odometry_trans`, `/cloud_registered_trans` |
| `fusion_repub` | `/Odometry_trans`, `/cloud_registered_trans`, `/drone_{id}_diff_odom`, `/drone_{id}_init_match_success` | `/drone_{id}_visual_slam/odom`, `/drone_{id}_cloud_registered`, `/mavros/vision_pose/pose`, `/mavros/companion_process/status` |

Spikive stable output：

| source | topic |
| --- | --- |
| `fusion_repub` | `/drone_{drone_id}_visual_slam/odom` |
| `fusion_repub` | `/drone_{drone_id}_cloud_registered` |

### `localization.launch`

主要用于单独启动 Spikive Localization 匹配节点。LIO、Motion Control 和 `fusion_repub` 由 `mapping_livox.launch` 主链路提供：

```text
/drone_{id}_localization_pcl + /drone_{id}_initialpose
  + /cloud_registered_trans + /Odometry_trans
  -> lsdc_global_match
  -> /drone_{id}_diff_odom + /drone_{id}_init_match_success + /drone_{id}_localization_match_status
```

关键输入：

| 参数/topic | 说明 |
| --- | --- |
| `drone_id` | 生成 `/drone_{id}_initialpose`、`/drone_{id}_localization_pcl` 和状态 topic |
| `localization/continuous_match` | 默认 `false`，成功后停止 ICP；设为 `true` 时可持续更新 correction |
| `localization/fitness_threshold` | ICP fitness 成功阈值，默认 `0.3` |
| `localization/success_confirm_count` | 连续成功确认次数，默认 `10` |
| `localization/match_timeout_sec` | 单次 initialpose 匹配超时，默认 `20` 秒 |
| `localization/status_publish_period_sec` | `matching` 状态心跳周期，默认 `1` 秒 |

### `uav.launch`

只启动 `lsdc_flight_controller`，适合已有 `/Odometry`、`/cloud_registered` 时做 UAV 输出适配验证。

## Node Topic 表

### LIO: `lsdc_mapping`

| 方向 | Topic/参数 | 类型/语义 |
| --- | --- | --- |
| sub | `common/imu_topic`，默认 `/livox/imu/` | `sensor_msgs/Imu` |
| sub | `{lidar_prefix}/common/lid_topic` | Livox `CustomMsg` |
| pub | `/Odometry` | `nav_msgs/Odometry`, `camera_init -> body` |
| pub | `/cloud_registered` | `sensor_msgs/PointCloud2`, frame `camera_init` |
| pub | `/cloud_registered_body` | `sensor_msgs/PointCloud2`, frame `body` |
| pub | `/cloud_effected` | effect points，当前主 launch 不强调 |
| pub | `/Laser_map` | map cloud，当前代码中发布器存在 |
| pub | `/path` | `nav_msgs/Path`, frame `camera_init` |

### INS preprocess: `lsdc_ins_preprocess`

| 方向 | Topic/参数 | 类型/语义 |
| --- | --- | --- |
| sub | `ins/gps_topic`，默认 `/rtk_gps` | `sensor_msgs/NavSatFix` |
| sub | `ins/imu_topic`，默认 `/rtk_imu` | `sensor_msgs/Imu` |
| pub | `/lsdc_rtk` | `nav_msgs/Odometry` 承载 LLA + orientation |

### RTK pose: `lsdc_rtk2pose`

| 方向 | Topic/参数 | 类型/语义 |
| --- | --- | --- |
| sub | `/lsdc_rtk` | LLA + orientation |
| pub | `/initial_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| pub | `/rtk_odom` | `nav_msgs/Odometry` |
| pub | `/rtk_path` | `nav_msgs/Path` |
| param | `rtk/use_map_origin` | 是否使用配置地图原点 |
| param | `rtk/frame_id` | 默认 `camera_init` |

### Global match: `lsdc_global_match`

| 方向 | Topic/参数 | 类型/语义 |
| --- | --- | --- |
| sub | `/drone_{id}_initialpose` | 前端 Pose Estimate，地图 world 下的当前位姿初值 |
| sub | `/drone_{id}_localization_pcl` | WayPoint 后端 latched 发布的定位地图 |
| sub | `/cloud_registered_trans` | 标定到运动中心后的当前帧点云 |
| sub | `/Odometry_trans` | 标定到运动中心后的 odom |
| pub | `/map` | 调试用 downsample 后地图，latched |
| pub | `/submap` | 视野内子地图，发布器存在 |
| pub | `/drone_{id}_diff_odom` | Localization correction，仅表示 startup world 到 WayPoint map world 的校正 |
| pub | `/drone_{id}_init_match_success` | latched 匹配是否成功，驱动 `fusion_repub` fallback/switch |
| pub | `/drone_{id}_localization_match_status` | latched JSON 状态，`idle/map_loaded/waiting_initialpose/matching/localized/failed` |
| pub | `/fov_sphere_marker` | 可视化 marker |

### Fusion repub: `lsdc_fusion_repub`

| 方向 | Topic | 类型/语义 |
| --- | --- | --- |
| sub | `/drone_{id}_diff_odom` | global match correction |
| sub | `/Odometry_trans` | 标定后的 fallback odom |
| sub | `/cloud_registered_trans` | 标定后的 fallback 当前帧点云 |
| sub | `/drone_{id}_init_match_success` | 初始化状态 |
| pub | `/localization_cloud_registered` | `world` 下调试 cloud；未 localized 时为 fallback cloud |
| pub | `/localization_odom` | `world` 下调试 odom；未 localized 时为 fallback odom |
| pub | `/drone_{id}_cloud_registered` | EGO/地面站稳定点云输入 |
| pub | `/drone_{id}_visual_slam/odom` | EGO/地面站稳定 odom 输入 |
| pub | `/mavros/vision_pose/pose` | MAVROS vision pose，统一由本节点发布 |
| pub | `/mavros/companion_process/status` | MAVROS companion status，统一由本节点发布 |

### UAV transform: `lsdc_flight_controller`

| 方向 | Topic/参数 | 类型/语义 |
| --- | --- | --- |
| sub | `/Odometry` | LIO odom |
| sub | `/cloud_registered` | LIO 当前帧点云 |
| pub | `/Odometry_trans` | startup `world` 下 odom，供 `global_match` / `fusion_repub` 使用 |
| pub | `/cloud_registered_trans` | startup `world` 下点云，供 `global_match` / `fusion_repub` 使用 |
| pub | 无 MAVROS 输出 | MAVROS pose/status 只由 `fusion_repub` 发布 |
| param | `R`, `T` | 标定到运动中心的旋转和平移 |
| private param | `init_x/y/z` | 输出平移偏置 |

### Save result: `lsdc_save_result`

| 方向 | Topic/参数 | 类型/语义 |
| --- | --- | --- |
| sub | `/Odometry` | LIO odom |
| sub | `/cloud_registered_body` | 保存 bag |
| sub | `/cloud_registered` | 累计 map |
| sub | `/initial_pose` | diff 初始化 |
| sub | `/rtk_odom` | RTK odom |
| pub | `/Odometry_enu` | PGO 默认 odom 输入之一 |
| pub | `/rtk_odom_enu` | PGO 默认 GPS 输入之一 |
| pub | `/pcl_map` | 累计点云 map |

## 配置文件关系

| 文件 | 用途 |
| --- | --- |
| `config/config.yaml` | 通用 LIO、publish、pcd_save、INS、UAV transform `R/T` |
| `config/config_avia.yaml` | Avia 雷达配置 |
| `config/config_mid360.yaml` | MID360 雷达配置 |
| `config/config_mid70.yaml` | MID70 雷达配置 |
| `config/config_a.yaml` | `uav.launch` 使用的 UAV transform 配置 |
| `config/velodyne.yaml` | Velodyne/标准点云配置 |

## 部署检查

1. `roslaunch lsdc_slam mapping_livox.launch` 后确认 `/laserMapping`、`/slam_to_uav_transform` 存在。
2. 确认 `/Odometry` 和 `/cloud_registered` 有数据。
3. 确认 `/mavros/vision_pose/pose` 有数据且 frame 为 `world`，发布者应为 `fusion_repub`。
4. 确认地面站 stable topic 存在：
   - `/drone_{id}_visual_slam/odom`
   - `/drone_{id}_cloud_registered`
5. 如果地面站 3D 跟随失败，检查 planner/odom_visualization 是否提供 `world -> base{id}`，不要先改 SLAM 输出。
