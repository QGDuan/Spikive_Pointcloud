# LSDC SLAM 坐标系关系

本文档是 `lsdc_slam` 后续修改坐标系、Localization、RTK、UAV 适配时的第一参考。坐标系说明基于当前源码和 launch，不代表理想化接口。

## 坐标系清单

| Frame / 坐标表达 | 来源 | 当前含义 |
| --- | --- | --- |
| LiDAR frame | 原始雷达驱动 | 单个雷达的点云坐标。多雷达时通过各自前缀配置转到主雷达/IMU相关坐标 |
| `body` | `laserMapping.cpp` | IMU/body 坐标，`/Odometry.child_frame_id`，`/cloud_registered_body.header.frame_id` |
| `camera_init` | `laserMapping.cpp` | LIO 建图世界系，FastLIO2 风格命名，`/Odometry.header.frame_id` |
| `map` | Localization | WayPoint 加载路线点云的地图坐标，`global_match` 内部用该 frame 表示匹配地图 |
| `world` | UAV/Spikive 稳定输出 | `/drone_{id}_*`、MAVROS 和 EGO 使用的稳定输出 frame；Localization 成功前是本次 SLAM 启动坐标，成功后对齐 WayPoint 地图坐标 |
| `base{id}` | Planner/地面站 | Spikive-Lichtblick 的 3D follow frame，通常由 `odom_visualization` 广播 |
| WGS84 LLA | RTK/INS | GPS 纬度、经度、高度 |
| ENU-like local | `save_result.cpp` / PGO | 使用 `origin_Q` 与 `kRot90` 旋转后的结果，供 `/Odometry_enu`、`/rtk_odom_enu` 和 PGO 默认链路使用 |

## LIO 内部坐标链

`lsdc_mapping` 的核心转换在 `laserMapping.cpp`：

```text
raw LiDAR point
  -> per-lidar extrinsic in pcl_preprocess.hpp
  -> IMU/body frame point
  -> state.rot * (offset_R_L_I * point_body + offset_T_L_I) + state.pos
  -> camera_init frame point
```

关键参数：

| 参数 | 作用 |
| --- | --- |
| `mapping/extrinsic_T` | 主雷达点云中心相对 IMU/body 的平移 |
| `mapping/extrinsic_R` | LiDAR 到 IMU/body 的旋转矩阵 |
| `mapping/extrinsic_E` | 若配置非零，代码会用 Z-Y-X 欧拉角生成旋转矩阵覆盖 `extrinsic_R` |
| `{lidar_type}_mapping/extrinsic_T/R` | 多雷达预处理时各雷达到主雷达坐标的外参 |

`/Odometry` 发布关系：

```text
header.frame_id = camera_init
child_frame_id  = body
pose            = body pose in camera_init
```

点云发布关系：

| Topic | frame_id | 内容 |
| --- | --- | --- |
| `/cloud_registered` | `camera_init` | 当前帧去畸变后转换到 LIO 世界系 |
| `/cloud_registered_body` | `body` | 当前帧去畸变后位于 IMU/body 系 |
| `/Laser_map` | `camera_init` | 局部/全局地图点云，当前 launch 中不一定启用发布 |

## Localization / world 坐标链

当前 Spikive Localization 拆成三层：

```text
/cloud_registered, /Odometry in camera_init
  -> flight_controller calibration adapter
  -> /cloud_registered_trans, /Odometry_trans in startup world
  -> global_match + /drone_{id}_localization_pcl + /drone_{id}_initialpose
  -> /drone_{id}_diff_odom + /drone_{id}_init_match_success
  -> fusion_repub stable outputs in world
```

`world` 的语义由 `fusion_repub` 统一收口：

| 阶段 | `/drone_{id}_*` 与 MAVROS 输出含义 |
| --- | --- |
| Localization 未成功 | 输出 `/cloud_registered_trans`、`/Odometry_trans` 的本次 SLAM 启动坐标；这是 fallback world，只能用于本地运行/显示，不能认为已对齐 WayPoint 路线地图 |
| Localization 成功 | 输出 `diff_odom * calibrated` 后的 WayPoint 地图 world；EGO、MAVROS 和地面站仍消费同一组 stable topic |

`global_match.cpp`：

| Topic | frame/语义 | 含义 |
| --- | --- | --- |
| `/drone_{id}_localization_pcl` | WayPoint map world | WayPoint 后端加载/录制后 latched 发布的定位地图点云 |
| `/drone_{id}_initialpose` | WayPoint map world | 前端 Pose Estimate 给出的当前无人机在地图 world 下的初始猜测 |
| `/cloud_registered_trans` | fallback startup world | 已经过标定 R/T 的当前帧点云 |
| `/Odometry_trans` | fallback startup world | 已经过标定 R/T 的运动中心 odom |
| `/drone_{id}_diff_odom` | map correction | Localization correction，只表示从 calibrated startup world 到 WayPoint map world 的校正，不混入标定 R/T |
| `/drone_{id}_init_match_success` | latched bool | `true` 后 `fusion_repub` 切到地图 world；`false` 时回退 startup world |
| `/drone_{id}_localization_match_status` | latched JSON | 后端权威匹配状态，状态值为 `idle`、`map_loaded`、`waiting_initialpose`、`matching`、`localized`、`failed` |

`/drone_{id}_initialpose` 到 `/drone_{id}_diff_odom` 的内部换算：

```text
T_diff_guess = T_initial_world * inverse(T_calibrated_current)
```

`fusion_repub.cpp`：

```text
if init_match_success:
  stable_odom = diff_odom * calibrated_odom
  stable_cloud = diff_odom * calibrated_cloud
else:
  stable_odom = calibrated_odom
  stable_cloud = calibrated_cloud
```

输出：

| Topic | frame | 含义 |
| --- | --- | --- |
| `/localization_odom` | `world` | 调试/观测用 localization odom，未成功时同 fallback odom |
| `/localization_cloud_registered` | `world` | 调试/观测用 localization cloud，未成功时同 fallback cloud |
| `/drone_{id}_visual_slam/odom` | `world` | EGO、地面站和下游稳定 odom 输入 |
| `/drone_{id}_cloud_registered` | `world` | EGO、地面站和下游稳定点云输入 |
| `/mavros/vision_pose/pose` | `world` | MAVROS 视觉定位输入，由 `fusion_repub` 统一发布 |
| `/mavros/companion_process/status` | `world` | MAVROS companion 状态，由 `fusion_repub` 统一发布 |

## RTK / WGS84 / ENU 关系

RTK 转换集中在 `include/lsdc_geo.hpp`。

关键状态：

| 名称 | 含义 |
| --- | --- |
| `origin_L` | 地图原点的 WGS84 LLA |
| `origin_Q` | 地图原点方向，缺省时为 `kRot90.inverse()` |
| `kRot90` | 绕 Z 轴 +90 度的旋转，用于北东/XY方向约定转换 |
| `T_rtk_wrt_body` / `E_rtk_wrt_body` | RTK 天线相对 body 的外参 |
| `T_car_wrt_rtk` / `E_car_wrt_rtk` | 车辆/机体参考点相对 RTK 的外参，当前对无人机场景也应记录是否实际使用 |

`lsdc_ins_preprocess`：

```text
/rtk_gps + /rtk_imu
  -> time sync
  -> /lsdc_rtk
```

`/lsdc_rtk` 使用 `nav_msgs/Odometry` 承载非标准语义：

| 字段 | 当前语义 |
| --- | --- |
| `pose.pose.position.x/y/z` | latitude / longitude / altitude |
| `pose.pose.orientation` | INS 姿态 |
| `child_frame_id` | `"OK"` 表示 RTK 可用于初始化；`"-"` 或 `"ERROR"` 表示不可用或回退 |

`lsdc_rtk2pose`：

```text
WGS84 LLA + INS orientation
  -> GeographicLib LocalCartesian(origin_L)
  -> apply kRot90 and RTK/body extrinsic
  -> odom_T / odom_Q
  -> /initial_pose and /rtk_odom
```

`save_result.cpp` 的 ENU 输出：

```text
origin_rot_odom.Q = kRot90 * origin_Q
Odometry_enu = origin_rot_odom * diff_odom * lio_odom
rtk_odom_enu = origin_rot_odom * rtk_odom
```

PGO 默认读取 `/Odometry_enu` 和 `/rtk_odom_enu`，因此是否运行 `lsdc_save_result` 或等价转换会直接影响 PGO 输入。

## Motion Control / 标定坐标链

`flight_controller.cpp` 当前保留源码名，但在 Spikive 链路中只作为 Motion Control / calibration adapter。它使用全局参数 `R`、`T` 和私有参数 `init_x/y/z` 把 LIO 输出转到运动中心的 startup world：

```text
lio_odom = /Odometry in camera_init
diff_odom.R = R
diff_odom.T = T
trans_odom = diff_odom * lio_odom * diff_odom^-1
trans_odom.T += init_translation
```

输出 odom：

| Topic | frame | 用途 |
| --- | --- | --- |
| `/Odometry_trans` | startup `world` | 给 `global_match` 和 `fusion_repub` 使用的已标定 fallback odom |

输出点云：

```text
point_world = R * point_camera_init + T + init_translation
```

| Topic | frame | 用途 |
| --- | --- | --- |
| `/cloud_registered_trans` | startup `world` | 给 `global_match` 和 `fusion_repub` 使用的已标定 fallback cloud |

### 重要约束

`R/T` 明确定义为雷达系到运动中心外参，`flight_controller` 启动时会校验 R/T 的尺寸、有限值、正交性和 `det(R)`。点云链路使用小队列，旧帧应丢弃而不是缓存。MAVROS pose 和 companion status 不由该节点发布，只有 `fusion_repub` 是最终发布者。

## Spikive-Lichtblick 期望

地面站 topic 命名当前集中在 `Spikive-Lichtblick/packages/suite-base/src/spikive/config/topicConfig.ts`：

| 期望 topic/frame | 来源 |
| --- | --- |
| `/drone_{id}_cloud_registered` | `fusion_repub` 从 `/cloud_registered_trans` fallback 或 localization correction 后统一发布 |
| `/drone_{id}_visual_slam/odom` | `fusion_repub` 从 `/Odometry_trans` fallback 或 localization correction 后统一发布 |
| `base{id}` | 通常由 planner/odom_visualization 发布 |
| `world -> base{id}` | 地面站跟随 3D 模型和轨迹依赖 |

SLAM 当前只直接提供 `world` 下 odom 和点云。`base{id}` 和 robot model/path 通常是 planner 或 odom_visualization 责任。
`/drone_{id}_odom_visualization/path` 不由 SLAM 重映射或发布。

## 修改坐标系前的检查清单

1. 先确认当前运行的是 `mapping_livox.launch` 还是 `localization.launch`。
2. 用 `rostopic echo -n 1` 检查 `/Odometry`、`/Odometry_trans`、`/cloud_registered`、`/cloud_registered_trans` 的 header frame。
3. 用 `tf_echo` 检查是否存在 `camera_init -> body`、`world -> base{id}`；当前 Spikive 收口链路不依赖 `map -> camera_init` TF。
4. 如果 PGO 输入异常，确认是否实际生成了 `/Odometry_enu` 和 `/rtk_odom_enu`。
5. 修改 `R/T/init_*` 时，同时验证 `/Odometry_trans`、`/cloud_registered_trans`、MAVROS pose 和地面站点云位置。
