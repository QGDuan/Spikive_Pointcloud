# LSDC PGO / Loop 数据链路

本文档按 `std_loop` 的实时、文件预处理和离线 PGO 三条路径记录 topic、文件、参数和坐标语义。总架构见 [architecture.md](architecture.md)。

## 参数入口

`std_loop.cpp` 读取的主要参数：

| 参数 | 代码默认值 | 说明 |
| --- | --- | --- |
| `run_pgo` | `false` | `false` 实时 loop；`true` 离线 PGO |
| `use_loop` | `true` | 是否添加 loop factor |
| `use_gps` | `false` | 是否添加 GPS factor |
| `use_file` | `false` | 是否先从文件载入历史数据 |
| `cloud_topic` | `/cloud_registered_body` | 点云输入 |
| `odom_topic` | `/Odometry_enu` | odom 输入 |
| `gps_topic` | `/rtk_odom_enu` | GPS/RTK 输入 |
| `save_path` | `${ROOT_DIR}MAP` | 输出目录 |
| `save_name` | `pgo` | 输出文件名前缀 |
| `input_path` | 空 | 离线输入目录 |
| `input_bag_names` | 空 | 多个 bag 名用 `-` 分隔 |
| `input_pose_names` | 空 | 多个 pose 名用 `-` 分隔 |
| `input_loop_names` | 空 | 多个 loop 名用 `-` 分隔 |
| `map_resolution` | `0.1` | PGO map 降采样分辨率 |

STD 参数来自 `config/config_loop.yaml` 或 `config/config_pgo.yaml`：

| 参数组 | 说明 |
| --- | --- |
| `loop/*` | 关键帧阈值、图更新频率、GPS factor 间隔 |
| pre process | `ds_size`, `maximum_corner_num` |
| key points | 平面检测、体素、投影和角点阈值 |
| std descriptor | 三角描述子邻域、边长、NMS、边长分辨率 |
| candidate search | 候选数量、近邻跳过、ICP/几何验证阈值 |

## 实时 loop 数据流

```text
cloud_topic + odom_topic + gps_topic
  -> timestamp pairing
  -> keyframe selection
  -> STD descriptor generation
  -> loop candidate search + ICP verification
  -> GTSAM factor graph
  -> ISAM2 update
  -> corrected odom/path/map
```

订阅：

| Topic 参数 | 默认/launch 常见值 | 语义 |
| --- | --- | --- |
| `cloud_topic` | `/cloud_registered_body` | body 系当前帧点云，来自 LIO |
| `odom_topic` | `/Odometry_enu` 或 `/Odometry` | LIO/ENU pose |
| `gps_topic` | `/rtk_odom_enu` 或 `/rtk_odom` | GPS/RTK 本地 pose |

发布：

| Topic | frame_id | 说明 |
| --- | --- | --- |
| `/aft_mapped_to_init` | 当前代码沿用 odom 设置 | 校正后 odom |
| `/cloud_current` | `camera_init` | 当前源点云可视化 |
| `/cloud_key_points` | `camera_init` | 当前关键点 |
| `/cloud_matched` | `camera_init` | 匹配目标点云 |
| `/cloud_matched_key_points` | `camera_init` | 匹配目标关键点 |
| `descriptor_line` | marker | STD 描述子匹配线 |
| `/cloud_correct` | `camera_init` | 校正点云 |
| `/odom_corrected` | 当前代码设置 | 校正 odom |
| `/pgo_map` | `camera_init` | 优化地图 |
| `/curr_cloud_correct` | `camera_init` | 当前校正点云 |
| `/lio_path` | `camera_init` | 原始轨迹 |
| `/pgo_path` | `camera_init` | 优化轨迹 |
| `/gps_path` | `camera_init` | GPS 轨迹 |
| `/gps_points` | `camera_init` | GPS 点集合 |
| `/gps_use_points` | `camera_init` | 实际加入图优化的 GPS 点 |

## 文件预处理数据流

`use_file=true` 时会先执行 `preprocessStd()`：

```text
input_path/name_pose.txt
input_path/name_cloud.bag
input_path/name_loop.txt optional
  -> read pose and cloud
  -> pair cloud with pose by timestamp
  -> generate descriptors
  -> add odometry, loop, GPS factors
  -> publish/save update results
```

文件命名约定：

| 输入 | 命名 |
| --- | --- |
| cloud bag | `${input_bag_name}_cloud.bag` |
| pose txt | `${input_pose_name}_pose.txt` |
| loop txt | `${input_loop_name}_loop.txt` |

`preprocessStd()` 内部硬编码读取：

| bag topic | 说明 |
| --- | --- |
| `/cloud_registered_body` | 点云 |
| `/rtk_odom` | GPS/RTK |

pose 文件格式由 `file_preprocess.hpp` / 读取函数使用，核心语义是：

```text
timestamp x y z qx qy qz qw
```

loop 文件保存/读取语义：

```text
target_timestamp source_timestamp dx dy dz qx qy qz qw
```

## 离线 PGO 数据流

`run_pgo=true` 时：

1. `FilePreprocess` 读取输入 bag 和 pose。
2. GPS 队列先通过 `gpsPreprocess()` 进入候选 GPS factor。
3. 每个点云/pose pair 进入 `msgpairPreprocess()`。
4. 关键帧进入 `saveGraphFactorMain(curr_stamp)`，添加里程计、loop、GPS 因子。
5. `isam.update(graph, initial)` 后多次迭代。
6. `pubUpdateResults(results, map_resolution)` 发布路径和地图。
7. 若地图点数足够，保存 `${save_name}_map.pcd`。

## 与 SLAM 输出的适配边界

当前 `lsdc_slam` 常见输出：

| SLAM topic | frame | 是否直接适合 PGO |
| --- | --- | --- |
| `/cloud_registered_body` | `body` | 是，PGO 默认 cloud 输入 |
| `/cloud_registered` | `camera_init` | 不建议替代 body cloud，除非重新确认算法期望 |
| `/Odometry` | `camera_init -> body` | 可用于部分实时 loop launch |
| `/Odometry_enu` | ENU-like | PGO 代码默认 odom 输入 |
| `/rtk_odom` | `camera_init` 或配置 frame | 可用于实时 launch |
| `/rtk_odom_enu` | ENU-like | PGO 代码默认 GPS 输入 |
| `/drone_{id}_cloud_registered` | `world` | 这是地面站输出，不是 PGO 默认输入 |
| `/drone_{id}_visual_slam/odom` | `world` | 这是地面站输出，不是 PGO 默认输入 |

推荐在无人机在线 PGO 改造前先选定一种模式：

| 模式 | 输入建议 | 风险 |
| --- | --- | --- |
| LIO 原始系 | `/cloud_registered_body`, `/Odometry`, `/rtk_odom` | GPS/RTK 与 LIO frame 需一致 |
| ENU 后处理系 | `/cloud_registered_body`, `/Odometry_enu`, `/rtk_odom_enu` | 需要 `lsdc_save_result` 或等价节点在线运行 |
| Spikive world 系 | `/drone_{id}_cloud_registered`, `/drone_{id}_visual_slam/odom` | 需要重审 PGO 期望 frame，不应直接替换默认输入 |

## 输出文件

`std_loop` 根据 `save_path` 和 `save_name` 构造：

| 文件 | 说明 |
| --- | --- |
| `${save_name}_pose.txt` | pose 输出 |
| `${save_name}_loop.txt` | loop 约束输出 |
| `${save_name}_map.pcd` | 优化地图 |
| `${save_name}_map_enu.pcd` | 路径变量存在，当前主保存逻辑主要保存 `_map.pcd` |

## 验证命令

文档或配置变更后建议先做静态核对：

```bash
rg -n "cloud_topic|odom_topic|gps_topic|run_pgo|use_file|use_gps" driver_ws/src/lsdc_loop
rg -n "/cloud_registered_body|/Odometry_enu|/rtk_odom_enu|/pgo_map|camera_init" driver_ws/src/lsdc_loop
```

运行时核对：

```bash
rosparam get /cloud_topic
rosparam get /odom_topic
rosparam get /gps_topic
rostopic hz /cloud_registered_body
rostopic echo -n 1 /Odometry
rostopic echo -n 1 /pgo_map/header
```

