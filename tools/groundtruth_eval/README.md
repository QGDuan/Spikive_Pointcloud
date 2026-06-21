# LAS 真值离线评估

本目录是额外的评价工具，不属于 SLAM/PGO/HBA 优化流程。`gld_fr_groundtruth.las`
只在这里读取，用来比较结果，不会写入优化图，也不会影响 PGO/HBA 输出。

## 评价口径

- `overall_icp`：整张结果地图独立刚体匹配 LAS，评价最终几何形状是否贴合真值。
- `start_segment_icp`：只用 bag 起始 `10s` 或最多 `100` 帧匹配 LAS，再把同一变换应用到全程，评价从起点固定后的漂移、分层和闭合一致性。
- 逐帧参差：从优化 bag 的每帧 world cloud 直接计算点到 LAS 的最近邻误差，主看每帧 `p95` 的均值、标准差、p95 和最差帧。

## 默认比较对象

`groundtruth_eval.yaml` 默认比较：

- `current_existing`
- `pgo_global_safe`，其中 HBA 是 4 层
- `pgo_global_safe_hba3`，其中 HBA 是 3 层

每个 case 内比较 `input`、`pgo`、`hba` 三个目标。这样可以验证肉眼观察到的
`global_safe` HBA4 是否在 LAS 真值下也更好。

## 运行

整体地图评价不需要 ROS：

```bash
python3 tools/groundtruth_eval/evaluate_groundtruth.py --map-only
```

完整逐帧评价需要在 ROS/Docker 环境内运行，因为需要读取 `.bag`：

```bash
python3 /workspace/project/tools/groundtruth_eval/evaluate_groundtruth.py
```

快速试跑可以限制帧数和点数：

```bash
python3 tools/groundtruth_eval/evaluate_groundtruth.py \
  --cases pgo_global_safe \
  --targets hba \
  --modes overall_icp \
  --map-only \
  --groundtruth-max-points 200000 \
  --map-max-points 200000 \
  --alignment-max-points 50000 \
  --metric-max-points 100000 \
  --no-save-aligned-map
```

## 输出

默认输出到 `output/groundtruth_eval/`：

- `summary_compare.yaml`：总排名和所有指标。
- `report.md`：Markdown 总报告。
- `{case}/{mode}/{target}/aligned_map.pcd`：对齐到 LAS 坐标系的地图。
- `{case}/{mode}/{target}/transform_lidar_to_las.yaml`：刚体变换矩阵。
- `{case}/{mode}/{target}/map_metrics.yaml`：整体地图误差。
- `{case}/{mode}/{target}/frame_metrics.csv`：逐帧误差。
- `{case}/{mode}/{target}/worst_frames.csv`：最差帧列表。

最终参数结论应同时看：

- `overall_icp_map_p95_rank`：最终地图贴合真值的能力。
- `start_segment_icp_frame_p95_rank`：起点固定后逐帧误差的稳定性。

