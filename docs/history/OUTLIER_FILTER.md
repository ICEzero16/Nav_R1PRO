# TF 过滤后进行单次距离分段离群点过滤

本次仅修改代码并复用已保存的三帧离线验证；没有启动雷达、导航、点云或网页服务。

处理顺序：原始点云 → 去除无效值与 TF 机身方盒内的点 → 单次统计离群点过滤。

`points_filtered` 保持原有语义（仅机身过滤）；`points_denoised` 是最终输出；
`points_removed` 仍只包含机身点；`points_outliers` 单独输出统计离群候选。
二维扫描入口已改为订阅 `/r1pro_nav/points_denoised`，不会对同一帧反复去噪。
网页代码支持原始、仅机身过滤、最终去噪三种来源，也可单独显示离群候选。

参数位于 `src/r1pro_waypoint_nav/config/self_filter.yaml`：

```yaml
outlier_enabled: true
outlier_mean_k: 16
outlier_method: range
outlier_std_multiplier: 3.0
outlier_range_edges: [2.0, 5.0, 10.0, 20.0]
outlier_min_bin_points: 64
outlier_global_floor_std: 2.0
```

每点在所有机身过滤后有效点中寻找排除自身的 16 个近邻，避免人为截断距离段边界上的邻居。
距离是原始雷达 XYZ 到雷达原点的三维距离，不是 base_link 或 map 原点距离。
按照 [0,2)、[2,5)、[5,10)、[10,20)、[20,+∞) 米分别计算邻居平均距离的均值 μ 和样本标准差 σ。
每段候选阈值为 μ+3σ；实际阈值取其、前面已统计距离段阈值、旧全局 μ+2σ 三者最大值。
因此有效统计段的阈值随距离不减，且默认参数下新版删除集合是旧版的子集。
某段少于 64 个点则全部保留，并在状态中报告原因；这可能也保留真正孤立的远点。
超过阈值才判为离群候选。阈值单位为米，表示近邻平均距离，不能理解为点的测距误差。
调大 multiplier 会保留更多点；关闭 enabled 时最终输出与机身过滤输出一致。
参数在节点启动时读取。点数不足 k+1 时保留所有点，非有限输入视为异常。
设置 method=global 且 std_multiplier=2.0 可恢复旧版算法。分段方法是本项目实现，并非 PCL 原生选项。
使用单线程 SciPy cKDTree，不引入 Open3D，不启动额外节点。

保留完整 PointCloud2 记录及原始顺序、header、所有字段（含逐点 timestamp）。
输出仅变为非组织化单行点云。离群失败会报告过滤错误，不用空点云冒充安全空间。

已保存三帧（2026-11-08 21:55:07.935～21:55:08.136，北京时间）的离线结果：

| 帧 | 机身过滤后 | 旧版候选 | 新版候选 | 新版保留 | 单帧离群计算耗时 |
|---|---:|---:|---:|---:|---:|
| 1 | 16117 | 708 | 238 | 15879 | 54.8 ms |
| 2 | 15878 | 667 | 198 | 15680 | 50.5 ms |
| 3 | 15785 | 712 | 234 | 15551 | 51.6 ms |

三帧 5–10 m 距离段阈值分别为 0.4674、0.4731、0.4767 m；旧全局阈值约 0.30–0.31 m。
三帧在 10 m 以上共只有 1 个点，不能据此验证更远距离的效果。

这是一次离线计时，不代表完整流水线的实时性能；未启动服务验证吞吐。
统计稀疏不等于物理噪声，远处表面、细杆和真实边缘也可能被筛除，需先查看对比再用于导航。

实现与验证：
- `r1pro_waypoint_nav/outliers.py`：纯数值算法与原始点索引映射。
- `tests/test_outliers.py`：孤立点、重复点、小点云、关闭过滤、参数异常、坐标变换与索引守恒。
- `scripts/filter_saved_outliers.py`：离线处理已有三帧，无 `rclpy.init`、无驱动启动。
- `captures/three_frames_20261108_215505/outliers_k16_s2/`：CDR、索引掩码、统计和可视化数据。
- `captures/three_frames_20261108_215505/outliers_range_k16_s3/`：新版分段结果，原结果保留。
- 全部 31 项测试通过，包含远处稀疏簇、少量样本保留、距离段边界、旋转不变性、字段与索引守恒。
- 编译使用 `PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select r1pro_waypoint_nav`，避免用户目录 setuptools 82 与 Humble 构建工具不兼容；未修改系统依赖。

算法参考：[PCL StatisticalOutlierRemoval](https://pointclouds.org/documentation/tutorials/statistical_outlier.html)。
