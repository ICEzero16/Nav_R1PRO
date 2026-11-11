R1PRO 二维 SLAM 初版 — 2026-11-10

后续更新：已接入建图专用跨帧确认与紧凑点簇运动过滤，详见 TEMPORAL_FILTER_README.txt。
新模块仅完成离线测试；下文两轮静止测试结果对应增加此模块之前的版本。

用途与范围
本次完成有超时退出的静止建图链路验证；不是已验收的自主导航系统。
EKF + slam_toolbox 足以作为当前平面场景的核心，但必须配套传感器适配、点云过滤、LaserScan 和一致的时间戳/TF。
当前不执行路径规划、避障控制或底盘运动，不启动 Nav2 控制器。

数据流
MID360 -> TF/URDF 46 个包围盒自身过滤（CUDA）
       -> 按距离分段的统计离群点过滤（单 CPU worker）
       -> pointcloud_to_laserscan -> /r1pro_nav/scan -> slam_toolbox
底盘速度 vx/vy -> /r1pro_nav/odom 的 twist ┐
底盘 IMU -> /r1pro_nav/imu_chassis_planar ┴-> planar EKF -> /r1pro_nav/odom_fused
EKF 发布 odom -> base_link；slam_toolbox 发布 map -> odom。
不把轮速积分位姿和同源速度重复融合；保留全向底盘真实的 vy。
IMU 只使用已验证方向与经验单位换算的角速度 Z，不融合未确认的姿态或加速度。

工作目录与启动入口（在 R1PRO 上执行）
cd /home/nvidia/ict_src/r1pro_navigation_ws
source scripts/env.bash
python3 scripts/run_slam_trial.py

这个入口先做 5 秒检查，再启动 40 秒静止建图；保存地图/位姿图并退出。
要求原厂基础系统已经运行、底盘保持静止，且没有现存雷达/建图组件或运动命令发布者。
启动时若发现重复组件、已有 odom TF、运动发布者或过期底盘反馈，会拒绝启动。
运行中检测反馈/扫描/IMU/融合里程计/状态断流和里程计故障；异常退出并保留 report.json。
不会杀死用户已有组件。原厂 R1PROBody.d 不由这个脚本管理。

launch/slam_trial.launch.py 是底层组合入口，包含传感器、EKF、SLAM。
直接 ros2 launch 不包含上述限时健康监督，应优先使用 run_slam_trial.py。
默认 start_lidar=false；监督脚本在检查无现存雷达后传入 true。
没有新增系统自启动项，也没有常驻导航服务。

配置与实现
src/r1pro_waypoint_nav/config/ekf_planar_candidate.yaml：平面 EKF 参数。
src/r1pro_waypoint_nav/config/chassis_imu.yaml：底盘陀螺适配参数。
src/r1pro_waypoint_nav/config/self_filter.yaml：CUDA 裁剪及按距离离群点参数。
src/r1pro_waypoint_nav/config/scan.yaml：雷达坐标系高度 -0.10~0.80 m；量程 0.30~12 m。
src/r1pro_waypoint_nav/config/slam.yaml：5 cm 栅格；trial launch 将最小使用距离覆盖为 0.31 m。
adapters.py：新增 25 帧连续反馈预热，预热前不发布里程计。
首次原点在预热通过时建立；预热中不要移动机器人。
初始化后的时间戳跳变或超过 0.2 秒的反馈间隔仍锁定故障，不补造丢失的位移。

结果文件
maps/stationary_trial_YYYYMMDD_HHMMSS/map.pgm + map.yaml：导航栅格地图。
同目录 posegraph.posegraph + posegraph.data：slam_toolbox 可继续建图的位姿图数据。
report.json：健康检查与计数；launch.log：完整运行/退出日志。
地图预览：白色为空闲、黑色为障碍、灰色为未知。

验证与已知限制
39 项单元测试通过，ROS 包构建通过。
两轮 40 秒静止建图通过；最终结果目录为 stationary_trial_20261110_152930。
最终记录 330 帧扫描、1083 帧融合里程计、7 次地图消息；里程计状态无故障。
地图 205×205 格，5 cm 分辨率；这是栅格边界范围，并非全部区域已观测。
该轮保存之后 IMU 适配器退出日志出现 ROS 上下文竞争异常，现已修正信号处理。
修正后再次通过 39 项测试和 3 次只读 IMU 启停验证，退出码均为 0；完整 SLAM 未因这一退出修正再次重跑。
静止测试已验证有效 scan、wheel/IMU/EKF、完整 map->base_link TF、地图和位姿图保存。
一次静止观测只能形成局部稀疏地图；MID360 扫描投影有空隙，遮挡区仍未知。
没有验收移动时的扫描匹配、转弯后墙面对齐、累计漂移和回环。
底盘距离比例尚未实测；calibration_confirmed 仍为 false。
当前 IMU 使用主机发布时间，未验证硬件采集时间；尚不具备完整 3D LIO 的标定依据。
观测过历史约 260 ms 底盘反馈中断；运行故障保护保留，长期稳定性仍需验证。
下一步是现场监督下低速移动建图，确认墙面重影、回环和定位重复性；随后再接 Nav2 定点导航。

参考
https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst
https://index.ros.org/p/slam_toolbox/
