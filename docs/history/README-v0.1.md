# R1Pro 定点导航初版 v0.1

工作空间：`/home/nvidia/ict_src/r1pro_navigation_ws`。ROS 2 Humble。

这是传感器适配、二维建图及 Nav2 规划预览初版，**尚不能驱动机器人自主到点**。
代码没有向真实底盘速度或制动话题发布的路径，也没有硬件执行开关。
不修改原厂安装文件，不启动原厂导航，不重启现有本体。

## 已提供

- Livox MID-360 独立启动入口，以及点云转 LaserScan。
- 底盘 `vx/vy/wz` 全向运动积分，输出 `/r1pro_nav/odom`。
- 时间戳、过期数据、反馈中断检查；里程计故障锁定，不虚构丢失期间位移。
- Livox IMU 单位适配入口；单位未确认时不输出。忽略驱动占位姿态四元数。
- slam_toolbox 建图入口与 Nav2 定位/规划预览入口，默认要求先确认标定。
- 命名点位保存、列表、删除、目标预览、向私有 Nav2 Action 发目标以及超时/中断取消。
- 速度预览限幅：平移合速度 0.05 m/s，旋转 0.15 rad/s；指令超时 0.3 s 清零。

## 1. 编译

```bash
cd /home/nvidia/ict_src/r1pro_navigation_ws
source scripts/env.bash
python3 scripts/prepare_configs.py
colcon build --symlink-install --packages-select r1pro_waypoint_nav
source scripts/env.bash
PYTHONPATH=src/r1pro_waypoint_nav python3 -m unittest discover -s tests -v
```

配置由本机 Humble 已安装组件的默认配置派生。`prepare_configs.py` 会重写本项目
`config/nav2.yaml` 和 `config/slam.yaml`；调参后不要重复运行它覆盖改动。

## 2. 默认传感器预览（不移动机器人）

本体原有驱动应已运行。确认没有另一个 Livox 驱动或本项目实例：

```bash
cd /home/nvidia/ict_src/r1pro_navigation_ws
source scripts/env.bash
ros2 launch r1pro_waypoint_nav sensors.launch.py start_lidar:=true
```

已有雷达驱动时使用 `start_lidar:=false`。不重复启动。
另一个已加载环境的终端查看：

```bash
ros2 topic echo /r1pro_nav/status --once
ros2 topic hz /r1pro_nav/scan
ros2 topic echo /r1pro_nav/odom --once
```

默认里程计是**待标定的调试估计**，假设原厂速度为 base_link 中的 m/s、rad/s，
不发布 `odom -> base_link`，也不向滤波器输入 IMU。没有给空 frame_id 自动赋予可信的物理含义。
它只用于检查链路。故障后先处理原因，再重启本项目；重启会重置里程计原点，
正在运行的建图/定位也应按同一个会话重新初始化。

`Ctrl+C` 关闭此次启动的传感器和适配器，不停止其他本体控制程序。

## 3. 标定完成后的建图入口

必须先核验：底盘速度单位和轴向、雷达与底盘外参、TF 单一发布源、点云机身过滤、
地面高度和扫描范围。当前机器人模型外参与旧 JSON 标定有差异，不能混用。
现有机身坐标系采用 `base_link`，不新建 `body`。

参数 `calibration_confirmed` 是人为核验声明，不是自动标定结果，不应为绕过检查而设置。
核验后，以以下方式替换第 2 节入口：

```bash
ros2 launch r1pro_waypoint_nav sensors.launch.py start_lidar:=true calibration_confirmed:=true publish_tf:=true
# 另一终端：
ros2 launch r1pro_waypoint_nav mapping.launch.py calibration_confirmed:=true
```

建图需要现场操作机器人移动采集；本项目不提供遥控或运动执行。
本版选择轮式里程计 + 点云投影 + slam_toolbox，尚未集成雷达惯性里程计。
Livox IMU 适配接口保留供后续融合，当前建图不依赖未标定 IMU。

保存地图：

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f /home/nvidia/ict_src/r1pro_navigation_ws/maps/lab_v1
```

## 4. Nav2 规划预览

停止建图节点，保留校准后的 sensors 链路。不要同时让 slam_toolbox 和 AMCL 发布 map -> odom。
确保没有其他 Nav2 节点占用相同节点名或动作服务。

```bash
ros2 launch r1pro_waypoint_nav navigation_preview.launch.py \
  calibration_confirmed:=true map:=/home/nvidia/ict_src/r1pro_navigation_ws/maps/lab_v1.yaml
```

在 RViz 中设置初始位姿，确认定位正确。所有 Nav2 速度发布到 `/r1pro_nav/cmd_vel`，
再经过适配器限幅到 `/r1pro_nav/preview/target_speed_chassis`。没有接真实底盘。
因此机器人不会跟随路径，非原地目标可能因没有进展而中止，属于此版本预期行为。
候选 footprint 是 0.9 × 0.8 m 的矩形加 0.05 m padding，**不是实测全身包络**；
未完成机身过滤、盲区评估和机械臂姿态约束前不能用于真实避障。

## 5. 命名点位

地图 ID 由操作者明确提供，必须与正在使用的地图版本一致；本版不自动验证地图文件哈希。
`save` 从新鲜的 `map -> base_link` 保存当前位置，不接受缺失/过期定位。

```bash
ros2 run r1pro_waypoint_nav waypoint save home --file /home/nvidia/ict_src/r1pro_navigation_ws/maps/waypoints.yaml --map-id lab_v1
ros2 run r1pro_waypoint_nav waypoint list --file /home/nvidia/ict_src/r1pro_navigation_ws/maps/waypoints.yaml --map-id lab_v1
ros2 run r1pro_waypoint_nav waypoint go home --file /home/nvidia/ict_src/r1pro_navigation_ws/maps/waypoints.yaml --map-id lab_v1
```

`go` 默认只打印目标。加 `--send-preview` 才向 `/r1pro_nav/navigate_to_pose` 提交，
默认 120 秒超时取消，Ctrl+C 也会请求取消。收到取消应答不代表物理停止。
覆盖点位需 `--replace`；删除使用 `delete NAME`。

## 实机检查依据与当前限制

2026-11-08 的短时采样：雷达约 9.6 Hz，每帧约 1.9 万点；Livox IMU 约 185 Hz；
底盘 IMU/原始反馈约 157 Hz，底盘速度约 49 Hz。两个 IMU 加速度模长均约 1，
疑似 g；IMU 单位、轴向和噪声尚未实测标定。速度反馈为空 frame_id。
底盘原始 JointState 的各数组长度不一致，所以本包不按标准关节顺序解析它。
头部视觉里程计参考系未接入机身且坐标异常大，本包不使用它。

里程计协方差、扫描高度、footprint 都是初始配置，不代表实际精度或完整障碍物覆盖。
本版无真实急停/遥控接管接口、控制权仲裁、硬件停止验证；硬件输出刻意未实现。
完成静态链路不等于完成动态导航验收。

参考：
- https://github.com/ros-perception/pointcloud_to_laserscan
- https://github.com/SteveMacenski/slam_toolbox
- https://github.com/Livox-SDK/livox_ros_driver2
- 本机 `/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml`

## 本次验证记录

2026-11-08：编译成功，7 项核心单元测试通过。真实雷达与底盘反馈联调通过；
IMU 未确认时输出为零条，真实速度和制动话题发布者未增加。
隔离 ROS_DOMAIN_ID=178 下限幅、指令超时归零、反馈故障锁定通过；
使用合成地图和 TF 时，Nav2 六个核心节点均进入 active。未发送真实导航目标。
报告位于 reports/sensors_smoke.json、reports/isolated_checks.json 和 reports/validation_summary.json。
