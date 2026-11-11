# R1PRO 导航模块

ROS 2 Humble 导航工作空间，包含底盘与 IMU 适配、TF 机身点云过滤、二维建图、AMCL 定位、Nav2 全向路径跟踪及网页操作界面。

当前代码基于 2026-11-11 的 R1PRO 工作区整理。最高平移速度为 0.20 m/s，最大角速度为 0.12 rad/s，到达位置容差为 0.05 m，朝向容差为 0.10 rad。网页执行导航时按剩余路径和转弯位置动态限速，直线路段逐步加速，接近目标和明显弯道时减速。0.20 m/s 配置尚未进行实车行驶验证。

## 目录

| 路径 | 内容 |
| --- | --- |
| `src/r1pro_waypoint_nav/` | 传感器适配、机身裁剪、离群点过滤、SLAM、导航配置与启动入口 |
| `src/r1pro_nav2_critics/` | Nav2 行驶中朝向和连续目标距离评分插件 |
| `web/` | 地图、目标选点、路径预览、执行导航、清除行驶轨迹和重置地图 |
| `scripts/` | 环境加载、原生库构建、配置生成及实机检查工具 |
| `config/` | RViz 配置 |
| `tests/` | 离线测试及隔离 ROS 域的模拟验证 |
| `reports/navigation_geometry.json` | 网页与导航配置生成所需的实测机身几何 |
| `docs/history/` | 各开发阶段的设计与验证记录 |

运行环境为 Linux。Windows 可通过浏览器和 SSH 端口转发访问网页。机器人原厂驱动、模型、TF、雷达驱动及消息定义由 R1PRO 系统提供。

## 构建

在 Linux 克隆本仓库作为 colcon 工作空间，并安装 ROS 2 Humble、Nav2、slam_toolbox、robot_localization、pointcloud_to_laserscan，以及 Python 的 numpy、scipy、PyYAML、Pillow 和 pytest。原厂环境应提供 `hdas_msg` 等消息和机器人模型。

`scripts/env.bash` 当前加载 `/home/nvidia/galaxea215/install/setup.bash`，使用 ROS 域 79。迁移到其他主机时按实际原厂安装路径和 ROS 域修改。

```bash
cd /path/to/r1pro_navigation_ws
source scripts/env.bash
bash scripts/build_cuda.sh
bash scripts/build_knn.sh
colcon build --symlink-install --packages-select r1pro_nav2_critics r1pro_waypoint_nav
source scripts/env.bash
```

CUDA 脚本针对 R1PRO 的 SM 87 编译。KNN 构建使用 `/opt/ros/humble/include/karto_sdk/nanoflann.hpp`，可通过 `NANOFLANN_INCLUDE_DIR` 指定头文件目录。原生库由源码构建，相关过滤参数见 `src/r1pro_waypoint_nav/config/self_filter.yaml`。

当前配置已纳入版本管理。`scripts/prepare_nav2.py` 根据实测几何重新生成 Nav2 参数。`scripts/prepare_configs.py` 属于早期配置生成工具，会覆盖 Nav2 与 SLAM 配置，使用前检查生成内容。

## 网页导航

原厂基础系统、传感器和定位应已就绪。使用已有地图启动导航组件。

```bash
source scripts/env.bash
ros2 launch r1pro_waypoint_nav navigation.launch.py \
  map:=/absolute/path/to/map.yaml start_planning:=true
```

传感器启动参数由现场已有节点决定，参见 `navigation.launch.py`。定位须有与当前地图一致的初始位姿。传感器输出和 `map -> odom -> base_link` 应完整。

启动网页，默认提供查看与规划预览。

```bash
source scripts/env.bash
python3 -u web/server.py --port 8088
```

需要通过网页确认目标并执行实车导航时，使用执行入口。

```bash
python3 -u web/server.py --port 8088 --enable-drive
```

在 Windows 保持 SSH 转发连接，并用浏览器打开 `http://127.0.0.1:8088/`。

```powershell
ssh -N -L 8088:127.0.0.1:8088 R1PRO
```

建图入口包括 `mapping.launch.py` 和 `slam_trial.launch.py`。具体组件与标定参数见对应 launch 文件。开发记录保存在 `docs/history/`，其中的参数和能力描述反映各自记录时间，当前实现以源码及提交配置为准。

## 验证

Linux 已加载机器人 ROS 环境后，可运行离线单元测试。

```bash
source scripts/env.bash
python3 -m pytest -q tests
```

全向跟踪模拟使用独立 ROS 域，模拟底盘和传感器。

```bash
ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1 python3 tests/check_holonomic_tracking.py
```

`scripts/` 中包含会控制实车的检查工具，例如 `test_axes_bounded.py`、`test_backward_pulse.py` 和 `rotate_and_map.py`，运行前按工具说明确认现场条件。

## Git 使用

采集数据、地图、日志、构建产物和本机凭据通过 `.gitignore` 排除。提交包含用于启动网页的实测几何文件。地图可自行放入工作空间的 `maps/` 目录，或在启动命令中指定路径。

本地初始分支为 `main`。准备好 GitHub 空仓库后，在本地关联并推送。

```bash
git remote add origin <GitHub 仓库地址>
git push -u origin main
```
