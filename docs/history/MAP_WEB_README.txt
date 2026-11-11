R1PRO 地图网页与保留原点的地图重置 — 2026-11-10

打开页面
本机浏览器：http://127.0.0.1:8088
机器人侧仅监听 127.0.0.1:8088，通过 SSH 转发访问，不公开到外网。
启动网页（不启动传感器或 SLAM）：
  cd /home/nvidia/ict_src/r1pro_navigation_ws
  bash scripts/start_web.sh
电脑重新建立访问通道：
  ssh -N -L 8088:127.0.0.1:8088 R1PRO
旧传感器观察台保留在 /index.html；新地图页为首页 /。

页面功能
显示 /map 栅格，支持平移、缩放、适应地图和查看原点。
显示分辨率、地图边界、实时机器人位置与轨迹（有效 map->base_link TF 时）。
灰色为未知，白色为空闲，黑色为障碍；橙色十字标记 map 的 (0,0)。
实时 /map 优先。没有实时地图时读取最近保存的 stationary_trial_*/map.yaml，并明确标注非实时。
保存地图不会叠加当前定位，因为它可能属于更早的地图会话。
只有当前建图管理器就绪、扫描与里程计新鲜、重置服务存在时才启用重置按钮。
重置会弹出确认框；确认前不会修改任何地图。

重置语义
只重建自己启动的 slam_toolbox 子进程及其内部历史图。
不重启 EKF、不清零底盘里程计、不停止/重启传感器、不发布运动指令。
保留 map 坐标系的原点位置和坐标方向，机器人当前位置不会被设为新的 (0,0)。
重置时先发布保持原栅格几何的全未知地图；新数据到来后重新建图。
清空页面轨迹，保持用户当前缩放和平移视角。
栅格左下角 info.origin 和宽高只是地图的外接边界，重新建图时可能变化；map 世界坐标原点不变。

实现与启动入口
src/r1pro_waypoint_nav/r1pro_waypoint_nav/slam_manager.py：拥有并管理一个 slam_toolbox 进程。
src/r1pro_waypoint_nav/r1pro_waypoint_nav/map_anchor.py：坐标对齐的纯数学实现。
web/map.html、map.js、map.css、map_geometry.js：网页与画布。
web/map_data.py：地图校验、PNG、旧地图载入；web/server.py：ROS 订阅及 HTTP 接口。
标准 slam_trial.launch.py 已使用管理器替代直接启动 slam_toolbox。
启动后 manager 是 /map 和 map->odom 的唯一发布者；native SLAM 使用每轮独立的私有地图/位姿/扫描话题，关闭自身 TF 广播。
旧独立 mapping.launch.py 不支持这个重置流程；网页对它只读。

在已准备好现场建图时，使用完整入口：
  cd /home/nvidia/ict_src/r1pro_navigation_ws
  source scripts/env.bash
  ros2 launch r1pro_waypoint_nav slam_trial.launch.py start_lidar:=true
仅在没有现存雷达驱动时设 start_lidar:=true。不要与已有建图、另一套 odom TF 或另一台导航启动器重复运行。
以上为持续建图命令，不会随网页打开而自动执行；Ctrl+C 退出。
scripts/run_slam_trial.py 仍是 40 秒限时验证入口，结束会自动关闭其拥有的组件。

保留原点的方法
记重置前的 map<-odom 修正为 T。
新一轮第一条有效 SLAM 位姿为 C（slam_local_map<-base），同一时间的里程计位姿为 O（odom<-base）。
计算固定对齐 A = T * O * inverse(C)。之后发布 map<-odom = A * C(t) * inverse(O(t))。
新地图栅格原点也通过同一个 A 转到 map，因此 TF 和网页地图保持一致。
等待对应时间的 TF，绝不使用“最新 TF”替代；通过独立话题防止上一轮延迟消息回流。
等待 DDS 订阅连接后才转发第一帧扫描，防止静止场景唯一有效位姿在订阅建立前丢失。

接口与记录
GET /api/map：地图元数据、建图状态、定位与重置能力。
GET /api/maps/{revision}.png：不可变的地图图像版本，避免几何和图像错配。
POST /api/map/reset：需要同源、运行期随机令牌、确认标志和当前 session/generation。
ROS /r1pro_nav/slam/reset_map（std_srvs/Trigger）返回请求是否接受；完成状态看 /r1pro_nav/slam/status。
ROS /r1pro_nav/slam/get_map 返回对齐后的公共地图；原生 slam_toolbox/dynamic_map 是内部坐标，不用于网页。
重置前栅格快照、每轮日志和对齐参数在 reports/slam_sessions/<session>/。
快照只备份栅格，不提供历史 SLAM 图的自动恢复；重置确实会清空那一轮 SLAM 历史。
限时脚本保存 posegraph.anchor.json，与 posegraph 文件配套保留；以后恢复历史图还需应用对应对齐参数。

验证
单元测试覆盖非零平移/旋转对齐、连续重置、未知像素、旋转地图原点、历史地图来源和重置就绪时序。
隔离 ROS_DOMAIN_ID=189 的端到端测试使用真实 slam_toolbox + 网页 HTTP + 合成扫描/里程计/TF。
模拟机器人位于 (2,-1) m、yaw=0.3 rad；验证重置请求、空白地图、新地图和重置前后 map<-odom 不变。
错误令牌被拒绝。测试退出后关闭其拥有的全部进程。
报告：reports/managed_reset_test/report.json；测试入口：scripts/check_managed_reset.py。
这些是离线/隔离验证，不是现场移动建图验收。此次没有启动现场雷达、EKF、导航，也没有重置现场地图。

旧地图兼容修正
早期导出的 PGM 用 205 表示未知，但 free_thresh=0.25 会把它错误解释为空闲。
网页对这些历史文件明确保留 205 为未知；没有改写旧地图。
后续限时导出的 YAML 使用 free_thresh=0.196，匹配 PGM 的未知灰度编码。
