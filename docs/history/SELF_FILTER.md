# 机器人方盒模型与点云自过滤

当前交付方式：仅代码。雷达、过滤器、扫描转换、网页及本机转发已按要求停止。
本次后续检查不启动任何 ROS 节点或后台服务。以下接口说明不是正在运行的状态。

## 模型

读取 `/robot_state_publisher` 的 `robot_description`，优先使用每个 link 的 collision 几何。
当前描述包含 47 个 link，其中 46 个含碰撞网格。STL 顶点在几何局部坐标中计算 AABB，
包含 URDF mesh scale 与 collision origin。将这些局部盒子通过对应时间戳的 TF 转换到点云坐标中，
得到跟随底盘、躯干、双臂姿态的有向方盒集合；不是用一个大方盒包住整个机器人。

支持 URDF 的 box、sphere、cylinder 和 STL mesh。未知网格格式或加载失败会令过滤器未就绪，
不会悄悄忽略该连杆。没有 collision 才回退到 visual，并在模型消息中标注来源。

默认 padding 为每个方向 0.025 m，即每个盒子每条边总共增加 0.05 m。
只去掉盒子及此余量内的点，不按机器人周围固定半径整圈删除。
这种近似仍可能删除盒子覆盖到的真实邻近物体，尤其是夹爪行程包络内的物体；需检查模型与回波对齐。

四个 gripper_finger link 如果没有可用 TF，会使用 URDF 声明的 prismatic 行程上下限生成全行程包围盒，
挂在有实时 TF 的夹爪父连杆上。该回退明确标注为 swept，并以黄色显示。
其他动态 TF 缺失时不使用最新姿态或零角度替代：短暂等待，超时丢弃该帧并报告未就绪。

## 数据流

```
/hdas/lidar_chassis_left  原始点云，不覆盖
  -> self_filter
     -> /r1pro_nav/points_filtered  保留的环境点
     -> /r1pro_nav/points_removed   方盒内剔除的有效点（调试）
     -> /r1pro_nav/self_filter/boxes  方盒角点与来源 JSON
     -> /r1pro_nav/self_filter/markers  RViz 线框
     -> /r1pro_nav/self_filter/status  数量、耗时、TF/模型状态
  -> pointcloud_to_laserscan（输入已改为 points_filtered）
     -> /r1pro_nav/scan
```

过滤处理全分辨率点云。保留点的完整记录按原顺序复制，保留 XYZ、intensity、tag、line、timestamp、
消息时间戳和 frame_id。输出使用单行非组织化 PointCloud2，宽度为保留点数。
NaN、Inf、距离传感器小于 5 cm 的无效/零点另行统计，不计入“删除机身点”。

性能实现先用整个模型的 AABB 缩小候选点集，再逐盒做准确包含判定；总包围盒不会直接删除点。
ROS 字节数组直接批量复制，避免 Python 逐字节类型检查。没有订阅者时不生成 RViz Marker 点列表。

## 网页代码

增加“过滤后点云 / 原始点云”选择、“机器人近景”、方盒显示和被过滤点显示。
青色线框为普通包围盒，黄色为手指全行程包围盒，被过滤点为粉红色。
只有坐标系一致时才叠加，失效盒子不继续冒充实时模型。
网页只是显示已运行节点的输出，无运动按钮，无底盘控制接口。

`scripts/start_web.sh` 已调整为默认只启动网页；只有显式传入 `--with-sensors` 才会启动传感器。
当前没有运行该脚本。服务启动留待用户明确要求。

## 离线验证

`tests/test_box_geometry.py` 验证旋转盒、动态位姿、collision origin、手指行程、STL scale、
带行填充的大端 PointCloud2、非有限值和未知模型格式处理，不创建 ROS 节点。
`scripts/check_self_filter.py` 是需要现有数据服务的实机验证脚本，当前阶段不运行。

当前 TF/模型外参沿用机器人现有定义，未完成物理标定。
采用整帧时间戳摆放盒子，不逐点运动补偿；快速关节运动时需进一步验证扫帧期间的几何误差。
这个功能不代表已完成 SLAM、避障或实机自主导航。
