# 单线程 CPU 工程优化与 CUDA 方盒裁剪

代码已集成，未启动雷达、导航、建图、点云或网页服务。
CUDA 只用于 TF 机身方盒裁剪；距离分段离群过滤仍使用 SciPy 单线程精确近邻查询。
不做体素降采样，不改 k=16、3σ 与距离分段规则，不改变原始点字段。

## 实现

- 每帧方盒角点和包围盒只准备一次，裁剪与可视化复用。
- CPU 路径只建立一次原始点到附近点的整数索引；各方盒使用布尔选择，仅变换尚未命中且位于粗包围盒内的点。
- 保留无效零点的提前排除，避免它们进入所有方盒的重复计算。
- 离群过滤通过原始布尔掩码回填，删除多余的 flatnonzero 和二次整数索引。
- 无订阅者时不构造调试点云和方盒 JSON/markers；输出点记录通过 buffer 复制，省去中间 bytes 对象。
- CUDA 使用持久化显存缓冲，容量增长时才重新分配；每点一个线程，命中后提前退出。
- 传入 float32 XYZ 和 float64 方盒变换，内部双精度几何判断，禁用 FMA 合并以减小与 CPU 的边界差异。
- 独立低优先级 CUDA stream；使用 blocking event 等待，不新增 CPU 计算线程池。不代表能够严格限制 GPU 占用或保证与 SLAM 并发时的延迟。
- GPU 错误明确报错并丢弃该帧，不伪造空环境或静默回退。

## 编译与选择

在 `/home/nvidia/ict_src/r1pro_navigation_ws` 下：

```bash
bash scripts/build_cuda.sh
source scripts/env.bash
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select r1pro_waypoint_nav
```

编译脚本针对本机 Jetson AGX Orin 的 sm_87、CUDA 12.2。其他 GPU 需修改架构参数重新编译。
无需 CuPy/PyTorch；通过 ctypes 调用原生共享库。
`config/self_filter.yaml` 已选择 `crop_backend: cuda`，下次明确启动节点时才生效。
改为 `crop_backend: cpu` 可选择优化后的单线程 CPU 路径。未编译 CUDA 时应选择 CPU。
启动参数只在节点初始化时读取，不提供运行中切换。

## 离线验证（2026-11-10）

保存的三帧逐点比较：CPU/CUDA 机身裁剪与旧版一致，最终离群过滤结果也一致。
31 项既有测试、15 组 CUDA 合成用例（空输入、空方盒、面边界、重复点、非有限值、旋转、缓冲扩容复用）通过。
三帧输出逐字节校验，点记录字段不变；已验证安装后的共享库能加载。未运行完整 ROS 服务。

每帧预热一次，之后各测 6 次；下表为各帧中位数范围：

| 路径 | 裁剪含方盒准备 | 裁剪与离群过滤 |
|---|---:|---:|
| 原 CPU | 7.40–7.58 ms | 58.62–59.53 ms |
| 优化 CPU | 5.94–6.10 ms | 57.15–58.09 ms |
| CUDA | 2.96–3.21 ms | 54.27–55.20 ms |

CUDA 耗时包括输入转换、主机/设备传输、同步和返回掩码；均不含 TF 查询、ROS 传输、输出点云打包、可视化。
只使用 3 帧历史数据，尚未验证与实时建图同时运行时的表现。
原始报告：`reports/cuda_crop_validation.json`。复测脚本：`scripts/check_cuda_crop.py`，不初始化 ROS。
