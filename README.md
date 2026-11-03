# underground-disturb-sim

地下工作面仿真平台。当前提交范围是**基础场景搭建与理想观测链路**，用于保存已完成的可运行基线，不代表扰动引擎或团队整链已完成。

## 已实现

- Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11.10.2；ROS 2 launch 统一启动 Gazebo、RViz、TF 和理想 IMU。
- SubT 拼块资源级复用：200 m 工作面、两条直段各 60 m 的巷道，16 个直段与 2 个弯段。
- 133 个简化液压支架、20 段输送机、煤壁与采煤机。设备参数为开发示意，待实际矿井资料确认。
- 模型内轨迹插件每个物理步控制采煤机；独立 p3d 插件发布 `/ground_truth`。
- 已完成完整往复验证及控制回跳修复。历史证据见 `evidence/control_fix_20261029.json`。

## 首次准备

在已安装 ROS 2 Humble 的 Ubuntu 22.04 中：

```bash
git clone https://github.com/wen37/underground-disturb-sim.git
cd underground-disturb-sim
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev ros-humble-rviz2 \
  ros-humble-tf2-ros python3-colcon-common-extensions python3-numpy
python3 scripts/restore_assets.py
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
bash scripts/run_scene.sh
```

模型纹理及网格约 113 MiB，保留本地离线归档，不重复提交进 Git。恢复脚本按固定 Fuel 版本下载并进行 SHA-256 校验，首次需要网络。若已有本项目归档的 `models/`，可完全离线恢复：

```bash
python3 scripts/restore_assets.py --from-local /path/to/archived/models
```

原开发机的归档路径为 `~/coal_sim_ws/src/coal_face_sim/scenarios/coal_face_v1/models`。该路径只是可选导入来源，运行代码不依赖该绝对路径。资源准备后场景运行无需 Fuel 联网。

## 日常启动

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/run_scene.sh
# 无窗口运行
bash scripts/run_scene.sh gui:=false rviz:=false
```

默认使用 ROS_DOMAIN_ID=90、Gazebo 端口 11990 与软件渲染，可通过环境变量覆盖。Ctrl+C 停止。一个端口只启动一个场景实例。

## 参数与接口

| 项目 | 当前设置 |
|---|---|
| 工作面几何长度 | 200 m；运动行程与此不同 |
| 采煤机中心 | X=0.6 m，Z=0.6605 m，Y=5～195 m |
| 轨迹 | 余弦往复，完整周期 1200 仿真秒，单程 190 m |
| 最大速度 | 约 0.4974 m/s，两端减速至零 |
| 物理步长 | 0.001 s；默认一倍速 |
| `/ground_truth`、`/shearer/reference` | nav_msgs/msg/Odometry，50 Hz |
| `/scan_raw` | sensor_msgs/msg/LaserScan，720 束，约 10 Hz |
| `/imu_raw` | sensor_msgs/msg/Imu，运动学基线，50 Hz |
| 坐标 | world → base_link → laser_link / imu_link |

参数配置在 `src/coal_face_sim/scenarios/coal_face_v1/worlds/coal_face.sdf` 的 `coal_face_trajectory` 插件中。布局实例见 `design_v02/layout_parameters.json`。

## 状态边界

本仓库尚未包含 UWB 原始测距、锚点正式配置、多径/NLOS 注入、激光粉尘误差模型、统一 QueryRay 接口或整链评估；这些需要后续对接。二维 LaserScan 不是三维点云。当前为预设运动学简化体，不验证采煤机牵引动力学。扫描边缘存在逐束采样时序导致的整帧几何核对差异；不能把整帧单一位姿比较的最大距离差当作随机传感器噪声。

第三方来源与修改见 [THIRD_PARTY.md](THIRD_PARTY.md)。运行截图、视频、完整 rosbag、构建产物和个人工作文档不纳入首版代码托管。历史测试不是 GitHub CI，也不等于在另一台电脑上的安装签收。
