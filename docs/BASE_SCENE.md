> 本页保留基础场景的安装、资源恢复和运行说明。接口字段以 [SIMHEADER_HANDOFF.md](SIMHEADER_HANDOFF.md) 和当前 msg/srv 为准；旧协议描述仅为历史记录。以下路径及命令以仓库根目录为起点。

# underground-disturb-sim

地下工作面仿真平台。当前提交范围是**基础场景搭建与理想观测链路**，用于保存已完成的可运行基线，不代表扰动引擎或团队整链已完成。

## 已实现

- Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11.10.2；ROS 2 launch 统一启动 Gazebo、RViz、TF 和理想 IMU。
- SubT 拼块资源级复用：200 m 工作面、两条直段各 60 m 的巷道，16 个直段与 2 个弯段。
- 133 个简化液压支架、20 段输送机、煤壁与采煤机。设备参数为开发示意，待实际矿井资料确认。
- 模型内轨迹插件每个物理步控制采煤机；独立 p3d 插件发布 `/ground_truth`。
- 已完成完整往复验证及控制回跳修复。历史证据见 `evidence/control_fix_20261029.json`。
- 2026-11-04：加入 `coal_platform_interfaces` 与 `coal_platform_bridge`，实际发布带场景/运行上下文的真值和二维扫描，支持恒等 `map→world`；原标准话题保留。新版入口由 `scripts/run_scene.sh` 调用。
- 2026-11-05：整理 LiDAR 开发交付基线与真实静态几何 QueryRay，提供完整模型ZIP的安装器和自动验收脚本。接口约定见 [LiDAR交付说明](LIDAR_HANDOFF.md) 和 [配置](../config/lidar_interfaces.yaml)。

## 首次准备

在已安装 ROS 2 Humble 的 Ubuntu 22.04 中：

```bash
git clone https://github.com/wen37/underground-disturb-sim.git
cd underground-disturb-sim
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev ros-humble-rviz2 \
  ros-humble-tf2-ros ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
  build-essential cmake python3-colcon-common-extensions python3-numpy python3-yaml
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

当前已提供开发锚点布局下的理想UWB测距、RobotState、真实场景静态碰撞几何QueryRay和独立模拟查询。多径/NLOS注入、粉尘误差模型及整链评估尚待接入。二维 LaserScan 不是三维点云。当前为预设运动学简化体，不验证采煤机牵引动力学。扫描边缘存在逐束采样时序导致的整帧几何核对差异；不能把整帧单一位姿比较的最大距离差当作随机传感器噪声。

## 新版真值与扫描接口

`src/coal_face_sim/config/scene_metadata.yaml` 配置当前场景 `coal_face_base`、版本0.2.0和设备ID。启动场景后，适配器从真实 `/ground_truth` 和 `/scan_raw` 发布 `/platform/v1/ground_truth` 与 `/platform/v1/scan_raw`，保留采样戳和数据，补入同一次启动共享的run_id及逐帧observation_id。每次启动适配器自动生成新run_id。`/platform/v1/run_context`为可靠、持久的上下文辅助话题。

当前只支持map与world完全重合；新版真值frame为map，扫描仍为laser_link。原p3d插件速度在world轴表达，在此恒等变换下直接保留；切换其他真值源或非恒等坐标前须实现相应变换。RobotState、UWB raw、真实QueryRay及独立QueryRay模拟服务现已实现。离线模型交付ZIP配套安装器会在收件电脑重新编译并自动验收；它不复制系统镜像。Git源码使用上述首次准备流程，完整ZIP使用包根目录install.sh。

第三方来源与修改见 [THIRD_PARTY.md](../THIRD_PARTY.md)。运行截图、视频、完整 rosbag、构建产物和个人工作文档不纳入首版代码托管。历史测试不是 GitHub CI，也不等于在另一台电脑上的安装签收。


## B C 开发输入

`coal-face`或`run_scene.sh`一并启动`bc_inputs.py`。`/platform/v1/robot_state`以50Hz发布工况与区域；`/platform/v1/uwb_ranges_raw`对9个开发锚点各10Hz发布无噪声理想测距。超120m时raw_valid=false、raw_range_m=NaN，truth_range_m保留。配置见`src/coal_platform_bridge/config/bc_inputs.json`。

`/mock/scene/query_ray`仅用于B的固定用例联调，响应均有MOCK_ONLY标记；不会占用真实服务名`/scene/query_ray`，不计算巷道实际遮挡与反射。B最小示例：`ros2 run coal_platform_bridge query_mock_example.py`。

默认工况按仿真时钟0～5s TRAVELLING、5～15s CUTTING、15s后TRAVELLING；区域为显式开发多边形，C从这些事实生成自己的粉尘状态。本节点不发布dust_active或注入噪声。场景版本0.2.0记录新增锚点/区域/工况语义，接口协议仍0.1.1。

## 真实 QueryRay

平台启动同时运行 `coal_scene_query`，服务名 `/scene/query_ray`，类型 `coal_platform_interfaces/srv/QueryRay`。它读取当前 `coal_face.sdf` 及本地模型的 collision，不使用固定模拟用例。`/platform/v1/query_info` 发布持久的实际几何来源、文件哈希及限制。实际调用示例：

```bash
ros2 run coal_platform_bridge query_real_example.py
```

需要向升级前已启动的场景单独接入查询服务时，可执行 `ros2 launch coal_platform_bridge real_query.launch.py`；若真实服务已经启动，请勿重复启动。请求复制对应 raw 的 context 和 observation_id，生成新的 request_id，输入 map 系米制起终点。当前只处理静态几何，排除本采煤机；未知的其他动态碰撞体会阻止节点启动。请求时间须属于当前 run，不能晚于已接收真值，也不能落后超过2仿真秒。半径范围0～5m，线段长度大于1μm且不超过500m。

返回按距离排序的碰撞交点与半径内最近表面候选。BOX有入/出点、厚度和穿过路径长度；MESH只提供表面交点，体厚与穿透长度用NaN表示，禁止当作0。材质与反射等级未知时返回UNKNOWN；候选表面不等于有效反射路径。共面BOX按闭实体处理，贴面穿行会带提示；相邻或重叠实体的路径区间需由消费端合并，不能直接累加。源文件改变后拒绝继续使用旧索引。服务不生成UWB偏差或粉尘误差，传播模型仍由B/C实现。详细记录见 `docs/真实QueryRay接入说明_20261104.md`。
