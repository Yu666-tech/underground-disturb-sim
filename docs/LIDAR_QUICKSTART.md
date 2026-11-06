# LiDAR 模块上手说明

已有 Ubuntu 22.04 和 ROS2 Humble 时，可先用公共 Mock 开发 LiDAR 粉尘扰动，不需要 Gazebo、模型下载或 A 的电脑目录。下列命令在自己的 Ubuntu 终端运行，默认从自己的用户主目录克隆；已经克隆过则进入现有目录，先更新 main，不重复克隆。

## 下载与构建

```bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions \
  ros-humble-tf2-ros ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime
cd ~
git clone https://github.com/wen37/underground-disturb-sim.git
cd underground-disturb-sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to coal_platform_devtools
source install/setup.bash
```

本次只构建接口和 Mock 工具，因此不会要求构建 Gazebo 场景包。这里的 `~/underground-disturb-sim` 是自己电脑的目录；若克隆到其他位置，后续 `cd` 路径相应调整。

## 运行输入与查看消息

终端一，保留运行：

```bash
cd ~/underground-disturb-sim
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=108
ros2 launch coal_platform_devtools fixtures.launch.py
```

另开终端二：

```bash
cd ~/underground-disturb-sim
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=108
ros2 topic list
ros2 topic echo /platform/v1/robot_state --once
```

`/platform/v1/scan_raw` 提供 720 束原始扫描，`/platform/v1/robot_state` 提供区域和工况；Mock 同时发布 `/clock`。算法节点使用同一 domain，并设置 `use_sim_time=true`。原始扫描为 BEST_EFFORT QoS，订阅端使用兼容的 sensor-data QoS；不要直接用默认 RELIABLE 订阅原始扫描。该数据带 MOCK_ONLY 标记，不是实际场景或粉尘算法效果。

自动检查公共输入时，先停止终端一的发布器，再执行：

```bash
ros2 run coal_platform_devtools check_fixtures.py --launch \
  --output fixture_verification.json
```

此命令自行启动和停止 Mock，只验证公共输入与服务。

## 开发与交付

先阅读 [统一接口说明](SIMHEADER_HANDOFF.md)。算法代码应放在独立 ROS 包中，例如 `src/coal_lidar_disturbance/`，不要把算法写入公共 Mock 发布器。

还需交付 `config/algorithms/lidar_dust.yaml`、粉尘时序与激光扰动算法、专项 fixtures 和回归测试。参数文件为非空 YAML mapping，包含字符串 `schema_version: '0.2.0'` 和非空 `params_version`；具体浓度、时间常数和扰动参数由模块负责人确定。

- 订阅 `/platform/v1/scan_raw` 和 `/platform/v1/robot_state`，根据 zone_id、work_mode 生成目标/实际粉尘浓度及 dust_active，再实现丢点、假点和测距扰动。
- 输出 `/disturbance_decision/lidar`、`/lidar_beam_state`、`/scan` 及彩色点云；保留 raw 的上下文和 observation_id，同帧回传共享 decision_id，不覆盖 raw。
- 六项公共身份字段在 `context.header`；robot_id、sequence、schema_version 在 `context`。
- `LidarBeamState` 为逐帧 N 项数组；total_beams 须等于六种光束状态计数之和；扫描保留 truth_ranges_m。两张 LiDAR CSV 表头位于 `config/csv_templates/`。

参数提交后检查：

```bash
python3 scripts/validate_bc_params.py C config/algorithms/lidar_dust.yaml
```

算法包创建后按其实际包名构建，例如 `colcon build --symlink-install --packages-up-to coal_lidar_disturbance`，重新 source 工作区，再使用该包的实际 launch 或可执行入口启动；当前仓库尚不包含该算法入口。

提交方式：工作区有未提交修改时先提交或妥善保存，再更新主分支并创建开发分支。

```bash
git switch main
git pull --ff-only
git switch -c feat/lidar-dust-initial
# 开发、构建和验证后，只添加本模块相关源码与参数
git add src/coal_lidar_disturbance config/algorithms/lidar_dust.yaml
git commit -m "feat: add initial lidar dust disturbance"
git push -u origin feat/lidar-dust-initial
```

随后在 GitHub 创建目标为 main 的 PR。没有仓库写权限时，先在 GitHub Fork，克隆自己的 Fork 后开发，再向原仓库 main 提 PR。不要直接推送 main，也不要提交 build/install/log 缓存。真实 Gazebo 联调及资源恢复见 [基础场景说明](BASE_SCENE.md)。
