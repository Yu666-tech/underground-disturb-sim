# UWB 模块上手与交付说明

2026-11-06 已完成最小 Ubuntu 22.04 + ROS2 Humble 隔离环境的首次上手验收，见 [验收方法与结果](ONBOARDING_VALIDATION.md)。

本文发给负责 UWB 误差模块的同学。已有 Ubuntu 22.04 和 ROS2 Humble 时，可以在自己的电脑使用公共 Mock 开发，无需 Gazebo 或场景模型。平台不包含已完成的 UWB 扰动算法，本文说明输入、接口和还需交付的内容。

## 下载与构建

以下命令用于首次下载。已经克隆过则进入现有目录，保存自己的修改后更新 main，不重复克隆。

```bash
sudo apt update
sudo apt install -y git build-essential cmake python3-dev \
  python3-colcon-common-extensions \
  ros-humble-tf2-ros ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime
cd ~
git clone https://github.com/wen37/underground-disturb-sim.git
cd underground-disturb-sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to coal_platform_devtools
source install/setup.bash
```

构建范围是公共接口和 Mock 工具，不要求构建 Gazebo 包。后续路径按自己实际的克隆位置调整。

## 运行输入与查询

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
python3 src/coal_platform_bridge/scripts/query_mock_example.py
```

这里直接执行仓库内的 Python 查询示例，不需要构建依赖 Gazebo 的 bridge 包。示例接收一条有效 raw，复制上下文和 observation_id，向 `/mock/scene/query_ray` 发起请求，输出带 `MOCK_ONLY` 标记的事实。

- 原始测距：`/platform/v1/uwb_ranges_raw`，每个 tag-anchor 观测一条消息。本分支发布端与 UWB/CSV 订阅端统一使用 RELIABLE，避免把传输缺失混作仿真测距失效；旧 BEST_EFFORT 订阅端仍兼容新发布端，旧 BEST_EFFORT 发布端不能与新的 RELIABLE 订阅端混用。
- 真值参考：`/platform/v1/ground_truth`。
- Mock 查询：`/mock/scene/query_ray`；真实场景查询：`/scene/query_ray`。Mock 不冒充真实查询服务。
- 自己的算法节点使用相同 domain，并设置 `use_sim_time=true`；Mock 发布 `/clock`。
- 几何相交与反射候选只是事实，LOS/NLOS/OUTAGE、触发原因及各类误差由 UWB 算法判定。

自动检查公共输入前，先停止终端一的发布器：

```bash
ros2 run coal_platform_devtools check_fixtures.py --launch \
  --output fixture_verification.json
```

该检查只验证公共输入与服务，不能代替专项算法验收。

## 还需交付

先阅读 [统一接口与交接](SIMHEADER_HANDOFF.md)，消息与服务定义位于 `src/coal_platform_interfaces/`。

- 参数：`config/algorithms/uwb.yaml`，非空 YAML mapping，包含字符串 `schema_version: '0.2.0'` 和非空字符串 `params_version`；具体参数由 UWB 模块负责人确定。
- 算法：传播状态判定、多径/NLOS/随机噪声误差，以及损坏输入、超量程和查询失败时的处理。代码放在独立 ROS 包中，例如 `src/coal_uwb_disturbance/`，不要修改公共 Mock 来代替算法。
- 回传：`/disturbance_decision/uwb` 的 `UwbDecision`、`/uwb_ranges` 的 `UwbRangeObserved`，以及 UWB 链路可视化。
- 关联：继承 raw 的 context、observation_id 和采样戳，判定与观测共享 decision_id，不覆盖 raw。六项公共身份字段位于 context.header。
- 测试：自己的专项 fixtures、状态转换及误差分项回归，并记录随机种子和参数版本。公共静态 LOS/零误差样例只用于消息传输验证。

UWB 五张 CSV 表头在 `config/csv_templates/`：`uwb_observations`、`state_transitions`、`query_trace`、`multipath_error_detail`、`uwb_query_arrays`，文件均带 `.template.csv` 后缀。当前交付的是表头，完整导出链路还需后续联调。

参数文件提交后检查：

```bash
python3 scripts/validate_bc_params.py B config/algorithms/uwb.yaml
```

该命令只检查结构与版本。算法包创建后，按其实际包名构建，例如 `colcon build --symlink-install --packages-up-to coal_uwb_disturbance`，重新 source 后用自己的实际入口启动；仓库当前没有该算法入口。

## 提交方式

开发前保存已有修改，再创建自己的分支：

```bash
git switch main
git pull --ff-only
git switch -c feat/uwb-initial
```

完成开发、构建和回归后：

```bash
git add src/coal_uwb_disturbance config/algorithms/uwb.yaml
git commit -m "feat: add initial uwb disturbance"
git push -u origin feat/uwb-initial
```

上述包名只是建议，按实际包名调整。随后在 GitHub 创建目标为 main 的 PR；无写权限时先 Fork，再向原仓库 main 提 PR。不要直接推送 main，不提交 build/install/log 缓存。

需要真实 Gazebo 联调时，按 [基础场景说明](BASE_SCENE.md) 准备依赖和模型。LiDAR 由对应模块负责人实现，场景资产由场景负责人提交，公共接口变化先与平台负责人对齐。
