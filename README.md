# 扰动仿真平台（Coal Disturbance Sim）

> ROS2 Humble · Gazebo 11 · 统一 SimHeader 接口 v0.2.0<br>
> 四人协作项目：A（平台）/ B（UWB误差）/ C（LiDAR粉尘）/ D（场景）

---

## 这是什么

一个面向煤矿井下工作面的**扰动仿真平台**：在 Gazebo 里模拟机器人运动，A 输出 ground truth / UWB raw / LiDAR raw，B/C 分别叠加 UWB 误差和粉尘扰动，目标是导出 9 张标准 CSV 用于算法验证。

目前已提供九张表头模板，完整导出链路待后续联调。

当前状态：**A 侧底座已完成本地验收**。

---

## 各模块交付入口

把对应链接发给模块负责人即可作为上手入口；文档包含下载、构建、运行、还需交付内容及 PR 提交方式，相关规范在文内链接。

| 负责模块 | 上手与交付说明 |
| --- | --- |
| UWB | [UWB 模块上手与交付](docs/UWB_QUICKSTART.md) |
| LiDAR | [LiDAR 模块上手说明](docs/LIDAR_QUICKSTART.md) |
| 场景 | [场景模块上手与交付](docs/SCENE_QUICKSTART.md) |

首次上手命令已进行独立环境验证：[验收方法与结果](docs/ONBOARDING_VALIDATION.md)。

## 快速开始

以下命令供首次接收仓库的 UWB / LiDAR 开发者，在各自的 Ubuntu 22.04（原生或 WSL）终端执行。已有工作区的维护者无需重复克隆；场景负责人按上方场景模块文档操作。

已有 ROS2 Humble 时，可以先使用 Mock 开发，无需 Gazebo 或场景模型。

```bash
# 下载源码到自己的电脑
sudo apt update
sudo apt install -y git build-essential cmake python3-dev \
  python3-colcon-common-extensions \
  ros-humble-tf2-ros ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime
cd ~
git clone https://github.com/wen37/underground-disturb-sim.git
cd underground-disturb-sim

# 仅构建公共接口与 Mock 工具
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to coal_platform_devtools
source install/setup.bash

# 启动 Mock（保留此终端运行）
ROS_DOMAIN_ID=108 ros2 launch coal_platform_devtools fixtures.launch.py
```

每个新终端都需要进入克隆目录并重新 `source` ROS 和工作区环境。

自己的算法节点也使用 `ROS_DOMAIN_ID=108`、`use_sim_time=true`，不要与真实仿真在同一 domain 同时启动。

LiDAR 从下载到开发的详细步骤见 [LiDAR 上手说明](docs/LIDAR_QUICKSTART.md)。

需要 Gazebo、模型恢复及完整验收时，按 [基础场景说明](docs/BASE_SCENE.md) 准备，再运行

```bash
bash scripts/accept_simheader.sh
```

---

## 分工与交付

### 🟦 B — UWB 误差模块

**还需交付：** 提交 `config/algorithms/uwb.yaml` + UWB 误差算法代码 + 专项 fixtures。

**重点看：**

- 话题：`/platform/v1/uwb_ranges_raw`（输入）→ `/disturbance_decision/uwb`、`/uwb_ranges`（输出）

- 接口：`UwbDecision.msg`（含 `anchor_id`）、`UwbRangeObserved.msg`

- 校验工具：

  ```bash
  python3 scripts/validate_bc_params.py B config/algorithms/uwb.yaml
  ```

  （检查结构与版本）

- 详情文档：[docs/SIMHEADER_HANDOFF.md](docs/SIMHEADER_HANDOFF.md)

### 🟥 C — LiDAR 粉尘扰动模块

**还需交付：** 提交 `config/algorithms/lidar_dust.yaml` + 粉尘时序与激光扰动算法 + 专项 fixtures。

**重点看：**

- 话题：`/platform/v1/scan_raw`、`/platform/v1/robot_state`（输入）→ `/disturbance_decision/lidar`、`/lidar_beam_state`、`/scan`（输出）

- 接口：`LidarDecision.msg`（含 `sensor_id`、`total_beams`）、`LidarBeamState.msg`（逐束数组）、`LidarFrameObserved.msg`（含 `truth_ranges_m`）

- 约束：`total_beams` 必须等于六种光束状态计数之和

- 校验工具：

  ```bash
  python3 scripts/validate_bc_params.py C config/algorithms/lidar_dust.yaml
  ```

  （检查结构与版本）

- 可视化：MarkerArray / PointCloud2 保留标准消息结构与 Header，业务身份从对应 decision 的 `context.header` 关联

- 详情文档：[docs/SIMHEADER_HANDOFF.md](docs/SIMHEADER_HANDOFF.md)

### 🟩 D — 场景制作

**还需交付：** 先交付空场景包（D0），再交付 `S01/S02/S03` 正式场景资产。

**重点看：**

- 场景包结构模板：`src/coal_scene_tools/scenes/reference_s01_skeleton/`

- 校验命令：

  ```bash
  ros2 run coal_scene_tools scene_validator.py ./你的场景包
  ```

- ⚠️ **D0 完成标准：你自己新建一个空场景包，能通过 validator 校验。

  A 给的 `reference_s01_skeleton` 只是模板，不算 D0，也不是完整可仿真的 S01。

  **

- 三个 yaml 文件：`scenario.yaml`、`scene_semantics.yaml`、`demo_route.yaml` 是程序解析的；`expected_events_suggestion.yaml` 只给人看，程序不读其内容

- 详情文档：[src/coal_scene_tools/docs/loader_constraints.md](src/coal_scene_tools/docs/loader_constraints.md)

---

## 仓库结构

```text
src/
├── coal_platform_interfaces/   # 公共 msg/srv 定义（SimHeader 在这里）
├── coal_platform_bridge/       # A：真实仿真桥接与公共输入
├── coal_platform_devtools/     # 无 Gazebo 的 mock / fixtures / 校验工具
├── coal_scene_tools/           # 场景包规范 + loader + validator
├── coal_scene_query/           # 射线查询 /scene/query_ray
└── coal_face_sim/              # Gazebo 仿真底座
config/
├── algorithms/                # UWB / LiDAR 算法参数（后续提交）
└── csv_templates/             # 9 张标准 CSV 表头模板
docs/                          # 接口与交接说明、验收记录
```

---

## 关键约定（所有人必读）

1. **公共字段位置**：六个公共字段位于 `context.header`；`robot_id`、`sequence`、`schema_version` 位于 `context`。

2. **话题前缀**：A 的业务输入统一用 `/platform/v1/`，避免和 Gazebo 原生话题冲突；真实查询为 `/scene/query_ray`，mock 查询为 `/mock/scene/query_ray`。

3. **ID 唯一性**：同一场景包内，`anchor_id` / `zone_id` / `geometry_id` 各自不重复。

4. **CSV 约定**：九张表头已提供；`stamp` 纳秒部分补齐九位，无效测距填 `NaN`，数组列编码为 JSON。

5. **建议提交流程**：从 `main` 拉功能分支 → PR → 评审合并；是否禁止直接 push main，由团队约定及分支保护设置决定。

---

## 许可证 & 资源

模型资产、场景素材的来源与许可信息，写在各自场景包的 `README.md` 里。
