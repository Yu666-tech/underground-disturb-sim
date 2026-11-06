# 扰动仿真平台（Coal Disturbance Sim）

> ROS2 Humble · Gazebo 11 · 统一 SimHeader 接口 v0.2.0<br>
> 四人协作项目：A（平台）/ B（UWB误差）/ C（LiDAR粉尘）/ D（场景）

---

## 这是什么

一个面向煤矿井下工作面的**扰动仿真平台**：在 Gazebo 里模拟机器人运动，A 输出 ground truth / UWB raw / LiDAR raw，B/C 分别叠加 UWB 误差和粉尘扰动，目标是导出 9 张标准 CSV 用于算法验证。目前已提供九张表头模板，完整导出链路待后续联调。

当前状态：**A 侧底座已完成本地验收**。

---

## 快速开始

首次克隆、依赖安装和模型恢复见 [基础场景说明](docs/BASE_SCENE.md)。以下路径按自己的克隆目录调整；原有场景和资源恢复方式保留。

```bash
# 已安装依赖的环境：WSL Ubuntu 22.04 + ROS2 Humble + Gazebo 11
cd ~/coal_sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 完整本地验收（包含真实 Gazebo 仿真）
bash scripts/accept_simheader.sh

# Mock 联调（不开 Gazebo，读取本地 fixtures）
ROS_DOMAIN_ID=108 ros2 launch coal_platform_devtools fixtures.launch.py
```

Mock 联调时，B/C 节点也使用 `ROS_DOMAIN_ID=108`；不要在同一 domain 同时启动真实仿真与 mock。

---

## 分工与交付

### 🟦 B — UWB 误差模块

**你要做的：** 提交 `config/B/uwb.yaml` + UWB 误差算法代码 + 专项 fixtures。

**重点看：**

- 话题：`/platform/v1/uwb_ranges_raw`（输入）→ `/disturbance_decision/uwb`、`/uwb_ranges`（输出）
- 接口：`UwbDecision.msg`（含 `anchor_id`）、`UwbRangeObserved.msg`
- 校验工具：`python3 scripts/validate_bc_params.py B config/B/uwb.yaml`（检查结构与版本）
- 详情文档：[docs/SIMHEADER_HANDOFF.md](docs/SIMHEADER_HANDOFF.md)

### 🟥 C — LiDAR 粉尘扰动模块

**你要做的：** 提交 `config/C/lidar_dust.yaml` + 粉尘时序与激光扰动算法 + 专项 fixtures。

**重点看：**

- 话题：`/platform/v1/scan_raw`、`/platform/v1/robot_state`（输入）→ `/disturbance_decision/lidar`、`/lidar_beam_state`、`/scan`（输出）
- 接口：`LidarDecision.msg`（含 `sensor_id`、`total_beams`）、`LidarBeamState.msg`（逐束数组）、`LidarFrameObserved.msg`（含 `truth_ranges_m`）
- 约束：`total_beams` 必须等于六种光束状态计数之和
- 校验工具：`python3 scripts/validate_bc_params.py C config/C/lidar_dust.yaml`（检查结构与版本）
- 可视化：MarkerArray / PointCloud2 保留标准消息结构与 Header，业务身份从对应 decision 的 `context.header` 关联
- 详情文档：[docs/SIMHEADER_HANDOFF.md](docs/SIMHEADER_HANDOFF.md)

### 🟩 D — 场景制作

**你要做的：** 先交付空场景包（D0），再交付 S01/S02/S03 正式场景资产。

**重点看：**

- 场景包结构模板：`src/coal_scene_tools/scenes/reference_s01_skeleton/`
- 校验命令：`ros2 run coal_scene_tools scene_validator.py ./你的场景包`
- ⚠️ **D0 完成标准：你自己新建一个空场景包，能通过 validator 校验。A 给的 `reference_s01_skeleton` 只是模板，不算 D0，也不是完整可仿真的 S01。**
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
├── B/                         # B 提交 uwb.yaml
├── C/                         # C 提交 lidar_dust.yaml
└── csv_templates/             # 9 张标准 CSV 表头模板
docs/                          # 接口与交接说明、验收记录
```

---

## 关键约定（所有人必读）

1. **公共字段位置**：六个公共字段位于 `context.header`；`robot_id`、`sequence`、`schema_version` 位于 `context`。
2. **话题前缀**：A 的业务输入统一用 `/platform/v1/`，避免和 Gazebo 原生话题冲突；真实查询为 `/scene/query_ray`，mock 查询为 `/mock/scene/query_ray`。
3. **ID 唯一性**：同一场景包内，`anchor_id` / `zone_id` / `geometry_id` 各自不重复。
4. **CSV 约定**：九张表头已提供；stamp 纳秒部分补齐九位，无效测距填 `NaN`，数组列编码为 JSON。
5. **建议提交流程**：从 `main` 拉功能分支 → PR → 评审合并；是否禁止直接 push main，由团队约定及分支保护设置决定。

---

## 许可证 & 资源

模型资产、场景素材的来源与许可信息，写在各自场景包的 `README.md` 里。
