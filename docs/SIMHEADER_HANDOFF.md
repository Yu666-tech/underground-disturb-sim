# 统一 SimHeader 接口与交接

本次首次向 B/C 交付的接口版本为 **0.2.0**。以 `src/coal_platform_interfaces` 的 `msg/srv` 和 `config/platform_interfaces.json` 为准。

模块使用入口：[UWB](UWB_QUICKSTART.md)、[LiDAR](LIDAR_QUICKSTART.md)、[场景](SCENE_QUICKSTART.md)。

## 公共字段与单位

`SimHeader` 含六项字段：

```text
stamp
frame_id
experiment_id
run_id
scene_id
scene_version
```

所有业务消息与 QueryRay 请求响应通过 `RunContext.header` 复用这些字段。`schema_version`、`robot_id`、`sequence` 仍在 `RunContext`。

字段访问例：

```python
raw.context.header.run_id
```

- `stamp` 是仿真采样时间。
- 距离、角度、时间使用 m/rad/s，浓度使用 mg/m³。
- 查询响应完整回显 `context`、`request_id`、`observation_id`。

## 消息结构与字段映射

坐标复用 `geometry_msgs/Point`，速度复用 `Twist`；CSV 的 x/y/z 从对应结构展开。

| 消息字段或 CSV 列 | 对应位置或字段 |
| --- | --- |
| `GroundTruth.robot_id`、`GroundTruth.sequence` | 取 `context`；其他消息同理 |
| QueryRay 响应的场景版本 | `served_scene_version` |
| `Intersection.distance_m` | `distance_from_origin_m` |
| CSV `target_dust_concentration` | `LidarDecision.target_concentration_mg_m3` |
| CSV `actual_dust_concentration` | `LidarDecision.actual_concentration_mg_m3` |
| CSV `drop_count`、`false_point_count` | `dropout_count`、`false_return_count` |
| robot_state CSV `cmd_linear_speed` | `commanded_linear_speed_mps` |
| robot_state CSV `cmd_yaw_rate` | `commanded_yaw_rate_radps` |

LiDAR 数组与计数约定：

- `LidarBeamState` 复用逐帧数组设计，`beam_index`、`beam_states`、`delta_range_m`、`observed_valid` 均为 N 项，CSV 逐束展开。
- `LidarFrameObserved` 包含 `truth_ranges_m`。
- `UwbDecision` 包含 `anchor_id`；`LidarDecision` 包含 `sensor_id`、`total_beams`。
- `total_beams` 必须等于六种状态计数之和。

## 输入、输出与查询

A 的业务输入沿用 `/platform/v1/` 前缀：

| 方向 | 话题 |
| --- | --- |
| A → 算法模块 | `/platform/v1/ground_truth` |
| A → 算法模块 | `/platform/v1/robot_state` |
| A → 算法模块 | `/platform/v1/uwb_ranges_raw` |
| A → 算法模块 | `/platform/v1/scan_raw` |
| UWB 判定回传 | `/disturbance_decision/uwb` |
| UWB 扰动观测回传 | `/uwb_ranges` |
| LiDAR 判定回传 | `/disturbance_decision/lidar` |
| LiDAR 逐束状态回传 | `/lidar_beam_state` |
| LiDAR 扰动观测回传 | `/scan` |

Gazebo 原生 `/ground_truth` 为 `Odometry`，`/scan_raw` 为 `LaserScan`，与公共业务消息类型不同。B/C 使用本基线话题或显式 remap，不能混用原生话题。

| 查询入口 | 用途 |
| --- | --- |
| `/scene/query_ray` | 真实场景几何查询 |
| `/mock/scene/query_ray` | Mock 几何事实查询 |

查询返回几何事实，不把几何命中事实当作 LOS/NLOS 结论。

## 公共 Mock 与可视化

公共 fixtures 位于 `coal_platform_devtools/config`：

- `fixture_cases.json` 提供原始输入数组与几何情形。
- `return_chain.json` 为 `MOCK_ONLY` 静态回传样例，其中 LOS/零误差只是传输样例，不是算法预期。

在隔离的 `ROS_DOMAIN_ID` 中运行：

```bash
ros2 launch coal_platform_devtools fixtures.launch.py
```

Mock 不启动真实查询服务。

`MarkerArray` 内各 `Marker.header` 与 `PointCloud2.header` 保持标准 Header，不能直接换成 SimHeader。B/C 必须保留同戳 frame，关联对应 decision 的 context；业务记录使用 decision 的六项身份字段。可视化不参与业务计算。

## CSV 约定

九张表头位于 `config/csv_templates`，旧 `csv_baseline` 仅作历史资料。

- 六项公共字段均取 `context.header`。
- stamp 导出为 `sec.nanosec`，纳秒补齐九位。
- 数组列表列编码为 JSON；无效测距用 NaN；布尔值统一 true/false。
- `run_manifest` 的提交号不得编造。未知时记录 UNKNOWN，上传现有仓库后按实际提交号填写。

## 还需交付

### UWB 模块

- `config/algorithms/uwb.yaml` 初版，声明 `schema_version: '0.2.0'` 和 `params_version`。
- 算法实现、专项 fixtures、状态转换与多径/NLOS/随机误差回归。
- 两类回传消息与 UWB 可视化。
- 保持 raw 身份、同戳、`decision_id` 与 `observation_id` 关联。

### LiDAR 模块

- `config/algorithms/lidar_dust.yaml` 初版，声明相同的接口及参数版本字段。
- 粉尘时序与扰动算法、专项 fixtures、逐束状态计数一致性与回归。
- 三类回传消息与彩色点云。

A 不代写 B/C 算法参数。参数提交后执行：

```bash
python3 scripts/validate_bc_params.py B config/algorithms/uwb.yaml
python3 scripts/validate_bc_params.py C config/algorithms/lidar_dust.yaml
```

该检查只保证 YAML 结构和接口版本，算法参数语义由 B/C 负责；尚不能声称已验证未提交文件。

### 场景模块

- 由 D 创建空场景包并通过 `scene_validator.py`，才算 D0；A 的 `reference_s01_skeleton` 通过不代表 D0 已完成。
- 后续交付 S01/S02/S03 的真实资产、语义、路线、真实 `preview.png`、README 资源来源与许可、已知问题及人工事件建议。
- 参考骨架不是完整可仿真 S01；约束见 `coal_scene_tools/docs/loader_constraints.md`。
- 场景 loader 解析 scenario、semantics、route；事件建议内容不被程序解析。
- 现有 loader 的 Gazebo 模式仅几何预览，真实场景的路线与挂载联调须单独验收。

## 仓库协作与安装边界

接口基线通过功能分支与 PR 合入 main；B/C/D 从最新 main 创建各自分支提交。发布状态以仓库实际 PR 为准。

本次作为接口联调源码提交，保留现有仓库的安装与资源恢复文件。现有 `install.sh` 面向完整离线包，依赖 `SHA256SUMS.json/RELEASE.json`，不属于本次已验证的源码安装入口。README 的构建步骤以依赖已安装的环境为前提。
