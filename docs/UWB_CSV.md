# UWB 分类与五张 CSV

本地 PR #5 测试分支的接口扩展，尚未合入主分支。`schema_version` 保留 0.2.0；新增字段改变 ROS 消息结构，生产者和消费者必须使用本分支重新构建的同一套接口，不能混用旧 install。原来的开发工作区不自动更新。

时序容差、可靠传输和负载评估见 [UWB_TIMING.md](UWB_TIMING.md)。

## 分类语义

`UwbDecision.nlos_type`、`UwbRangeObserved.nlos_type`、`UwbQueryTrace.nlos_type` 使用小写 `none/soft/hard`。

| 传播状态 | nlos_type | 含义 |
| --- | --- | --- |
| LOS | none | 直达无遮挡 |
| NLOS | soft | UWB 模块判为软遮挡 |
| NLOS | hard | UWB 模块判为硬遮挡 |
| OUTAGE / INPUT_INVALID / UNKNOWN | none | 没有可用 NLOS 分类，必须结合 valid/state 判断，不能当成 LOS |

本改动传递现有算法分类，不改薄墙阈值、反射候选判断、噪声和偏置参数。查询调用异常记为 `INTERNAL_ERROR`；没有调用时仍记为 `NOT_CALLED`。`none` 是无分类值，不是所有情况下都表示无误差。

新增 `/disturbance_query/uwb`（`UwbQueryTrace`），由实际查询发起者回传请求 ID、结果状态、几何数组、处理耗时和对应的 decision/classification。无效 raw、服务尚未就绪时不伪造查询行。查询返回空几何数组时，数组明细表不会增加占位行。

## 五张 CSV

| 表 | 新增内容 |
| --- | --- |
| uwb_observations.csv | nlos_type、valid、tag_id |
| state_transitions.csv | nlos_type、old_nlos_type、new_nlos_type、tag_id |
| query_trace.csv | nlos_type、valid、decision_id、tag_id、processing_time_ms、error_message |
| multipath_error_detail.csv | nlos_type、valid、tag_id |
| uwb_query_arrays.csv | nlos_type、valid、decision_id、anchor_id、tag_id、query_array_kind |

状态切换表只记录同一 run/robot/tag/anchor 的相邻状态变化；比较 `(propagation_state, nlos_type)`，因此 `NLOS/soft → NLOS/hard` 也会记录。`nlos_type` 等于切换后的 `new_nlos_type`。每条链路的第一条观测只建立初始状态，不伪造上一状态。

录制器按业务身份和 observation_id 关联 raw、decision、observed、query，检查 context、anchor/tag、decision_id、分类一致性后写入。重复消息不重复写行；矛盾的同 ID 数据报错，停止本次录制。SQLite 文件保留关联事实，避免将长期实验数据全部缓存在 Python 内存中。

观测、误差和查询表按接收完成顺序写出，保留原始采样 stamp；状态切换表每秒及正常退出时，按链路和采样 stamp/sequence 重建快照并原子替换，避免查询回包乱序造成虚假切换。消费者需按采样 stamp 排序，不能把 CSV 行号视为时间顺序。这里不做因子图插值或修改采样时间。

公共六字段来自 context.header，stamp 纳秒补齐九位；数组列为 JSON，布尔值为 true/false，无效观测为 NaN。`valid` 是观测分类有效性；query_trace 的 `status` 才表示查询是否成功。

## 构建与运行

在此测试仓库的 Ubuntu 终端运行，先 source ROS，再构建：

首次克隆后启动真实场景前，先按 [基础场景说明](BASE_SCENE.md) 恢复模型资源，再构建安装；在构建之后才新增资源时，需要重新安装场景包。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

先启动原有 Mock 或真实场景；在另一个使用相同 ROS_DOMAIN_ID、相同 install 的终端启动 UWB 算法和录制器。输出目录必须尚不存在，避免覆盖旧实验：

```bash
# Mock：另一个终端运行 fixtures.launch.py
ROS_DOMAIN_ID=108 ros2 launch coal_platform_bridge uwb_recording.launch.py \
  query_service:=/mock/scene/query_ray \
  config_file:=$PWD/config/algorithms/uwb.yaml \
  output_dir:=$HOME/uwb_csv_runs/mock_001

# 真实场景：先按原有入口运行 coal_face_interfaces.launch.py
# 在与真实场景相同的 ROS_DOMAIN_ID 下执行：
ros2 launch coal_platform_bridge uwb_recording.launch.py \
  query_service:=/scene/query_ray \
  config_file:=$PWD/config/algorithms/uwb.yaml \
  output_dir:=$HOME/uwb_csv_runs/scene_001
```

不要同时运行两个 UWB 算法实例。为保留录制末尾的查询结果，建议分终端运行算法和录制器：先停止 raw 发布，等待查询排空，再 Ctrl+C 停止录制器。录制目录内的 recording_summary.json 报告未完成观测关联与未匹配查询数；不能将非零计数默认为零丢包。强制杀进程会失去正常退出汇总，需要重新核查。

本节点只实现五张 UWB 表，不声称已实现 LiDAR 两表或公共两表。原始接口、算法专项 fixtures、可视化、动态反射路径验证和 episode 偏置重置，按既定职责另行交付。

## 本步验证入口

```bash
python3 scripts/test_uwb_query_failures.py
python3 scripts/test_uwb_csv.py
ROS_DOMAIN_ID=117 python3 scripts/check_uwb_csv_chain.py /tmp/uwb_csv_contract_new
```

最后一项通过实际 UWB 节点、ROS 消息和录制器验证 none/soft/hard 以及失败输入的文件输出，数据明确为 MOCK_ONLY。它不替代 UWB 同学的物理模型与算法专项验收。时序容差仍是现有实现，50/100ms 对照实验在下一审批阶段执行。
