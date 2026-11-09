# UWB 查询时序容差与负载评估

本说明对应本地 PR #5 测试分支。用户于 2026-11-09 确认采用 100ms，现已设为真实查询服务和启动入口的默认值；50ms 保留为对照评估起点，不是论文或 ROS 对本服务的统一规定。ROS 官方近似同步示例使用 0.05 秒消息匹配窗口，概念与本服务的查询准入容差不同，不能当作物理精度保证。

## 时序规则

真实 `/scene/query_ray` 提供 `future_tolerance_ms`（已审批默认 100.0）。静态场景允许：

```text
run 起点 <= request.stamp <= latest_truth.stamp + future_tolerance_ms
latest_truth.stamp - request.stamp <= 2 秒
```

实验、运行、机器人、场景及版本检查继续保留。容差不修改请求或回传的采样时间，不等待指定时长，也不外推机器人位姿；射线端点来自请求，查询的仍是当前加载的静态碰撞几何。动态场景需要另行设计时间状态查询，不能直接沿用这个放宽规则。

可选 `publish_timing_diagnostics:=true` 在 `/platform/v1/query_timing` 发布 JSON 诊断，包括请求/观测 ID、请求与最新真值的时间差、准入原因及服务处理耗时。默认关闭。`TIME_UNAVAILABLE` 的 error_message 区分 NO_TRUTH、IDENTITY_MISMATCH、BEFORE_RUN_START、FUTURE_BEYOND_TOLERANCE、OLDER_THAN_2S。

```bash
# 与原有场景启动参数兼容，gui/rviz 按本机环境调整。
ros2 launch coal_platform_bridge coal_face_interfaces.launch.py \
  future_tolerance_ms:=100.0 publish_timing_diagnostics:=true \
  gui:=false rviz:=false
```

## 完整回传与录制

UWB raw 的真实与公共 Mock 发布者、UWB 节点、observed 发布者及录制器统一使用 RELIABLE。旧 BEST_EFFORT 订阅者仍能接收新 RELIABLE 发布者；旧 BEST_EFFORT 发布者不能满足新 RELIABLE 订阅者，联调必须更新同一分支的完整链路。LiDAR 的原始扫描 QoS 不变。

服务客户端和服务端使用显式 RELIABLE、depth=512，避免沿用默认很小的服务历史队列。UWB 节点按单调墙钟设置 `query_timeout_s`（默认 5.0），查询没有结果时输出一次无效观测，status=INTERNAL_ERROR、追踪原因 RPC_TIMEOUT；释放 pending Future，迟到结果不会重复发布。这是异常处理期限，不是采样时序容差，也不是主动增加时延。

`publish_processing_diagnostics:=true` 可启用 `/platform/v1/uwb_processing`，记录收到 raw、已发查询、已完成、超时、待处理与已发布数量。两类诊断都是可选开发工具，不代替业务消息。

几何查询复用原有三角形最近点公式：网格采用静态空间索引，盒体采用批量的原 slab 方程。测试中的旧参考实现从第 2 步提交固定保存，只用于回归比较。仍返回完整命中与候选，不裁剪候选数量、不近似坐标、不把反射候选当作有效反射路径。

五张 UWB CSV 的字段与运行入口见 [UWB_CSV.md](UWB_CSV.md)。录制器的时间切换快照按采样时刻生成；其他表保持到达顺序。后续因子图应使用业务采样戳关联，不能用 CSV 行号或消息到达时刻替代。

CSV 行仍实时刷新到文件。用于关联/去重的 SQLite 索引改为每 64 条事实、定时刷新及正常退出时提交，减少逐消息提交的磁盘阻塞；评估器只在索引已提交且实际文件核对完成后认定落盘完成。应正常停止录制器后再收集结果，强制杀进程后的索引汇总不能用来证明完整性。

## 可重复的评估入口

完整构建并 source 本分支 install 后，每组使用独立 ROS_DOMAIN_ID、Gazebo 端口和新的输出目录。真实场景需要先按 [BASE_SCENE.md](BASE_SCENE.md) 恢复模型资源。

```bash
python3 scripts/test_query_timing.py
python3 scripts/test_query_acceleration.py
python3 scripts/test_timing_assessment.py
python3 scripts/test_uwb_query_failures.py

ROS_DOMAIN_ID=131 python3 scripts/check_query_time_contract.py \
  /tmp/query_boundary_50.json --tolerance-ms 50.0

ROS_DOMAIN_ID=132 python3 scripts/evaluate_uwb_timing.py \
  /tmp/timing_mock_50_new --profile mock --tolerance-ms 50.0 --duration-s 30

ROS_DOMAIN_ID=133 GAZEBO_MASTER_URI=http://127.0.0.1:11383 \
  python3 scripts/evaluate_uwb_timing.py \
  /tmp/timing_default_50_new --profile scene_default --tolerance-ms 50.0 --duration-s 60

ROS_DOMAIN_ID=134 GAZEBO_MASTER_URI=http://127.0.0.1:11384 \
  python3 scripts/evaluate_uwb_timing.py \
  /tmp/timing_stress_50_new --profile scene_90hz --tolerance-ms 50.0 --duration-s 60
```

`scene_90hz` 只在测试输出目录生成锚点配置，将九个锚点放在有效量程内，维持每条链路 10 次/仿真秒；真实碰撞场景和 UWB 数值参数不变。报告同时给出仿真时间与墙钟时间速率，不把 90 次/仿真秒等同于 90 次/墙钟秒。

Mock 保留其自身的预置查询规则；它验证公共数据集与完整回传，不直接验证真实服务的 50/100ms 门限。真实服务的纳秒边界由单独的 ROS QueryRay 测试覆盖。

评估从全部 raw 独立统计序号连续性；停止输入后保留真值与查询服务排空。另读取录制器已提交的 completed/query 状态，再逐项核对实际 CSV 的观测、误差、查询与几何数组，不能只看探针已收到结果。保留原始 transport.jsonl、alignment.csv、assessment.json 和五张 CSV。

本轮预设工程判据为：稳定区间内原始观测速率约 90/仿真秒、未来时间拒绝率不超过 0.1%、无其他查询拒绝、无未完成回传/CSV 行、无 ID/采样戳错配或无法由误差分项解释的跳变；真实场景至少 1000 次有效查询。这不是论文通用标准，也不保证在其他机器或长期运行中同样成立。回包到达乱序单列报告，不重写采样戳。

因子图尚未实现（用户已确认）。本步只能检查精确采样时间关联与可用于后端的观测数据，不能声称验证了实际因子图优化或乱序处理实现。
