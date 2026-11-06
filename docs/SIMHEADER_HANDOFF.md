# 统一 SimHeader 接口与交接

本次首次向 B/C 交付的接口版本为 0.2.0。以 src/coal_platform_interfaces 的 msg/srv 和 config/platform_interfaces.json 为准。

SimHeader 含 stamp、frame_id、experiment_id、run_id、scene_id、scene_version 六项，所有业务消息与 QueryRay 请求响应通过 RunContext.header 复用；schema_version、robot_id、sequence 仍在 RunContext。字段访问例：raw.context.header.run_id。stamp 是仿真采样时间，m/rad/s，浓度 mg/m³。查询响应完整回显 context、request_id、observation_id。

复用现有结构：坐标为 geometry_msgs/Point，速度为 Twist；CSV 的 x/y/z 从对应结构展开。GroundTruth.robot_id/sequence 取 context；其他消息同理。QueryRay.scene_version 对应 served_scene_version；Intersection.distance_m 对应 distance_from_origin_m。不把几何命中事实当作 LOS/NLOS 结论。

话题：A 输入沿用 /platform/v1/{ground_truth,robot_state,uwb_ranges_raw,scan_raw}，以避开现有 Gazebo /ground_truth 的 Odometry 与 /scan_raw 的 LaserScan 类型冲突；B/C 使用本基线话题或显式 remap，不能混用原生话题。真实查询 /scene/query_ray，mock 查询 /mock/scene/query_ray。回传为 /disturbance_decision/uwb、/uwb_ranges、/disturbance_decision/lidar、/lidar_beam_state、/scan。

LidarBeamState 复用逐帧数组设计，beam_index、beam_states、delta_range_m、observed_valid 均为 N 项。CSV 逐束展开。LidarFrameObserved 新增 truth_ranges_m。UwbDecision 新增 anchor_id，LidarDecision 新增 sensor_id、total_beams。LidarDecision.target_concentration_mg_m3/actual_concentration_mg_m3 对应 CSV target_dust_concentration/actual_dust_concentration；dropout_count/false_return_count 对应 drop_count/false_point_count；总束数为 total_beams 且必须等于六种状态计数之和。robot_state CSV cmd_linear_speed/cmd_yaw_rate 对应 commanded_linear_speed_mps/commanded_yaw_rate_radps。

MarkerArray 内各 Marker.header 与 PointCloud2.header 保持标准 Header，不能直接换成 SimHeader；B/C 必须保留同戳 frame，关联对应 decision 的 context，业务记录使用 decision 的六项身份字段。可视化不参与业务计算。

公共 fixtures 位于 coal_platform_devtools/config。fixture_cases.json 提供原始输入数组与几何情形，return_chain.json 为 MOCK_ONLY 静态回传样例。后者的 LOS/零误差只是传输样例，不是算法预期。ros2 launch coal_platform_devtools fixtures.launch.py 在隔离 ROS_DOMAIN_ID 使用。mock 不启动真实查询服务。

CSV 九表位于 config/csv_templates。stamp 导出为 sec.nanosec（纳秒补齐九位），六项公共字段均取 context.header；数组列表列编码为 JSON；无效测距用 NaN，布尔值统一 true/false。run_manifest 的提交号不得编造；提交号未知时记录 UNKNOWN，上传现有仓库后按实际提交号填写。旧 csv_baseline 仅作历史资料。

B 后续提交：config/algorithms/uwb.yaml 初版（schema_version 0.2.0、params_version）、算法实现、专项 fixtures、状态转换与多径/NLOS/随机误差回归、两类回传消息与 UWB 可视化。保持 raw 身份、同戳、decision_id 与 observation_id 关联。

C 后续提交：config/algorithms/lidar_dust.yaml 初版（同上）、粉尘时序与扰动算法、专项 fixtures、逐束状态计数一致性与回归、三类回传消息与彩色点云。A 不代写 B/C 算法参数。提交后运行 python3 scripts/validate_bc_params.py B config/algorithms/uwb.yaml 或 python3 scripts/validate_bc_params.py C config/algorithms/lidar_dust.yaml；该检查只保证 YAML 结构和接口版本，算法参数语义由 B/C 负责，尚不能声称已验证未提交文件。

D 后续提交：由 D 创建的空场景包并通过 scene_validator.py 才算 D0；A 的 reference_s01_skeleton 通过不代表 D0。之后交付 S01/S02/S03 的真实资产、语义、路线、真实 preview.png、README 资源来源与许可、已知问题及人工事件建议。参考骨架不是完整可仿真 S01；详情见 coal_scene_tools/docs/loader_constraints.md。场景 loader 解析 scenario、semantics、route；事件建议内容不被程序解析。现有 loader 的 Gazebo 模式仅几何预览，后续真实场景的路线与挂载联调须单独验收。

接口基线通过功能分支与 PR 合入 main；B/C/D 从最新 main 创建各自分支提交。发布状态以仓库实际 PR 为准。

上传说明：本次作为接口联调源码提交，保留现有仓库的安装与资源恢复文件。现有 install.sh 面向完整离线包，依赖 SHA256SUMS.json/RELEASE.json，不属于本次已验证的源码安装入口。README 的快速开始以依赖已安装的环境为前提。
