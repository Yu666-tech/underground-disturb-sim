# D 标准场景包规范

## 1 身份和版本

scene_id允许小写英文开头的英文数字下划线名称，例如当前基础场景 `coal_face_base`；也兼容D的 S01_straight、S02_corner、S03_workface。目录必须与ID相同。一个ID对应场景功能身份，内容调整递增scene_version（SemVer）；几何、语义、路线、安装位或锚点变化都应升版。schema_version=0.1.0是文件格式，与scene版本、计划V1.7、平台发布版本各自独立。D填写具体ID与版本；A负责命名登记、加载和追溯。现有200m工作面尚未转换成此格式，不自动登记成D的S03。

## 2 必交目录

```text
scenes/S01_straight/
  scenario.yaml                       # 场景入口与安装位
  scene_semantics.yaml                # 几何材料和区域事实
  demo_route.yaml                     # 时间化路线和工况
  expected_events_suggestion.yaml     # 仅期望区段建议
  world/scene.sdf                     # Gazebo Classic SDF世界
  assets/models/...                   # 自包含model.sdf、网格、贴图，按需建立
  preview.svg                        # 当前骨架预览；正式交付可换preview.png
  README.md                          # 启动、来源许可、完成项、已知限制
```

scenario中的preview字段决定图片文件名；支持PNG/SVG。V1.7建议的preview.png在正式包中优先使用。示例不依赖外部模型，因此assets列表为空、无需创建空目录。不得打包自己的build/install/log/rosbag缓存替代资产源码。

## 3 YAML逐文件约束

三个业务文件采用UTF-8、显式schema_version/scene_id/scene_version，跨文件一致。严格字段见对应JSON Schema；不接受重复YAML键、未知字段或NaN/Inf配置。事件建议文件仅供人工参考，其示例schema不作为程序输入校验。新增字段先由A更新schema和协议说明，避免拼写错误静默被忽略。

scenario：global_frame=map、world_frame=world，map_to_world按TF父map子world定义。world/semantics/route/expected_events/preview/assets均为包内相对路径；禁止Windows绝对路径、../逃逸、外部软链接、HTTP/Fuel在线依赖。robot声明唯一robot_id、base_link初始位姿和物理外包尺寸。sensors声明真实安装位姿（相对base_link）、唯一激光/标签frame、频率、量程；anchors全部在map中，z是坐标绝对值，不是默认距地板高度。标签xyz是机体安装偏移，不能当map点。锚点是参考配置，不保证定位可观性。

semantics：geometry_id在本场景唯一，model_name对应SDF实体；BOX声明center_m、size_m、可选rpy_rad（省略为0）；MESH声明mesh_uri（包内）、center_m、rpy_rad与scale，变换应与collision一致。几何采用collision而非visual；center/normal等在map表达，若world与map非恒等，A必须变换后比对。当前检查器的BOX与SDF核对只支持恒等map/world变换，正式非恒等场景需A扩展检查器后接收。material可UNKNOWN，reflection_level为LOW/MEDIUM/HIGH/UNKNOWN；thickness_m是标称法向厚度，不得用墙轴线或外包尺寸冒充真实遮挡厚度。未知厚度当前schema不允许填NaN；应在README列明待补，不能将候选提交为已验收。is_dynamic=true仅声明动态事实，A需要观测时刻快照。D不提供传播状态或误差大小。

zone：非自交且无重复首尾点的多边形，至少3点、非零面积，z_min<z_max；zone_id唯一，NONE保留。priority为整数，重叠选择规则见BC接口。zone_type取REFERENCE/DUST_REGION/OTHER，仅标环境用途，不直接生成粉尘浓度或dust_active。

route：必须指定robot_id、linear插值、loop和至少2个waypoint；time_s从0严格递增，xyz/yaw及work_mode明确。起点与scenario初始位姿一致；loop=true时末点位姿回起点。边界时刻取后一个waypoint的工况，区间内保持前一工况；yaw取最短角插值（恰好π时取正方向）。当前骨架不执行路线，正式路线需A的驱动层实现这些规则后复测。不得让机器人穿墙，也不得仅凭端点无碰撞就宣称全路径通过。

expected_events_suggestion：advisory_only=true；时间窗在路线时长内。只写“经过障碍附近/进入某区域”等建议；B/C输出的LOS/NLOS、dust_active是算法结果，不由此文件硬编码或对齐改写。D可与B/C讨论期望区段，但公共接口变更统一由A收口。

## 4 Validator与Loader

```bash
python3 tools/validate_scene.py scenes/S01_straight
python3 tools/scene_loader.py --scene-id S01_straight --allow-skeleton
# 只想查看静态几何时，已安装Gazebo的终端可加：
python3 tools/scene_loader.py --scene-id S01_straight --allow-skeleton --launch-geometry
```

validator非0退出表示拒绝；JSON中passed=true仅表示当前检查项通过。可检查三个业务文件schema、跨文件版本、frame和ID、直接资源路径、区域自交、路线时序、部分BOX语义与SDF一致性、零yaw直线段的AABB扫掠。不能替代递归纹理引用、MESH/旋转模型精确碰撞、实际TF、传感器、运动控制、query和离线网络隔离试验；warnings必须随包保留。当前骨架也不实现候选包签收门或完整动态模型支持。

loader默认只解析验证并输出绝对路径计划，不更改环境、不发布数据。status=skeleton时必须显式--allow-skeleton；status=candidate只是D申报待验收，不代表A已签收。可选launch只运行固定gzserver参数，永不执行场景包自带shell/Python。静态几何查看默认端口11995，不继承主场景端口；可用--gazebo-master-uri指定其他本机http端口。它不是V1.7最终full_visual，不能运行B/C模块或热切场景。D可提供独立launch_scene.py辅助查看，但A不自动信任执行。

## 5 交付与签收

D提交源码资产、版本、预览、来源许可、单独启动命令、路线完成记录和已知问题；A验证结构→真实加载→TF/ID→truth/raw/state/query→记录回放。通过后记录scene_version、配置hash、四人源码版本与检查结果。S01优先稳定，之后S02拐角、S03作业面；不要在S01骨架上预置B/C算法代码。
