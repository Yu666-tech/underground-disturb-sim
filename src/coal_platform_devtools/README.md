# B/C 无 Gazebo 输入样例

本包提供合成事实。

场景ID固定为`mock_bc_fixture`，`experiment_id`=`MOCK_FIXTURES_ONLY`，不能作为实际采煤面几何或误差效果的证据。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash  # 在自己的工作区根目录执行
export ROS_DOMAIN_ID=108
ros2 launch coal_platform_devtools fixtures.launch.py
```

B/C自己的节点也设`ROS_DOMAIN_ID=108`和`use_sim_time=true`，按正式topic和QoS订阅输入。

B的服务参数设为`/mock/scene/query_ray`；若代码仅支持固定服务名，在B节点上显式添加`--ros-args -r /scene/query_ray:=/mock/scene/query_ray`。

本包不发布真实`/scene/query_ray`。

不要与真实场景在同一ROS域运行。

- `GroundTruth/RobotState` 50Hz；720束`LidarFrameRaw` 10Hz；9个锚点`UwbRangeRaw`各10Hz；发布/`clock`和`map`→`world`→`base_link`→`laser_link`。

- 公共消息协议0.2.0；场景版本0.1.0；每次启动生成新`run_id`、观测ID不重复；派生消息须继承raw的`context`与`observation_id`。

- `A00/A08`空查询；A01合成阻挡；A02反射候选；A03两者同时；A04查询失败；A05损坏输入；A06传感器不可用；A07超量程。

- 每帧激光第0束+Inf、第1束NaN、第2束低于量程、第3束超过量程，其余716束为5m。

  它们是输入异常样例，不是C注入的粉尘结果。

- 默认每4秒切换一段演示路径/工况：TRAVELLING→CUTTING→TRAVELLING→IDLE循环；按位置进入/离开fixture_dust_zone。

  可用`phase_duration_s:=0.5`缩短周期，位置连续、阶段速度突变，仅供触发联调。

- `/platform/v1/fixture_info`为可靠、持久的样例说明；查询响应均带`MOCK_ONLY`，版本/`ID`/时间/坐标错误显式拒绝。

- 本包不输出`UwbDecision`、`UwbRangeObserved`、`LidarDecision`、`LidarBeamState`或`LidarFrameObserved`，不替B/C决定传播状态、粉尘浓度或误差。

自动检查（默认启动隔离ROS域108）：

```bash
ros2 run coal_platform_devtools check_fixtures.py --launch --output fixture_verification.json
```

输入模式和预期查询事实见`config/fixture_cases.json`。

切换真实平台时使用`coal_face_base`及`/scene/query_ray`，不复制算法节点，仅更换输入域/服务参数；算法若限制`scene_id`白名单，需要接受实际选定的场景ID。
