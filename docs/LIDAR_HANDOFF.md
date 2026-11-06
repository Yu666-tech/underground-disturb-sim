> 当前接口为 0.2.0，以 [SIMHEADER_HANDOFF.md](SIMHEADER_HANDOFF.md) 为准。本页保留原始交付背景。

# LiDAR 开发交付基线（2026-11-05）

场景 `coal_face_base/0.2.0`，消息协议 `0.1.1`。

面向已有 `Ubuntu22.04`、ROS2 Humble、Gazebo Classic11 的 C 同学，提供场景、理想扫描、真值和工况输入。

二维720束，约10Hz；不包含C的粉尘退化实现。

机器可读约定：[config/lidar_interfaces.yaml](../config/lidar_interfaces.yaml)。

输入为 `/platform/v1/scan_raw`、`/platform/v1/ground_truth`、`/platform/v1/robot_state`，另有持久上下文 `/platform/v1/run_context`。

米、弧度、仿真采样时间；扫描`frame`=`laser_link`，真值`frame`=`map`，`map/world`恒等。

truth是参考，raw是扰动前输入，observed由C生成。

C输出话题沿用已有A0约定：

| 话题                                      | 消息               | QoS                  |
| ----------------------------------------- | ------------------ | -------------------- |
| `/platform/v1/disturbance_decision/lidar` | LidarDecision      | Reliable，depth100   |
| `/platform/v1/lidar_beam_state`           | LidarBeamState     | Reliable，depth20    |
| `/platform/v1/scan`                       | LidarFrameObserved | Best Effort，depth20 |

以上均`volatile/keep_last`，每帧raw对应一组输出；复制raw的`context/observation_id`，三个输出共享`decision_id`。

C在自己的ROS2包开发，不覆盖raw。

逐束数组长度720，`intensities`可空或720；无返回/丢点用+Inf，损坏用NaN；非有限距离不计算有效差值。

粉尘浓度、`dust_active`、随机种子与参数版本由C给出，工况与区域是开发示意配置。

真实 `/scene/query_ray` 可选使用，能力和限制见真实QueryRay说明；模拟服务仍在独立 `/mock/scene/query_ray`，不能当成场景几何。

安装ZIP包含本地模型与校验值，`install.sh` 创建独立工作区，在本机编译后自动无窗口验收，成功才注册 `coal-lidar`、`coal_lidar_env`。

Ubuntu依赖若已齐全无需下载模型或依赖；缺少系统依赖时需要网络和sudo，不升级ROS或Gazebo主版本。

支持Classic11，目标同学已知版本为11.10.2。

验证记录见 `evidence/platform_acceptance_20261104.json`、`query_native_reference_20261104.json`、`lidar_install_20261105.json`。

历史运动测试与本次短时安装验收区分保留；本次不是重跑20分钟全周期，也不等于另一台电脑已签收。

接收方还需确认GUI、性能和C模型接入结果。
