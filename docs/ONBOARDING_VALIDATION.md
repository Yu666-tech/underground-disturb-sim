# 首次上手隔离环境验收

2026-11-06，对已有 Ubuntu 22.04 与 ROS2 Humble 的接收方，验证三个模块文档的首次上手流程。

## 从主目录开始即可

接收方不需要提前创建维护者的 `~/coal_sim_ws`。在自己的终端执行 `cd ~` 后，`git clone` 会创建 `~/underground-disturb-sim`；该仓库根目录本身就是 ROS 工作区，里面已有 `src/`。

`colcon build` 自动生成 `build/`、`install/`、`log/`，随后 source 本工作区的 `install/setup.bash`。工作区目录不必以 `_ws` 结尾。

仅在维护者电脑上换到主目录或新建目录，仍会共用已有系统软件及环境，不能作为首次安装验收。

## 本次隔离方法

- 从 Ubuntu 22.04 最小根文件系统开始，只安装 ROS2 Humble 的 `ros-base` 基线。
- 基线中没有 gcc、g++、colcon。创建普通用户 receiver，其主目录没有本项目、旧工作区或构建产物；启动时清空外部环境变量。
- 算法模块和场景模块使用两份独立的 overlay 根文件系统，避免前一条流程安装的依赖被后一条借用。
- 按文档提取首次命令块执行，直接从 GitHub HTTPS 克隆；没有复制维护者的源码目录、build 或 install。
- 复用了下载过的 `.deb` 包缓存以减少下载；所有软件在隔离系统中重新安装，未复用宿主已安装文件。
- 两份环境克隆的代码提交均为 `147c323c178a251abda849af9cf1c2eda4c94a95`。本次后续仓库更新仅涉及依赖说明与验收记录，平台源码和接口未改变。

## 结果

| 模块 | 实际执行 | 结果 |
| --- | --- | --- |
| UWB / LiDAR 共用安装步骤 | apt 安装依赖 → GitHub 克隆 → 构建 coal_platform_interfaces、coal_platform_devtools → source | 通过，两个包构建成功 |
| UWB | 启动 Mock，执行 query_mock_example.py | 通过，查询返回 OK 且标记 MOCK_ONLY |
| LiDAR 公共输入 | topic list、robot_state 单帧接收、公共 fixtures 验收 | 通过，真值 50 Hz、扫描 10 Hz，720 束特殊输入与工况/区域验证通过 |
| 场景工具 | 独立环境安装依赖 → GitHub 克隆 → 构建 coal_scene_tools | 通过，一个包构建成功 |
| 场景包预检 | 参考骨架校验、新建 scene_empty_demo、validator 与 loader | 通过，loader 为 plan_only |

UWB 与 LiDAR 的首次安装命令块逐字相同，因此共用安装验收，并分别检查了 UWB 查询和 LiDAR 公共输入。场景工具使用独立环境，不共享算法流程安装的依赖。

## 文档修正

原文没有显式安装编译工具；最低 ROS 基线中确实没有 gcc/g++。现已补列：

- UWB / LiDAR：`build-essential`、`cmake`、`python3-dev`。
- 场景工具：`build-essential`、`cmake`、`ros-humble-ament-cmake`。
- 完整场景准备说明补列 `python3-dev` 与 `python3-jsonschema`；该完整 Gazebo 路径不计入本次最小上手验收。

## 如何自行复验

在独立 Ubuntu 22.04 虚拟机、新 WSL 实例或同等隔离系统中安装 ROS2 Humble，使用没有旧工作区的新用户，按对应入口从下载步骤开始执行：

- [UWB 上手与交付](UWB_QUICKSTART.md)
- [LiDAR 上手说明](LIDAR_QUICKSTART.md)
- [场景上手与交付](SCENE_QUICKSTART.md)

验收不能只看命令退出：还要确认包构建成功、source 成功、消息实际到达，或 validator/loader 输出 passed=true。B/C 文内的公共 fixtures 自动检查会自行启动并停止 Mock，运行前先停止手工启动的发布器。

## 范围与前提

需有 sudo 权限，可访问 GitHub 与 Ubuntu/ROS 软件源，且 ROS2 Humble apt 源已配置。上表证明本次明确基线中的公共上手步骤可运行，不保证任意网络和任意安装方式都相同。

参数校验命令须在负责人创建参数文件后执行；算法构建命令须在对应算法包创建后执行。Git 提交/推送另需用户身份配置及仓库写权限或 Fork。

本次没有验收尚未提交的 UWB/LiDAR 算法、正式 D0 交付、完整 CSV 导出、Gazebo 首次安装或图形性能。A 的场景示例验证不代替场景负责人的 D0 提交。

原始结构化结果见 [cold_onboarding_20261106.json](../evidence/cold_onboarding_20261106.json)。本记录来自独立系统实测，不是 GitHub Actions CI 检查。
