# 场景模块上手与交付说明

本文发给负责场景制作的同学。先完成自己创建的空场景包及校验，再交付 S01/S02/S03 正式场景。结构预检只需要 Ubuntu 22.04、ROS2 Humble 和 Python 依赖，不需要 Gazebo；几何预览及正式仿真另行准备。

## 下载与构建工具

首次下载在自己的 Ubuntu 终端执行；已克隆过则进入现有目录，保存修改并更新 main，不重复克隆。

```bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions \
  python3-yaml python3-jsonschema
cd ~
git clone https://github.com/wen37/underground-disturb-sim.git
cd underground-disturb-sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select coal_scene_tools
source install/setup.bash
```

后续每个新终端都需要进入自己的克隆目录，重新 source ROS 和工作区环境。

## 先识别参考骨架

```bash
ros2 run coal_scene_tools scene_validator.py \
  src/coal_scene_tools/scenes/reference_s01_skeleton
```

`passed: true` 只表示结构及当前检查项通过。参考骨架不是完整可仿真 S01，也不代表场景负责人的 D0 已完成。

## 创建自己的空场景包

以下以 `scene_empty_demo` 为例，仅在该目录尚不存在时执行。名称按实际情况调整；目录名必须等于 scene_id，不使用人员代号命名。

先创建开发分支：

```bash
git switch main
git pull --ff-only
git switch -c feat/scene-empty-package
```

复制骨架，并同步修改 YAML 与入口中的 scene_id：

```bash
cp -a src/coal_scene_tools/scenes/reference_s01_skeleton \
  src/coal_scene_tools/scenes/scene_empty_demo
sed -i 's/reference_s01_skeleton/scene_empty_demo/g' \
  src/coal_scene_tools/scenes/scene_empty_demo/*.yaml \
  src/coal_scene_tools/scenes/scene_empty_demo/launch_scene.py
```

填写自己的场景说明、来源和已知问题，再执行：

```bash
ros2 run coal_scene_tools scene_validator.py \
  src/coal_scene_tools/scenes/scene_empty_demo
ros2 run coal_scene_tools scene_loader \
  --scenes-root src/coal_scene_tools/scenes \
  --scene-id scene_empty_demo --allow-skeleton
```

loader 默认输出解析结果，不会启动完整仿真。自己创建的包通过校验后，提交源码与校验结果；由场景负责人提交并通过检查，才算 D0，A 提供的参考包不算替代交付。

## 正式场景还需交付

详细约束见 [场景包加载约束](../src/coal_scene_tools/docs/loader_constraints.md) 和 [场景包规范](../src/coal_scene_tools/docs/D_SCENE_SPEC.md)。

| 文件或目录 | 内容 |
| --- | --- |
| `scenario.yaml` | scene_id、版本、world、机器人初始位姿、锚点、传感器安装位和路线引用 |
| `scene_semantics.yaml` | 区域多边形、geometry_id、材料、厚度、反射等级、动态标记 |
| `demo_route.yaml` | 时间化航点、速度相关配置与 work_mode 工况区段，包括需要的 CUTTING 区段 |
| `expected_events_suggestion.yaml` | 给算法负责人的人工事件参考；程序不解析其内容 |
| `world/` 与模型资产 | 可被包内相对路径引用的场景几何与资源 |
| `preview.png` | 正式场景真实预览，替换模板中的灰色占位图 |
| `README.md` | 资源来源、许可、场景说明和已知问题 |
| `launch_scene.py` 或统一 loader 入口 | 与实际 scene_id 一致的入口 |

- 场景 YAML 的 schema_version 为 0.1.0，与 ROS 接口版本 0.2.0 独立。
- 三份业务 YAML 的 scene_id、scene_version、schema_version 一致；同一包内 anchor_id、zone_id、geometry_id 各自唯一。
- 多边形不能自交或退化；frame 和传感器 parent 满足约束；资源引用不能越出场景包。
- 场景只定义环境事实，不写 LOS/NLOS、多径误差或粉尘浓度判定算法。
- validator 不能代替路线执行、传感器挂载、TF、QueryRay 或完整仿真验收。loader 当前的 Gazebo 模式仅几何预览，正式场景随后与平台负责人联调。

## 提交方式

以上示例已经创建了开发分支。验证通过后，只提交自己的包：

```bash
git add src/coal_scene_tools/scenes/scene_empty_demo
git commit -m "feat: add validated scene skeleton"
git push -u origin feat/scene-empty-package
```

在 GitHub 创建目标为 main 的 PR，附校验结果与已知问题。正式场景后续另开分支提交。没有仓库写权限时，先 Fork 再开发并向原仓库 main 提 PR；不直接推送 main，不提交 build/install/log 缓存。

现有 Gazebo 基础场景及资源恢复见 [基础场景说明](BASE_SCENE.md)。UWB 和 LiDAR 扰动由各自模块负责人实现，场景负责人不需要代写其算法或参数。
