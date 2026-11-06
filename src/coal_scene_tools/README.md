# D 场景制作与预检工具

本包将既有 A0 模板集中纳入正式平台。

版本 0.1.0；场景文件 `schema_version`=0.1.0，与 ROS 消息 `schema_version`=0.2.0 独立。

## 安装与检查

在 Ubuntu 22.04、ROS 2 Humble 工作区根目录执行：

```bash
sudo apt install python3-yaml python3-jsonschema
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select coal_scene_tools
source install/setup.bash
ros2 run coal_scene_tools scene_loader --scene-id reference_s01_skeleton --allow-skeleton
```

## D 复制模板后开发

```bash
share=$(ros2 pkg prefix --share coal_scene_tools)
mkdir -p ~/coal_scene_work/scenes
cp -rL "$share/scenes/reference_s01_skeleton" ~/coal_scene_work/scenes/
ros2 run coal_scene_tools validate_scene ~/coal_scene_work/scenes/reference_s01_skeleton
ros2 run coal_scene_tools scene_loader --scenes-root ~/coal_scene_work/scenes --scene-id reference_s01_skeleton --allow-skeleton
# 已安装 Gazebo Classic 时，可查看静态几何；Ctrl+C 停止：
ros2 run coal_scene_tools scene_loader --scenes-root ~/coal_scene_work/scenes --scene-id reference_s01_skeleton --allow-skeleton --launch-geometry
# 上述命令启动服务器；另开终端显示三维窗口：
GAZEBO_MASTER_URI=http://127.0.0.1:11995 LIBGL_ALWAYS_SOFTWARE=1 gzclient
```

复制目录可任意放置，不依赖 A 电脑路径。

改名时同步修改目录名、三份业务 YAML 的 `scene_id`（事件建议仅供人工参考）；修改几何、路线、安装位或锚点时递增 `scene_version`。

详见 `docs/D_SCENE_SPEC.md`。

## 当前完成与限制

- 校验目录资源、三份业务 YAML、版本、ID、坐标、部分 BOX 语义与路线碰撞，保留 warnings。

- Loader 默认仅预检并输出加载计划；可选只启动静态几何。

  不执行包内自带脚本。

- 静态几何查看默认独立Gazebo端口11995，不继承主场景端口；同时看多个场景时可传`--gazebo-master-uri http://127.0.0.1:11996`。

- S01 是静态骨架，传感器、路线与工况为配置示例，尚不自动发布 `truth/raw/state/query`。

- 当前 coal_face_base 已提供公共输入与真实静态 QueryRay，但其主 launch 尚未由本 loader 驱动。

- 不支持据此宣称完整多场景加载、动态查询、精确 MESH 路线碰撞或真实 B/C 扰动算法已完成。

## D 交回 A

提交完整场景目录、四份 YAML、SDF/本地模型、预览、README（来源许可、独立启动方法、已知问题）和校验结果。

正式 S01 后续还需路线、传感器、TF 和运行测试。
