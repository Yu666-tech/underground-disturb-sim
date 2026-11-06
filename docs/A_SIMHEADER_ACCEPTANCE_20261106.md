# A 统一 SimHeader 本地实现与验收记录

2026-11-06，WSL Ubuntu22，工作区 `/home/ll/coal_sim_ws`。

本记录描述 A 的本地实现与验收；GitHub 发布与合并结果以对应 PR 为准。

## 实现

- 六字段 `SimHeader.msg`，通过现有 `RunContext.header` 统一复用，接口版本 0.2.0。

  同步迁移 bridge、真实 query、mock 与验收工具，新增返回链路所缺 `anchor_id`、`sensor_id`、`total_beams`、`truth_ranges_m`、`beam_index`。

- 复用既有场景工具，新增 reference_s01_skeleton、PNG 占位图、入口与 `loader_constraints.md`，安装 `scene_validator.py`。

  loader 输出解析后的 scenario、semantics、route。

  人工事件建议不再作为程序输入。

- mock 读取本地 `fixture_cases.json` 数组与案例，明确 `MOCK_ONLY`；补充 `return_chain.json` 静态 ROS 回传消息与关联验证，保留真实/模拟 query 服务隔离。

- 九张 CSV 表头、机器可读接口基线、B/C 参数目录与结构校验器、可复现验收入口、交接与变更记录。

## 验收结果

- colcon build：6 个包全部通过。

- 场景校验：16 项正负例全部通过；已安装 validator 能识别参考骨架；loader 解析成功。

- mock：真值 50 Hz、扫描 10 Hz，9 个 UWB 案例、5 个非法请求、工况与区域切换通过。

- 回传：5 类 ROS 消息传输、3 组 decision–observed 关联通过，仅代表公共静态样例。

- 真实 Gazebo：15 秒仿真，751 组真值、150 组扫描、1,359 条 UWB，原始数据与桥接数据一致，时间戳保留，真实查询通过。

- 九表公共字段与唯一列名检查通过；迁移后的 Python 代码编译检查通过。

## B C D 后续提交

B：`config/algorithms/uwb.yaml`、UWB 算法、专项 fixtures、状态切换与误差分项回归、回传及可视化。

C：`config/algorithms/lidar_dust.yaml`、粉尘与激光扰动算法、专项 fixtures、逐束计数与时序回归、回传及彩色点云。

D：亲自提交可被 validator 识别的空场景包才能完成 D0；后续提交实际 `S01/S02/S03` 资产、语义、路线、真实预览、来源许可与说明。

A 的参考骨架通过不代表 D0 已完成。

A 不编写 B/C 算法参数；尚未收到的参数文件无法做实际内容验收。

当前参数工具检查非空 YAML mapping 和接口/参数版本，算法参数语义由 B/C 定义并审核。

现有任意场景 loader 的启动能力限于几何预览，完整场景路线和传感器挂载在 D 真实包交付后另行联调。

## 使用与证据

在 WSL 工作区执行

```bash
bash scripts/accept_simheader.sh
```

可复现主要验收。

详细接口及字段映射见 `docs/SIMHEADER_HANDOFF.md`。

A 输入话题沿用 `/platform/v1/` 前缀，避免与 Gazebo 原生 `Odometry/LaserScan` 类型冲突；B/C 按说明订阅或 remap。

结果：`evidence/simheader_build.log`、`simheader_fixtures.json`、`return_chain_verification.json`、`simheader_gazebo_verification.json`、`reference_s01_validation.json`、`simheader_loader.json`、`simheader_csv_verification.json`。

变更及 SHA256 见 `evidence/simheader_changes.json`。

修改前备份：`backups/pre_simheader_20261106.tar.gz`。

桌面交付仅为记录与交接说明，源码、模板、fixtures 和完整证据保留在指定 WSL 工作区。

发布使用现有 GitHub 仓库的功能分支与 PR，结果以实际 PR 为准。

## 上传前复核

已删除不适用于首次交付的旧版兼容提示；修复回传验收脚本写死工作区目录、缺少 evidence 目录的问题，统一接口包版本为 0.2.0。

README 已同步到 WSL 根目录。

上传应从现有仓库 main 新建功能分支，按相对路径合入源码与文档，保留原有 .gitattributes、`THIRD_PARTY.md`、安装及资源恢复文件。

不要整目录覆盖或删除原有文件。

排除 build/、install/、log/、backups/、data/、Python 缓存及临时日志。

当前验证基于依赖与模型已齐备的本地环境，不等于空系统或 GitHub 全新克隆的安装验收。

B/C/D 未提交内容、CSV 完整导出及 D 正式场景联调仍待后续完成。

复核结果：修正后本地接口包与 mock 包重新构建通过；在不同路径、只复制源码的干净工作区，接口/`mock`/场景工具三个包构建通过，公共回传验证、mock 输入/服务验证、16 项场景测试和参考骨架校验通过。

该检查使用当前机器既有 ROS 系统依赖，不等于空系统安装验收。

详见 `evidence/preupload_clean_check.log`、`preupload_clean_summary.json`。
