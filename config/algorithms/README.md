# 算法参数

本目录按功能存放参数，不使用人员代号作为目录名。

- `uwb.yaml`：UWB 误差算法参数，后续由该模块负责人提交。

- `lidar_dust.yaml`：LiDAR 粉尘扰动参数，后续由该模块负责人提交。

当前仅预留位置，不提供或编造算法参数。

文件应为非空 YAML mapping，包含字符串 `schema_version: '0.2.0'` 和非空字符串 `params_version`。

提交后执行：

```bash
python3 scripts/validate_bc_params.py B config/algorithms/uwb.yaml
python3 scripts/validate_bc_params.py C config/algorithms/lidar_dust.yaml
```

B/C 只是校验脚本现有的模块选择参数，不参与目录命名。

此检查只验证结构与版本，不验证算法效果。
