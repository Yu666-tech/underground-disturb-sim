# coal_uwb_disturbance

B 侧 UWB 扰动节点（初版）。

## 输入

- `/platform/v1/uwb_ranges_raw`（`UwbRangeRaw`，best effort）
- `/scene/query_ray` 或 `/mock/scene/query_ray`（`QueryRay` service）

## 输出

- `/disturbance_decision/uwb`（`UwbDecision`，reliable）
- `/uwb_ranges`（`UwbRangeObserved`，best effort）

## 内部状态与上报映射

| 内部 | propagation_state | cause |
|---|---|---|
| LOS | LOS | DIRECT_CLEAR |
| SOFT | NLOS | THIN_WALL |
| HARD | NLOS | SINGLE_REFLECTION |
| OUTAGE | OUTAGE | BLOCKED_NO_REFLECTOR |
| INPUT_INVALID | INPUT_INVALID | INPUT_INVALID |
| QUERY_FAILED | UNKNOWN | QUERY_FAILED |

`UwbRangeObserved.state` 与 `UwbDecision.propagation_state` 保持一致。

## 运行

```bash
ros2 run coal_uwb_disturbance uwb_node --ros-args \
  -p use_sim_time:=true \
  -p config_file:=$HOME/underground-disturb-sim/config/algorithms/uwb.yaml
```

真实场景把查询服务换成：

```bash
-p query_service:=/scene/query_ray
```

## 已知限制

- HARD 目前只按“有反射候选”判定，未做反射路径长度与可达性校验；
- `nlos_type` 尚不能写入消息（平台 schema 无此字段），只在内部与后续 CSV 适配中保留；
- 查询失败时 `valid=false`、`propagation_state=UNKNOWN`，不默认成 LOS。
