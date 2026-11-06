# 场景包加载约束

模板版本 0.1.0。

参考 `scenes/reference_s01_skeleton`；它只是联调骨架，不是完整 S01。

- 目录名必须等于 `scene_id`，所有资源为包内相对路径，禁止越界与网络资源。

- `map/world` 当前必须恒等；body 与传感器 `frame` 唯一，传感器 parent 为 body `frame`。

- `anchor_id`、`geometry_id`、`zone_id` 各自唯一。多边形不得自交或退化，`NONE` 为保留区域。

- 场景描述、语义、路线的 `scene_id/scene_version/schema_version` 必须一致。

- `expected_events_suggestion.yaml` 仅为人工备注，`loader/validator` 不解析其内容。

- `preview.png` 是灰色占位图，D 必须替换；README 必须说明来源、许可与已知问题。

- D0 必须由 D 提交空场景包并通过 `scene_validator.py`；A 的参考骨架通过不代表 D0 已完成。

- 启动入口统一使用

  ```bash
  ros2 run coal_scene_tools scene_loader --scene-id ID
  ```

  骨架须 --allow-skeleton。

- 当前 --launch-geometry 只启动几何预览，不承诺自动执行任意 D 路线或挂载任意资产的传感器。
