"""链路状态滞回：连续多帧一致才切换，避免边界抖动。"""

LOS, SOFT, HARD, OUTAGE, INPUT_INVALID, QUERY_FAILED = range(6)

# 这些状态表示“没有可用观测”，出现时应立即生效，不等待滞回。
IMMEDIATE = (OUTAGE, INPUT_INVALID, QUERY_FAILED)


class StateHysteresis:
    """按 (tag_id, anchor_id) 维护已确认状态。

    - 进入 NLOS（SOFT/HARD）需要连续 enter_frames 帧；
    - 回到 LOS 需要连续 exit_frames 帧；
    - 进入 OUTAGE/INPUT_INVALID/QUERY_FAILED 立即生效；
    - 从上述状态恢复时立即生效。
    """

    def __init__(self, enter_frames: int = 3, exit_frames: int = 2):
        self.enter_frames = max(1, int(enter_frames))
        self.exit_frames = max(1, int(exit_frames))
        self._state = {}
        self._pending = {}

    def update(self, key, observed: int) -> int:
        committed = self._state.get(key)
        if committed is None:
            self._state[key] = observed
            return observed

        if observed == committed:
            self._pending.pop(key, None)
            return committed

        if observed in IMMEDIATE:
            self._state[key] = observed
            self._pending.pop(key, None)
            return observed

        if committed in IMMEDIATE:
            self._state[key] = observed
            self._pending.pop(key, None)
            return observed

        need = self.enter_frames if observed in (SOFT, HARD) else self.exit_frames
        candidate, count = self._pending.get(key, (None, 0))
        count = count + 1 if candidate == observed else 1
        if count >= need:
            self._state[key] = observed
            self._pending.pop(key, None)
            return observed
        self._pending[key] = (observed, count)
        return committed

    def reset(self):
        self._state.clear()
        self._pending.clear()
