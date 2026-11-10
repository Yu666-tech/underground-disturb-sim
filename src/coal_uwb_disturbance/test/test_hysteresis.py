"""状态滞回单元测试（不依赖 ROS）。"""

import unittest

from coal_uwb_disturbance.hysteresis import (
    HARD, INPUT_INVALID, LOS, OUTAGE, SOFT, StateHysteresis,
)


class TestStateHysteresis(unittest.TestCase):
    def setUp(self):
        self.h = StateHysteresis(enter_frames=3, exit_frames=2)
        self.key = ("tag_01", "A01")

    def test_initial_state_commits(self):
        self.assertEqual(self.h.update(self.key, LOS), LOS)

    def test_enter_nlos_needs_three_frames(self):
        self.assertEqual(self.h.update(self.key, LOS), LOS)
        self.assertEqual(self.h.update(self.key, HARD), LOS)
        self.assertEqual(self.h.update(self.key, HARD), LOS)
        self.assertEqual(self.h.update(self.key, HARD), HARD)

    def test_single_frame_blip_is_suppressed(self):
        self.h.update(self.key, LOS)
        self.assertEqual(self.h.update(self.key, HARD), LOS)
        self.assertEqual(self.h.update(self.key, LOS), LOS)

    def test_exit_needs_two_frames(self):
        self.h.update(self.key, LOS)
        for _ in range(3):
            self.h.update(self.key, HARD)
        self.assertEqual(self.h.update(self.key, LOS), HARD)
        self.assertEqual(self.h.update(self.key, LOS), LOS)

    def test_outage_is_immediate(self):
        self.h.update(self.key, LOS)
        self.assertEqual(self.h.update(self.key, OUTAGE), OUTAGE)

    def test_recovery_from_outage_is_immediate(self):
        self.h.update(self.key, OUTAGE)
        self.assertEqual(self.h.update(self.key, LOS), LOS)

    def test_input_invalid_is_immediate(self):
        self.h.update(self.key, LOS)
        self.assertEqual(self.h.update(self.key, INPUT_INVALID), INPUT_INVALID)

    def test_soft_and_hard_share_nlos_gate(self):
        self.h.update(self.key, LOS)
        self.h.update(self.key, SOFT)
        self.h.update(self.key, SOFT)
        self.assertEqual(self.h.update(self.key, SOFT), SOFT)
        self.assertEqual(self.h.update(self.key, HARD), SOFT)
        self.assertEqual(self.h.update(self.key, HARD), SOFT)
        self.assertEqual(self.h.update(self.key, HARD), HARD)

    def test_links_are_independent(self):
        other = ("tag_01", "A02")
        self.h.update(self.key, LOS)
        self.h.update(other, HARD)
        self.h.update(other, HARD)
        self.h.update(other, HARD)
        self.assertEqual(self.h.update(other, HARD), HARD)
        self.assertEqual(self.h.update(self.key, LOS), LOS)


if __name__ == "__main__":
    unittest.main()
