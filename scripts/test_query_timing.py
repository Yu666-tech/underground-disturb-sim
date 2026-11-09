#!/usr/bin/env python3
"""Boundary/identity regression for the real static-scene temporal guard."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/coal_scene_query/scripts'))
from query_timing import rejection_reason, tolerance_ns
from coal_platform_interfaces.msg import RunContext


class TimeGuard(unittest.TestCase):
    def setUp(self):
        self.truth = RunContext()
        self.truth.header.experiment_id = 'experiment'
        self.truth.header.run_id = 'run'
        self.truth.header.scene_id = 'scene'
        self.truth.header.scene_version = '1'
        self.truth.robot_id = 'robot'
        self.truth.header.stamp.sec = 10

    def context(self, delta_ns):
        c = deepcopy(self.truth)
        ns = 10000000000 + delta_ns
        c.header.stamp.sec, c.header.stamp.nanosec = divmod(ns, 1000000000)
        return c

    def check(self, delta_ns, ms=50., start=0):
        return rejection_reason(self.context(delta_ns), self.truth, start, tolerance_ns(ms))

    def test_50ms_exact_boundary(self):
        for delta in (0, 1, 20000000, 49999999, 50000000):
            self.assertIsNone(self.check(delta))
        self.assertEqual(self.check(50000001), 'FUTURE_BEYOND_TOLERANCE')

    def test_100ms_exact_boundary(self):
        self.assertIsNone(self.check(100000000, 100.))
        self.assertEqual(self.check(100000001, 100.), 'FUTURE_BEYOND_TOLERANCE')

    def test_original_zero_tolerance(self):
        self.assertIsNone(self.check(0, 0.))
        self.assertEqual(self.check(1, 0.), 'FUTURE_BEYOND_TOLERANCE')

    def test_older_than_two_seconds_still_rejected(self):
        self.assertIsNone(self.check(-2000000000))
        self.assertEqual(self.check(-2000000001), 'OLDER_THAN_2S')

    def test_run_start_still_enforced(self):
        self.assertEqual(self.check(-1, start=10000000000), 'BEFORE_RUN_START')

    def test_foreign_identity_still_rejected(self):
        for field in ('experiment_id', 'run_id', 'scene_id', 'scene_version'):
            c = self.context(0)
            setattr(c.header, field, 'foreign')
            self.assertEqual(rejection_reason(c, self.truth, 0, tolerance_ns(100.)), 'IDENTITY_MISMATCH')
        c = self.context(0)
        c.robot_id = 'foreign'
        self.assertEqual(rejection_reason(c, self.truth, 0, tolerance_ns(100.)), 'IDENTITY_MISMATCH')

    def test_invalid_parameter_values_rejected(self):
        for value in (-1., float('nan'), float('inf'), 2001.):
            with self.assertRaises(ValueError):
                tolerance_ns(value)


if __name__ == '__main__':
    unittest.main()
