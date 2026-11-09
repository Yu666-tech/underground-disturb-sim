#!/usr/bin/env python3
"""Regression for independent raw-sequence vs output completeness statistics."""
from types import SimpleNamespace
import unittest
from evaluate_uwb_timing import analyze, truth_key
from test_uwb_csv import facts
from uwb_csv import identity


class AssessmentRegression(unittest.TestCase):
    def probe(self, indices, missing_outputs=()):
        probe = SimpleNamespace(messages={k: {} for k in ('raw', 'decision', 'observed', 'query')},
            arrivals={k: {} for k in ('raw', 'decision', 'observed', 'query')}, truths={},
            diagnostics={}, duplicates={}, query_info=None, processing_snapshots=[])
        keys = []
        for i in indices:
            sample = facts(i)
            for message in sample.values():
                message['anchor_id'] = 'A00'
            raw = sample['raw']
            raw['tag_position_m'] = dict(x=0., y=0., z=0.)
            raw['anchor_position_m'] = dict(x=5., y=0., z=0.)
            key = identity(raw)
            keys.append(key)
            probe.truths[truth_key(raw)] = {}
            for kind in probe.messages:
                if kind != 'raw' and i in missing_outputs:
                    continue
                probe.messages[kind][key] = sample[kind]
                probe.arrivals[kind][key] = i * 100000000
        return probe, keys

    def assess(self, indices, missing_outputs=()):
        probe, keys = self.probe(indices, missing_outputs)
        duration = len(keys) / 90.
        return analyze(probe, keys, SimpleNamespace(profile='mock', tolerance_ms=50.), duration, duration)

    def test_missing_output_is_not_raw_loss(self):
        result = self.assess([0, 1, 2], missing_outputs=[1])
        self.assertEqual(result['missing_raw_sequences'], 0)
        self.assertEqual(result['incomplete_count'], 1)
        self.assertFalse(result['suitable'])

    def test_actual_raw_gap_is_counted(self):
        result = self.assess([0, 2])
        self.assertEqual(result['missing_raw_sequences'], 1)
        self.assertEqual(result['incomplete_count'], 0)

    def test_complete_cohort(self):
        result = self.assess([0, 1, 2])
        self.assertEqual(result['missing_raw_sequences'], 0)
        self.assertEqual(result['incomplete_count'], 0)
        self.assertTrue(result['suitable'])


if __name__ == '__main__':
    unittest.main()
