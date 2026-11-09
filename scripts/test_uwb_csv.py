#!/usr/bin/env python3
"""CSV contract regression: values on disk, topic reordering and identity isolation."""
import copy
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/coal_platform_bridge/scripts'))
from uwb_csv import UwbCsvWriter


def facts(index, state='LOS', kind='none', valid=True, run='run'):
    context = dict(header=dict(stamp=dict(sec=2, nanosec=index * 100000001),
                               frame_id='map', experiment_id='CSV_TEST', run_id=run,
                               scene_id='fixture', scene_version='0.1.0'),
                   robot_id='robot', sequence=index, schema_version='0.2.0')
    shared = dict(context=context, observation_id=f'obs-{index}', anchor_id='anchor')
    raw = dict(shared, tag_id='tag', truth_range_m=5., raw_range_m=5., raw_valid=True)
    decision = dict(shared, decision_id=f'dec-{index}', propagation_state=state,
                    nlos_type=kind, valid=valid, query_status='OK' if valid else 'UNSUPPORTED',
                    cause='fixture', multipath_error_m=0., nlos_error_m=.1 if valid else 0.,
                    random_noise_m=0.)
    observed = dict(shared, tag_id='tag', decision_id=f'dec-{index}', state=state,
                    nlos_type=kind, valid=valid, truth_range_m=5., raw_range_m=5.,
                    observed_range_m=5.1 if valid else float('nan'),
                    error_m=.1 if valid else float('nan'))
    hits = [] if kind == 'none' else [dict(geometry_id='wall', geometry_type='BOX',
        material='FIXTURE', reflection_level='LOW', distance_from_origin_m=2.)]
    refs = [] if kind != 'hard' else [dict(geometry_id='reflector', geometry_type='BOX',
        material='FIXTURE', reflection_level='HIGH', distance_to_link_m=.5)]
    trace = dict(shared, tag_id='tag', decision_id=f'dec-{index}', request_id=f'req-{index}',
                 nlos_type=kind, valid=valid, status=decision['query_status'],
                 processing_time_ms=.25, error_message='MOCK_ONLY',
                 direct_intersections=hits, reflector_candidates=refs)
    return dict(raw=raw, decision=decision, observed=observed, query=trace)


class CsvContract(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.output = Path(self.temp.name) / 'recording'
        self.writer = UwbCsvWriter(self.output, ROOT / 'config/csv_templates')

    def tearDown(self):
        self.writer.close()
        self.temp.cleanup()

    def rows(self, table):
        with (self.output / f'{table}.csv').open(newline='') as f:
            return list(csv.DictReader(f))

    def feed(self, sample):
        for kind in ('query', 'observed', 'decision', 'raw'):
            self.writer.ingest(kind, sample[kind])

    def test_all_five_tables_and_ordered_soft_to_hard(self):
        samples = [facts(0), facts(1, 'NLOS', 'soft'), facts(2, 'NLOS', 'hard'),
                   facts(3, 'UNKNOWN', 'none', False)]
        for index in (3, 1, 0, 2):
            self.feed(samples[index])
        self.writer.refresh_transitions()
        for table in ('uwb_observations', 'query_trace', 'multipath_error_detail'):
            rows = self.rows(table)
            self.assertEqual(len(rows), 4)
            self.assertEqual({r['nlos_type'] for r in rows}, {'none', 'soft', 'hard'})
            self.assertEqual(next(r for r in rows if r['observation_id'] == 'obs-1')['stamp'],
                             '2.100000001')
        invalid = next(r for r in self.rows('uwb_observations') if r['observation_id'] == 'obs-3')
        self.assertEqual((invalid['valid'], invalid['observed_range_m']), ('false', 'NaN'))
        transitions = self.rows('state_transitions')
        self.assertEqual([(r['old_nlos_type'], r['new_nlos_type']) for r in transitions],
                         [('none', 'soft'), ('soft', 'hard'), ('hard', 'none')])
        self.assertEqual((transitions[1]['old_state'], transitions[1]['new_state']), ('NLOS', 'NLOS'))
        self.assertEqual(transitions[1]['nlos_type'], 'hard')
        arrays = self.rows('uwb_query_arrays')
        self.assertEqual(len(arrays), 3)
        self.assertEqual({r['nlos_type'] for r in arrays}, {'soft', 'hard'})
        self.assertEqual({r['query_array_kind'] for r in arrays}, {'intersection', 'reflector'})
        q = next(r for r in self.rows('query_trace') if r['observation_id'] == 'obs-2')
        self.assertEqual(json.loads(q['reflector_geometry_id_list']), ['reflector'])

    def test_duplicate_messages_do_not_duplicate_rows(self):
        sample = facts(1, 'NLOS', 'soft')
        self.feed(sample)
        self.feed(sample)
        self.assertEqual(len(self.rows('uwb_observations')), 1)
        self.assertEqual(len(self.rows('query_trace')), 1)

    def test_run_identity_isolation(self):
        self.feed(facts(0, 'NLOS', 'soft', run='first'))
        self.feed(facts(0, 'NLOS', 'hard', run='second'))
        self.writer.refresh_transitions()
        self.assertEqual(len(self.rows('uwb_observations')), 2)
        self.assertEqual(self.rows('state_transitions'), [])

    def test_no_fabricated_geometry_for_empty_or_failed_query(self):
        self.feed(facts(0))
        self.feed(facts(1, 'UNKNOWN', 'none', False))
        self.assertEqual(self.rows('uwb_query_arrays'), [])

    def test_incomplete_joins_are_reported(self):
        self.writer.ingest('raw', facts(0)['raw'])
        report = self.writer.close()
        self.assertEqual(report['incomplete_observations'], 1)
        self.assertEqual(self.rows('uwb_observations'), [])

    def test_mismatched_classification_is_rejected(self):
        sample = facts(0, 'NLOS', 'soft')
        sample['observed']['nlos_type'] = 'hard'
        with self.assertRaisesRegex(ValueError, 'classification mismatch'):
            self.feed(sample)

    def test_conflicting_duplicate_is_rejected(self):
        sample = facts(0)
        self.writer.ingest('raw', sample['raw'])
        changed = copy.deepcopy(sample['raw'])
        changed['raw_range_m'] = 6.
        with self.assertRaisesRegex(ValueError, 'Conflicting duplicate'):
            self.writer.ingest('raw', changed)

    def test_existing_output_is_protected(self):
        with self.assertRaises(FileExistsError):
            UwbCsvWriter(self.output, ROOT / 'config/csv_templates')


if __name__ == '__main__':
    unittest.main()
