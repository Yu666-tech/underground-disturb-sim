"""UWB CSV storage; preserve sample identities and join independently arriving topics."""
import csv
import json
import math
import sqlite3
from pathlib import Path


TABLES = ('uwb_observations', 'state_transitions', 'query_trace',
          'multipath_error_detail', 'uwb_query_arrays')


def encoded(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN'
    if isinstance(value, (list, dict)):
        return json.dumps(value, ensure_ascii=False, allow_nan=False)
    return value


def identity(message):
    c = message['context']
    h = c['header']
    return json.dumps([h[k] for k in ('experiment_id', 'run_id', 'scene_id',
                                     'scene_version')] +
                     [c['robot_id'], message['observation_id']])


def common(message):
    h = message['context']['header']
    row = {k: h[k] for k in ('frame_id', 'experiment_id', 'run_id',
                             'scene_id', 'scene_version')}
    row['stamp'] = f"{h['stamp']['sec']}.{h['stamp']['nanosec']:09d}"
    row['observation_id'] = message['observation_id']
    return row


def validate_classification(message, state):
    kind = message['nlos_type']
    if kind not in ('none', 'soft', 'hard'):
        raise ValueError('nlos_type must be none/soft/hard')
    if state == 'NLOS' and message['valid']:
        if kind == 'none':
            raise ValueError('Valid NLOS requires soft/hard')
    elif kind != 'none':
        raise ValueError('Non-NLOS/invalid observations require nlos_type=none')


class UwbCsvWriter:
    """Disk-backed joins/deduplication, with atomically refreshed ordered transitions.

    SQLite keeps incomplete topic joins off the Python heap. Observation/error/query
    rows stream to disk immediately; transitions are derived in sample-time order,
    so service-response reordering cannot invent backwards state changes.
    """

    def __init__(self, output_dir, template_dir):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.headers = {}
        self.files = {}
        self.writers = {}
        for name in TABLES:
            with (Path(template_dir) / f'{name}.template.csv').open(newline='') as f:
                self.headers[name] = next(csv.reader(f))
            if name != 'state_transitions':
                f = (self.output_dir / f'{name}.csv').open('x', newline='', encoding='utf-8')
                self.files[name] = f
                self.writers[name] = csv.DictWriter(f, self.headers[name])
                self.writers[name].writeheader()
                f.flush()
        self.db = sqlite3.connect(self.output_dir / 'uwb_join.sqlite3')
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('PRAGMA synchronous=NORMAL')
        self.db.executescript('''
            CREATE TABLE facts (id TEXT, kind TEXT, data TEXT,
                                PRIMARY KEY (id, kind));
            CREATE TABLE completed (id TEXT PRIMARY KEY, link TEXT, stamp INTEGER,
                                    sequence INTEGER, data TEXT);
            CREATE TABLE queries (id TEXT, request_id TEXT, data TEXT, written INTEGER,
                                  PRIMARY KEY (id, request_id));
            CREATE INDEX sample_order ON completed (link, stamp, sequence, id);
        ''')
        self.counts = {name: 0 for name in TABLES}
        self.transitions_dirty = True
        self.closed = False
        self.refresh_transitions()

    def _write(self, table, row):
        self.writers[table].writerow({k: encoded(row.get(k, ''))
                                     for k in self.headers[table]})
        self.files[table].flush()
        self.counts[table] += 1

    def ingest(self, kind, message):
        if kind not in ('raw', 'decision', 'observed', 'query'):
            raise ValueError(f'Unknown fact type: {kind}')
        key = identity(message)
        data = json.dumps(message, sort_keys=True)
        if kind == 'query':
            request_id = message['request_id']
            if not request_id:
                raise ValueError('Actual query traces require request_id')
            old = self.db.execute('SELECT data FROM queries WHERE id=? AND request_id=?',
                                  (key, request_id)).fetchone()
            if old and old[0] != data:
                raise ValueError('Conflicting duplicate query trace')
            self.db.execute('INSERT OR IGNORE INTO queries VALUES (?, ?, ?, 0)',
                            (key, request_id, data))
        else:
            old = self.db.execute('SELECT data FROM facts WHERE id=? AND kind=?',
                                  (key, kind)).fetchone()
            if old and old[0] != data:
                raise ValueError(f'Conflicting duplicate {kind}')
            self.db.execute('INSERT OR IGNORE INTO facts VALUES (?, ?, ?)', (key, kind, data))
        self._flush(key)
        self.db.commit()

    def _flush(self, key):
        facts = {kind: json.loads(data) for kind, data in self.db.execute(
            'SELECT kind, data FROM facts WHERE id=?', (key,))}
        if not all(k in facts for k in ('raw', 'decision', 'observed')):
            return
        raw, decision, observed = (facts[k] for k in ('raw', 'decision', 'observed'))
        if not (raw['context'] == decision['context'] == observed['context']):
            raise ValueError('Raw/decision/observed sample contexts differ')
        if not (raw['anchor_id'] == decision['anchor_id'] == observed['anchor_id']):
            raise ValueError('Anchor association mismatch')
        if raw['tag_id'] != observed['tag_id']:
            raise ValueError('Tag association mismatch')
        if decision['decision_id'] != observed['decision_id']:
            raise ValueError('Decision association mismatch')
        if (decision['propagation_state'], decision['nlos_type'], decision['valid']) != (
                observed['state'], observed['nlos_type'], observed['valid']):
            raise ValueError('Decision/observed classification mismatch')
        validate_classification(decision, decision['propagation_state'])
        row = common(raw)
        row.update({k: observed[k] for k in ('decision_id', 'anchor_id', 'tag_id',
                   'truth_range_m', 'raw_range_m', 'observed_range_m', 'error_m',
                   'nlos_type', 'valid')})
        row.update(robot_id=raw['context']['robot_id'],
                   propagation_state=decision['propagation_state'], cause=decision['cause'],
                   raw_valid=raw['raw_valid'])
        if not self.db.execute('SELECT 1 FROM completed WHERE id=?', (key,)).fetchone():
            self._write('uwb_observations', row)
            detail = dict(row)
            detail.update({k: decision[k] for k in
                           ('multipath_error_m', 'nlos_error_m', 'random_noise_m')})
            self._write('multipath_error_detail', detail)
            h = raw['context']['header']
            link = json.dumps([row[k] for k in ('experiment_id', 'run_id', 'scene_id',
                              'scene_version', 'robot_id', 'tag_id', 'anchor_id')])
            stamp = h['stamp']['sec'] * 1000000000 + h['stamp']['nanosec']
            self.db.execute('INSERT INTO completed VALUES (?, ?, ?, ?, ?)',
                            (key, link, stamp, raw['context']['sequence'], json.dumps(row)))
            self.transitions_dirty = True
        for request_id, data in self.db.execute(
                'SELECT request_id, data FROM queries WHERE id=? AND written=0',
                (key,)).fetchall():
            trace = json.loads(data)
            if (trace['context'] != raw['context'] or
                    trace['decision_id'] != decision['decision_id'] or
                    trace['anchor_id'] != raw['anchor_id'] or
                    trace['tag_id'] != raw['tag_id'] or
                    trace['nlos_type'] != decision['nlos_type'] or
                    trace['valid'] != decision['valid'] or
                    trace['status'] != decision['query_status']):
                raise ValueError('Query trace association mismatch')
            q = dict(row, request_id=request_id, status=trace['status'],
                     processing_time_ms=trace['processing_time_ms'],
                     error_message=trace['error_message'],
                     intersect_geometry_id_list=[h['geometry_id'] for h in trace['direct_intersections']],
                     reflector_geometry_id_list=[h['geometry_id'] for h in trace['reflector_candidates']])
            self._write('query_trace', q)
            for kind, hits, distance in (
                    ('intersection', trace['direct_intersections'], 'distance_from_origin_m'),
                    ('reflector', trace['reflector_candidates'], 'distance_to_link_m')):
                for hit in hits:
                    item = dict(q, geometry_id=hit['geometry_id'],
                                geometry_type=hit['geometry_type'], material=hit['material'],
                                reflection_level=hit['reflection_level'],
                                distance_m=hit[distance], query_array_kind=kind)
                    self._write('uwb_query_arrays', item)
            self.db.execute('UPDATE queries SET written=1 WHERE id=? AND request_id=?',
                            (key, request_id))

    def refresh_transitions(self):
        if not self.transitions_dirty:
            return
        path = self.output_dir / 'state_transitions.csv'
        temporary = path.with_suffix('.csv.tmp')
        count = 0
        previous_link = previous = None
        with temporary.open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, self.headers['state_transitions'])
            writer.writeheader()
            for link, data in self.db.execute(
                    'SELECT link, data FROM completed ORDER BY link, stamp, sequence, id'):
                row = json.loads(data)
                state = (row['propagation_state'], row['nlos_type'])
                if previous_link == link and previous != state:
                    transition = dict(row, old_state=previous[0], new_state=state[0],
                                      old_nlos_type=previous[1], new_nlos_type=state[1],
                                      trigger_cause=row['cause'])
                    writer.writerow({k: encoded(transition.get(k, ''))
                                     for k in self.headers['state_transitions']})
                    count += 1
                previous_link, previous = link, state
        temporary.replace(path)
        self.counts['state_transitions'] = count
        self.transitions_dirty = False

    def close(self):
        if self.closed:
            return
        self.refresh_transitions()
        pending = self.db.execute('SELECT COUNT(DISTINCT id) FROM facts '
                                  'WHERE id NOT IN (SELECT id FROM completed)').fetchone()[0]
        pending_queries = self.db.execute('SELECT COUNT(*) FROM queries WHERE written=0').fetchone()[0]
        report = dict(rows=self.counts, incomplete_observations=pending,
                      unmatched_query_traces=pending_queries,
                      ordering='Transitions ordered by sample stamp/sequence per run/tag/anchor; '
                               'other CSV rows retain arrival order and original sample stamps.')
        (self.output_dir / 'recording_summary.json').write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        for f in self.files.values():
            f.close()
        self.db.commit()
        self.db.close()
        self.closed = True
        return report
