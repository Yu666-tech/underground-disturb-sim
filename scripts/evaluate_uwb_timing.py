#!/usr/bin/env python3
"""Stage-3 reproducible timing evaluation; no graph optimizer is implemented here.

Run each profile in its own ROS_DOMAIN_ID, with installed resources and a NEW
output directory. The measured interval excludes discovery/startup and shutdown.
"""
import argparse
from collections import Counter, defaultdict
import csv
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src/coal_platform_devtools/scripts'),
               str(ROOT / 'src/coal_platform_bridge/scripts')]
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String
from coal_platform_interfaces.msg import (GroundTruth, UwbDecision, UwbRangeObserved,
                                           UwbRangeRaw, UwbQueryTrace)
from coal_platform_interfaces.srv import QueryRay
from fixture_inputs import FixtureInputs
from uwb_csv import identity


def stamp(message):
    s = message['context']['header']['stamp']
    return s['sec'] * 1000000000 + s['nanosec']


def truth_key(message):
    c = message['context']
    h = c['header']
    return (h['experiment_id'], h['run_id'], h['scene_id'], h['scene_version'],
            c['robot_id'], stamp(message))


def percentile(values, q):
    if not values:
        return None
    values = sorted(values)
    return values[round((len(values) - 1) * q)]


class Probe(Node):
    def __init__(self, output):
        super().__init__('uwb_timing_probe')
        self.messages = {kind: {} for kind in ('raw', 'decision', 'observed', 'query')}
        self.arrivals = {kind: {} for kind in self.messages}
        self.duplicates = Counter()
        self.truths = {}
        self.latest_truth_ns = None
        self.diagnostics = {}
        self.query_info = None
        self.processing_snapshots = []
        self.log = (output / 'transport.jsonl').open('w')
        self.subs = []
        sensor = QoSProfile(depth=10000, reliability=ReliabilityPolicy.RELIABLE)
        reliable = QoSProfile(depth=10000, reliability=ReliabilityPolicy.RELIABLE)
        for kind, cls, topic, qos in (
            ('raw', UwbRangeRaw, '/platform/v1/uwb_ranges_raw', sensor),
            ('decision', UwbDecision, '/disturbance_decision/uwb', reliable),
            ('observed', UwbRangeObserved, '/uwb_ranges', sensor),
            ('query', UwbQueryTrace, '/disturbance_query/uwb', reliable)):
            self.subs.append(self.create_subscription(cls, topic,
                lambda msg, k=kind: self.receive(k, msg), qos))
        self.subs.append(self.create_subscription(GroundTruth, '/platform/v1/ground_truth', self.truth, reliable))
        self.subs.append(self.create_subscription(String, '/platform/v1/query_timing', self.timing, reliable))
        self.subs.append(self.create_subscription(String, '/platform/v1/uwb_processing',
            lambda msg: self.processing_snapshots.append(json.loads(msg.data)), reliable))
        self.subs.append(self.create_subscription(String, '/platform/v1/query_info', self.info,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def receive(self, kind, msg):
        data = message_to_ordereddict(msg)
        key = identity(data)
        if key in self.messages[kind]:
            self.duplicates[kind] += 1
        self.messages[kind][key] = data
        self.arrivals[kind][key] = time.monotonic_ns()
        self.log.write(json.dumps(dict(kind=kind, arrival_ns=self.arrivals[kind][key], message=data)) + '\n')

    def truth(self, msg):
        data = message_to_ordereddict(msg)
        self.truths[truth_key(data)] = data
        self.latest_truth_ns = stamp(data)
        self.log.write(json.dumps(dict(kind='truth', message=data)) + '\n')

    def timing(self, msg):
        data = json.loads(msg.data)
        self.diagnostics[data['request_id']] = data
        self.log.write(json.dumps(dict(kind='timing', message=data)) + '\n')

    def info(self, msg):
        self.query_info = json.loads(msg.data)


def analyze(probe, keys, args, wall_seconds, sim_seconds):
    raw = probe.messages['raw']
    decisions = probe.messages['decision']
    observed = probe.messages['observed']
    queries = probe.messages['query']
    mismatches = []
    incomplete = []
    future_rejections = 0
    query_status = Counter()
    timing_reason = Counter()
    deltas = []
    processing = []
    latency = []
    links = defaultdict(list)
    mock_mismatch = []
    errors = []
    for key in keys:
        r = raw[key]
        # Input sequence continuity must use ALL raw, independently of replies.
        links[(truth_key(r)[:5], r['tag_id'], r['anchor_id'])].append(key)
        if key not in decisions or key not in observed:
            incomplete.append(r['observation_id'])
            continue
        d, o = decisions[key], observed[key]
        if r['context'] != d['context'] or r['context'] != o['context']:
            mismatches.append([r['observation_id'], 'raw/decision/observed context'])
        if d['decision_id'] != o['decision_id'] or d['nlos_type'] != o['nlos_type']:
            mismatches.append([r['observation_id'], 'decision/observed ID or classification'])
        if truth_key(r) not in probe.truths:
            mismatches.append([r['observation_id'], 'no ground truth at exact sample stamp'])
        if r['raw_valid']:
            if key not in queries:
                incomplete.append(r['observation_id'] + '/query')
            else:
                q = queries[key]
                query_status[q['status']] += 1
                if (q['context'] != r['context'] or q['nlos_type'] != d['nlos_type'] or
                        q['decision_id'] != d['decision_id'] or q['status'] != d['query_status']):
                    mismatches.append([r['observation_id'], 'query correlation'])
                if args.profile != 'mock':
                    diagnostic = probe.diagnostics.get(q['request_id'])
                    if diagnostic is None:
                        incomplete.append(r['observation_id'] + '/diagnostic')
                    else:
                        reason = diagnostic['guard_reason']
                        timing_reason[reason] += 1
                        future_rejections += reason == 'FUTURE_BEYOND_TOLERANCE'
                        if diagnostic['request_stamp_ns'] != stamp(r):
                            mismatches.append([r['observation_id'], 'query diagnostic sample stamp'])
                        if diagnostic['request_latest_delta_ms'] is not None:
                            deltas.append(diagnostic['request_latest_delta_ms'])
                        processing.append(diagnostic['processing_time_ms'])
                latency.append((probe.arrivals['query'][key] - probe.arrivals['raw'][key]) / 1e6)
        if o['valid']:
            if not math.isfinite(o['observed_range_m']):
                mismatches.append([r['observation_id'], 'valid observation nonfinite'])
            injected = d['random_noise_m'] + d['nlos_error_m'] + d['multipath_error_m']
            if not math.isclose(o['observed_range_m'] - r['raw_range_m'], injected, abs_tol=1e-9):
                mismatches.append([r['observation_id'], 'unexplained range-error jump'])
            if not math.isclose(o['observed_range_m'] - r['truth_range_m'], o['error_m'], abs_tol=1e-9):
                mismatches.append([r['observation_id'], 'truth-error mismatch'])
            errors.append(o['error_m'])
        elif not math.isnan(o['observed_range_m']):
            mismatches.append([r['observation_id'], 'invalid observation is not NaN'])
        if args.profile == 'mock':
            anchor = r['anchor_id']
            if anchor in ('A00', 'A02', 'A08'):
                expected = ('LOS', 'none')
            elif anchor in ('A01', 'A03'):
                width = min(.3, r['truth_range_m'] * .1)
                expected = ('NLOS', 'soft') if width < .25 else (
                    ('OUTAGE', 'none') if anchor == 'A01' else ('NLOS', 'hard'))
            elif anchor == 'A04':
                expected = ('UNKNOWN', 'none')
            elif anchor in ('A05', 'A06'):
                expected = ('INPUT_INVALID', 'none')
            else:
                expected = ('OUTAGE', 'none')
            if (d['propagation_state'], d['nlos_type']) != expected:
                mock_mismatch.append(r['observation_id'])
    missing_sequences = 0
    raw_order_inversions = 0
    output_order_inversions = 0
    unexplained_truth_jumps = 0
    error_jumps = []
    for sample_keys in links.values():
        samples = sorted(sample_keys, key=lambda k: stamp(raw[k]))
        sequences = [raw[k]['context']['sequence'] for k in samples]
        missing_sequences += max(sequences) - min(sequences) + 1 - len(set(sequences))
        arrivals = sorted(samples, key=lambda k: probe.arrivals['raw'][k])
        raw_order_inversions += sum(stamp(raw[b]) <= stamp(raw[a]) for a, b in zip(arrivals, arrivals[1:]))
        replies = sorted((k for k in samples if k in observed), key=lambda k: probe.arrivals['observed'][k])
        output_order_inversions += sum(stamp(raw[b]) <= stamp(raw[a]) for a, b in zip(replies, replies[1:]))
        for a, b in zip(samples, samples[1:]):
            x, y = raw[a], raw[b]
            travel = sum(math.dist([x[p][axis] for axis in 'xyz'], [y[p][axis] for axis in 'xyz'])
                         for p in ('tag_position_m', 'anchor_position_m'))
            if abs(y['truth_range_m'] - x['truth_range_m']) > travel + 1e-8:
                unexplained_truth_jumps += 1
            if a in observed and b in observed and observed[a]['valid'] and observed[b]['valid']:
                error_jumps.append(abs(observed[b]['error_m'] - observed[a]['error_m']))
    count = len(keys)
    attempted = sum(query_status.values())
    eligible = sum(raw[k]['raw_valid'] for k in keys)
    future_rate = future_rejections / eligible if eligible else 1.
    other_rejections = sum(n for status, n in query_status.items() if status != 'OK') - future_rejections
    # Thresholds are declared before evaluation; scripted Mock errors are expected.
    suitable = (not mismatches and not incomplete and not missing_sequences and
                not raw_order_inversions and not unexplained_truth_jumps and not mock_mismatch and
                future_rate <= .001 and abs(count / sim_seconds - 90.) <= 1. and (args.profile == 'mock' or other_rejections == 0) and
                (args.profile == 'mock' or eligible >= 1000))
    return dict(profile=args.profile, future_tolerance_ms=args.tolerance_ms,
        suitable=suitable, cohort_raw=count, cohort_valid_raw=eligible, cohort_query_attempts=attempted,
        wall_seconds=wall_seconds, simulation_seconds=sim_seconds,
        raw_rate_per_sim_second=count / sim_seconds, query_rate_per_wall_second=attempted / wall_seconds,
        real_time_factor=sim_seconds / wall_seconds, query_status_counts=dict(query_status),
        timing_reason_counts=dict(timing_reason), future_rejection_rate=future_rate,
        incomplete_count=len(incomplete), incomplete_examples=incomplete[:10],
        association_mismatches=len(mismatches), mismatch_examples=mismatches[:10],
        missing_raw_sequences=missing_sequences, raw_arrival_order_inversions=raw_order_inversions,
        observed_arrival_order_inversions=output_order_inversions,
        unexplained_truth_jumps=unexplained_truth_jumps, mock_dataset_mismatches=len(mock_mismatch),
        request_minus_latest_truth_ms=dict(p50=percentile(deltas, .5), p95=percentile(deltas, .95),
                                          p99=percentile(deltas, .99), maximum=max(deltas) if deltas else None),
        service_processing_ms=dict(p50=percentile(processing, .5), p99=percentile(processing, .99)),
        raw_to_query_arrival_ms=dict(p50=percentile(latency, .5), p99=percentile(latency, .99),
                                     negative_count=sum(v < 0 for v in latency)),
        error_jump_m=dict(p99=percentile(error_jumps, .99), maximum=max(error_jumps) if error_jumps else None),
        duplicate_messages=dict(probe.duplicates), query_info=probe.query_info,
        processing_diagnostics=probe.processing_snapshots[-10:],
        factor_graph='Not implemented (confirmed by user); exact sample-time association only.',
        gate=dict(max_future_rejection_rate=.001, max_missing=0, max_association_mismatch=0,
                  max_unexplained_jump=0, minimum_real_eligible_queries=1000))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--profile', choices=('mock', 'scene_default', 'scene_90hz'), required=True)
    parser.add_argument('--tolerance-ms', type=float, default=50.)
    parser.add_argument('--duration-s', type=float, default=30.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, GAZEBO_MODEL_DATABASE_URI='', QT_QPA_PLATFORM='offscreen')
    processes = {}

    def start(name, command):
        stream = (args.output / f'{name}.log').open('w')
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes[name] = (process, stream)

    def stop(name):
        if name not in processes:
            return
        process, stream = processes[name]
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        stream.close()

    rclpy.init(args=[])
    probe = Probe(args.output)
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    fixture = None
    try:
        if args.profile == 'mock':
            fixture = FixtureInputs()
            executor.add_node(fixture)
        else:
            share = Path(get_package_share_directory('coal_face_sim'))
            scene = share / 'scenarios/coal_face_v1'
            config = json.loads((ROOT / 'src/coal_platform_bridge/config/bc_inputs.json').read_text())
            if args.profile == 'scene_90hz':
                config['anchors'] = config['anchors'][:5] + [
                    dict(anchor_id='A05', position_m=[-2., 5.25, 1.2]),
                    dict(anchor_id='A06', position_m=[1.6, 35., 1.9]),
                    dict(anchor_id='A07', position_m=[1.7, 65., 2.2]),
                    dict(anchor_id='A08', position_m=[1.55, 90., 1.8])]
            config_path = args.output / 'source_config.json'
            config_path.write_text(json.dumps(config, indent=2))
            start('physics', ['ros2', 'launch', 'coal_face_sim', 'coal_face_full.launch.py', 'gui:=false', 'rviz:=false'])
            start('bridge', ['ros2', 'run', 'coal_platform_bridge', 'platform_bridge.py', '--ros-args', '-p', 'use_sim_time:=true'])
            start('query', ['ros2', 'run', 'coal_scene_query', 'scene_query.py', '--ros-args',
                '-p', f'world_file:={scene}/worlds/coal_face.sdf', '-p', f'models_root:={scene}/models',
                '-p', f'future_tolerance_ms:={args.tolerance_ms}', '-p', 'publish_timing_diagnostics:=true'])
            start('inputs', ['ros2', 'run', 'coal_platform_bridge', 'bc_inputs.py', '--ros-args',
                '-p', 'use_sim_time:=true', '-p', f'config_file:={config_path}'])
        start('algorithm', ['ros2', 'run', 'coal_uwb_disturbance', 'uwb_node', '--ros-args',
            '-p', 'use_sim_time:=true', '-p', f'query_service:={"/mock/scene/query_ray" if fixture else "/scene/query_ray"}',
            '-p', 'publish_processing_diagnostics:=true',
            '-p', f'config_file:={ROOT}/config/algorithms/uwb.yaml'])
        start('recorder', ['ros2', 'run', 'coal_platform_bridge', 'uwb_csv_recorder.py', '--ros-args',
            '-p', f'output_dir:={args.output}/csv'])
        deadline = time.monotonic() + 300
        start_ns = None
        start_wall = None
        next_progress = time.monotonic()
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            for name, (proc, _) in processes.items():
                if proc.poll() is not None:
                    raise RuntimeError(f'{name} terminated: inspect {name}.log')
            if start_ns is None:
                ready = len(probe.messages['query']) >= 30 and (
                    fixture is not None or probe.query_info is not None)
                csv_path = args.output / 'csv/uwb_observations.csv'
                if ready:
                    if csv_path.exists():
                        with csv_path.open(newline='') as f:
                            ready = len(list(csv.DictReader(f))) >= 20
                    else:
                        ready = False
                if ready and probe.latest_truth_ns is not None:
                    start_ns = probe.latest_truth_ns + 100000000
            elif probe.latest_truth_ns >= start_ns:
                if start_wall is None:
                    start_wall = time.monotonic()
                if probe.latest_truth_ns >= start_ns + round(args.duration_s * 1e9):
                    break
            if time.monotonic() >= next_progress:
                print(json.dumps(dict(progress=args.profile, tolerance=args.tolerance_ms,
                    simulation_ns=probe.latest_truth_ns, raw_seen=len(probe.messages['raw']),
                    queries_seen=len(probe.messages['query']))), flush=True)
                next_progress = time.monotonic() + 10
        else:
            raise TimeoutError('Could not collect the prescribed simulation interval')
        wall_seconds = time.monotonic() - start_wall
        end_ns = start_ns + round(args.duration_s * 1e9)
        if fixture is not None:
            fixture.timer.cancel()
        else:
            stop('inputs')
        # Keep truth/query services alive while draining all already-produced inputs.
        drain = time.monotonic() + 20
        while time.monotonic() < drain:
            executor.spin_once(timeout_sec=.02)
            raw = probe.messages['raw']
            complete = all(k in probe.messages['decision'] and k in probe.messages['observed'] and
                (not r['raw_valid'] or probe.messages['decision'][k]['query_status'] == 'NOT_CALLED'
                 or k in probe.messages['query']) for k, r in raw.items())
            if complete:
                for _ in range(50):
                    executor.spin_once(timeout_sec=.01)
                break
        keys = [k for k, r in probe.messages['raw'].items() if start_ns <= stamp(r) < end_ns]
        # Wait for the independent recorder to persist the SAME observation cohort.
        expected_ids = {probe.messages['raw'][k]['observation_id'] for k in keys}
        expected_queries = {k for k in keys if k in probe.messages['query']}
        csv_observations = args.output / 'csv/uwb_observations.csv'
        database = args.output / 'csv/uwb_join.sqlite3'
        csv_deadline = time.monotonic() + 20
        persisted = {}
        while time.monotonic() < csv_deadline:
            executor.spin_once(timeout_sec=.02)
            if database.exists():
                with sqlite3.connect(f'file:{database}?mode=ro', uri=True) as db:
                    completed = {row[0] for row in db.execute('SELECT id FROM completed')}
                    written_queries = {row[0] for row in db.execute('SELECT id FROM queries WHERE written=1')}
                if set(keys).issubset(completed) and expected_queries.issubset(written_queries):
                    break
            time.sleep(.05)
        with csv_observations.open(newline='') as f:
            persisted = {r['observation_id']: r for r in csv.DictReader(f)}
        result = analyze(probe, keys, args, wall_seconds, args.duration_s)
        result['missing_cohort_csv'] = len(expected_ids - set(persisted))
        csv_mismatches = 0
        for table in ('uwb_observations', 'multipath_error_detail', 'query_trace', 'uwb_query_arrays'):
            with (args.output / f'csv/{table}.csv').open(newline='') as f:
                rows = [r for r in csv.DictReader(f) if r['observation_id'] in expected_ids]
            for row in rows:
                source = persisted.get(row['observation_id'])
                if source is None or (row['stamp'], row['decision_id'], row['nlos_type']) != (
                        source['stamp'], source['decision_id'], source['nlos_type']):
                    csv_mismatches += 1
            expected = len(keys) if table in ('uwb_observations', 'multipath_error_detail') else (
                len(expected_queries) if table == 'query_trace' else sum(
                    len(probe.messages['query'][k]['direct_intersections']) +
                    len(probe.messages['query'][k]['reflector_candidates']) for k in expected_queries))
            csv_mismatches += abs(len(rows) - expected)
        result['csv_association_mismatches'] = csv_mismatches
        result['suitable'] = result['suitable'] and result['missing_cohort_csv'] == 0 and csv_mismatches == 0
        (args.output / 'assessment.json').write_text(json.dumps(result, indent=2))
        with (args.output / 'alignment.csv').open('w', newline='') as f:
            fields = ('observation_id', 'stamp_ns', 'anchor_id', 'sequence', 'decision_id', 'nlos_type',
                      'query_status', 'query_delta_ms', 'truth_at_exact_stamp')
            writer = csv.DictWriter(f, fields)
            writer.writeheader()
            for k in keys:
                r = probe.messages['raw'][k]
                d = probe.messages['decision'].get(k, {})
                q = probe.messages['query'].get(k, {})
                diagnostic = probe.diagnostics.get(q.get('request_id'), {})
                writer.writerow(dict(observation_id=r['observation_id'], stamp_ns=stamp(r),
                    anchor_id=r['anchor_id'], sequence=r['context']['sequence'],
                    decision_id=d.get('decision_id'), nlos_type=d.get('nlos_type'),
                    query_status=d.get('query_status'), query_delta_ms=diagnostic.get('request_latest_delta_ms'),
                    truth_at_exact_stamp=truth_key(r) in probe.truths))
        print(json.dumps(result, indent=2), flush=True)
    finally:
        if fixture is not None:
            fixture.timer.cancel()
        for name in ('algorithm', 'recorder', 'inputs', 'query', 'bridge', 'physics'):
            stop(name)
        executor.shutdown()
        probe.log.close()
        probe.destroy_node()
        if fixture is not None:
            fixture.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
