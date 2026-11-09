#!/usr/bin/env python3
"""MOCK_ONLY interface test: actual UWB algorithm + ROS transport + five CSV files.

This tests the new transport/export contract, not physical correctness of the UWB
model. Algorithm-owner specialist fixtures remain a separate deliverable.
"""
import argparse
import csv
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src/coal_uwb_disturbance'),
               str(ROOT / 'src/coal_platform_bridge/scripts')]
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from coal_platform_interfaces.msg import Intersection, ReflectorCandidate, UwbRangeRaw
from coal_platform_interfaces.srv import QueryRay
from coal_uwb_disturbance.node import UwbDisturbance
from uwb_csv_recorder import UwbCsvRecorder


def run(output):
    rclpy.init(args=['--ros-args', '-p', f'output_dir:={output}'])
    harness = Node('mock_only_csv_chain')
    algorithm = UwbDisturbance()
    recorder = UwbCsvRecorder()
    cases = [('LOS', 'none', True), ('NLOS', 'soft', True),
             ('NLOS', 'hard', True), ('UNKNOWN', 'none', False),
             ('INPUT_INVALID', 'none', False)]

    def query(req, res):
        index = req.context.sequence
        res.context = req.context
        res.request_id = req.request_id
        res.observation_id = req.observation_id
        res.served_scene_version = '0.1.0'
        res.status = 'UNSUPPORTED' if index == 3 else 'OK'
        res.error_message = 'MOCK_ONLY deterministic export-contract facts'
        if index in (1, 2):
            hit = Intersection(geometry_id='contract-wall', geometry_type='BOX',
                               material='FIXTURE', reflection_level='LOW',
                               thickness_m=.1 if index == 1 else 1.,
                               distance_from_origin_m=2.)
            res.direct_intersections = [hit]
        if index == 2:
            res.reflector_candidates = [ReflectorCandidate(
                geometry_id='contract-reflector', geometry_type='BOX', material='FIXTURE',
                reflection_level='HIGH', distance_to_link_m=.5)]
        return res

    service = harness.create_service(QueryRay, '/mock/scene/query_ray', query)
    pub = harness.create_publisher(UwbRangeRaw, '/platform/v1/uwb_ranges_raw',
        QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    executor = SingleThreadedExecutor()
    for node in (harness, algorithm, recorder):
        executor.add_node(node)

    def spin_until(condition, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if condition():
                return
        raise AssertionError('ROS discovery/CSV completion timed out')

    try:
        spin_until(lambda: pub.get_subscription_count() >= 2 and
                   algorithm.client.service_is_ready() and
                   algorithm.decision_pub.get_subscription_count() >= 1 and
                   algorithm.observed_pub.get_subscription_count() >= 1 and
                   algorithm.query_trace_pub.get_subscription_count() >= 1)
        for index, _ in enumerate(cases):
            raw = UwbRangeRaw(tag_id='tag', anchor_id='anchor',
                              observation_id=f'csv-contract-{index}',
                              truth_range_m=5., raw_range_m=5.,
                              raw_valid=index != 4, raw_status='OK' if index != 4 else 'INPUT_INVALID')
            raw.context.header.stamp.sec = 4
            raw.context.header.stamp.nanosec = index * 100000001
            raw.context.header.frame_id = 'map'
            raw.context.header.experiment_id = 'MOCK_ONLY_CSV_CONTRACT'
            raw.context.header.run_id = 'csv-contract'
            raw.context.header.scene_id = 'csv-contract'
            raw.context.header.scene_version = '0.1.0'
            raw.context.schema_version = '0.2.0'
            raw.context.robot_id = 'robot'
            raw.context.sequence = index
            pub.publish(raw)
            spin_until(lambda: recorder.writer.counts['uwb_observations'] == index + 1)
        spin_until(lambda: recorder.writer.counts['query_trace'] == 4)
        report = recorder.writer.close()
        tables = {}
        for name in report['rows']:
            with (output / f'{name}.csv').open(newline='') as f:
                tables[name] = list(csv.DictReader(f))
        assert report['incomplete_observations'] == report['unmatched_query_traces'] == 0
        observations = tables['uwb_observations']
        assert [(r['propagation_state'], r['nlos_type'], r['valid'] == 'true')
                for r in observations] == cases
        for i, row in enumerate(observations):
            assert row['stamp'] == f'4.{i * 100000001:09d}'
            assert row['observation_id'] == f'csv-contract-{i}'
        assert tables['state_transitions'][1]['old_nlos_type'] == 'soft'
        assert tables['state_transitions'][1]['new_nlos_type'] == 'hard'
        assert len(tables['uwb_query_arrays']) == 3
        result = dict(passed=True, source='MOCK_ONLY_CSV_CONTRACT',
                      classifications=[list(c) for c in cases], **report)
        (output / 'contract_test.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
    finally:
        recorder.writer.close()
        executor.shutdown()
        for node in (harness, algorithm, recorder):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    run(parser.parse_args().output)
