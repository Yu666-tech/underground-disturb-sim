#!/usr/bin/env python3
"""Actual ROS service checks for configured tolerance and unchanged identity/age guards."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/coal_scene_query/scripts'))
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from coal_platform_interfaces.msg import GroundTruth
from coal_platform_interfaces.srv import QueryRay
from scene_query import SceneQuery


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', type=Path)
    parser.add_argument('--tolerance-ms', type=float, default=50.)
    args = parser.parse_args()
    share = Path(get_package_share_directory('coal_face_sim')) / 'scenarios/coal_face_v1'
    rclpy.init(args=['--ros-args', '-p', f'world_file:={share}/worlds/coal_face.sdf',
                    '-p', f'models_root:={share}/models', '-p', f'future_tolerance_ms:={args.tolerance_ms}'])
    server = SceneQuery()
    client_node = Node('time_contract_client')
    client = client_node.create_client(QueryRay, '/scene/query_ray')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client_node)
    latest = GroundTruth()
    c = latest.context
    c.schema_version = '0.2.0'
    c.robot_id = 'robot'
    c.header.frame_id = 'map'
    c.header.experiment_id = 'MOCK_TIME_CONTRACT'
    c.header.run_id = 'time-contract'
    c.header.scene_id = 'coal_face_base'
    c.header.scene_version = '0.2.0'
    c.header.stamp.sec = 90
    server.on_truth(deepcopy(latest))
    c.header.stamp.sec = 100
    server.on_truth(latest)
    results = []
    try:
        deadline = time.monotonic() + 10
        while not client.service_is_ready() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert client.service_is_ready()
        future_ns = round(args.tolerance_ms * 1e6)
        cases = [('present', 0, 'OK'), ('20ms', 20000000, 'OK'),
                 ('exact_future_boundary', future_ns, 'OK'),
                 ('future_boundary_plus_1ns', future_ns + 1, 'TIME_UNAVAILABLE'),
                 ('exact_old_boundary', -2000000000, 'OK'),
                 ('old_boundary_minus_1ns', -2000000001, 'TIME_UNAVAILABLE'),
                 ('before_run', -11000000000, 'TIME_UNAVAILABLE'),
                 ('foreign_run', 0, 'TIME_UNAVAILABLE')]
        for name, delta, expected in cases:
            request = QueryRay.Request()
            request.context = deepcopy(c)
            stamp_ns = 100000000000 + delta
            request.context.header.stamp.sec, request.context.header.stamp.nanosec = divmod(stamp_ns, 1000000000)
            if name == 'foreign_run':
                request.context.header.run_id = 'foreign-run'
            request.request_id = name
            request.observation_id = 'contract/' + name
            request.origin_m.x = request.target_m.x = .6
            request.origin_m.y, request.target_m.y = 5., 6.
            request.origin_m.z = request.target_m.z = 1.2
            future = client.call_async(request)
            deadline = time.monotonic() + 5
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            assert future.done(), name
            response = future.result()
            assert response.status == expected, (name, response.status, response.error_message)
            assert response.context == request.context
            assert response.request_id == name and response.observation_id == request.observation_id
            results.append(dict(name=name, delta_ns=delta, status=response.status,
                                reason=response.error_message, sample_context_preserved=True))
        args.output.write_text(json.dumps(dict(passed=True, tolerance_ms=args.tolerance_ms,
                                              calls=results), indent=2))
        print(json.dumps(dict(passed=True, tolerance_ms=args.tolerance_ms, cases=len(results))))
    finally:
        executor.shutdown()
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
