#!/usr/bin/env python3
"""Receive a real raw range and query the actual scene collision service."""
import json
import time
import uuid
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from coal_platform_interfaces.msg import UwbRangeRaw
from coal_platform_interfaces.srv import QueryRay


def main():
    rclpy.init()
    node = Node('query_real_example')
    node.declare_parameter('anchor_id', 'A01')
    anchor = node.get_parameter('anchor_id').value
    latest = []

    def receive(msg):
        if msg.anchor_id == anchor and msg.raw_valid:
            latest[:] = [msg]

    sub = node.create_subscription(UwbRangeRaw, '/platform/v1/uwb_ranges_raw', receive,
                                  QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    client = node.create_client(QueryRay, '/scene/query_ray')
    try:
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and (not latest or not client.service_is_ready()):
            rclpy.spin_once(node, timeout_sec=.1)
        if not latest or not client.service_is_ready():
            raise RuntimeError('No valid raw range or real query service; start coal-face and source coal_env')
        raw = latest[-1]
        req = QueryRay.Request()
        req.context = deepcopy(raw.context)
        req.request_id = str(uuid.uuid4())
        req.observation_id = raw.observation_id
        req.origin_m = deepcopy(raw.tag_position_m)
        req.target_m = deepcopy(raw.anchor_position_m)
        req.reflector_search_radius_m = 2.
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done():
            raise RuntimeError('Real geometry query timed out')
        result = future.result()
        assert result.context == req.context and result.request_id == req.request_id
        assert result.observation_id == req.observation_id
        print(json.dumps(dict(query_mode='ACTUAL_SCENE_COLLISION_GEOMETRY',
                              scene_id=raw.context.header.scene_id, scene_version=raw.context.header.scene_version,
                              run_id=raw.context.header.run_id, observation_id=raw.observation_id,
                              anchor_id=anchor, status=result.status,
                              intersections=len(result.direct_intersections),
                              hit_geometry_ids=[h.geometry_id for h in result.direct_intersections],
                              reflectors=len(result.reflector_candidates),
                              processing_time_ms=result.processing_time_ms,
                              message=result.error_message), ensure_ascii=False, indent=2))
        if result.status != 'OK':
            raise RuntimeError('Query rejected: ' + result.status)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
