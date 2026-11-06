#!/usr/bin/env python3
"""B's minimal consumer: receive one ideal range and query the MOCK endpoint."""
import time,uuid,json
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from coal_platform_interfaces.msg import UwbRangeRaw
from coal_platform_interfaces.srv import QueryRay
def main():
    rclpy.init();node=Node('query_mock_example');node.declare_parameter('anchor_id','A01');anchor=node.get_parameter('anchor_id').value;latest=[]
    sub=node.create_subscription(UwbRangeRaw,'/platform/v1/uwb_ranges_raw',lambda m:latest.append(m) if m.anchor_id==anchor and m.raw_valid else None,QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
    client=node.create_client(QueryRay,'/mock/scene/query_ray')
    try:
        deadline=time.monotonic()+15
        while time.monotonic()<deadline and (not latest or not client.service_is_ready()):rclpy.spin_once(node,timeout_sec=.1)
        if not latest or not client.service_is_ready():raise RuntimeError('未收到该锚点有效raw或mock服务；先启动coal-face，并在本终端执行coal_env')
        raw=latest[-1];req=QueryRay.Request();req.context=deepcopy(raw.context);req.request_id=str(uuid.uuid4());req.observation_id=raw.observation_id;req.origin_m=deepcopy(raw.tag_position_m);req.target_m=deepcopy(raw.anchor_position_m);req.reflector_search_radius_m=2.
        future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=3.)
        if not future.done():raise RuntimeError('mock查询超时')
        result=future.result();assert result.request_id==req.request_id and result.observation_id==req.observation_id
        print(json.dumps(dict(query_mode='MOCK_FIXTURES_ONLY',scene_id=raw.context.header.scene_id,scene_version=raw.context.header.scene_version,run_id=raw.context.header.run_id,observation_id=raw.observation_id,anchor_id=raw.anchor_id,truth_range_m=raw.truth_range_m,raw_range_m=raw.raw_range_m,status=result.status,intersections=len(result.direct_intersections),reflectors=len(result.reflector_candidates),message=result.error_message),ensure_ascii=False,indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
