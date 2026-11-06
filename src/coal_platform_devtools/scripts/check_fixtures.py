#!/usr/bin/env python3
"""Exercise fixture facts and service contracts; never assert B/C algorithms."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--launch', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('fixture_verification.json'))
    args = parser.parse_args()
    if args.launch:
        os.environ['ROS_DOMAIN_ID'] = os.environ.get('COAL_FIXTURE_VERIFY_DOMAIN_ID', '108')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    from coal_platform_interfaces.msg import GroundTruth, RobotState, UwbRangeRaw, LidarFrameRaw
    from coal_platform_interfaces.srv import QueryRay
    rclpy.init()
    node = Node('check_bc_fixtures')
    proc = None
    log = None
    first = None
    latest = None
    truths = []
    scans = []
    states = set()
    zones = set()
    raws = {}
    info = []
    ids = set()
    query_results = {}
    sensor = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def ns(context):
        return context.header.stamp.sec*1000000000+context.header.stamp.nanosec

    def check_context(m):
        c = m.context
        assert c.header.experiment_id=='MOCK_FIXTURES_ONLY' and c.header.scene_id=='mock_bc_fixture'
        assert c.header.scene_version=='0.1.0' and c.schema_version=='0.2.0'
        assert c.header.run_id and c.robot_id=='shearer_01'
        if hasattr(m, 'observation_id'):
            assert m.observation_id not in ids
            ids.add(m.observation_id)

    def truth(m):
        nonlocal first, latest
        check_context(m)
        assert m.context.header.frame_id=='map'
        latest = ns(m.context)
        first = first if first is not None else latest
        truths.append(latest)

    def state(m):
        check_context(m)
        states.add(m.work_mode)
        zones.add(m.zone_id)
        assert m.is_moving == (abs(m.commanded_velocity_map_mps.y)>.001)

    def raw(m):
        check_context(m)
        distance = math.dist([m.tag_position_m.x,m.tag_position_m.y,m.tag_position_m.z],
                             [m.anchor_position_m.x,m.anchor_position_m.y,m.anchor_position_m.z])
        assert math.isclose(distance,m.truth_range_m,rel_tol=1e-12)
        if m.anchor_id in ['A05','A06','A07']:
            assert not m.raw_valid and math.isnan(m.raw_range_m)
            assert m.raw_status=={'A05':'INPUT_INVALID','A06':'SENSOR_UNAVAILABLE','A07':'OUT_OF_RANGE'}[m.anchor_id]
            if m.anchor_id=='A07':assert m.truth_range_m>m.sensor_max_range_m
        else:
            assert m.raw_valid and m.raw_status=='OK' and m.raw_range_m==m.truth_range_m
        raws[m.anchor_id]=m

    def scan(m):
        check_context(m)
        assert m.context.header.frame_id=='laser_link'
        assert m.beam_count==720 and len(m.raw_ranges_m)==len(m.truth_ranges_m)==len(m.raw_valid)==len(m.raw_status)==720
        assert m.raw_ranges_m[0]==math.inf and math.isnan(m.raw_ranges_m[1])
        assert m.raw_ranges_m[2]<m.range_min_m and m.raw_ranges_m[3]>m.range_max_m
        assert list(m.raw_status[:4])==[1,2,3,2] and not any(m.raw_valid[:4])
        assert all(m.raw_valid[4:]) and len(m.intensities)==0
        scans.append(ns(m.context))

    subscriptions = [
        node.create_subscription(GroundTruth,'/platform/v1/ground_truth',truth,sensor),
        node.create_subscription(RobotState,'/platform/v1/robot_state',state,sensor),
        node.create_subscription(UwbRangeRaw,'/platform/v1/uwb_ranges_raw',raw,sensor),
        node.create_subscription(LidarFrameRaw,'/platform/v1/scan_raw',scan,sensor),
        node.create_subscription(String,'/platform/v1/fixture_info',lambda m:info.append(json.loads(m.data)),latched)]
    client = node.create_client(QueryRay,'/mock/scene/query_ray')

    def request(anchor, mutate=None):
        m=copy.deepcopy(raws[anchor])
        req=QueryRay.Request()
        req.context=copy.deepcopy(m.context)
        req.observation_id=m.observation_id
        req.request_id=str(uuid.uuid4())
        req.origin_m=copy.deepcopy(m.tag_position_m)
        req.target_m=copy.deepcopy(m.anchor_position_m)
        req.reflector_search_radius_m=1.
        if mutate:mutate(req)
        future=client.call_async(req)
        deadline=time.monotonic()+3
        while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
        assert future.done(),'Mock service timed out'
        res=future.result()
        assert res.context==req.context and res.request_id==req.request_id and res.observation_id==req.observation_id
        assert res.error_message.startswith('MOCK_ONLY')
        return res

    try:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        if args.launch:
            log=args.output.with_suffix('.log').open('w')
            proc=subprocess.Popen(['ros2','launch','coal_platform_devtools','fixtures.launch.py','phase_duration_s:=0.5'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+25
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if proc and proc.poll() is not None:raise RuntimeError('Fixture node exited')
            if first is not None and latest-first>=3000000000 and len(raws)==9 and len(scans)>=25 and states=={'TRAVELLING','CUTTING','IDLE'} and zones=={'NONE','fixture_dust_zone'} and info and client.service_is_ready():break
        assert first is not None and latest-first>=3000000000
        assert states=={'TRAVELLING','CUTTING','IDLE'} and zones=={'NONE','fixture_dust_zone'}
        assert info[-1]['is_mock'] and len(raws)==9 and len(scans)>=25
        for anchor,expected in {'A00':('OK',0,0),'A01':('OK',1,0),'A02':('OK',0,1),'A03':('OK',1,1),
                                'A04':('UNSUPPORTED',0,0),'A05':('UNSUPPORTED',0,0),
                                'A06':('UNSUPPORTED',0,0),'A07':('UNSUPPORTED',0,0),'A08':('OK',0,0)}.items():
            res=request(anchor)
            result=(res.status,len(res.direct_intersections),len(res.reflector_candidates))
            assert result==expected,(anchor,result)
            query_results[anchor]=result
        mutations=[('SCENE_MISMATCH',lambda r:setattr(r.context.header,'scene_id','wrong_scene')),
                   ('VERSION_MISMATCH',lambda r:setattr(r.context.header,'scene_version','9.0.0')),
                   ('INVALID_REQUEST',lambda r:setattr(r.context.header,'frame_id','laser_link')),
                   ('INVALID_REQUEST',lambda r:setattr(r,'target_m',copy.deepcopy(r.origin_m))),
                   ('TIME_UNAVAILABLE',lambda r:setattr(r.context.header.stamp,'sec',r.context.header.stamp.sec+100))]
        for expected,change in mutations:assert request('A00',change).status==expected
        real=node.create_client(QueryRay,'/scene/query_ray')
        assert not real.service_is_ready(),'Fixture run must not impersonate the real query service'
        truth_rate=(len(truths)-1)*1e9/(truths[-1]-truths[0])
        scan_rate=(len(scans)-1)*1e9/(scans[-1]-scans[0])
        assert 49<=truth_rate<=51 and 9.8<=scan_rate<=10.2
        result=dict(passed=True,source='MOCK_FIXTURES_ONLY',truth_rate_hz=truth_rate,scan_rate_hz=scan_rate,
                    scan_special_inputs_verified=True,uwb_anchors=list(raws),query_cases=query_results,
                    invalid_request_cases=len(mutations),work_modes=sorted(states),zones=sorted(zones),
                    observation_ids_unique=True,real_query_not_advertised=True,
                    limitations='Tests A input fixtures only, not B/C algorithms, Gazebo geometry or recipient installation')
        args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2))
        print(json.dumps(result,ensure_ascii=False,indent=2))
    except BaseException as exc:
        args.output.write_text(json.dumps(dict(passed=False,error=str(exc)),ensure_ascii=False,indent=2))
        raise
    finally:
        if proc and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                proc.wait(timeout=5)
        if log:log.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    main()
