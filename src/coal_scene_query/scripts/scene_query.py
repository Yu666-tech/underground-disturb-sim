#!/usr/bin/env python3
"""Query actual loaded SDF collision geometry. Mock remains a separate server."""
import json,math,time
from copy import deepcopy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import Point,Vector3
from std_msgs.msg import String
from coal_platform_interfaces.msg import GroundTruth,Intersection,ReflectorCandidate
from coal_platform_interfaces.srv import QueryRay
from geometry_engine import Engine
from query_timing import rejection_reason, run_identity, tolerance_ns
def ns(stamp):return stamp.sec*1000000000+stamp.nanosec
def point(x):return Point(x=float(x[0]),y=float(x[1]),z=float(x[2]))
def vector(x):return Vector3(x=float(x[0]),y=float(x[1]),z=float(x[2]))
class SceneQuery(Node):
    def __init__(self):
        super().__init__('coal_scene_query')
        for k,v in [('world_file',''),('models_root',''),('self_model','shearer'),('scene_id','coal_face_base'),('scene_version','0.2.0')]:self.declare_parameter(k,v)
        self.expected={k:self.get_parameter(k).value for k in ['scene_id','scene_version']}
        self.engine=Engine(self.get_parameter('world_file').value,self.get_parameter('models_root').value,self.get_parameter('self_model').value)
        self.declare_parameter('future_tolerance_ms',50.0)
        self.future_tolerance_ms=float(self.get_parameter('future_tolerance_ms').value)
        self.future_tolerance_ns=tolerance_ns(self.future_tolerance_ms)
        self.declare_parameter('publish_timing_diagnostics',False)
        self.timing_pub=self.create_publisher(String,'/platform/v1/query_timing',QoSProfile(depth=2000,reliability=ReliabilityPolicy.RELIABLE)) if self.get_parameter('publish_timing_diagnostics').value else None
        self.latest=None;self.run_start=None;self.timeline=None
        self.create_subscription(GroundTruth,'/platform/v1/ground_truth',self.on_truth,QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE))
        self.info_pub=self.create_publisher(String,'/platform/v1/query_info',QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        info=dict(future_tolerance_ms=self.future_tolerance_ms,time_policy='STATIC_GEOMETRY_REQUEST_ENDPOINTS_NO_POSE_EXTRAPOLATION',query_source='ACTUAL_SDF_COLLISION_GEOMETRY',is_mock=False,world_file=str(self.engine.world),geometry_sha256=self.engine.geometry_hash,source_files=self.engine.manifest,collision_objects=len(self.engine.objects),triangle_count=sum(len(o['tri']) for o in self.engine.objects),self_models_excluded=self.engine.excluded,material_policy='UNKNOWN unless supplied; visual texture is not a propagation material',mesh_policy='surface intersections only; thickness/path length NaN',reflector_policy='nearest surface within radius, not a validated reflection path',max_radius_m=5.,max_segment_length_m=500.)
        self.info_pub.publish(String(data=json.dumps(info)))
        self.query_group=ReentrantCallbackGroup()
        self.create_service(QueryRay,'/scene/query_ray',self.query,callback_group=self.query_group,
            qos_profile=QoSProfile(depth=512,reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info('REAL collision QueryRay active: '+str(len(self.engine.objects))+' objects, world SHA256='+self.engine.files[self.engine.world]['sha256'])
    def on_truth(self,msg):
        if self.latest is None or run_identity(msg.context)!=run_identity(self.latest.context):self.run_start=ns(msg.context.header.stamp)
        self.latest=msg
        # Publish a consistent (truth, run-start) snapshot to concurrent query callbacks.
        self.timeline=(msg,self.run_start)
    def query(self,req,res):
        start=time.perf_counter();res.context=deepcopy(req.context);res.request_id=req.request_id;res.observation_id=req.observation_id;res.served_scene_version=self.expected['scene_version']
        snapshot=None;timing_reason='NOT_EVALUATED';latest_ns=None
        try:
            xyz=[req.origin_m.x,req.origin_m.y,req.origin_m.z,req.target_m.x,req.target_m.y,req.target_m.z];radius=req.reflector_search_radius_m
            if not req.request_id or not req.observation_id or req.context.header.frame_id!='map' or not all(math.isfinite(v) for v in xyz+[radius]) or not 0<=radius<=5:
                res.status='INVALID_REQUEST';res.error_message='Finite map coordinates, IDs, and radius in [0,5]m required';return res
            length=math.dist(xyz[:3],xyz[3:])
            if not 1e-6<length<=500:res.status='INVALID_REQUEST';res.error_message='Segment length must be in (1e-6,500]m';return res
            if req.context.header.scene_id!=self.expected['scene_id']:res.status='SCENE_MISMATCH';return res
            if req.context.header.scene_version!=self.expected['scene_version'] or not self.engine.unchanged():res.status='VERSION_MISMATCH';res.error_message='Scene version or source files changed; reload geometry';return res
            if req.context.schema_version!='0.2.0':res.status='UNSUPPORTED';return res
            snapshot=self.timeline
            if snapshot is None:
                timing_reason='NO_TRUTH';res.status='TIME_UNAVAILABLE';res.error_message=timing_reason;return res
            truth,run_start=snapshot;c=truth.context;latest_ns=ns(c.header.stamp)
            if c.header.scene_id!=self.expected['scene_id'] or c.header.scene_version!=self.expected['scene_version']:res.status='VERSION_MISMATCH';return res
            reason=rejection_reason(req.context,c,run_start,self.future_tolerance_ns)
            timing_reason=reason or 'ACCEPTED'
            if reason:
                res.status='TIME_UNAVAILABLE';res.error_message=reason;return res
            hits,candidates,flags=self.engine.query(xyz[:3],xyz[3:],radius)
            if len(hits)>4096 or len(candidates)>256:res.status='UNSUPPORTED';res.error_message='Result bound exceeded; no partial results returned';return res
            for h in hits:
                o=h['object'];m=Intersection();m.geometry_id=o['id'];m.geometry_type=o['kind'];m.entry_point_m=point(h['entry']);m.exit_point_m=point(h['exit']);m.distance_from_origin_m=float(h['distance']);m.entry_normal=vector(h['normal']);m.material=o['material'];m.thickness_m=float(h['thickness']);m.path_length_m=float(h['path_length']);m.reflection_level=o['reflection_level'];m.is_dynamic=False;res.direct_intersections.append(m)
            for c in candidates:
                o=c['object'];m=ReflectorCandidate();m.geometry_id=o['id'];m.geometry_type=o['kind'];m.closest_point_m=point(c['point']);m.distance_to_link_m=float(c['distance']);m.normal=vector(c['normal']);m.material=o['material'];m.reflection_level=o['reflection_level'];m.is_dynamic=False;res.reflector_candidates.append(m)
            res.status='OK';res.error_message='; '.join(flags);return res
        except Exception as exc:
            res.status='INTERNAL_ERROR';res.error_message=str(exc);res.direct_intersections=[];res.reflector_candidates=[];return res
        finally:
            res.processing_time_ms=(time.perf_counter()-start)*1000
            if self.timing_pub is not None:
                h=req.context.header;requested_ns=ns(h.stamp)
                diagnostic=dict(request_id=req.request_id,observation_id=req.observation_id,
                    experiment_id=h.experiment_id,run_id=h.run_id,scene_id=h.scene_id,
                    robot_id=req.context.robot_id,request_stamp_ns=requested_ns,
                    latest_truth_stamp_ns=latest_ns,
                    request_latest_delta_ms=(requested_ns-latest_ns)/1e6 if latest_ns is not None else None,
                    future_tolerance_ms=self.future_tolerance_ms,guard_reason=timing_reason,
                    status=res.status,processing_time_ms=res.processing_time_ms)
                self.timing_pub.publish(String(data=json.dumps(diagnostic)))
def main():
    rclpy.init();node=SceneQuery();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
