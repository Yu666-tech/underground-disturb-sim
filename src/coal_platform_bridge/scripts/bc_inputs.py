#!/usr/bin/env python3
"""Live truth-derived state/ranges plus an explicitly separated query fixture."""
import json,math,time,hashlib
from copy import deepcopy
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import Point,Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from coal_platform_interfaces.msg import GroundTruth,RobotState,UwbRangeRaw,Intersection,ReflectorCandidate
from coal_platform_interfaces.srv import QueryRay
def ns(stamp):return stamp.sec*1000000000+stamp.nanosec
def rotate(q,v):
    # Unit quaternion rotates body-frame tag offset into map axes.
    x,y,z,w=q.x,q.y,q.z,q.w
    return [(1-2*(y*y+z*z))*v[0]+2*(x*y-z*w)*v[1]+2*(x*z+y*w)*v[2],
            2*(x*y+z*w)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z-x*w)*v[2],
            2*(x*z-y*w)*v[0]+2*(y*z+x*w)*v[1]+(1-2*(x*x+y*y))*v[2]]
def in_polygon(x,y,polygon):
    inside=False
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        cross=(b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0])
        if abs(cross)<1e-9 and min(a[0],b[0])-1e-9<=x<=max(a[0],b[0])+1e-9 and min(a[1],b[1])-1e-9<=y<=max(a[1],b[1])+1e-9:return True
        if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:inside=not inside
    return inside

class BCInputs(Node):
    def __init__(self):
        super().__init__('coal_bc_inputs')
        self.declare_parameter('config_file','')
        path=Path(self.get_parameter('config_file').value)
        self.cfg=json.loads(path.read_text())
        self.validate_config()
        self.period_ns=round(1e9/self.cfg['uwb_rate_hz'])
        self.latest=None;self.last_uwb=None;self.batch=0;self.references={}
        self.max_command_age_ns=50000000
        self.reliable=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub=self.create_publisher(RobotState,'/platform/v1/robot_state',self.reliable)
        self.uwb_pub=self.create_publisher(UwbRangeRaw,'/platform/v1/uwb_ranges_raw',QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.info_pub=self.create_publisher(String,'/platform/v1/bc_input_info',QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(GroundTruth,'/platform/v1/ground_truth',self.on_truth,self.reliable)
        self.create_subscription(Odometry,'/shearer/reference',self.on_reference,self.reliable)
        self.create_service(QueryRay,'/mock/scene/query_ray',self.query)
        self.info=dict(state_source='GAZEBO_TRUTH_AND_DEMO_WORK_SCHEDULE',uwb_source='IDEAL_DISTANCE_FROM_GAZEBO_TRUTH',query_source='MOCK_FIXTURES_NOT_SCENE_GEOMETRY',query_service='/mock/scene/query_ray',query_is_mock=True,config_file=str(path),config_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),configuration=self.cfg)
        self.info_pub.publish(String(data=json.dumps(self.info)))
        self.get_logger().warning('MOCK query at /mock/scene/query_ray only; work_mode/zone use development config. UWB raw is noiseless ideal distance.')

    def validate_config(self):
        c=self.cfg
        assert math.isfinite(c['uwb_rate_hz']) and 0<c['uwb_rate_hz']<=50
        assert math.isfinite(c['sensor_max_range_m']) and c['sensor_max_range_m']>0
        anchors=c['anchors'];assert anchors and len({a['anchor_id'] for a in anchors})==len(anchors)
        for v in [c['tag_offset_body_m']]+[a['position_m'] for a in anchors]:assert len(v)==3 and all(math.isfinite(x) for x in v)
        schedule=c['work_schedule'];assert schedule and schedule[0]['time_s']==0
        assert all(b['time_s']>a['time_s'] for a,b in zip(schedule,schedule[1:]))
        assert all(s['work_mode'] in ['IDLE','TRAVELLING','CUTTING','UNKNOWN'] for s in schedule)
        assert all(v in ['EMPTY','INTERSECTION','REFLECTOR','BOTH','ERROR'] for v in c['mock_query_cases'].values())
        assert len({z['zone_id'] for z in c['zones']})==len(c['zones'])
        for z in c['zones']:assert z['z_min_m']<z['z_max_m'] and len(z['polygon_xy_m'])>=3

    def on_reference(self,msg):
        self.references[ns(msg.header.stamp)]=msg.twist.twist
        if len(self.references)>100:self.references.pop(min(self.references))

    def on_truth(self,msg):
        t=ns(msg.context.header.stamp)
        if msg.context.header.frame_id!='map':return
        if self.latest is not None:
            if msg.context.header.run_id!=self.latest.context.header.run_id:
                self.last_uwb=None;self.batch=0
                self.references={k:v for k,v in self.references.items() if k<=t}
            elif t<=ns(self.latest.context.header.stamp):return
        self.latest=msg
        p=msg.pose.position;state=RobotState();state.context=deepcopy(msg.context)
        state.is_moving=math.sqrt(sum(v*v for v in [msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z]))>.001
        elapsed=t/1e9-self.cfg['work_schedule_epoch_sim_s'];state.work_mode='UNKNOWN'
        for s in self.cfg['work_schedule']:
            if elapsed>=s['time_s']:state.work_mode=s['work_mode']
        zones=[z for z in self.cfg['zones'] if z['z_min_m']<=p.z<=z['z_max_m'] and in_polygon(p.x,p.y,z['polygon_xy_m'])]
        state.zone_id=sorted(zones,key=lambda z:(-z['priority'],z['zone_id']))[0]['zone_id'] if zones else 'NONE'
        valid_refs=[k for k in self.references if 0<=t-k<=self.max_command_age_ns]
        if valid_refs:
            command=self.references[max(valid_refs)];state.commanded_velocity_map_mps=deepcopy(command.linear)
            q=msg.pose.orientation;world=[command.linear.x,command.linear.y,command.linear.z]
            inv=deepcopy(q);inv.x=-q.x;inv.y=-q.y;inv.z=-q.z
            state.commanded_linear_speed_mps=rotate(inv,world)[0]
            state.commanded_yaw_rate_radps=command.angular.z
        else:
            state.commanded_linear_speed_mps=math.nan;state.commanded_yaw_rate_radps=math.nan
            state.commanded_velocity_map_mps=Vector3(x=math.nan,y=math.nan,z=math.nan)
        self.state_pub.publish(state)
        if self.last_uwb is not None and t-self.last_uwb<self.period_ns:return
        self.last_uwb=t;offset=rotate(msg.pose.orientation,self.cfg['tag_offset_body_m']);tag=[p.x+offset[0],p.y+offset[1],p.z+offset[2]]
        for anchor in self.cfg['anchors']:
            out=UwbRangeRaw();out.context=deepcopy(msg.context);out.context.sequence=self.batch
            out.tag_id=self.cfg['tag_id'];out.anchor_id=anchor['anchor_id']
            out.observation_id='/'.join([out.context.header.run_id,out.context.robot_id,out.tag_id,out.anchor_id,str(self.batch)])
            out.tag_position_m=Point(x=tag[0],y=tag[1],z=tag[2]);a=anchor['position_m'];out.anchor_position_m=Point(x=float(a[0]),y=float(a[1]),z=float(a[2]))
            out.truth_range_m=math.dist(tag,a);out.sensor_max_range_m=self.cfg['sensor_max_range_m'];out.raw_valid=out.truth_range_m<=out.sensor_max_range_m
            out.raw_range_m=out.truth_range_m if out.raw_valid else math.nan;out.raw_status='OK' if out.raw_valid else 'OUT_OF_RANGE'
            self.uwb_pub.publish(out)
        self.batch+=1

    def query(self,req,res):
        begin=time.perf_counter();res.context=deepcopy(req.context);res.request_id=req.request_id;res.observation_id=req.observation_id
        res.served_scene_version=self.latest.context.header.scene_version if self.latest else ''
        res.error_message='MOCK_ONLY: synthetic fixtures, not scene geometry'
        try:
            values=[req.origin_m.x,req.origin_m.y,req.origin_m.z,req.target_m.x,req.target_m.y,req.target_m.z,req.reflector_search_radius_m]
            if not req.request_id or not req.observation_id or req.context.header.frame_id!='map' or not all(math.isfinite(v) for v in values) or req.reflector_search_radius_m<0:res.status='INVALID_REQUEST';return res
            if self.latest is None:res.status='TIME_UNAVAILABLE';return res
            current=self.latest.context
            if req.context.header.scene_id!=current.header.scene_id:res.status='SCENE_MISMATCH';return res
            if req.context.header.scene_version!=current.header.scene_version:res.status='VERSION_MISMATCH';return res
            if req.context.schema_version!=current.schema_version:res.status='UNSUPPORTED';return res
            if req.context.header.run_id!=current.header.run_id or req.context.robot_id!=current.robot_id or not 0<=ns(current.header.stamp)-ns(req.context.header.stamp)<=2000000000:res.status='TIME_UNAVAILABLE';return res
            parts=req.observation_id.split('/')
            if len(parts)!=5 or parts[0]!=current.header.run_id or parts[1]!=current.robot_id or parts[2]!=self.cfg['tag_id'] or parts[-1]!=str(req.context.sequence):res.status='INVALID_REQUEST';return res
            anchor=parts[-2];case=self.cfg['mock_query_cases'].get(anchor)
            if case is None:res.status='INVALID_REQUEST';return res
            a=values[:3];b=values[3:6];distance=math.dist(a,b)
            if distance<1e-9:res.status='INVALID_REQUEST';return res
            if case=='ERROR':res.status='UNSUPPORTED';res.error_message+='; scripted query failure';return res
            direction=[(y-x)/distance for x,y in zip(a,b)]
            if case in ['INTERSECTION','BOTH']:
                length=min(.35,distance*.1);center=[(x+y)/2 for x,y in zip(a,b)];hit=Intersection()
                hit.geometry_id='MOCK_BLOCK_'+anchor;hit.geometry_type='BOX';hit.material='MOCK_CONCRETE';hit.reflection_level='MEDIUM'
                hit.entry_point_m=Point(**dict(zip(['x','y','z'],[x-d*length/2 for x,d in zip(center,direction)])))
                hit.exit_point_m=Point(**dict(zip(['x','y','z'],[x+d*length/2 for x,d in zip(center,direction)])))
                hit.entry_normal=Vector3(x=-direction[0],y=-direction[1],z=-direction[2]);hit.distance_from_origin_m=distance/2-length/2;hit.thickness_m=length;hit.path_length_m=length;res.direct_intersections=[hit]
            if case in ['REFLECTOR','BOTH'] and req.reflector_search_radius_m>0:
                normal=[-direction[1],direction[0],0.];norm=math.sqrt(sum(v*v for v in normal))
                normal=[v/norm for v in normal] if norm>1e-9 else [1.,0.,0.];offset=min(.5,req.reflector_search_radius_m)
                candidate=ReflectorCandidate();candidate.geometry_id='MOCK_REFLECTOR_'+anchor;candidate.geometry_type='BOX';candidate.material='MOCK_METAL';candidate.reflection_level='HIGH';candidate.distance_to_link_m=offset
                point=[(x+y)/2+v*offset for x,y,v in zip(a,b,normal)];candidate.closest_point_m=Point(**dict(zip(['x','y','z'],point)));candidate.normal=Vector3(**dict(zip(['x','y','z'],[-v for v in normal])));res.reflector_candidates=[candidate]
            res.status='OK';res.error_message+='; case='+case;return res
        except Exception as exc:
            res.status='INTERNAL_ERROR';res.error_message+='; '+str(exc);res.direct_intersections=[];res.reflector_candidates=[];return res
        finally:res.processing_time_ms=(time.perf_counter()-begin)*1000

def main():
    rclpy.init();node=BCInputs()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
