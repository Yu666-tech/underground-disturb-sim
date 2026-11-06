#!/usr/bin/env python3
"""Runtime acceptance for the recipient's freshly compiled platform."""
import argparse,copy,json,math,os,signal,subprocess,time,uuid
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('--launch',action='store_true');parser.add_argument('--workspace',type=Path);parser.add_argument('--output',type=Path,default=Path('install_verification.json'));args=parser.parse_args()
if args.launch:
    os.environ.update(ROS_DOMAIN_ID=os.environ.get('COAL_VERIFY_DOMAIN_ID','103'),GAZEBO_MASTER_URI='http://127.0.0.1:12003',GAZEBO_MODEL_DATABASE_URI='',LIBGL_ALWAYS_SOFTWARE='1')
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_prefix
from coal_platform_interfaces.msg import GroundTruth,LidarFrameRaw,RobotState,LidarDecision,LidarFrameObserved,LidarBeamState,UwbRangeRaw,UwbDecision,UwbRangeObserved
from coal_platform_interfaces.srv import QueryRay
rclpy.init();node=Node('lidar_delivery_acceptance');subs=[];proc=None;log=None
old_truth={};new_truth={};old_scan={};new_scan={};truth_pairs=set();scan_pairs=set();ids=set();state_count=0;identity=False;source=[];latest=[];stamps=[]
uwb_samples={};uwb_invalid_count=0
def key(m):return (m.header.stamp.sec,m.header.stamp.nanosec)
def same_array(a,b):return len(a)==len(b) and all(x==y or (math.isnan(x) and math.isnan(y)) for x,y in zip(a,b))
def pair_truth(k):
    if k not in old_truth or k not in new_truth or k in truth_pairs:return
    old,new=old_truth[k],new_truth[k]
    assert old.pose.pose==new.pose and old.twist.twist==new.twist,'Truth payload changed'
    assert new.context.header.frame_id=='map'
    truth_pairs.add(k)
def pair_scan(k):
    if k not in old_scan or k not in new_scan or k in scan_pairs:return
    old,new=old_scan[k],new_scan[k]
    assert new.beam_count==720 and len(old.ranges)==720
    assert same_array(old.ranges,new.raw_ranges_m) and same_array(new.truth_ranges_m,new.raw_ranges_m)
    assert same_array(old.intensities,new.intensities)
    for field in ['raw_valid','raw_status']:assert len(getattr(new,field))==720
    for field,adapted in [('angle_min','angle_min_rad'),('angle_max','angle_max_rad'),('angle_increment','angle_increment_rad'),('range_min','range_min_m'),('range_max','range_max_m'),('scan_time','scan_time_s'),('time_increment','time_increment_s')]:
        assert math.isclose(getattr(old,field),getattr(new,adapted),rel_tol=1e-6,abs_tol=1e-8)
    assert new.context.header.frame_id=='laser_link'
    scan_pairs.add(k)
def receive_old_truth(m):k=key(m);old_truth[k]=m;pair_truth(k)
def receive_truth(m):
    c=m.context;assert c.header.scene_id=='coal_face_base' and c.header.scene_version=='0.2.0' and c.schema_version=='0.2.0'
    assert m.observation_id not in ids;ids.add(m.observation_id)
    k=(c.header.stamp.sec,c.header.stamp.nanosec);new_truth[k]=m;latest[:]=[m];stamps.append(k[0]+k[1]*1e-9);pair_truth(k)
def receive_old_scan(m):k=key(m);old_scan[k]=m;pair_scan(k)
def receive_scan(m):
    assert m.observation_id not in ids;ids.add(m.observation_id)
    k=(m.context.header.stamp.sec,m.context.header.stamp.nanosec);new_scan[k]=m;pair_scan(k)
def receive_state(m):
    global state_count
    assert m.context.header.scene_id=='coal_face_base' and m.context.header.scene_version=='0.2.0'
    assert m.work_mode in ['IDLE','TRAVELLING','CUTTING','UNKNOWN'];state_count+=1

def receive_uwb(m):
    global uwb_invalid_count
    c=m.context
    assert c.header.scene_id=='coal_face_base' and c.header.scene_version=='0.2.0' and c.schema_version=='0.2.0'
    assert c.header.frame_id=='map' and m.tag_id=='tag_01'
    assert m.observation_id not in ids;ids.add(m.observation_id)
    distance=math.dist([m.tag_position_m.x,m.tag_position_m.y,m.tag_position_m.z],
                       [m.anchor_position_m.x,m.anchor_position_m.y,m.anchor_position_m.z])
    assert math.isclose(distance,m.truth_range_m,rel_tol=1e-12,abs_tol=1e-10)
    if distance<=m.sensor_max_range_m:
        assert m.raw_valid and m.raw_status=='OK' and m.raw_range_m==m.truth_range_m
    else:
        assert not m.raw_valid and m.raw_status=='OUT_OF_RANGE' and math.isnan(m.raw_range_m)
        uwb_invalid_count+=1
    uwb_samples.setdefault(m.anchor_id,[]).append(c.header.stamp.sec+c.header.stamp.nanosec*1e-9)
def receive_static(m):
    global identity
    for t in m.transforms:
        if t.header.frame_id=='map' and t.child_frame_id=='world':
            assert t.transform.translation.x==t.transform.translation.y==t.transform.translation.z==0
            assert t.transform.rotation.x==t.transform.rotation.y==t.transform.rotation.z==0 and t.transform.rotation.w==1
            identity=True
sensor=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT)
for typ,topic,callback in [(Odometry,'/ground_truth',receive_old_truth),(GroundTruth,'/platform/v1/ground_truth',receive_truth),(LaserScan,'/scan_raw',receive_old_scan),(LidarFrameRaw,'/platform/v1/scan_raw',receive_scan),(RobotState,'/platform/v1/robot_state',receive_state),(UwbRangeRaw,'/platform/v1/uwb_ranges_raw',receive_uwb)]:subs.append(node.create_subscription(typ,topic,callback,sensor))
latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
subs.append(node.create_subscription(TFMessage,'/tf_static',receive_static,latched));subs.append(node.create_subscription(String,'/platform/v1/query_info',lambda m:source.append(json.loads(m.data)),latched))
client=node.create_client(QueryRay,'/scene/query_ray')
def query(a,b):
    req=QueryRay.Request();req.context=copy.deepcopy(latest[-1].context);req.observation_id=latest[-1].observation_id;req.request_id=str(uuid.uuid4());req.origin_m=Point(x=float(a[0]),y=float(a[1]),z=float(a[2]));req.target_m=Point(x=float(b[0]),y=float(b[1]),z=float(b[2]));req.reflector_search_radius_m=0.
    future=client.call_async(req);deadline=time.monotonic()+5
    while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
    assert future.done(),'Query timeout';result=future.result()
    assert result.status=='OK',(result.status,result.error_message)
    assert result.context==req.context and result.observation_id==req.observation_id and result.request_id==req.request_id
    return result
try:
    args.output.parent.mkdir(parents=True,exist_ok=True)
    if args.workspace:
        for package in ['coal_face_sim','coal_platform_interfaces','coal_platform_bridge','coal_scene_query']:
            assert Path(get_package_prefix(package)).is_relative_to(args.workspace.resolve()),'Loaded an old workspace: '+package
    if args.launch:
        log=args.output.with_suffix('.launch.log').open('w')
        proc=subprocess.Popen(['ros2','launch','coal_platform_bridge','coal_face_interfaces.launch.py','gui:=false','rviz:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    deadline=time.monotonic()+100
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        if proc and proc.poll() is not None:raise RuntimeError('Scene exited; see launch log')
        if len(stamps)>1 and stamps[-1]-stamps[0]>=15 and len(truth_pairs)>=100 and len(scan_pairs)>=30 and state_count>=100 and identity and source and client.service_is_ready():break
    assert stamps and stamps[-1]-stamps[0]>=15,'No 15-second simulation stream'
    assert len(truth_pairs)>=100 and len(scan_pairs)>=30 and state_count>=100 and identity
    assert source and source[-1]['is_mock'] is False and source[-1]['collision_objects']==209
    clear=query([.6,5,1.2105],[.6,25,1.2105]);assert not clear.direct_intersections
    solid=query([.6,.75,1.2],[-2,.75,1.2]);hit=next(h for h in solid.direct_intersections if h.geometry_id=='support_000/link/collision')
    assert abs(hit.distance_from_origin_m-1.05)<1e-8 and abs(hit.path_length_m-1.2)<1e-8
    delta=stamps[-1]-stamps[0];truth_rate=(len(stamps)-1)/delta
    scan_times=sorted(k[0]+k[1]*1e-9 for k in scan_pairs);scan_rate=(len(scan_times)-1)/(scan_times[-1]-scan_times[0])
    assert 49<=truth_rate<=51 and 9.8<=scan_rate<=10.2
    # Import output types as well: recipient can compile C against the complete contract.
    assert all(t().get_fields_and_field_types() for t in [LidarDecision,LidarFrameObserved,LidarBeamState,UwbDecision,UwbRangeObserved])
    assert set(uwb_samples)=={'A%02d'%i for i in range(9)}
    uwb_rates={anchor:(len(times)-1)/(times[-1]-times[0]) for anchor,times in uwb_samples.items()}
    assert all(9.8<=rate<=10.2 for rate in uwb_rates.values())
    report=dict(passed=True,scene_id='coal_face_base',scene_version='0.2.0',schema_version='0.2.0',truth_pairs=len(truth_pairs),scan_pairs=len(scan_pairs),truth_rate_hz=truth_rate,scan_rate_hz=scan_rate,state_messages=state_count,simulation_duration_s=delta,old_new_payloads_equal=True,sampling_timestamps_preserved=True,scan_720=True,map_world_identity=True,observation_ids_unique=True,query_real_clear_and_support_passed=True,output_types_imported=True,package_prefixes={p:get_package_prefix(p) for p in ['coal_face_sim','coal_platform_interfaces','coal_platform_bridge','coal_scene_query']},limitations='Fresh local installation test; remote recipient GUI/performance and C disturbance implementation still require recipient acceptance')
    report.update(uwb_messages=sum(map(len,uwb_samples.values())),uwb_anchor_rates_hz=uwb_rates,
                  uwb_ideal_distance_and_validity_checked=True,uwb_out_of_range_samples=uwb_invalid_count)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False,indent=2))
except BaseException as exc:
    args.output.write_text(json.dumps(dict(passed=False,error=str(exc)),ensure_ascii=False,indent=2));raise
finally:
    if proc and proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
    if log:log.close()
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
