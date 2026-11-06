#!/usr/bin/env python3
"""MOCK_ONLY static return-chain transport test. No B/C algorithms."""
import json, os, time
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='109'
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.set_message import set_message_fields
from coal_platform_interfaces import msg
rclpy.init(); n=Node('mock_only_return_chain_check')
r=Path(__file__).resolve().parents[1]; (r/'evidence').mkdir(exist_ok=True); data=json.loads((r/'src/coal_platform_devtools/config/return_chain.json').read_text())
received={}; pubs=[];subs=[]
for name,spec in data.items():
 cls=getattr(msg,name);m=cls();set_message_fields(m,spec['message'])
 pubs.append((n.create_publisher(cls,spec['topic'],10),m))
 subs.append(n.create_subscription(cls,spec['topic'],lambda m,k=name:received.update({k:m}),10))
try:
 deadline=time.monotonic()+8
 while time.monotonic()<deadline and len(received)<len(data):
  for p,m in pubs:p.publish(m)
  rclpy.spin_once(n,timeout_sec=.1)
 assert len(received)==len(data)
 for decision,observed in [('UwbDecision','UwbRangeObserved'),('LidarDecision','LidarFrameObserved'),('LidarDecision','LidarBeamState')]:
  a,b=received[decision],received[observed]
  assert a.observation_id==b.observation_id and a.decision_id==b.decision_id and a.context==b.context
 assert received['UwbRangeObserved'].observed_range_m-received['UwbRangeObserved'].truth_range_m==received['UwbRangeObserved'].error_m
 scan=received['LidarFrameObserved']; beams=received['LidarBeamState']
 assert len(scan.truth_ranges_m)==len(scan.observed_ranges_m)==len(beams.beam_index)==len(beams.beam_states)==3
 assert list(beams.beam_index)==[0,1,2]
 assert scan.context.header.frame_id == 'laser_link'
 result={'passed':True,'source':'MOCK_ONLY','transport_messages':len(received),'association_pairs':3,'limitation':'Static A transport fixtures only; B/C algorithm acceptance pending'}
 (r/'evidence/return_chain_verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
finally:
 n.destroy_node();rclpy.shutdown()
