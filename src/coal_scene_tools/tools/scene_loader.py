#!/usr/bin/env python3
"""Validate and resolve a scene; optional geometry-only Gazebo launch."""
import argparse,json,os,subprocess,sys,hashlib
from urllib.parse import urlparse
from pathlib import Path
import yaml
from validate_scene import validate,local,ROOT
def main():
 p=argparse.ArgumentParser();p.add_argument('--scene-id',required=True);p.add_argument('--scenes-root',default=str(ROOT/'scenes'));p.add_argument('--allow-skeleton',action='store_true');p.add_argument('--launch-geometry',action='store_true');p.add_argument('--gazebo-master-uri',default='http://127.0.0.1:11995');args=p.parse_args()
 endpoint=urlparse(args.gazebo_master_uri)
 if endpoint.scheme!='http' or endpoint.hostname not in ['127.0.0.1','localhost'] or endpoint.port is None or not 1<=endpoint.port<=65535 or endpoint.username or endpoint.password or endpoint.path not in ['', '/'] or endpoint.query or endpoint.fragment:raise ValueError('geometry viewer需要本机http主控地址及有效端口')
 root=Path(args.scenes_root).resolve();scene=(root/args.scene_id).resolve()
 if scene.parent!=root:raise ValueError('scene-id必须为直属场景目录名')
 result=validate(scene);s=yaml.safe_load((scene/'scenario.yaml').read_text())
 if s['scene_id']!=args.scene_id:raise ValueError('目录名与scene_id不一致')
 if s['status']=='skeleton' and not args.allow_skeleton:raise ValueError('骨架不是正式场景；预检请显式--allow-skeleton')
 plan=dict(**result,world=str(local(scene,s['world'])),global_frame=s['global_frame'],map_to_world=s['map_to_world'],route=str(local(scene,s['route'])),mode='geometry_only' if args.launch_geometry else 'plan_only',scenario_sha256=hashlib.sha256((scene/'scenario.yaml').read_bytes()).hexdigest())
 plan['scenario'] = s
 plan['semantics'] = yaml.safe_load(local(scene,s['semantics']).read_text())
 plan['demo_route'] = yaml.safe_load(local(scene,s['route']).read_text())
 print(json.dumps(plan,ensure_ascii=False,indent=2),flush=True)
 if args.launch_geometry:
  env=os.environ.copy();env['GAZEBO_MODEL_PATH']=str(scene/'assets/models');env['GAZEBO_MODEL_DATABASE_URI']='';env['GAZEBO_MASTER_URI']=args.gazebo_master_uri
  # No scene-authored Python/shell execution, no ROS topics or route claimed.
  child=subprocess.Popen(['gzserver',plan['world']],env=env)
  try:return child.wait()
  except KeyboardInterrupt:
   child.terminate()
   try:child.wait(timeout=10)
   except subprocess.TimeoutExpired:child.kill();child.wait()
 return 0
if __name__=='__main__':
 try:sys.exit(main())
 except Exception as e:print(str(e),file=sys.stderr);sys.exit(1)
