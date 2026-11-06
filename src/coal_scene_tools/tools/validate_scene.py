#!/usr/bin/env python3
"""A0 structural and primitive-geometry validator, no Gazebo side effects."""
import argparse,json,math,re,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml,jsonschema
ROOT=Path(__file__).resolve().parents[1]
def local(root,relative):
 if not isinstance(relative,str) or not relative or '\\' in relative or ':' in relative:raise ValueError('资源必须为包内相对路径: '+str(relative))
 p=(root/relative).resolve()
 if Path(relative).is_absolute() or not p.is_relative_to(root.resolve()):raise ValueError('资源越出包目录: '+relative)
 if not p.is_file():raise ValueError('资源缺失: '+relative)
 return p
def finite(value):
 if isinstance(value,float) and not math.isfinite(value):raise ValueError('YAML配置不允许NaN/Inf')
 if isinstance(value,dict):
  for x in value.values():finite(x)
 if isinstance(value,list):
  for x in value:finite(x)
class UniqueLoader(yaml.SafeLoader):pass
def mapping(loader,node,deep=False):
 result={}
 for key,value in node.value:
  k=loader.construct_object(key,deep=deep)
  if k in result:raise ValueError('重复YAML键: '+str(k))
  result[k]=loader.construct_object(value,deep=deep)
 return result
UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,mapping)
def read(root,name,schema):
 data=yaml.load(local(root,name).read_text(encoding='utf-8'),Loader=UniqueLoader);finite(data)
 spec=json.loads((ROOT/'schemas'/schema).read_text());jsonschema.Draft7Validator(spec).validate(data);return data
def unique(items,label):
 if len(items)!=len(set(items)):raise ValueError(label+' 不唯一')
def inside_segment(a,b,lo,hi):
 # Strict interior intersection; floor contact alone is allowed.
 lower,upper=0.,1.
 for k in range(3):
  low,high=lo[k]+1e-8,hi[k]-1e-8;delta=b[k]-a[k]
  if abs(delta)<1e-12:
   if not low<a[k]<high:return False
  else:
   t1,t2=(low-a[k])/delta,(high-a[k])/delta
   lower=max(lower,min(t1,t2));upper=min(upper,max(t1,t2))
   if lower>=upper:return False
 return lower<upper
def intersect2(a,b,c,d):
 def cross(p,q,r):return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
 def on(p,q,r):return min(p[0],q[0])-1e-10<=r[0]<=max(p[0],q[0])+1e-10 and min(p[1],q[1])-1e-10<=r[1]<=max(p[1],q[1])+1e-10
 x,y,z,w=cross(a,b,c),cross(a,b,d),cross(c,d,a),cross(c,d,b)
 return x*y<0 and z*w<0 or any(abs(v)<1e-10 and on(p,q,r) for v,p,q,r in [(x,a,b,c),(y,a,b,d),(z,c,d,a),(w,c,d,b)])
def validate(root):
 root=Path(root).resolve();s=read(root,'scenario.yaml','scenario.schema.json')
 sem=read(root,s['semantics'],'semantics.schema.json');route=read(root,s['route'],'route.schema.json')
 warnings=[]
 if any(abs(v)>1e-10 for v in s['map_to_world']['xyz_m']+s['map_to_world']['rpy_rad']):raise ValueError('当前骨架validator不支持非恒等map/world几何核对，需A扩展后接收')
 for data in [sem,route]:
  for key in ['scene_id','scene_version','schema_version']:
   if data[key]!=s[key]:raise ValueError(key+' 跨文件不一致')
 for key in ['world','preview']:local(root,s[key])
 for asset in s['assets']:local(root,asset)
 unique([a['anchor_id'] for a in s['anchors']],'anchor_id');unique([g['geometry_id'] for g in sem['geometry']],'geometry_id');unique([z['zone_id'] for z in sem['zones']],'zone_id')
 if any(z['zone_id']=='NONE' for z in sem['zones']):raise ValueError('NONE是无区域保留ID')
 if route['robot_id']!=s['robot']['robot_id']:raise ValueError('route robot_id不匹配')
 frames=[s['global_frame'],s['world_frame'],s['robot']['body_frame'],s['sensors']['lidar']['frame'],s['sensors']['uwb']['frame']];unique(frames,'frame')
 if any(not re.fullmatch('[A-Za-z][A-Za-z0-9_]*',frame) for frame in frames):raise ValueError('单机器人基线frame必须为无斜线的英文标识')
 for sensor in s['sensors'].values():
  if sensor['parent_frame']!=s['robot']['body_frame']:raise ValueError('sensor parent必须为base_link')
 l=s['sensors']['lidar']
 if l['angle_min_rad']>=l['angle_max_rad'] or l['range_min_m']>=l['range_max_m']:raise ValueError('LiDAR角度/量程不合法')
 if l['beam_count']!=720:warnings.append('非当前720束基线，C须确认数组兼容')
 way=route['waypoints'];times=[w['time_s'] for w in way]
 if times[0]!=0 or any(b<=a for a,b in zip(times,times[1:])):raise ValueError('路线必须0起始且时间严格递增')
 if way[0]['xyz_m']!=s['robot']['initial_pose']['xyz_m'] or abs(way[0]['yaw_rad']-s['robot']['initial_pose']['rpy_rad'][2])>1e-9:raise ValueError('路线起点与初始位姿不一致')
 if route['loop'] and (way[0]['xyz_m']!=way[-1]['xyz_m'] or way[0]['yaw_rad']!=way[-1]['yaw_rad']):raise ValueError('循环路线末点必须回到起点位姿')
 for zone in sem['zones']:
  poly=zone['polygon_xy_m'];n=len(poly)
  if zone['z_min_m']>=zone['z_max_m']:raise ValueError('zone高度无效')
  if len(set(map(tuple,poly)))!=n:raise ValueError('多边形顶点重复；不要重复末点')
  area=sum(poly[i][0]*poly[(i+1)%n][1]-poly[(i+1)%n][0]*poly[i][1] for i in range(n))
  if abs(area)<1e-9:raise ValueError('多边形面积为零')
  for i in range(n):
   for j in range(i+1,n):
    if j==i+1 or i==0 and j==n-1:continue
    if intersect2(poly[i],poly[(i+1)%n],poly[j],poly[(j+1)%n]):raise ValueError('多边形自交')
 tree=ET.parse(local(root,s['world']));world=tree.find('world')
 if world is None:raise ValueError('SDF缺少world')
 unique([m.get('name') for m in world.findall('model')],'SDF model name')
 for uri in tree.findall('.//uri'):
  text=uri.text or ''
  if text.startswith('model://'):
   rest=text[8:];key=rest.split('/')[0];rel='assets/models/'+rest
   if '/' not in rest:rel+='/model.sdf'
   local(root,rel)
  else:local(root,text)
 for g in sem['geometry']:
  if g['geometry_type']=='MESH':local(root,g['mesh_uri']);warnings.append('MESH几何需Gazebo/三角形查询复核: '+g['geometry_id']);continue
  model=world.find("model[@name='%s']"%g['model_name'])
  if model is None:raise ValueError('语义模型不存在: '+g['model_name'])
  box=model.find('.//collision/geometry/box/size');pose=model.find('pose')
  vals=list(map(float,pose.text.split())) if pose is not None else [0]*6
  if box is None or list(map(float,box.text.split()))!=g['size_m'] or vals[:3]!=g['center_m'] or vals[3:]!=g.get('rpy_rad',[0,0,0]):raise ValueError('BOX语义与SDF collision不一致: '+g['geometry_id'])
  if g.get('is_dynamic'):warnings.append('动态体需要A提供时间快照: '+g['geometry_id']);continue
  if any(abs(v)>1e-9 for v in g.get('rpy_rad',[0,0,0])):warnings.append('旋转BOX路线扫掠需仿真复核: '+g['geometry_id']);continue
  half=[v/2 for v in s['robot']['footprint_size_m']];lo=[c-size/2-h for c,size,h in zip(g['center_m'],g['size_m'],half)];hi=[c+size/2+h for c,size,h in zip(g['center_m'],g['size_m'],half)]
  if any(abs(w['yaw_rad'])>1e-9 for w in way):warnings.append('非零yaw路线不进行本骨架AABB碰撞验收');continue
  if any(inside_segment(a['xyz_m'],b['xyz_m'],lo,hi) for a,b in zip(way,way[1:])):raise ValueError('路线扫掠与BOX重叠: '+g['geometry_id'])
 warnings+=['仅结构/部分原语几何检查；不证明TF已发布、传感器已挂载、路线已执行或query已实现']
 return dict(passed=True,scene_id=s['scene_id'],scene_version=s['scene_version'],status=s['status'],warnings=warnings)
def main():
 p=argparse.ArgumentParser();p.add_argument('scene');args=p.parse_args()
 try:result=validate(args.scene)
 except Exception as e:print(json.dumps(dict(passed=False,error=str(e)),ensure_ascii=False,indent=2));return 1
 print(json.dumps(result,ensure_ascii=False,indent=2));return 0
if __name__=='__main__':sys.exit(main())
