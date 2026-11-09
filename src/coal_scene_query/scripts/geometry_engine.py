"""Real SDF collision queries, no cached handoff geometry or propagation model."""
import hashlib,math,ctypes,os,weakref
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from collision_loader import pose,transform,dae_triangles
EPS=1e-6

def ray_tri(a,b,tri):
    direction=b-a;e1=tri[:,1]-tri[:,0];e2=tri[:,2]-tri[:,0]
    h=np.cross(direction,e2);det=np.einsum('ij,ij->i',e1,h);s=a-tri[:,0];q=np.cross(s,e1)
    with np.errstate(divide='ignore',invalid='ignore'):
        u=np.einsum('ij,ij->i',s,h)/det;v=q@direction/det;t=np.einsum('ij,ij->i',e2,q)/det
    u=np.nan_to_num(u,nan=-2.,posinf=-2.,neginf=-2.);v=np.nan_to_num(v,nan=-2.,posinf=-2.,neginf=-2.);t=np.nan_to_num(t,nan=-2.,posinf=-2.,neginf=-2.)
    ok=(abs(det)>1e-12)&(u>=-1e-10)&(v>=-1e-10)&(u+v<=1+1e-10)&(t>=0)&(t<=1)
    return t,ok

def nearest_segment_tri(a,b,tri):
    """Exact minimum segment/triangle distance, with point on triangle."""
    n=len(tri);best=np.full(n,np.inf);point=np.zeros((n,3));u=b-a;aa=float(u@u)
    def offer(q,p,valid=None):
        nonlocal best,point
        q=np.broadcast_to(q,(n,3));p=np.broadcast_to(p,(n,3));dist=np.sum((q-p)**2,axis=1)
        mask=dist<best
        if valid is not None:mask&=valid
        best[mask]=dist[mask];point[mask]=q[mask]
    v0=tri[:,0];v1=tri[:,1]-v0;v2=tri[:,2]-v0;normal=np.cross(v1,v2);nn=np.sum(normal*normal,axis=1)
    for endpoint in [a,b]:
        with np.errstate(divide='ignore',invalid='ignore'):projection=endpoint-(np.sum((endpoint-v0)*normal,axis=1)/nn)[:,None]*normal
        w=projection-v0;d00=np.sum(v1*v1,axis=1);d01=np.sum(v1*v2,axis=1);d11=np.sum(v2*v2,axis=1);d20=np.sum(w*v1,axis=1);d21=np.sum(w*v2,axis=1);den=d00*d11-d01*d01
        with np.errstate(divide='ignore',invalid='ignore'):s=(d11*d20-d01*d21)/den;t=(d00*d21-d01*d20)/den
        offer(projection,endpoint,(nn>1e-20)&(s>=0)&(t>=0)&(s+t<=1))
    for i,j in [(0,1),(1,2),(2,0)]:
        c=tri[:,i];d=tri[:,j];v=d-c;cc=np.sum(v*v,axis=1);bb=v@u;w=a-c;dd=w@u;ee=np.sum(v*w,axis=1);den=aa*cc-bb*bb
        with np.errstate(divide='ignore',invalid='ignore'):s=(bb*ee-cc*dd)/den;t=(aa*ee-bb*dd)/den
        valid=(den>1e-20)&(s>=0)&(s<=1)&(t>=0)&(t<=1);s=np.nan_to_num(s);t=np.nan_to_num(t)
        offer(c+t[:,None]*v,a+s[:,None]*u,valid)
        for endpoint in [a,b]:
            with np.errstate(divide='ignore',invalid='ignore'):t=np.clip(np.sum((endpoint-c)*v,axis=1)/cc,0,1)
            offer(c+t[:,None]*v,endpoint,cc>1e-20)
        for endpoint in [c,d]:
            s=np.clip((endpoint-a)@u/aa,0,1);offer(endpoint,a+s[:,None]*u)
    t,ok=ray_tri(a,b,tri);hit=a+t[:,None]*u;offer(hit,hit,ok)
    return np.sqrt(best),point,normal/np.maximum(np.sqrt(nn),1e-30)[:,None]

def box_tri(size):
    h=np.array(size)/2;result=[]
    for axis in range(3):
        others=[i for i in range(3) if i!=axis]
        for sign in [-1,1]:
            corners=[]
            for x,y in [(-1,-1),(1,-1),(1,1),(-1,1)]:
                p=np.zeros(3);p[axis]=sign*h[axis];p[others[0]]=x*h[others[0]];p[others[1]]=y*h[others[1]];corners.append(p)
            for ids in [(0,1,2),(0,2,3)]:
                t=np.array([corners[i] for i in ids]);n=np.cross(t[1]-t[0],t[2]-t[0])
                if n[axis]*sign<0:t=t[[0,2,1]]
                result.append(t)
    return np.array(result)

class Engine:
    def __init__(self,world,models,self_model='shearer',use_bvh=True):
        library=os.environ.get('COAL_QUERY_MATH_LIBRARY',str(Path(__file__).parent/'libcoal_query_math.so'))
        if not Path(library).is_file():
            from ament_index_python.packages import get_package_prefix
            library=str(Path(get_package_prefix('coal_scene_query'))/'lib/coal_scene_query/libcoal_query_math.so')
        self.library=ctypes.CDLL(library);self.native=self.library.nearest_surface;ptr=ctypes.POINTER(ctypes.c_double)
        self.native.argtypes=[ptr,ctypes.c_int,ptr,ptr,ptr];self.native.restype=ctypes.c_double
        self.use_bvh=use_bvh
        self.library.build_surface_bvh.argtypes=[ptr,ctypes.c_int];self.library.build_surface_bvh.restype=ctypes.c_void_p
        self.library.destroy_surface_bvh.argtypes=[ctypes.c_void_p]
        self.library.query_surface_bvh.argtypes=[ctypes.c_void_p,ptr,ptr,ctypes.c_double,ptr];self.library.query_surface_bvh.restype=ctypes.c_double
        self.library.ray_surface_bvh_candidates.argtypes=[ctypes.c_void_p,ptr,ptr,ctypes.POINTER(ctypes.c_int),ctypes.c_int];self.library.ray_surface_bvh_candidates.restype=ctypes.c_int
        self.world=Path(world).resolve();self.models=Path(models).resolve();self.files={};self.objects=[];self.dynamic=[];self.cache={};self.excluded=[];self.asset_roots=[]
        tree=ET.parse(self.record(self.world));root=tree.find('world')
        if root is None:raise ValueError('SDF world missing')
        for inc in root.findall('include'):
            uri=inc.findtext('uri','')
            if not uri.startswith('model://'):raise ValueError('Only local model:// includes supported')
            folder=(self.models/uri[8:]).resolve();assert folder.is_relative_to(self.models)
            model_path=self.record(folder/'model.sdf');self.asset_roots.append(model_path.parent)
            model=ET.parse(model_path).find('model')
            if model is None:raise ValueError('Included model missing')
            name=inc.findtext('name') or model.get('name');matrix=pose(inc.find('pose'))@pose(model.find('pose'))
            self.load_model(model,name,matrix,model_path.parent,self_model)
        for model in root.findall('model'):self.load_model(model,model.get('name'),pose(model.find('pose')),self.world.parent,self_model)
        if self_model not in self.excluded:raise ValueError('Expected self robot model not found; refuse wrong self-exclusion')
        if self.dynamic:raise ValueError('Unsupported other dynamic geometry: '+str(self.dynamic))
        for o in self.objects:
            o['low']=o['tri'].min(axis=(0,1));o['high']=o['tri'].max(axis=(0,1));o['tri_low']=o['tri'].min(axis=1);o['tri_high']=o['tri'].max(axis=1)
            if use_bvh and o['kind']=='MESH':
                tri=np.ascontiguousarray(o['tri'],dtype=np.float64)
                o['bvh']=self.library.build_surface_bvh(tri.ctypes.data_as(ptr),len(tri))
                if not o['bvh']:raise MemoryError('Cannot build static mesh query index')
                weakref.finalize(self,self.library.destroy_surface_bvh,o['bvh'])
        self.object_low=np.array([o['low'] for o in self.objects]).reshape(-1,3);self.object_high=np.array([o['high'] for o in self.objects]).reshape(-1,3)
        self.box_indices=np.array([i for i,o in enumerate(self.objects) if o['kind']=='BOX'],dtype=int)
        self.box_inverse=np.array([self.objects[i]['inverse'] for i in self.box_indices])
        self.box_half=np.array([self.objects[i]['size']/2 for i in self.box_indices])
        self.manifest={str(p):dict(sha256=d['sha256'],size=d['size']) for p,d in self.files.items()}
        self.geometry_hash=hashlib.sha256(''.join(sorted(d['sha256'] for d in self.files.values())).encode()).hexdigest()

    def record(self,path):
        p=Path(path).resolve()
        if p not in self.files:
            st=p.stat();self.files[p]=dict(sha256=hashlib.sha256(p.read_bytes()).hexdigest(),mtime_ns=st.st_mtime_ns,size=st.st_size)
        return p
    def nearest(self,a,b,tri):
        tri=np.ascontiguousarray(tri,dtype=np.float64);a=np.ascontiguousarray(a,dtype=np.float64);b=np.ascontiguousarray(b,dtype=np.float64);out=np.zeros(6,dtype=np.float64);ptr=ctypes.POINTER(ctypes.c_double)
        distance=self.native(tri.ctypes.data_as(ptr),len(tri),a.ctypes.data_as(ptr),b.ctypes.data_as(ptr),out.ctypes.data_as(ptr));return distance,out[:3],out[3:]
    def unchanged(self):
        try:return all(p.stat().st_mtime_ns==d['mtime_ns'] and p.stat().st_size==d['size'] for p,d in self.files.items())
        except OSError:return False
    def load_model(self,model,name,matrix,folder,self_model):
        if model.findall('model') or model.findall('include'):raise ValueError('Nested models require explicit support')
        if name==self_model:self.excluded.append(name);return
        if model.findtext('static','false').lower()!='true':
            if model.findall('.//collision'):self.dynamic.append(name)
            return
        for link in model.findall('link'):
            lm=matrix@pose(link.find('pose'))
            for collision in link.findall('collision'):
                matrix_full=lm@pose(collision.find('pose'));geom=collision.find('geometry');box=geom.find('box');mesh=geom.find('mesh')
                o=dict(id=name+'/'+link.get('name')+'/'+collision.get('name'),material='UNKNOWN',reflection_level='UNKNOWN',matrix=matrix_full)
                if box is not None:
                    size=np.fromstring(box.findtext('size'),sep=' ');assert len(size)==3 and np.all(size>0)
                    o.update(kind='BOX',size=size,inverse=np.linalg.inv(matrix_full));tri=box_tri(size)
                elif mesh is not None:
                    uri=mesh.findtext('uri','');path=(self.models/uri[8:]) if uri.startswith('model://') else folder/uri
                    path=path.resolve()
                    if not (path.is_relative_to(self.models) or any(path.is_relative_to(root) for root in self.asset_roots)):raise ValueError('Mesh outside explicitly loaded local model roots')
                    self.record(path)
                    if path.suffix.lower()!='.dae':raise ValueError('Only current COLLADA meshes supported')
                    if path not in self.cache:self.cache[path]=dae_triangles(path)
                    instances=self.cache[path];sub=mesh.find('submesh')
                    if sub is not None:
                        if sub.findtext('center','false')=='true':raise ValueError('Centered submesh unsupported')
                        instances=[i for i in instances if sub.findtext('name') in i[:3]]
                    if not instances:raise ValueError('No collision triangles')
                    tri=np.concatenate([i[3] for i in instances]).copy();scale=np.fromstring(mesh.findtext('scale','1 1 1'),sep=' ');assert len(scale)==3 and np.all(scale>0);tri*=scale;o['kind']='MESH'
                else:raise ValueError('Unsupported collision geometry')
                o['tri']=transform(tri.reshape(-1,3),matrix_full).reshape(-1,3,3)
                self.objects.append(o)

    def query(self,a,b,radius):
        a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float);length=float(np.linalg.norm(b-a));hits=[];candidates=[];flags=set()
        low=np.minimum(a,b)-radius;high=np.maximum(a,b)+radius
        selected=np.all(self.object_high>=low,axis=1)&np.all(self.object_low<=high,axis=1)
        # Evaluate the unchanged slab equations for selected boxes in one NumPy batch.
        box_rows=np.flatnonzero(selected[self.box_indices])
        if len(box_rows):
            inv=self.box_inverse[box_rows];half=self.box_half[box_rows]
            start=np.einsum('nij,j->ni',inv[:,:3,:3],a)+inv[:,:3,3]
            end=np.einsum('nij,j->ni',inv[:,:3,:3],b)+inv[:,:3,3];direction=end-start
            parallel=np.abs(direction)<1e-12;divisor=np.where(parallel,1.,direction)
            t1=(-half-start)/divisor;t2=(half-start)/divisor
            near=np.where(parallel,-np.inf,np.minimum(t1,t2));far=np.where(parallel,np.inf,np.maximum(t1,t2))
            enter=np.maximum(0.,near.max(axis=1));exit=np.minimum(1.,far.min(axis=1))
            possible=~np.any(parallel&((start < -half-1e-10)|(start > half+1e-10)),axis=1)
            contact=np.any(parallel&(np.abs(np.abs(start)-half)<=1e-10),axis=1)
            valid=possible&(exit>enter)&((exit-enter)*length>EPS)&(exit*length>EPS)&(enter*length<length-EPS)
            for row in np.flatnonzero(valid):
                o=self.objects[self.box_indices[box_rows[row]]];en=float(enter[row]);ex=float(exit[row]);n=np.zeros(3)
                if contact[row]:flags.add('BOX_FACE_COPLANAR_CONTACT_CLOSED_SOLID_CONVENTION')
                if near[row].max()>0:
                    axis=int(near[row].argmax());n[axis]=-1. if direction[row,axis]>0 else 1.;thickness=float(o['size'][axis])
                else:thickness=math.nan;flags.add('ORIGIN_INSIDE_BOX_NORMAL_UNDEFINED')
                hits.append(dict(object=o,distance=en*length,entry=a+en*(b-a),exit=a+ex*(b-a),normal=o['matrix'][:3,:3]@n,thickness=thickness,path_length=(ex-en)*length))
        for index in np.flatnonzero(selected):
            o=self.objects[index]
            tri=o['tri']
            if self.use_bvh and o['kind']=='MESH':
                indices=np.empty(len(tri),dtype=np.int32);ptr=ctypes.POINTER(ctypes.c_double)
                count=self.library.ray_surface_bvh_candidates(o['bvh'],a.ctypes.data_as(ptr),b.ctypes.data_as(ptr),indices.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),len(indices))
                if count<0:raise RuntimeError('Mesh candidate buffer exceeded; no partial result')
                indices=indices[:count]
                mask=np.all(o['tri_high'][indices]>=low,axis=1)&np.all(o['tri_low'][indices]<=high,axis=1)
                tri=tri[indices[mask]]
            else:
                mask=np.all(o['tri_high']>=low,axis=1)&np.all(o['tri_low']<=high,axis=1);tri=tri[mask]
            if o['kind']=='MESH':
                ts,valid=ray_tri(a,b,tri);valid&=(ts*length>EPS)&((1-ts)*length>EPS)
                order=np.where(valid)[0][np.argsort(ts[valid])];last=-math.inf
                for idx in order:
                    distance=float(ts[idx]*length)
                    if distance-last<=EPS:continue
                    last=distance;point=a+ts[idx]*(b-a);normal=np.cross(tri[idx,1]-tri[idx,0],tri[idx,2]-tri[idx,0]);normal/=np.linalg.norm(normal)
                    hits.append(dict(object=o,distance=distance,entry=point,exit=point,normal=normal,thickness=math.nan,path_length=math.nan));flags.add('MESH_SURFACE_HITS_VOLUME_THICKNESS_UNKNOWN')
            if radius>0 and (len(tri) or self.use_bvh and o['kind']=='MESH'):
                if self.use_bvh and o['kind']=='MESH':
                    out=np.zeros(6,dtype=np.float64);ptr=ctypes.POINTER(ctypes.c_double)
                    distance=self.library.query_surface_bvh(o['bvh'],a.ctypes.data_as(ptr),b.ctypes.data_as(ptr),radius,out.ctypes.data_as(ptr))
                    point,normal=out[:3],out[3:]
                else:distance,point,normal=self.nearest(a,b,tri)
                if distance<=radius+1e-10:candidates.append(dict(object=o,distance=distance,point=point,normal=normal))
        hits.sort(key=lambda h:(h['distance'],h['object']['id']));candidates.sort(key=lambda c:(c['distance'],c['object']['id']))
        return hits,candidates,sorted(flags)
