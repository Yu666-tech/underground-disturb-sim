"""COLLADA loader reused from this project's verified collision measurement tools.
Includes COLLADA units, scene graph, SDF-compatible xyz/rpy transforms.
"""
import math
import xml.etree.ElementTree as ET
import numpy as np
NS={'c':'http://www.collada.org/2005/11/COLLADASchema'}
def transform(points,matrix):
    return (np.column_stack((points,np.ones(len(points))))@matrix.T)[:,:3]

def rotation(axis,angle):
    axis=np.asarray(axis,dtype=float)
    length=np.linalg.norm(axis)
    if length<1e-15:raise ValueError('Zero rotation axis')
    x,y,z=axis/length;c=math.cos(angle);s=math.sin(angle);v=1-c
    matrix=np.eye(4)
    matrix[:3,:3]=[[x*x*v+c,x*y*v-z*s,x*z*v+y*s],
                   [y*x*v+z*s,y*y*v+c,y*z*v-x*s],
                   [z*x*v-y*s,z*y*v+x*s,z*z*v+c]]
    return matrix

def pose(element):
    matrix=np.eye(4)
    if element is None:return matrix
    if element.get('relative_to'):raise ValueError('Named pose frames require explicit resolution')
    values=np.fromstring(element.text or '',sep=' ')
    if len(values)!=6:raise ValueError('Expected xyz/rpy six-value pose')
    x,y,z,r,p,yaw=values
    matrix=rotation([0,0,1],yaw)@rotation([0,1,0],p)@rotation([1,0,0],r)
    matrix[:3,3]=[x,y,z]
    return matrix

def dae_triangles(path):
    root=ET.parse(path).getroot()
    meter=float(root.find('c:asset/c:unit',NS).get('meter','1'))
    assert root.findtext('c:asset/c:up_axis',namespaces=NS)=='Z_UP'
    geometries={}
    for geometry in root.findall('c:library_geometries/c:geometry',NS):
        mesh=geometry.find('c:mesh',NS);sources={};vertices={};triangles=[]
        for source in mesh.findall('c:source',NS):
            accessor=source.find('c:technique_common/c:accessor',NS)
            if accessor is None:continue
            array=source.find("c:float_array[@id='%s']"%accessor.get('source')[1:],NS)
            if array is None:continue
            stride=int(accessor.get('stride','1'));offset=int(accessor.get('offset','0'));count=int(accessor.get('count'))
            sources[source.get('id')]=np.fromstring(array.text or '',sep=' ')[offset:offset+stride*count].reshape(count,stride)
        for v in mesh.findall('c:vertices',NS):
            position=v.find("c:input[@semantic='POSITION']",NS)
            if position is not None:vertices[v.get('id')]=position.get('source')[1:]
        for primitive in list(mesh):
            kind=primitive.tag.split('}')[-1]
            if kind in ['source','vertices','extra']:continue
            if kind not in ['triangles','polylist','polygons']:raise ValueError('Unsupported primitive '+kind)
            inputs=primitive.findall('c:input',NS)
            vertex=next((e for e in inputs if e.get('semantic')=='VERTEX'),None)
            if vertex is None:continue
            width=max(int(e.get('offset','0')) for e in inputs)+1
            offset=int(vertex.get('offset','0'))
            points=sources[vertices[vertex.get('source')[1:]]][:,:3]
            for p in primitive.findall('c:p',NS):
                ids=np.fromstring(p.text or '',sep=' ',dtype=int).reshape(-1,width)[:,offset]
                if not len(ids):continue
                if kind=='triangles':triangles.append(points[ids.reshape(-1,3)])
                else:
                    counts=np.fromstring(primitive.findtext('c:vcount',namespaces=NS),sep=' ',dtype=int) if kind=='polylist' else [len(ids)]
                    start=0
                    for count in counts:
                        face=ids[start:start+count];start+=count
                        for j in range(1,count-1):triangles.append(points[[face[0],face[j],face[j+1]]][None,:,:])
        if triangles:geometries[geometry.get('id')]=np.concatenate(triangles)
    reference=root.find('c:scene/c:instance_visual_scene',NS).get('url')[1:]
    scene=root.find("c:library_visual_scenes/c:visual_scene[@id='%s']"%reference,NS)
    instances=[]
    def visit(node,parent):
        local=np.eye(4)
        for child in node:
            tag=child.tag.split('}')[-1]
            if tag in ['matrix','translate','scale','rotate']:
                vals=np.fromstring(child.text or '',sep=' ');part=np.eye(4)
                if tag=='matrix':part=vals.reshape(4,4).T
                elif tag=='translate':part[:3,3]=vals
                elif tag=='scale':part[:3,:3]=np.diag(vals)
                else:part=rotation(vals[:3],math.radians(vals[3]))
                local=local@part
            elif tag in ['lookat','skew','instance_node']:raise ValueError('Unsupported transform '+tag)
        full=parent@local
        for instance in node.findall('c:instance_geometry',NS):
            key=instance.get('url')[1:]
            if key in geometries:
                tri=geometries[key]
                tri=(transform(tri.reshape(-1,3),full)*meter).reshape(-1,3,3)
                instances.append((node.get('name'),node.get('id'),key,tri))
        for child in node.findall('c:node',NS):visit(child,full)
    for node in scene.findall('c:node',NS):visit(node,np.eye(4))
    return instances
