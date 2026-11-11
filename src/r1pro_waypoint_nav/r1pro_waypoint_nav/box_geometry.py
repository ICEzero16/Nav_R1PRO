"""URDF collision-local bounding boxes. No assumed robot dimensions."""
from pathlib import Path
import itertools
import math
import struct
import xml.etree.ElementTree as ET
import numpy as np


def vector(text,default):
    out=np.asarray([float(v) for v in text.split()] if text else default,dtype=float)
    if out.shape!=(3,) or not np.isfinite(out).all():raise ValueError('Invalid URDF vector')
    return out


def origin(element):
    t=np.eye(4)
    if element is None:return t
    r,p,y=vector(element.get('rpy'),[0,0,0]);cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
    t[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    t[:3,3]=vector(element.get('xyz'),[0,0,0]);return t


def tf_matrix(transform):
    q=transform.rotation;a=transform.translation;x,y,z,w=q.x,q.y,q.z,q.w
    norm=x*x+y*y+z*z+w*w
    if not math.isfinite(norm) or norm<1e-12:raise ValueError('Invalid TF quaternion')
    s=2/norm;t=np.eye(4)
    t[:3,:3]=[[1-s*(y*y+z*z),s*(x*y-z*w),s*(x*z+y*w)],
               [s*(x*y+z*w),1-s*(x*x+z*z),s*(y*z-x*w)],
               [s*(x*z-y*w),s*(y*z+x*w),1-s*(x*x+y*y)]]
    t[:3,3]=[a.x,a.y,a.z]
    if not np.isfinite(t).all():raise ValueError('Nonfinite TF')
    return t


def corners(low,high):
    return np.array(list(itertools.product(*zip(low,high))),dtype=float)


def stl_bounds(path,scale):
    path=Path(path)
    with path.open('rb') as f:
        header=f.read(84)
        count=struct.unpack_from('<I',header,80)[0] if len(header)==84 else 0
        if count and path.stat().st_size==84+count*50:
            dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
            triangles=np.fromfile(f,dtype=dtype,count=count)
            points=triangles['vertices'].reshape(-1,3)
        else:
            f.seek(0);points=np.array([[float(v) for v in line.split()[1:4]] for line in f.read().decode('ascii').splitlines() if line.strip().startswith('vertex ')])
    if not len(points) or not np.isfinite(points).all():raise ValueError('Empty/nonfinite mesh: '+str(path))
    low=points.min(axis=0)*scale;high=points.max(axis=0)*scale
    return np.minimum(low,high),np.maximum(low,high)


def build_boxes(xml,resolve):
    root=ET.fromstring(xml);joints={j.find('child').get('link'):j for j in root.findall('joint')};boxes=[]
    for link in root.findall('link'):
        name=link.get('name');geometries=link.findall('collision');source='collision'
        if not geometries:geometries=link.findall('visual');source='visual_fallback'
        for index,shape in enumerate(geometries):
            g=shape.find('geometry')
            if g is None:raise ValueError('Missing geometry: '+name)
            mesh=g.find('mesh');box=g.find('box');sphere=g.find('sphere');cylinder=g.find('cylinder')
            if mesh is not None:
                path=resolve(mesh.get('filename'))
                if Path(path).suffix.lower()!='.stl':raise ValueError('Unsupported mesh (expected STL): '+str(path))
                low,high=stl_bounds(path,vector(mesh.get('scale'),[1,1,1]))
            elif box is not None:
                half=vector(box.get('size'),[0,0,0])/2;low,high=-half,half
            elif sphere is not None:
                half=np.full(3,float(sphere.get('radius')));low,high=-half,half
            elif cylinder is not None:
                half=np.array([float(cylinder.get('radius'))]*2+[float(cylinder.get('length'))/2]);low,high=-half,half
            else:raise ValueError('Unsupported collision geometry: '+name)
            if not np.isfinite(low).all() or not np.isfinite(high).all() or np.any(high<low):raise ValueError('Invalid bounds')
            t=origin(shape.find('origin'));entry=dict(name=f'{name}:{index}',frame=name,matrix=t,low=low,high=high,source=source)
            joint=joints.get(name)
            # Missing prismatic finger TF: bound the entire declared stroke, not a guessed joint angle.
            if joint is not None and joint.get('type')=='prismatic' and 'gripper_finger' in name:
                limits=joint.find('limit');axis_element=joint.find('axis')
                if limits is None:raise ValueError('Finger stroke missing')
                lower,upper=float(limits.get('lower')),float(limits.get('upper'))
                if not math.isfinite(lower+upper) or lower>upper:raise ValueError('Invalid finger limits')
                axis=vector(axis_element.get('xyz') if axis_element is not None else None,[1,0,0])
                if np.linalg.norm(axis)<1e-9:raise ValueError('Invalid joint axis')
                axis=axis/np.linalg.norm(axis)
                local_axis=t[:3,:3].T@axis
                delta0,delta1=local_axis*lower,local_axis*upper
                entry['fallback']=dict(frame=joint.find('parent').get('link'),matrix=origin(joint.find('origin'))@t,
                    low=low+np.minimum(delta0,delta1),high=high+np.maximum(delta0,delta1))
            boxes.append(entry)
    if not boxes:raise ValueError('Robot has no bounding geometry')
    return boxes


def box_mask(points,matrix,low,high):
    """Broad-phase AABB followed by exact membership in the oriented box."""
    world=corners(low,high)@matrix[:3,:3].T+matrix[:3,3]
    candidates=np.flatnonzero(np.all((points>=world.min(axis=0))&(points<=world.max(axis=0)),axis=1))
    selected=np.zeros(len(points),dtype=bool)
    if len(candidates):
        local=(points[candidates]-matrix[:3,3])@matrix[:3,:3]
        selected[candidates]=np.all((local>=low-1e-7)&(local<=high+1e-7),axis=1)
    return selected,world


def unpack_cloud(msg):
    fields={f.name:f for f in msg.fields}
    if not all(k in fields and fields[k].datatype==7 for k in ('x','y','z')):raise ValueError('XYZ float32 fields required')
    if msg.width<=0 or msg.height<=0 or msg.point_step<=0:raise ValueError('Empty cloud')
    if msg.row_step<msg.width*msg.point_step or len(msg.data)<msg.height*msg.row_step:raise ValueError('Truncated cloud')
    raw=np.ndarray((msg.height,msg.width,msg.point_step),dtype=np.uint8,buffer=msg.data,strides=(msg.row_step,msg.point_step,1)).reshape(-1,msg.point_step)
    endian='>' if msg.is_bigendian else '<'
    xyz=np.column_stack([np.ndarray((msg.height,msg.width),dtype=endian+'f4',buffer=msg.data,offset=fields[k].offset,strides=(msg.row_step,msg.point_step)).reshape(-1) for k in ('x','y','z')])
    return raw,xyz


def invalid_points(xyz):
    finite=np.isfinite(xyz).all(axis=1)
    return (~finite) | ((np.where(finite[:,None],xyz,0.)**2).sum(axis=1)<.0025)
