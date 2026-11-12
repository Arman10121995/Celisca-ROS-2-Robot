"""Mass and spatial inertia of the URDF component welded to its root."""
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def fixed_root_inertial(urdf):
    root=ET.fromstring(urdf)
    links={e.get('name'):e for e in root.findall('link')}
    children={j.find('child').get('link') for j in root.findall('joint')}
    roots=set(links)-children
    if len(roots)!=1:raise ValueError('Expected one URDF root for welded inertia')
    total=0.;moment=np.zeros(3);tensor=np.zeros((3,3));visited=set()
    def pose(origin):
        if origin is None:return np.zeros(3),Rotation.identity()
        xyz=np.array(list(map(float,origin.get('xyz','0 0 0').split())))
        angles=np.array(list(map(float,origin.get('rpy','0 0 0').split())))
        if xyz.shape!=(3,) or angles.shape!=(3,) or not np.all(np.isfinite([xyz,angles])):
            raise ValueError('Invalid URDF inertial transform')
        return xyz,Rotation.from_euler('xyz',angles)
    def shift(m,p):return m*(np.dot(p,p)*np.eye(3)-np.outer(p,p))
    def visit(name,xyz,rotation):
        nonlocal total,moment,tensor
        if name in visited:raise ValueError('Cycle in fixed URDF component')
        visited.add(name)
        inertial=links[name].find('inertial')
        if inertial is not None:
            mass=float(inertial.find('mass').get('value'))
            a=inertial.find('inertia').attrib
            local_tensor=np.array([[float(a['ixx']),float(a.get('ixy',0)),float(a.get('ixz',0))],
                [float(a.get('ixy',0)),float(a['iyy']),float(a.get('iyz',0))],
                [float(a.get('ixz',0)),float(a.get('iyz',0)),float(a['izz'])]])
            if not np.isfinite(mass) or mass<=0 or not np.all(np.isfinite(local_tensor)) or np.min(np.linalg.eigvalsh(local_tensor))<=0:
                raise ValueError('Invalid source inertia in fixed root: '+name)
            position,frame=pose(inertial.find('origin'))
            center=xyz+rotation.apply(position);frame=rotation*frame
            total+=mass;moment+=mass*center
            tensor+=frame.as_matrix()@local_tensor@frame.as_matrix().T+shift(mass,center)
        for joint in root.findall('joint'):
            if joint.get('type')!='fixed' or joint.find('parent').get('link')!=name:continue
            position,frame=pose(joint.find('origin'))
            visit(joint.find('child').get('link'),xyz+rotation.apply(position),rotation*frame)
    visit(next(iter(roots)),np.zeros(3),Rotation.identity())
    if total<=0:raise ValueError('The fixed root component has no authored mass')
    com=moment/total;inertia=tensor-shift(total,com)
    return dict(mass=format(total,'.16g'),pos=' '.join(format(v,'.16g') for v in com),
                fullinertia=' '.join(format(inertia[i,j],'.16g') for i,j in [(0,0),(1,1),(2,2),(0,1),(0,2),(1,2)]))
