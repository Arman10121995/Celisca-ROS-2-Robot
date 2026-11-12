"""Portable physical roller derivative of the source myAGV Pro wheel assemblies.

FUJI placement/handedness and geometry: DaiGuard/fuji_mecanum at
646431a5e107448e0e2bdeeba4575db9aa3665ff (MIT, copyright 2022 DaiGuard).
Vendor CAD visuals remain. Original wheel mass, COM and rigid-assembly inertia
are conserved at the zero roller pose; individual rollers are free in physics.
This is a declared lab wheel derivative, not vendor hardware calibration.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .myagv_pro import ROLES


def translate_inertia(mass, xyz):
    return mass*(np.dot(xyz,xyz)*np.eye(3)-np.outer(xyz,xyz))


def matrix(element):
    a=element.attrib
    return np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],
                     [float(a['ixy']),float(a['iyy']),float(a['iyz'])],
                     [float(a['ixz']),float(a['iyz']),float(a['izz'])]])


def write_matrix(element, tensor):
    for key,i,j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]:
        element.set(key,format(float(tensor[i,j]),'.16g'))


def vector(values):
    return ' '.join(format(float(v),'.16g') for v in values)


def physical_rollers(root, assets):
    assets=Path(assets).resolve()
    for file in ('wheel.stl','roller_collision.stl','LICENSE','README.md'):
        if not (assets/file).is_file():raise ValueError('Missing pinned FUJI resource: '+file)
    scale=.1524/.205
    rotation=Rotation.from_euler('z',math.pi/2)
    metadata=[]
    # Replace the analytic contact surrogate, retaining its original geometric
    # corner resolution, source servos and separate passive vertical compliance.
    for role,name in ROLES.items():
        link=root.find(f"link[@name='{name}_link']")
        inertial=link.find('inertial')
        if inertial is None:raise ValueError('Missing vendor wheel inertia')
        origin=inertial.find('origin')
        if any(abs(float(v))>1e-10 for v in origin.get('rpy','0 0 0').split()):
            raise ValueError('Unreviewed source wheel inertial rotation')
        mass=float(inertial.find('mass').get('value'))
        com=np.array(list(map(float,origin.get('xyz','0 0 0').split())))
        tensor=matrix(inertial.find('inertia'))
        original_origin_tensor=tensor+translate_inertia(mass,com)
        center=np.array([0.,-.02317416 if role.endswith('left') else .02317416,0.])
        for block in list(root.findall('gazebo')):
            if block.get('reference')==name+'_link':root.remove(block)
        for collision in list(link.findall('collision')):link.remove(collision)
        hub=ET.SubElement(link,'collision')
        ET.SubElement(hub,'origin',xyz=vector(center),rpy=f'0 0 {math.pi/2}')
        ET.SubElement(ET.SubElement(hub,'geometry'),'mesh',filename=str(assets/'wheel.stl'),scale=vector([scale]*3))
        direction=-1 if role in ('front_left','rear_right') else 1
        roller_mass=mass*.1/15
        local_tensor=np.diag([.00024,.00076,.00076])*scale**2*(roller_mass/.3)
        moment=np.zeros(3);roller_origin_tensor=np.zeros((3,3));children=[]
        for index in range(15):
            theta=2*math.pi*index/15
            local_rpy=[math.atan(-math.tan(theta)*math.sqrt(2)),
                       math.asin(direction*math.cos(theta)/math.sqrt(2)),
                       math.atan(direction*math.sin(theta))]
            pose=rotation*Rotation.from_euler('xyz',local_rpy)
            xyz=center+rotation.apply([0.,.0895*math.cos(theta)*scale,.0895*math.sin(theta)*scale])
            child=name+f'_lab_roller_{index+1}'
            body=ET.SubElement(root,'link',name=child)
            inertia=ET.SubElement(body,'inertial')
            ET.SubElement(inertia,'mass',value=str(roller_mass))
            element=ET.SubElement(inertia,'inertia');write_matrix(element,local_tensor)
            collision=ET.SubElement(body,'collision')
            ET.SubElement(ET.SubElement(collision,'geometry'),'mesh',filename=str(assets/'roller_collision.stl'),scale=vector([scale]*3))
            joint=ET.SubElement(root,'joint',name=child+'_joint',type='continuous')
            ET.SubElement(joint,'parent',link=name+'_link');ET.SubElement(joint,'child',link=child)
            ET.SubElement(joint,'origin',xyz=vector(xyz),rpy=vector(pose.as_euler('xyz')))
            ET.SubElement(joint,'axis',xyz='1 0 0')
            ET.SubElement(joint,'dynamics',damping='.00001',friction='.000001')
            friction=ET.SubElement(root,'gazebo',reference=child)
            ET.SubElement(friction,'mu1').text='.8';ET.SubElement(friction,'mu2').text='.8'
            world_tensor=pose.as_matrix()@local_tensor@pose.as_matrix().T
            roller_origin_tensor+=world_tensor+translate_inertia(roller_mass,xyz)
            moment+=roller_mass*xyz
            children.append(child+'_joint')
        core_mass=mass-15*roller_mass
        core_com=(mass*com-moment)/core_mass
        core_tensor=original_origin_tensor-roller_origin_tensor-translate_inertia(core_mass,core_com)
        if not np.all(np.linalg.eigvalsh(core_tensor)>0):
            raise ValueError('Physical roller split gives a nonpositive source core inertia')
        inertial.find('mass').set('value',str(core_mass));origin.set('xyz',vector(core_com))
        write_matrix(inertial.find('inertia'),core_tensor)
        reconstructed=core_tensor+translate_inertia(core_mass,core_com)+roller_origin_tensor
        if (not np.allclose(reconstructed,original_origin_tensor,atol=1e-12)
                or not np.allclose(core_mass*core_com+moment,mass*com,atol=1e-12)):
            raise ValueError('Roller split changed the source rigid-assembly inertia')
        metadata.append(dict(wheel=name,role=role,original_mass_kg=mass,core_mass_kg=core_mass,
            original_com_m=com.tolist(),core_com_m=core_com.tolist(),original_inertia=tensor.tolist(),
            reassembled_origin_inertia=reconstructed.tolist(),roller_joints=children,
            roller_mass_kg=roller_mass,geometry_scale=scale))
    return metadata
