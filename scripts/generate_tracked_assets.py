#!/usr/bin/env python3
"""Preserve Gazebo's complete tracked/flipper example and export its RViz tree.

Source: gazebosim/gz-sim, ign-gazebo6 examples/worlds/tracked_vehicle_simple.sdf,
Apache-2.0. Physics uses the original contact-surface tracks, not wheel aliases.
"""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]


def matrix(text):
    values=list(map(float,(text or '0 0 0 0 0 0').split()))
    transform=np.eye(4);transform[:3,3]=values[:3]
    transform[:3,:3]=Rotation.from_euler('xyz',values[3:]).as_matrix()
    return transform


def origin(parent, transform):
    ET.SubElement(parent,'origin',xyz=' '.join(map(str,transform[:3,3])),
        rpy=' '.join(map(str,Rotation.from_matrix(transform[:3,:3]).as_euler('xyz'))))


def generate(source=Path('/usr/share/ignition/ignition-gazebo6/worlds/tracked_vehicle_simple.sdf')):
    folder=ROOT/'src/robot_lab_robots/lab_tracked';folder.mkdir(parents=True,exist_ok=True)
    raw=source.read_bytes();model=ET.fromstring(raw).find("world/model[@name='simple_tracked']")
    if model is None:raise ValueError('Complete Gazebo tracked example missing')
    # Preserve source poses, masses, colliders, joints and track-controller
    # settings. Keyboard triggered publishers belong to the upstream demo,
    # and are replaced by Robot Lab's explicit bounded command owner.
    model=copy.deepcopy(model);model.set('name','lab_tracked');model.find('pose').text='0 0 .1 0 0 0'
    for plugin in list(model.findall('plugin')):
        if plugin.get('name','').endswith('TriggeredPublisher'):model.remove(plugin)
    control=next(p for p in model.findall('plugin') if p.get('name','').endswith('TrackedVehicle'))
    for key,value in dict(topic='/tracked/cmd_vel',odom_topic='/tracked/odometry',tf_topic='/tracked/odom_tf',
        frame_id='odom',child_frame_id='base_footprint').items():ET.SubElement(control,key).text=value
    for tag,cap in (('linear_velocity',.3),('angular_velocity',.5)):
        limit=ET.SubElement(control,tag)
        for key,value in dict(min_velocity=-cap,max_velocity=cap,min_acceleration=-.4,max_acceleration=.4).items():ET.SubElement(limit,key).text=str(value)
    # Correct the source's unused plural spelling; Fortress's actual API is
    # track_height. Preserve the source value and record the adaptation.
    height=control.find('tracks_height')
    if height is not None:height.tag='track_height'
    urdf=ET.Element('robot',name='lab_tracked');ET.SubElement(urdf,'link',name='base_footprint')
    joint=ET.SubElement(urdf,'joint',name='footprint_fixed',type='fixed')
    ET.SubElement(joint,'parent',link='base_footprint');ET.SubElement(joint,'child',link='base_link')
    links={l.get('name'):matrix(l.findtext('pose')) for l in model.findall('link')}
    for source_link in model.findall('link'):
        link=ET.SubElement(urdf,'link',name=source_link.get('name'))
        inertia=source_link.find('inertial')
        if inertia is not None:
            target=ET.SubElement(link,'inertial');origin(target,matrix(inertia.findtext('pose')))
            ET.SubElement(target,'mass',value=inertia.findtext('mass'))
            ET.SubElement(target,'inertia',**{k:inertia.findtext('inertia/'+k,'0') for k in ('ixx','ixy','ixz','iyy','iyz','izz')})
        for kind in ('visual','collision'):
            for shape in source_link.findall(kind):
                target=ET.SubElement(link,kind,name=shape.get('name'));origin(target,matrix(shape.findtext('pose')))
                geometry=ET.SubElement(target,'geometry');g=shape.find('geometry')
                if g.find('box') is not None:ET.SubElement(geometry,'box',size=g.findtext('box/size'))
                elif g.find('cylinder') is not None:ET.SubElement(geometry,'cylinder',radius=g.findtext('cylinder/radius'),length=g.findtext('cylinder/length'))
                elif g.find('sphere') is not None:ET.SubElement(geometry,'sphere',radius=g.findtext('sphere/radius'))
                else:raise ValueError('Unexported source geometry')
                if kind=='visual':
                    material=ET.SubElement(target,'material',name=shape.get('name')+'_color')
                    ET.SubElement(material,'color',rgba=shape.findtext('material/diffuse','.25 .35 .45 1'))
    for source_joint in model.findall('joint'):
        parent,child=source_joint.findtext('parent'),source_joint.findtext('child')
        target=ET.SubElement(urdf,'joint',name=source_joint.get('name'),type=source_joint.get('type'))
        ET.SubElement(target,'parent',link=parent);ET.SubElement(target,'child',link=child)
        origin(target,np.linalg.inv(links[parent])@links[child])
        if source_joint.get('type')!='fixed':
            ET.SubElement(target,'axis',xyz=source_joint.findtext('axis/xyz'))
            limit=source_joint.find('axis/limit')
            ET.SubElement(target,'limit',lower=limit.findtext('lower','-3.14159'),upper=limit.findtext('upper','3.14159'),effort='25',velocity='1')
    sdf=ET.Element('sdf',version='1.7');sdf.append(model)
    for path,tree in ((folder/'model.sdf',sdf),(folder/'robot.urdf',urdf)):
        ET.indent(tree);path.write_bytes(ET.tostring(tree,encoding='utf-8',xml_declaration=True))
    (folder/'SOURCE.json').write_text(json.dumps(dict(upstream='https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/tracked_vehicle_simple.sdf',
        installed_source=str(source),source_sha256=hashlib.sha256(raw).hexdigest(),license='Apache-2.0',
        adaptations=['demo keyboard publishers removed','explicit bounded topics/frames','track_height spelling corrected','RViz URDF export'],
        scope='Complete source tracked model; native runtime qualification separate'),indent=2)+'\n')


if __name__=='__main__':generate()
