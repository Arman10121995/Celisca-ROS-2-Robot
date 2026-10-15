#!/usr/bin/env python3
"""Adapt pinned CC-BY Tethys and native Fortress example, preserving originals.

Source geometry/masses/joints remain intact. The marine runtime adds declared
internal trim ballast for the chosen fluid, native bounded propulsion and
actual observation bridges. This authoring step is not mission qualification.
"""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
from robot_lab_utils.mesh_assets import parse_collada_mesh,write_binary_stl

ROOT=Path(__file__).resolve().parents[1]
FOLDER=ROOT/'src/robot_lab_robots/tethys_auv'
SOURCE=FOLDER/'source'
ZIP_SHA='81cded5384f86c77afa1afa56050f024463379db912c33a60d938d27d28f4a15'


def add(parent,tag,text=None,**attrs):
    result=ET.SubElement(parent,tag,{k:str(v) for k,v in attrs.items()})
    if text is not None:result.text=str(text)
    return result


def pose(text):
    values=list(map(float,(text or '0 0 0 0 0 0').split()))
    if len(values)!=6 or not np.isfinite(values).all():raise ValueError('Finite SDF pose required')
    return values[:3],values[3:]


def save(root,path):
    path.parent.mkdir(parents=True,exist_ok=True);ET.indent(root)
    path.write_bytes(ET.tostring(root,encoding='utf-8',xml_declaration=True))


def ballast(model,density):
    """Seal an explicit internal mass; it adds no displaced collision volume."""
    source_mass=sum(float(link.findtext('inertial/mass')) for link in model.findall('link'))
    volume=0.;moment=np.zeros(3)
    for source_link in model.findall('link'):
        for collision in source_link.findall('collision'):
            part_volume=float(np.prod(list(map(float,collision.findtext('geometry/box/size').split()))))
            center=np.array(pose(source_link.findtext('pose'))[0])+pose(collision.findtext('pose'))[0]
            volume+=part_volume;moment+=part_volume*center
    center=moment/volume
    mass=max(0.,density*volume-source_mass)
    if mass>1e-8:
        link=add(model,'link',name='lab_trim_ballast');add(link,'pose',f'{center[0]} {center[1]} -.08 0 0 0')
        inertia=add(link,'inertial');add(inertia,'mass',mass)
        values=add(inertia,'inertia')
        for key in ('ixx','iyy','izz','ixy','ixz','iyz'):add(values,key,.4*mass*.03**2 if key in ('ixx','iyy','izz') else 0.)
        joint=add(model,'joint',name='lab_trim_ballast_joint',type='fixed')
        add(joint,'parent','base_link');add(joint,'child','lab_trim_ballast')
    return dict(source_mass_kg=source_mass,displaced_volume_m3=float(volume),ballast_mass_kg=mass,
                complete_mass_kg=source_mass+mass,fluid_density_kg_m3=density,
                center_of_buoyancy_m=center.tolist(),
                ballast_note='Declared sealed internal mass below center of buoyancy (z=-0.08 m), no added collision volume; original mass/inertia untouched')


def add_systems(model,density):
    # The source declares unlimited propeller effort and huge finite angle
    # limits. DART rejects velocity control of that combination. Keep the
    # source untouched and declare the lab actuator's finite torque/rate.
    limits=model.find("joint[@name='propeller_joint']/axis/limit")
    limits.find('effort').text='2.0';limits.find('velocity').text='100.0'
    example=ET.parse(FOLDER/'source_physics/auv_controls_fortress.sdf').getroot().find('world/include')
    for item in example.findall('plugin'):
        plugin=copy.deepcopy(item);name=plugin.get('name')
        if name.endswith('::JointPositionController'):
            add(plugin,'cmd_max',2.);add(plugin,'cmd_min',-2.);add(plugin,'use_velocity_commands','false')
        elif name.endswith('::Thruster'):
            plugin.find('namespace').text='tethys_auv';plugin.find('fluid_density').text=str(density)
            add(plugin,'min_thrust_cmd',-35.);add(plugin,'max_thrust_cmd',35.);add(plugin,'velocity_control','true')
        elif name.endswith('::LiftDrag'):plugin.find('air_density').text=str(density)
        elif name.endswith('::Hydrodynamics'):
            for field in plugin:
                if field.tag!='link_name':
                    coefficient=float(field.text)
                    # Fortress 6.18 applies -Ma*acceleration. The bundled
                    # example predates that convention: negative stability
                    # derivatives must become positive mass diagonals.
                    if 'Dot' in field.tag:coefficient=abs(coefficient)
                    field.text=str(coefficient*density/1000.)
        model.append(plugin)
    observer=add(model,'plugin',filename='ignition-gazebo-odometry-publisher-system',name='ignition::gazebo::systems::OdometryPublisher')
    for key,value in dict(dimensions=3,odom_frame='map',robot_base_frame='base_link',odom_publish_frequency=50,
                          odom_topic='/marine/odometry',tf_topic='/marine/truth_pose').items():add(observer,key,value)
    states=add(model,'plugin',filename='ignition-gazebo-joint-state-publisher-system',name='ignition::gazebo::systems::JointStatePublisher')
    add(states,'topic','/marine/joint_states')
    imu=add(model.find("link[@name='base_link']"),'sensor',name='auv_imu',type='imu')
    for key,value in dict(always_on='true',update_rate=50,topic='/marine/imu',gz_frame_id='base_link').items():add(imu,key,value)


def resolve_assets(model,base):
    for item in model.iter():
        if item.tag in ('uri','albedo_map','normal_map','metalness_map','roughness_map') and item.text:
            relative=Path(item.text.strip())
            if not relative.is_absolute() and '://' not in item.text:
                item.text=str((base/relative).resolve())


def display_urdf(source_model):
    """Separate link visual submeshes; never repeat the entire robot per fin."""
    model=copy.deepcopy(source_model);calibration=ballast(model,1025.)
    urdf=ET.Element('robot',name='tethys_auv');submeshes={}
    mesh_dir=FOLDER/'meshes';mesh_dir.mkdir(exist_ok=True)
    for source_link in model.findall('link'):
        link=add(urdf,'link',name=source_link.get('name'))
        src_inertia=source_link.find('inertial');inertia=add(link,'inertial')
        xyz,rpy=pose(src_inertia.findtext('pose'));add(inertia,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)))
        add(inertia,'mass',value=src_inertia.findtext('mass'))
        add(inertia,'inertia',**{item.tag:item.text for item in src_inertia.find('inertia')})
        for collision in source_link.findall('collision'):
            item=add(link,'collision',name=source_link.get('name')+'_'+collision.get('name'))
            xyz,rpy=pose(collision.findtext('pose'));add(item,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)))
            geometry=add(item,'geometry');add(geometry,'box',size=collision.findtext('geometry/box/size'))
        for visual in source_link.findall('visual'):
            name=visual.findtext('geometry/mesh/submesh/name')
            if name not in submeshes:
                dae=ET.parse(SOURCE/'meshes/tethys.dae');root=dae.getroot();ns={'c':root.tag.split('}')[0][1:]}
                for scene in root.findall('.//c:library_visual_scenes/c:visual_scene',ns):
                    for node in list(scene):
                        if node.tag.endswith('node') and node.get('name')!=name:scene.remove(node)
                # Converter respects original scene transforms, scale and axis.
                import os
                cache=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))/'auv_asset_build'
                cache.mkdir(parents=True,exist_ok=True);temporary=cache/(name+'.dae');dae.write(temporary)
                vertices,faces=parse_collada_mesh(temporary)
                target=mesh_dir/(name+'.stl');write_binary_stl(target,vertices,faces);submeshes[name]=target
            item=add(link,'visual',name=source_link.get('name')+'_visual');xyz,rpy=pose(visual.findtext('pose'))
            add(item,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)))
            add(add(item,'geometry'),'mesh',filename='package://robot_lab_robots/tethys_auv/meshes/'+submeshes[name].name)
            add(add(item,'material',name='tethys_color'),'color',rgba='.85 .55 .08 1')
    for src_joint in model.findall('joint'):
        joint=add(urdf,'joint',name=src_joint.get('name'),type='continuous' if src_joint.get('name')=='propeller_joint' else src_joint.get('type'))
        add(joint,'parent',link=src_joint.findtext('parent'));child=src_joint.findtext('child');add(joint,'child',link=child)
        xyz,rpy=pose(model.find("link[@name='%s']"%child).findtext('pose'))
        add(joint,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)))
        if src_joint.get('type')!='fixed':
            add(joint,'axis',xyz=src_joint.findtext('axis/xyz'))
            limits=src_joint.find('axis/limit')
            add(joint,'limit',lower=limits.findtext('lower'),upper=limits.findtext('upper'),effort=2.,velocity=100.)
    return urdf,calibration


def main():
    source=ET.parse(SOURCE/'model.sdf').getroot().find('model')
    description,calibration=display_urdf(source);save(description,FOLDER/'urdf/tethys_auv.urdf')
    original=ET.Element('sdf',version='1.6');model=copy.deepcopy(source);model.set('name','tethys_auv')
    calibration=ballast(model,1025.);add_systems(model,1025.)
    # Runtime resolves relative resources against the installed source folder.
    original.append(model);save(original,FOLDER/'model.sdf')
    calibration.update(source_fuel_version=3,source_zip_sha256=ZIP_SHA,source_model_sha256=hashlib.sha256((SOURCE/'model.sdf').read_bytes()).hexdigest(),
                       source_physics_sha256=hashlib.sha256((FOLDER/'source_physics/auv_controls_fortress.sdf').read_bytes()).hexdigest(),
                       thrust_limit_N=35.,fin_limit_rad=.261799,source_forward_yaw=3.141592653589793,
                       added_mass_convention='Fortress 6.18 applies -Ma*acceleration; derived mass diagonals are positive, original example retained unchanged',
                       scope='Native source AUV forces/fins; authored trim and experimental guidance, no independent estimation/SLAM/navigation')
    (FOLDER/'calibration.json').write_text(json.dumps(calibration,indent=2)+'\n')
    print(json.dumps(calibration,indent=2))


if __name__=='__main__':main()
