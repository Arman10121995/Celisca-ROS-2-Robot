#!/usr/bin/env python3
"""Install the complete official myAGV Pro with declared lab simulation adapters."""
import argparse
import fcntl
import hashlib
import json
import os
import re
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/robot_lab_utils'))
from robot_lab_utils.myagv_pro import ROLES, adapt_source, sensor_kit, lab_compliance
from robot_lab_utils.drive_kinematics import drive_from_config
from catalog_external_assets import checkout, digest, load_source
from provision_extension_assets import checkpoint_robots, robot_entity, source_package_prefix, write_json, write_yaml


def provision(store, download=False):
    source=load_source('elephant_myagv_pro')
    if download:
        for entry in ('agv_pro_description','agv_pro_gazebo','agv_pro_base'):
            checkout(source,entry,store)
    root=store/'sources'/source['id']/source['revision']
    if subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()!=source['revision']:
        raise ValueError('Unexpected official source revision')
    if subprocess.check_output(['git','status','--porcelain'],cwd=root,text=True):
        raise ValueError('Preserve edited vendor source before provisioning')
    env=source_package_prefix([root/'agv_pro_description'],store/'prefixes/elephant_myagv_pro')
    entry='agv_pro_description/urdf/agv_pro.urdf'
    expanded=subprocess.check_output(['xacro',str(root/entry),'namespace:='],env=env,text=True,timeout=45)
    document=ET.fromstring(expanded)
    name='asset_elephant_myagv_pro';document.set('name',name)
    original=ET.fromstring(expanded)
    resources={}
    for mesh in document.findall('.//mesh'):
        uri=mesh.get('filename','')
        if not uri.startswith('package://agv_pro_description/'):
            raise ValueError('Unexpected vendor mesh '+uri)
        path=(root/'agv_pro_description'/uri.removeprefix('package://agv_pro_description/')).resolve()
        if not path.is_relative_to(root.resolve()) or not path.is_file():raise ValueError('Missing source mesh '+uri)
        mesh.set('filename',str(path));resources[str(path)]=digest(path)
    wheels=adapt_source(document)
    compliance=lab_compliance(document)
    sensors=sensor_kit(document)
    drive=dict(type='mecanum', **{role+'_wheel_joint':joint+'_joint' for role,joint in ROLES.items()},
        wheel_radius=.0762,wheelbase=.3431,wheel_separation=.31345248,roller_angle=.7853981633974483,
        max_speed=.3,max_accel=.3,max_angular_speed=.4,max_angular_accel=.5,max_wheel_speed=8.,wheel_force_limit=5.,
        gazebo_wheel_velocity_gain=1.,gazebo_wheel_armature=.01)
    identity=hashlib.sha256(ET.tostring(document)+json.dumps(drive,sort_keys=True).encode()).hexdigest()
    target=store/'models'/name/identity;target.mkdir(parents=True,exist_ok=True)
    controls=target/'controllers.yaml'
    joints=drive_from_config(drive).wheel_joints
    config=dict(controller_manager=dict(ros__parameters=dict(update_rate=100,use_sim_time=True,
        joint_state_broadcaster=dict(type='joint_state_broadcaster/JointStateBroadcaster'),
        mecanum_wheel_controller=dict(type='velocity_controllers/JointGroupVelocityController'))),
        joint_state_broadcaster=dict(ros__parameters=dict(use_sim_time=True)),
        mecanum_wheel_controller=dict(ros__parameters=dict(use_sim_time=True,joints=joints)))
    write_yaml(controls,config)
    system=ET.SubElement(document,'ros2_control',name='LabMyAGVProSystem',type='system')
    ET.SubElement(ET.SubElement(system,'hardware'),'plugin').text='ign_ros2_control/IgnitionSystem'
    for name_joint in joints:
        joint=ET.SubElement(system,'joint',name=name_joint)
        command=ET.SubElement(joint,'command_interface',name='velocity')
        ET.SubElement(command,'param',name='min').text='-8'
        ET.SubElement(command,'param',name='max').text='8'
        for state in ('position','velocity','effort'):ET.SubElement(joint,'state_interface',name=state)
    for name_joint in compliance['joints']:
        joint=ET.SubElement(system,'joint',name=name_joint)
        # Native passive springs remain uncommanded. Measured travel is still
        # required by robot_state_publisher to place the original wheel frames.
        for state in ('position','velocity','effort'):ET.SubElement(joint,'state_interface',name=state)
    gazebo=ET.SubElement(document,'gazebo')
    plugin=ET.SubElement(gazebo,'plugin',filename='ign_ros2_control-system',name='ign_ros2_control::IgnitionROS2ControlPlugin')
    ET.SubElement(plugin,'parameters').text=str(controls)
    truth=ET.SubElement(gazebo,'plugin',name='ignition::gazebo::systems::OdometryPublisher',filename='ignition-gazebo-odometry-publisher-system')
    for key,value in dict(odom_frame='world',robot_base_frame='base_footprint',odom_topic='/model/'+name+'/odometry_truth',
        tf_topic='/model/'+name+'/truth_pose',odom_publish_frequency='50',dimensions='3').items():ET.SubElement(truth,key).text=value
    output=target/'drive.urdf';output.write_text(ET.tostring(document,encoding='unicode'))
    converted=subprocess.check_output(['ign','sdf','-p',str(output)],text=True,timeout=30)
    (target/'converted.sdf').write_text(converted)
    parseable=converted if 'xmlns:ignition=' in converted else re.sub(
        r'(<sdf\b)',r'\1 xmlns:ignition="http://ignitionrobotics.org/schema"',converted,count=1)
    native=ET.fromstring(parseable)
    if (len(native.findall('.//fdir1'))!=4
            or len(native.findall("model/link[@name='base_footprint']/sensor"))!=3):
        raise ValueError('Converted plant lost a wheel friction direction or native sensor')
    (target/'source-expanded.urdf').write_text(expanded)
    (target/'LICENSE').write_bytes((root/'LICENSE').read_bytes())
    # Both physics and RViz retain the complete source chassis, all wheel
    # inertias and source TF. Only wheel contact/dynamics are declared adapters.
    for link in original.findall('link'):
        inertial=link.find('inertial')
        if inertial is not None:
            actual=document.find(f"link[@name='{link.get('name')}']/inertial")
            if ET.tostring(inertial)!=ET.tostring(actual):raise ValueError('Changed source body inertia')
    profile=dict(package='robot_lab_robots',name=name,xacro=str(output),type='mobile',external_asset=True,
        asset_origin='extension',source_id=source['id'],source_revision=source['revision'],source_entry=entry,
        drive=drive,sensor_config=sensors,drive_in_display=True,controllers_yaml=str(controls),
        controllers=['joint_state_broadcaster','mecanum_wheel_controller'],
        supported_simulators=['gazebo'],supported_modes=['display'],default_mode='display',default_map='nav_empty',
        experimental_modes_by_simulator={'gazebo':['display','loc','slam','3d_slam','nav']},spawn={'z':.006},
        features=['ros2_control','velocity_base','holonomic_base','lidar_2d','rgbd_camera','imu'],
        default_algorithms={'control':'native_holonomic_controller'},
        implementation_state='experimental',validation_state='pending',qualification_state='experimental_unqualified',
        notes='Complete official BSD-3-Clause myAGV Pro CAD chassis/four wheels, original visual meshes, body masses/inertias and sensor frames. '
              'Declared lab Fortress adapter: 0.0762 m cylindrical wheel envelopes with anisotropic contact (rigid source STL rollers), '
              'bounded passive tire-compliance surrogate with 0.02 kg added carriers (not vendor suspension), '
              'geometric corner mapping, 5 Nm/8 rad-s simulation caps (not hardware ratings), native holonomic controller and simulated LiDAR/IMU/RGB-D at source mounts. '
              'Recorded 2026-11-12: Gazebo/nav_empty Display drive/sensors/timeout and one normal-GUI Nav2 arrival (0.125 m, 2.23 degrees); '
              'exact revision/route receipts: docs/status/evidence/myagv-pro-2026-11-12/README.md. '
              'Higher modes require experimental opt-in. Source CAD mass 20.316 kg differs from documented hardware 40 kg. '
              'Other backends, calibrated contact fidelity, wider missions and hardware serial/autocharge unqualified.')
    changes=['source CAD body/inertias/frames preserved','vendor broken controller/joint names replaced by actual joint mapping',
             'CAD wheel radius used instead of vendor Gazebo 0.05 m placeholder',
             'declared cylindrical/anisotropic wheel contact approximation',
             'bounded passive vertical tire-compliance surrogate; 0.02 kg added carriers, original inertias retained',
             'conservative lab 5 Nm/8 rad-s wheel limits; not source hardware specifications',
             'native Fortress control/timeout and simulated sensors at original frame mounts']
    receipt=dict(repository=source['repository'],source_id=source['id'],revision=source['revision'],license='BSD-3-Clause',
        original=str(root/entry),source_sha256=digest(root/entry),source_expanded_sha256=digest(target/'source-expanded.urdf'),
        description=str(output),description_sha256=digest(output),source_meshes_sha256=resources,
        converted_sdf_sha256=digest(target/'converted.sdf'),converted_contact_directions=4,converted_sensors=3,
        lab_compliance=compliance,added_carrier_mass_kg=.02,
        source_mass_kg=sum(float(mass.get('value')) for mass in original.findall('link/inertial/mass')),
        wheel_corner_mapping=ROLES,source_wheels=wheels,drive=drive,lab_changes=changes,runtime_mission_qualified=False)
    evidence=target/'integration.json';write_json(evidence,receipt)
    entity=robot_entity(name,profile,source,entry,evidence)
    entity.update(name='Elephant Robotics myAGV Pro',robot_class='mobile',
        capabilities=['display','control','holonomic_base','lidar_2d','rgbd_camera','imu'],
        tags=['extension','myagv_pro','four_wheels','mecanum','complete_robot','experimental_control'])
    entity['locomotion']=dict(type='mecanum',dof=3,wheels=4,max_velocity=.3,max_acceleration=.3)
    entity['frames']=list(dict.fromkeys(['base_footprint',*entity.get('frames',[])]))
    for item in entity['evidence']:item['date']='2026-11-12'
    checkpoint_robots(store,{name:profile},[entity],[dict(source_id=source['id'],revision=source['revision'],
        entry=entry,status='installed',check=str(evidence),profiles=[dict(id=name,kind='robot',support='Experimental Fortress source-model adapter',notes=profile['notes'])])])
    return receipt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--download',action='store_true')
    parser.add_argument('--store',type=Path,default=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))/'external_assets')
    args=parser.parse_args();store=args.store.resolve()
    store.mkdir(parents=True,exist_ok=True)
    if store.stat().st_dev==Path('/').stat().st_dev:parser.error('Use the mounted SSD')
    (store/'locks').mkdir(exist_ok=True)
    with (store/'locks/provision.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        print(json.dumps(provision(store,args.download),indent=2))


if __name__=='__main__':main()
