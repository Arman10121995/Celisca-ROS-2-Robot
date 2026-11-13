#!/usr/bin/env python3
"""Install the complete official myAGV Pro with declared lab simulation adapters.

Variants: ``fortress`` (Gazebo), ``rollers`` (MuJoCo/PyBullet/Isaac) and
``pro630``: the roller base with the official myCobot Pro630 arm on the deck
(robot_lab_utils.myagv_pro630; ROADMAP R14.3)."""
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


def provision(store, download=False, variant='fortress'):
    if variant not in ('fortress','rollers','pro630'):raise ValueError('Unknown myAGV physical adapter')
    portable=variant in ('rollers','pro630')
    with_arm=variant=='pro630'
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
    name='asset_elephant_myagv_pro'+('_pro630' if with_arm else '_rollers' if portable else '');document.set('name',name)
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
    roller_contract=[]
    fuji=ROOT/'src/robot_lab_robots/holonomic_wheels/third_party/fuji_mecanum'
    if portable:
        from robot_lab_utils.myagv_rollers import physical_rollers
        roller_contract=physical_rollers(document,fuji)
    articulation=arm_receipt=arm_source=None
    if with_arm:
        from robot_lab_utils.myagv_pro630 import compose
        arm_source=load_source('elephant_mycobot_ros2')
        if download:
            for entry_arm in arm_source['entries']:
                checkout(arm_source,entry_arm,store)
        arm_root=store/'sources'/arm_source['id']/arm_source['revision']
        if subprocess.check_output(['git','rev-parse','HEAD'],cwd=arm_root,text=True).strip()!=arm_source['revision']:
            raise ValueError('Unexpected official arm source revision')
        if subprocess.check_output(['git','status','--porcelain'],cwd=arm_root,text=True):
            raise ValueError('Preserve edited arm source before provisioning')
        articulation,arm_receipt=compose(document,arm_root,store/'model_geometry'/'myagv_pro630'/arm_source['revision'])
    drive=dict(type='mecanum', **{role+'_wheel_joint':joint+'_joint' for role,joint in ROLES.items()},
        wheel_radius=.0762,wheelbase=.3431,wheel_separation=.31345248,roller_angle=.7853981633974483,
        max_speed=.3,max_accel=.3,max_angular_speed=.4,max_angular_accel=.5,max_wheel_speed=8.,wheel_force_limit=5.,
        gazebo_wheel_velocity_gain=1.,gazebo_wheel_armature=.01)
    if portable:
        drive['mujoco_wheel_housing_exclusions']=True
        drive['mujoco_preserve_fixed_root_inertia']=True
        drive['mujoco_expected_mass_kg']=sum(float(mass.get('value')) for mass in document.findall('link/inertial/mass'))
        drive.update(mujoco_wheel_velocity_gain=10.,mujoco_wheel_armature=.02,mujoco_implicit_wheel_control=True)
        drive.update(pybullet_explicit_massless_frames=True, pybullet_preserve_passive_springs=True,
                     pybullet_expected_mass_kg=drive['mujoco_expected_mass_kg']+1e-6)
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
    if (len(native.findall('.//fdir1'))!=(0 if portable else 4)
            or len(native.findall("model/link[@name='base_footprint']/sensor"))!=3):
        raise ValueError('Converted plant lost a wheel friction direction or native sensor')
    (target/'source-expanded.urdf').write_text(expanded)
    (target/'LICENSE').write_bytes((root/'LICENSE').read_bytes())
    if portable:(target/'LICENSE-FUJI-MIT').write_bytes((fuji/'LICENSE').read_bytes())
    # Both physics and RViz retain the complete source chassis, all wheel
    # inertias and source TF. Only wheel contact/dynamics are declared adapters.
    for link in original.findall('link'):
        inertial=link.find('inertial')
        if inertial is not None:
            if portable and link.get('name') in {n+'_link' for n in ROLES.values()}:
                continue  # Exact aggregate mass/COM/tensor checked by physical_rollers.
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
    if portable:
        # Native mobile backends expose an ideal root-state IMU observer.
        # Do not mislabel it with the vendor's differently rotated IMU frame.
        sensors['imu_link_name']='base_footprint'
        profile.update(supported_simulators=['mujoco','pybullet','isaac'],
            experimental_modes_by_simulator={sim:['display','loc','slam','3d_slam','nav'] for sim in ('mujoco','pybullet','isaac')},
            notes='Complete official myAGV Pro CAD base with a declared portable MIT FUJI physical-roller adapter: '
                  '15 passive rollers per driven wheel, original vendor wheel assembly mass/COM/inertia conserved, '
                  'original visual meshes and source sensor mounts. Lab 0.02 kg vertical-compliance kit, bounded 5 Nm/8 rad-s wheel control, '
                  'source-mount LiDAR/RGB-D and explicitly root-frame ideal IMU observer. Original rotated imu_link remains in TF. '
                  'Native physics drives the root; no chassis-pose/velocity override. '
                  'Source CAD mass differs from hardware documentation. Other-mode missions and physical calibration unqualified; '
                  'use the separate measured Fortress adapter for Gazebo.')
        changes=['source CAD chassis/visuals/frames retained',
                 'original wheel mass/COM/inertia decomposed into core plus 15 passive FUJI rollers',
                 'source FUJI geometry/handedness scaled to 152.4 mm diameter',
                 'same declared passive vertical-compliance kit and simulated sensors',
                 'existing native backend wheel actuation, springs, measured feedback and command lease']
    if with_arm:
        profile.update(type='mobile_manipulator',articulation=articulation,
            features=[*profile['features'],'articulated_arm'],
            notes='Official myAGV Pro (BSD-3-Clause) with the physical-roller adapter, and the official myCobot Pro630 arm '
                  '(BSD-2-Clause, mycobot_ros2 d42ff61a) fixed on the deck: source kinematics, axes, URDF limits, collision primitives '
                  'and visuals (Collada converted to metre STL). Declared lab estimates, not vendor figures: 8 kg arm mass spread by '
                  'hull volume, 1 rad/s joint cap, efforts 1.5 x (gravity + inertia x 2 rad/s^2) with the 2 kg payload, centred mount '
                  'on the deck rails. Bounded position control of six joints with Stop, heartbeat watchdog and base inhibition '
                  '(urdf_articulation). No gripper (no official Pro630 gripper description). Missions unqualified.')
        changes=[*changes,'official Pro630 arm fixed on the deck (declared mount)','Collada visuals converted to metre STL',
                 'declared arm mass/inertia/effort/speed estimates','bounded six-joint position articulation']
    receipt=dict(repository=source['repository'],source_id=source['id'],revision=source['revision'],license='BSD-3-Clause',
        original=str(root/entry),source_sha256=digest(root/entry),source_expanded_sha256=digest(target/'source-expanded.urdf'),
        description=str(output),description_sha256=digest(output),source_meshes_sha256=resources,
        converted_sdf_sha256=digest(target/'converted.sdf'),converted_contact_directions=len(native.findall('.//fdir1')),converted_sensors=3,
        lab_compliance=compliance,added_carrier_mass_kg=.02,
        variant=variant,physical_roller_contract=roller_contract,
        source_mass_kg=sum(float(mass.get('value')) for mass in original.findall('link/inertial/mass')),
        wheel_corner_mapping=ROLES,source_wheels=wheels,drive=drive,lab_changes=changes,runtime_mission_qualified=False,
        arm=arm_receipt,articulation=articulation)
    evidence=target/'integration.json';write_json(evidence,receipt)
    entity=robot_entity(name,profile,source,entry,evidence)
    entity.update(name='Elephant Robotics myAGV Pro'+(' + myCobot Pro630' if with_arm else ''),robot_class='mobile',
        capabilities=['display','control','holonomic_base','lidar_2d','rgbd_camera','imu'],
        tags=['extension','myagv_pro','four_wheels','mecanum','complete_robot','experimental_control'])
    entity['locomotion']=dict(type='mecanum',dof=3,wheels=4,max_velocity=.3,max_acceleration=.3)
    entity['frames']=list(dict.fromkeys(['base_footprint',*entity.get('frames',[])]))
    if portable:
        for item in entity.get('sensors',[]):
            if item.get('type')=='imu':item['frame']=sensors['imu_link_name']
    if portable:
        extra=dict(repository='https://github.com/DaiGuard/fuji_mecanum',revision='646431a5e107448e0e2bdeeba4575db9aa3665ff',
                   license='MIT',resource_sha256={file:digest(fuji/file) for file in ('wheel.stl','roller_collision.stl','LICENSE','README.md')})
        receipt['auxiliary_sources']=[extra]
        if with_arm:
            receipt['auxiliary_sources'].append(dict(repository=arm_source['repository'],revision=arm_source['revision'],
                license='BSD-2-Clause',resource_sha256={'mycobot_pro_630.urdf':arm_receipt['source_urdf_sha256']}))
            entity['tags']=list(dict.fromkeys([*entity['tags'],'mobile_manipulator','arm','mycobot_pro630']))
            entity['capabilities']=list(dict.fromkeys([*entity['capabilities'],'manipulation']))
        write_json(evidence,receipt)
        entity['source']['license']='BSD-3-Clause vendor base; MIT FUJI wheel geometry; declared lab derivative'+(
            '; BSD-2-Clause myCobot Pro630 arm' if with_arm else '')
    for item in entity['evidence']:item['date']='2026-11-12'
    checkpoint_robots(store,{name:profile},[entity],[dict(source_id=source['id'],revision=source['revision'],
        entry=entry+('/physical-rollers+pro630' if with_arm else '/physical-rollers' if portable else ''),status='installed',check=str(evidence),profiles=[dict(id=name,kind='robot',
            support='Experimental physical-roller mobile manipulator' if with_arm else 'Experimental physical-roller native adapter' if portable else 'Experimental Fortress source-model adapter',notes=profile['notes'])])])
    return receipt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--download',action='store_true')
    parser.add_argument('--variant',choices=['fortress','rollers','pro630'],default='fortress')
    parser.add_argument('--store',type=Path,default=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))/'external_assets')
    args=parser.parse_args();store=args.store.resolve()
    store.mkdir(parents=True,exist_ok=True)
    if store.stat().st_dev==Path('/').stat().st_dev:parser.error('Use the mounted SSD')
    (store/'locks').mkdir(exist_ok=True)
    with (store/'locks/provision.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        print(json.dumps(provision(store,args.download,args.variant),indent=2))


if __name__=='__main__':main()
