#!/usr/bin/env python3
"""Install reviewed open-source counterparts into the normal robot catalog.

Existing source profiles/qualification are preserved. New variants explicitly
separate implemented experimental workflows from measured runtime support.
"""
import argparse
import ast
import copy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
import subprocess
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.vacuum_sensors import create3_hazard_derivative
from provision_extension_assets import checkpoint_robots, digest, load_source, robot_entity, write_json, source_package_prefix
from catalog_external_assets import checkout


def octopod(store, download=False):
    """Install the MIT segmented source plant; only its scene/start pose changes."""
    import math
    import mujoco
    import numpy as np
    from robot_lab_utils.native_mjcf_assets import export_display_urdf
    source = load_source('light_robotics_octopod')
    entry = 'robot_rl/bodies/octopod/robot.xml'
    if download:
        checkout(source, 'robot_rl/bodies/octopod', store)
    root = store/'sources'/source['id']/source['revision']
    if subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip() != source['revision']:
        raise ValueError('Unexpected octopod revision')
    if subprocess.check_output(['git', 'status', '--porcelain'], cwd=root, text=True):
        raise ValueError('Preserve edited source octopod before provisioning')
    original = root/entry
    config = original.parent/'config.py'
    # Read data, never execute the external training/JAX package to import a robot.
    constants = {}
    for node in ast.parse(config.read_text()).body:
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            try:
                constants[node.target.id] = ast.literal_eval(node.value)
            except (ValueError, TypeError):
                pass
    names = constants['JOINT_NAMES']
    nominal = [0. if n.endswith('COXA') or n.startswith('spine') else
               (.2 if n.startswith('L') else -.2) if n.endswith('FEMUR') else
               -(math.pi/2+.2) if n.startswith('L') else math.pi/2+.2 for n in names]
    source_spec = mujoco.MjSpec.from_file(str(original))
    source_model = source_spec.compile()
    # Bake settotalmass into explicit inertias before canonical serialization.
    # Keeping the compiler's global rescale would rescale the robot when a
    # selected world's massive fixed geometry is attached by the native bridge.
    for body in source_spec.bodies:
        index = mujoco.mj_name2id(source_model, mujoco.mjtObj.mjOBJ_BODY, body.name)
        if index > 0:
            body.mass = source_model.body_mass[index]
            body.inertia = source_model.body_inertia[index]
            body.ipos = source_model.body_ipos[index]
            body.iquat = source_model.body_iquat[index]
            body.explicitinertial = True
    source_spec.compiler.settotalmass = 0.
    source_spec.compiler.saveinertial = True
    document = ET.fromstring(source_spec.to_xml())
    world = document.find('worldbody')
    floor = world.find("geom[@name='floor']")
    if floor is None or floor.get('type') != 'plane':
        raise ValueError('Unexpected source floor')
    world.remove(floor)
    home = ET.SubElement(ET.SubElement(document, 'keyframe'), 'key', name='home')
    home.set('qpos', ' '.join(map(str, [0., 0., .18, 1., 0., 0., 0.]+nominal)))
    home.set('ctrl', ' '.join(map(str, nominal)))
    text = ET.tostring(document, encoding='unicode')
    name = 'asset_light_robotics_octopod'
    output = store/'models'/name/hashlib.sha256(text.encode()).hexdigest()
    output.mkdir(parents=True, exist_ok=True)
    native = output/'robot.xml'; native.write_text(text)
    model = mujoco.MjModel.from_xml_path(str(native))
    protected = ['body_mass', 'body_inertia', 'body_ipos', 'body_iquat', 'body_pos', 'body_quat',
                 'jnt_range', 'jnt_axis', 'dof_damping', 'dof_armature', 'dof_frictionloss',
                 'actuator_gainprm', 'actuator_biasprm', 'actuator_forcerange', 'actuator_gear']
    for field in protected:
        if not np.allclose(getattr(source_model, field), getattr(model, field), rtol=1e-5, atol=1e-9):
            raise ValueError('Canonical source plant changed: '+field)
    if not math.isclose(sum(model.body_mass), 7.5, rel_tol=1e-6):
        raise ValueError('Source mass differs')
    description = output/'display.urdf'
    geometry = export_display_urdf(model, description)
    (output/'LICENSE').write_bytes((root/'LICENSE').read_bytes())
    manifest = output/'locomotion.yaml'
    resources = [original, config, original.parent/'scripted_gait_octopod.py', root/'LICENSE', native]
    contract = dict(schema_version=1, controller='light_robotics_octopod', robot=name,
        repository=source['repository'], revision=source['revision'], native_model=str(native),
        joint_names=names, nominal=nominal, resources=[dict(path=str(p), sha256=digest(p)) for p in resources],
        command_timeout_s=.5, command_limits=[.15, 0., .35], scripted_prior=True,
        lab_changes=['selected world owns floor', 'source nominal pose/0.18m config start height',
                     'canonical compiled inertias preserve source mass under world attachment',
                     'NumPy flat-prior port; bounded differential side scaling and transitions',
                     'wall command lease; finite-state/fall latch; explicit reset'],
        qualification='experimental scripted source gait; no learned checkpoint or mission qualification')
    manifest.write_text(yaml.safe_dump(contract, sort_keys=False))
    profile = dict(package='robot_lab_robots', name=name, xacro=str(description), native_mjcf=str(native),
        source_id=source['id'], source_revision=source['revision'], source_entry=entry,
        external_asset=True, asset_origin='extension', type='legged', leg_count=8,
        supported_simulators=['mujoco'], supported_modes=['display'],
        supported_modes_by_simulator={'mujoco':['display']}, default_mode='display', default_map='none',
        features=['native_locomotion', 'octopod', 'segmented_body'], locomotion_policy_config=str(manifest),
        locomotion_drive_limits=dict(max_speed=.15, max_angular_speed=.35, max_accel=.2, max_angular_accel=.4),
        implementation_state='experimental', validation_state='pending', qualification_state='experimental_unqualified',
        notes='Complete pinned MIT light-robotics 7.5 kg segmented octopod: 24 leg and three pitch-spine servos. '
              'Original native position actuators/gains, inertia, contacts and 2.45 Nm force caps. '
              'Enable Native locomotion on MuJoCo Display for scripted flat gait and Drive/WASD. '
              'Velocity tracking is uncalibrated. No learned policy, lateral gait, terrain, SLAM or navigation support claim.')
    evidence = output/'integration.json'
    receipt = dict(source_id=source['id'], revision=source['revision'], license='MIT',
        native_model=str(native), native_sha256=digest(native), source_sha256=digest(original),
        geometry=geometry, mass_kg=float(sum(model.body_mass)), actuator_count=model.nu,
        force_ranges_Nm=model.actuator_forcerange.tolist(), protected_compiled_fields=protected,
        contract=str(manifest), runtime_mission_qualified=False, lab_changes=contract['lab_changes'])
    write_json(evidence, receipt)
    entity = robot_entity(name, profile, source, entry, evidence)
    entity.update(robot_class='legged', tags=['extension', 'octopod', 'eight_legged', 'segmented_body', 'complete_robot', 'experimental_control'])
    entity['command_interfaces'] = ['geometry_msgs/Twist']
    checkpoint_robots(store, {name:profile}, [entity], [dict(source_id=source['id'], revision=source['revision'],
        entry=entry, status='installed', check=str(evidence), profiles=[dict(id=name, kind='robot',
            support='MuJoCo Display; opt-in experimental scripted locomotion', notes=profile['notes'])])])
    return receipt


def marine_sensor_kit(root, position):
    """Declared massless lab RGB-D/ENU IMU kit; does not alter source forces."""
    for name,parent,xyz,rpy in [('oakd_rgb_camera_frame','base_link',position,'0 0 0'),
            ('oakd_rgb_camera_optical_frame','oakd_rgb_camera_frame','0 0 0','-1.5707963267948966 0 -1.5707963267948966')]:
        ET.SubElement(root,'link',name=name)
        joint=ET.SubElement(root,'joint',name=name+'_mount',type='fixed')
        ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=name)
        ET.SubElement(joint,'origin',xyz=xyz,rpy=rpy)
    # Massless TF links are intentionally lumped; attach the physical sensor
    # directly to the massive source hull rather than preserving a massless link.
    block=ET.SubElement(root,'gazebo',reference='base_link')
    sensor=ET.SubElement(block,'sensor',name='rgbd_camera',type='rgbd_camera')
    ET.SubElement(sensor,'pose').text=position+' 0 0 0'
    for key,text in dict(always_on='true',update_rate='5',gz_frame_id='oakd_rgb_camera_optical_frame').items():
        ET.SubElement(sensor,key).text=text
    camera=ET.SubElement(sensor,'camera')
    ET.SubElement(camera,'horizontal_fov').text='1.25'
    image=ET.SubElement(camera,'image')
    for key,text in dict(width='320',height='240',format='R8G8B8').items():ET.SubElement(image,key).text=text
    for element in (ET.SubElement(camera,'clip'),ET.SubElement(ET.SubElement(camera,'depth_camera'),'clip')):
        ET.SubElement(element,'near').text='.1';ET.SubElement(element,'far').text='8'
    sensor=ET.SubElement(ET.SubElement(root,'gazebo',reference='base_link'),'sensor',name='marine_imu',type='imu')
    for key,text in dict(always_on='true',update_rate='50',topic='/marine/imu',gz_frame_id='base_link').items():
        ET.SubElement(sensor,key).text=text
    for kind,short,values in [
        ('OdometryPublisher','odometry-publisher',dict(odom_frame='map',robot_base_frame='base_link',
            odom_topic='/marine/odometry',tf_topic='/marine/truth_pose',odom_publish_frequency='50',dimensions='3')),
        ('JointStatePublisher','joint-state-publisher',dict(topic='/marine/joint_states'))]:
        plugin=ET.SubElement(ET.SubElement(root,'gazebo'),'plugin',
            name='ignition::gazebo::systems::'+kind,filename='ignition-gazebo-'+short+'-system')
        for key,text in values.items():ET.SubElement(plugin,key).text=text


def wamv(store, download=False):
    """WAM-V H source chassis/propellers and actual upstream water equations."""
    from build_vrx_fortress import build
    source=load_source('vrx_vendor')
    root=store/'sources'/source['id']/source['revision']
    if download:checkout(source,'vrx_urdf',store)
    libraries=build(store,download)
    env=source_package_prefix([root/'vrx_urdf'],store/'prefixes/vrx')
    entry='vrx_urdf/wamv_gazebo/urdf/wamv_gazebo.urdf.xacro'
    expanded=subprocess.check_output(['xacro',str(root/entry),'namespace:=wamv','thruster_config:=H'],
                                     text=True,env=env,timeout=60)
    document=ET.fromstring(expanded)
    name='asset_vrx_wamv_h'
    document.set('name',name)
    # The lab's frame convention uses base_link, without an embedded slash.
    for node in document.iter():
        for key,text in node.attrib.items():node.set(key,text.replace('wamv/',''))
        if node.text:node.text=node.text.replace('wamv/','')
    for mesh in document.findall('.//mesh'):
        uri=mesh.get('filename')
        if uri.startswith(('package://','model://')):
            relative=uri.split('://',1)[1]
            path=root/'vrx_urdf'/relative
            if not path.is_file():raise ValueError('Missing source mesh '+str(path))
            mesh.set('filename',str(path))
    for node in document.iter():
        if node.tag.endswith('_map') and node.text and node.text.startswith('model://'):
            resource=root/'vrx_urdf'/node.text.removeprefix('model://')
            if not resource.is_file():raise ValueError('Missing source material '+str(resource))
            node.text=str(resource)
    for block in document.findall('gazebo'):
        for plugin in block.findall('plugin'):
            kind=plugin.get('name','').split('::')[-1]
            if kind in libraries['libraries']:
                plugin.set('filename',libraries['libraries'][kind])
                if kind=='Surface':
                    # Session launch sets these absolute world water parameters.
                    ET.SubElement(plugin,'fluid_density').text='1025'
            elif kind in ('Thruster','JointPositionController'):
                plugin.set('name','ignition::gazebo::systems::'+kind)
                plugin.set('filename','ignition-gazebo-'+('thruster' if kind=='Thruster' else 'joint-position-controller')+'-system')
                if kind=='Thruster':
                    plugin.find('namespace').text=name
                    # Explicit Fortress default reverse limit, preserving upstream max.
                    ET.SubElement(plugin,'min_thrust_cmd').text='-1000'
                else:
                    ET.SubElement(plugin,'initial_position').text='0'
            else:block.remove(plugin)
    marine_sensor_kit(document,'.75 0 1.5')
    text=ET.tostring(document,encoding='unicode')
    from robot_lab_utils.marine_model import SurfaceModel
    plant=SurfaceModel(text,1025.)
    hydro=document.find("gazebo/plugin[@name='vrx::SimpleHydrodynamics']")
    yaw_inertia=float(document.find("link[@name='base_link']/inertial/inertia").get('izz'))
    tuning=dict(surge_gain=2*plant.mass,surge_drag_linear=float(hydro.findtext('xU')),
        surge_drag_quadratic=float(hydro.findtext('xUU')),yaw_gain=2*yaw_inertia,
        yaw_drag_linear=float(hydro.findtext('nR')),yaw_drag_quadratic=float(hydro.findtext('nRR')),
        max_surge=.4,max_yaw=.1)
    identity=hashlib.sha256(text.encode()).hexdigest()
    target=store/'models'/name/identity
    target.mkdir(parents=True,exist_ok=True)
    (target/'source-expanded.urdf').write_text(expanded)
    output=target/'marine.urdf';output.write_text(text)
    profile=dict(package='robot_lab_robots',xacro=str(output),name=name,type='aquatic',
        asset_origin='extension',source_id=source['id'],source_revision=source['revision'],source_entry=entry,
        external_asset=True,marine_model=True,marine_propulsion='surface_twin_thrusters',
        marine_native_watchdog=True,marine_buoyancy='vrx_surface',marine_navigation_radius=3.3,marine_slam_camera=True,
        marine_camera_link='base_link',marine_camera_sensor='rgbd_camera',
        supported_simulators=['gazebo'],supported_modes=['display'],default_mode='display',default_map='surface_harbor',
        experimental_modes_by_simulator={'gazebo':['display','marine']},
        spawn_by_map={'surface_harbor':dict(x=-5.,y=0.,z=0.,yaw=0.)},
        qualification_state='experimental_unqualified',implementation_state='experimental',validation_state='pending',
        drive=dict(type='marine',max_speed=.4,max_accel=.4,max_angular_speed=.1,max_angular_accel=.4),
        surface_control=tuning,
        features=['marine_dynamics','surface_vessel','differential_thrust','rgbd_camera','imu'],
        notes='Pinned Apache-2.0 VRX WAM-V H source chassis, battery masses, two zero-held azimuth engines and propellers. '
              'Private Fortress ABI port retains upstream Surface/SimpleHydrodynamics equations. Lab massless RGB-D/ENU IMU kit, '
              'bounded differential-force controller and native watchdog. Marine mode requires explicit experimental opt-in; '
              'wave/current calibration and mission qualification pending.')
    receipt=dict(source_id=source['id'],revision=source['revision'],entry=entry,license='Apache-2.0',
        source_expanded_sha256=digest(target/'source-expanded.urdf'),description_sha256=digest(output),
        plugin_build=libraries,mass_kg=plant.mass,collision_volume_m3=plant.displacement,
        thrusters=plant.joints,force_bounds_N=plant.bounds.tolist(),allocation=plant.matrix.tolist(),surface_control=tuning,
        lab_changes=['frame namespace stripping','Fortress ABI port','explicit default -1000N reverse limit',
                     'zero azimuth hold','massless RGB-D/ENU IMU kit','1025 kg/m3 water environment'],runtime_mission_qualified=False)
    evidence=target/'integration.json';write_json(evidence,receipt)
    entity=robot_entity(name,profile,source,entry,evidence)
    entity.update(robot_class='aquatic',capabilities=['display'],tags=['extension','vrx','surface_vessel','complete_robot','experimental_control'])
    checkpoint_robots(store,{name:profile},[entity],[dict(source_id=source['id'],revision=source['revision'],
        entry=entry,status='installed',check=str(evidence),profiles=[dict(id=name,kind='robot',support='Experimental native Fortress marine control',notes=profile['notes'])])])
    return receipt


def bluerov2_heavy(store,download=False):
    from robot_lab_utils.bluerov_source import heavy_description
    from robot_lab_utils.marine_model import MarineModel
    source=load_source('bluerov2_vendor')
    entry='models/bluerov2_heavy/model.sdf'
    if download:checkout(source,'models/bluerov2_heavy',store)
    root=store/'sources'/source['id']/source['revision']
    if subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()!=source['revision']:
        raise ValueError('Unexpected BlueROV2 source revision')
    if subprocess.check_output(['git','status','--porcelain'],cwd=root,text=True):
        raise ValueError('Preserve edited BlueROV2 source before provisioning')
    name='asset_bluerov2_heavy'
    document,contract=heavy_description(root/entry,name)
    marine_sensor_kit(document,'.25 0 .13')
    text=ET.tostring(document,encoding='unicode');plant=MarineModel(text)
    target=store/'models'/name/hashlib.sha256(text.encode()).hexdigest()
    target.mkdir(parents=True,exist_ok=True)
    output=target/'marine.urdf';output.write_text(text)
    profile=dict(package='robot_lab_robots',xacro=str(output),name=name,type='aquatic',external_asset=True,
        asset_origin='extension',source_id=source['id'],source_revision=source['revision'],source_entry=entry,
        marine_model=True,marine_propulsion='eight_thrusters',marine_native_watchdog=True,
        marine_navigation_radius=.6,marine_slam_camera=True,marine_camera_link='base_link',marine_camera_sensor='rgbd_camera',
        supported_simulators=['gazebo'],supported_modes=['display'],default_mode='display',default_map='marine_tank',
        experimental_modes_by_simulator={'gazebo':['display','marine']},
        implementation_state='experimental',qualification_state='experimental_unqualified',validation_state='pending',
        source_license_review_required=True,
        drive=dict(type='marine',max_speed=.35,max_accel=.4,max_angular_speed=.35,max_angular_accel=.5),
        features=['marine_dynamics','underwater','eight_thrusters','rgbd_camera','imu'],
        notes='Pinned BlueROV2 Heavy source hull, eight propellers, original mesh scales/inertias/collision volume and hydrodynamics. '
              'Fortress native-force derivative with lab allocator, ±50 N source normalized-control envelope, native watchdog and '
              'massless RGB-D/ENU IMU kit. Original FRD IMU retained separately; ArduSub firmware is not run. '
              'Marine control requires experimental opt-in. Package declares MIT, root license/third-party mesh redistribution review '
              'remains pending; assets stay in the pinned local SSD cache. No mission qualification.')
    receipt=dict(source_id=source['id'],revision=source['revision'],entry=entry,source_sdf_sha256=digest(root/entry),
        description_sha256=digest(output),mass_kg=plant.mass,displaced_volume_m3=plant.displacement,
        allocation=plant.matrix.tolist(),force_bounds_N=plant.bounds.tolist(),source_contract=contract,
        source_mesh_sha256={uri:digest(Path(path)) for uri,path in contract['meshes'].items()},
        license_note=source['license_note'],runtime_mission_qualified=False,
        lab_changes=['SDF joint/link-frame-preserving URDF derivative','Fortress built-in system ABI names',
                     'existing lab allocator replaces ArduSub','native force timeout','massless ENU IMU/RGB-D kit'])
    evidence=target/'integration.json';write_json(evidence,receipt)
    entity=robot_entity(name,profile,source,entry,evidence)
    entity.update(robot_class='aquatic',capabilities=['display'],tags=['extension','bluerov2','underwater','complete_robot','experimental_control'])
    checkpoint_robots(store,{name:profile},[entity],[dict(source_id=source['id'],revision=source['revision'],
        entry=entry,status='installed_candidate',check=str(evidence),profiles=[dict(id=name,kind='robot',
        support='Experimental Fortress marine candidate; license review and missions pending',notes=profile['notes'])])])
    return receipt


def summit_steel(store,download=False):
    """Complete Robotnik source base with a declared lab sensing/control adapter."""
    from robot_lab_utils.source_ground import ground_sensor_kit, summit_fortress_contacts
    from robot_lab_utils.drive_kinematics import drive_from_config
    from provision_extension_assets import write_yaml
    source=load_source('robotnik_description_vendor')
    if download:
        for entry in ('urdf','meshes/bases/rbsummit_steel','meshes/wheels','simulators/gazebo_ignition/rbsummit_steel'):
            checkout(source,entry,store)
    root=store/'sources'/source['id']/source['revision']
    if subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()!=source['revision']:
        raise ValueError('Unexpected Robotnik source revision')
    if subprocess.check_output(['git','status','--porcelain'],cwd=root,text=True):
        raise ValueError('Preserve edited Robotnik source before provisioning')
    env=source_package_prefix([root],store/'prefixes/robotnik')
    recipes=store/'recipes';recipes.mkdir(exist_ok=True)
    wrapper=recipes/'summit-steel-base.xacro'
    wrapper.write_text('''<robot name="summit_steel" xmlns:xacro="http://www.ros.org/wiki/xacro">
<xacro:arg name="gazebo_ignition" default="true"/><xacro:arg name="prefix" default=""/>
<xacro:include filename="$(find robotnik_description)/urdf/bases/rbsummit_steel/rbsummit_steel_base.xacro"/>
<xacro:rbsummit_steel prefix="" namespace="" gazebo_ignition="true"/>
</robot>''')
    expanded=subprocess.check_output(['xacro',str(wrapper)],env=env,text=True,timeout=60)
    document=ET.fromstring(expanded)
    ET.register_namespace('gz','http://gazebosim.org/schema')
    name='asset_robotnik_summit_steel';document.set('name',name)
    contact_frames=summit_fortress_contacts(document)
    resources={}
    for node in document.findall('.//mesh'):
        uri=node.get('filename','')
        if not uri.startswith('package://robotnik_description/'):
            raise ValueError('Unreviewed source mesh: '+uri)
        path=(root/uri.removeprefix('package://robotnik_description/')).resolve()
        if not path.is_relative_to(root.resolve()) or not path.is_file():raise ValueError('Missing source mesh '+uri)
        node.set('filename',str(path));resources[str(path)]=digest(path)
    sensors=ground_sensor_kit(document)
    drive=dict(type='mecanum',front_left_wheel_joint='front_left_wheel_joint',
        front_right_wheel_joint='front_right_wheel_joint',rear_left_wheel_joint='back_left_wheel_joint',
        rear_right_wheel_joint='back_right_wheel_joint',wheel_radius=.127,wheelbase=.445,wheel_separation=.409,
        roller_angle=0.7853981633974483,max_speed=.3,max_accel=.4,max_angular_speed=.4,max_angular_accel=.6)
    # The complete body/collision/inertia and source interface bounds remain.
    control=document.find('ros2_control')
    control.find('hardware/plugin').text='ign_ros2_control/IgnitionSystem'
    original_bounds={j.get('name'):{p.get('name'):float(p.text) for p in j.findall('command_interface/param')}
                     for j in control.findall('joint')}
    if set(original_bounds)!=set(drive_from_config(drive).wheel_joints):
        raise ValueError('Expected four source mecanum wheel velocity interfaces')
    identity=hashlib.sha256(ET.tostring(document)+json.dumps(drive,sort_keys=True).encode()).hexdigest()
    target=store/'models'/name/identity;target.mkdir(parents=True,exist_ok=True)
    config_path=target/'controllers.yaml'
    config=dict(controller_manager=dict(ros__parameters=dict(update_rate=100,use_sim_time=True,
        joint_state_broadcaster=dict(type='joint_state_broadcaster/JointStateBroadcaster'),
        mecanum_wheel_controller=dict(type='velocity_controllers/JointGroupVelocityController'))),
        joint_state_broadcaster=dict(ros__parameters=dict(use_sim_time=True)),
        mecanum_wheel_controller=dict(ros__parameters=dict(use_sim_time=True,joints=drive_from_config(drive).wheel_joints)))
    write_yaml(config_path,config)
    plugin=document.find("gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
    if plugin is None:raise ValueError('Expected the source ros2_control plugin')
    plugin.set('filename','ign_ros2_control-system');plugin.set('name','ign_ros2_control::IgnitionROS2ControlPlugin')
    plugin.find('parameters').text=str(config_path)
    ros=plugin.find('ros')
    if ros is not None:plugin.remove(ros)
    truth=ET.SubElement(ET.SubElement(document,'gazebo'),'plugin',name='ignition::gazebo::systems::OdometryPublisher',
                        filename='ignition-gazebo-odometry-publisher-system')
    for key,text in dict(odom_frame='world',robot_base_frame='base_footprint',
        odom_topic='/model/'+name+'/odometry_truth',tf_topic='/model/'+name+'/truth_pose',
        odom_publish_frequency='50',dimensions='3').items():ET.SubElement(truth,key).text=text
    output=target/'drive.urdf';output.write_text(ET.tostring(document,encoding='unicode'))
    (target/'source-expanded.urdf').write_text(expanded)
    profile=dict(package='robot_lab_robots',xacro=str(output),name=name,type='mobile',source_id=source['id'],
        source_revision=source['revision'],source_entry='urdf/bases/rbsummit_steel/rbsummit_steel_base.xacro',
        asset_origin='extension',drive=drive,sensor_config=sensors,drive_in_display=True,
        controllers=['joint_state_broadcaster','mecanum_wheel_controller'],
        controllers_yaml=str(config_path),supported_simulators=['gazebo'],supported_modes=['display'],
        experimental_modes_by_simulator={'gazebo':['display','loc','slam','3d_slam','nav']},
        default_mode='display',default_map='nav_empty',
        features=['ros2_control','velocity_base','holonomic_base','lidar_2d','rgbd_camera','imu'],
        default_algorithms={'control':'native_holonomic_controller'},
        qualification_state='experimental_unqualified',implementation_state='experimental',validation_state='pending',
        notes='Complete BSD-3-Clause Robotnik RB-SUMMIT-STEEL source chassis and four mecanum wheels. '
              'Original mass, inertia, sphere contacts, anisotropic friction and ±10/20 rad/s source interfaces retained. '
              'Fortress ros2_control ABI port, existing bounded holonomic controller and declared massless lab LiDAR/IMU/RGB-D kit. '
              'Gazebo/nav_empty Display forward, reverse, strafe, turn, neutral and publisher-loss motion measured; '
              'higher modes require experimental opt-in. Source contact fidelity, other maps/backends and full missions remain unqualified.')
    receipt=dict(source_id=source['id'],revision=source['revision'],license='BSD-3-Clause',
        source_expanded_sha256=digest(target/'source-expanded.urdf'),description_sha256=digest(output),
        controller_yaml_sha256=digest(config_path),source_wheel_bounds_rad_s=original_bounds,contact_frame_port=contact_frames,
        mass_kg=sum(float(m.get('value')) for m in document.findall('link/inertial/mass')),
        source_resource_sha256=resources,sensor_config=sensors,drive=drive,
        lab_changes=['base macro assembled without optional vendor sensors','Fortress ros2_control ABI and root namespace',
                     'Fortress friction-frame namespace; original source body frame/directions preserved',
                     'bounded lab holonomic controller','massless lab sensing kit'],runtime_mission_qualified=False)
    evidence=target/'integration.json';write_json(evidence,receipt)
    entity=robot_entity(name,profile,source,profile['source_entry'],evidence)
    entity.update(robot_class='mobile',tags=['extension','robotnik','four_wheels','mecanum','complete_robot','experimental_control'])
    entity['locomotion'].update(type='mecanum',dof=4)
    entity['capabilities']=sorted(set(entity['capabilities'])|{'holonomic_base'})
    entity['frames']=list(dict.fromkeys(['base_footprint',*entity['frames']]))
    checkpoint_robots(store,{name:profile},[entity],[dict(source_id=source['id'],revision=source['revision'],
        entry=profile['source_entry'],status='installed_candidate',check=str(evidence),profiles=[dict(id=name,kind='robot',
        support='Experimental native Fortress mecanum base',notes=profile['notes'])])])
    return receipt


def turtlebot4_cleaning(store):
    original_id, name = 'asset_turtlebot4_lite', 'asset_turtlebot4_lite_cleaning'
    installed = store / 'installed'
    profiles = yaml.safe_load((installed / 'robots.yaml').read_text())['robots']
    base = profiles[original_id]
    source = load_source('turtlebot4_vendor')
    check_path = installed / 'checks' / ('turtlebot4_vendor-' + hashlib.sha256(
        base['source_entry'].encode()).hexdigest()[:16] + '.json')
    check = json.loads(check_path.read_text())
    description = Path(base['xacro'])
    upstream = description.parent / 'input.urdf'
    if (check.get('revision') != source['revision'] or digest(description) != check['drive_urdf_sha256']
            or digest(upstream) != check['model_sha256']):
        raise ValueError('Original TurtleBot source/drive bytes differ from their installed import receipt')
    text, sensors = create3_hazard_derivative(description.read_text(), upstream.read_text(), name)
    identity = hashlib.sha256((text + '\ncreate3-hazards-v1').encode()).hexdigest()
    target = store / 'models' / name / identity
    target.mkdir(parents=True, exist_ok=True)
    output = target / 'cleaning.urdf'
    if output.exists() and output.read_text() != text:
        raise ValueError('A content-addressed cleaning derivative was changed')
    output.write_text(text)
    profile = copy.deepcopy(base)
    for key in ('runtime_screens', 'qualification_recheck', 'qualification_state', 'default_algorithms',
                'supported_modes_by_simulator', 'experimental_modes_by_simulator'):
        profile.pop(key, None)
    profile.update(name=name, xacro=str(output), supported_simulators=['gazebo'],
        supported_modes=['display'], default_mode='display', default_map='nav_empty',
        supported_modes_by_simulator={'gazebo': ['display']},
        experimental_modes_by_simulator={'gazebo': ['display', 'loc', 'slam', '3d_slam', 'nav']},
        qualification_state='experimental_unqualified', supports_room_vacuum=True,
        features=['velocity_base', 'lidar_2d', 'rgbd_camera', 'bumper', 'cliff'],
        gazebo_max_physics_step_s=.002,
        vacuum=dict(radius=.171, cleaning_width=.24, side_brush_reach=0., max_speed=.3,
            base_frame='base_link', bumper_topic='/bumper',
            cliff_topics=['/cliff/front_left', '/cliff/front_right', '/cliff/side_left', '/cliff/side_right'],
            cliff_absolute_threshold_m=.08, require_cliff_feedback=True),
        notes='Source TurtleBot 4 Lite / Create 3 cleaning variant: unchanged chassis, wheels, suspension, '
              'controller and sensor geometry; source bumper and four cliff sensors restored at 20 Hz. '
              'Cleaning coverage/battery are lab models; there is no vacuum hardware. Gazebo Display drive '
              'implemented, higher modes require explicit experimental opt-in; new runtime acceptance pending.')
    evidence = target / 'integration.json'
    receipt = dict(recipe='create3-hazards-v1', source_id=source['id'], revision=source['revision'],
        source_drive=str(description), source_drive_sha256=digest(description),
        source_expanded_description=str(upstream), source_expanded_sha256=digest(upstream),
        description=str(output), description_sha256=digest(output), restored_sensors=sensors,
        source_dependencies=check.get('source_dependencies', []), runtime_mission_qualified=False,
        scope='Source/native hazard integration; original profile and its certificates unchanged')
    write_json(evidence, receipt)
    entity = robot_entity(name, profile, source, base['source_entry'], evidence)
    record = dict(source_id=source['id'], revision=source['revision'],
        entry='robot-lab/Create3-cleaning-variant-v1', status='installed', check=str(evidence),
        profiles=[dict(id=name, kind='robot', support='Gazebo Display; explicit experimental cleaning modes',
                       notes=profile['notes'])])
    checkpoint_robots(store, {name: profile}, [entity], [record])
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', choices=['turtlebot4_cleaning','wamv','bluerov2_heavy','summit_steel','octopod'], default='turtlebot4_cleaning')
    parser.add_argument('--download',action='store_true',help='Fetch the pinned upstream source before provisioning')
    parser.add_argument('--store', type=Path, default=Path(os.environ.get(
        'ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'external_assets')
    args = parser.parse_args()
    store = args.store.resolve()
    if store.stat().st_dev == Path('/').stat().st_dev:
        parser.error('Counterpart assets must remain on the mounted SSD')
    with (store / 'locks/provision.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        result=(turtlebot4_cleaning(store) if args.robot=='turtlebot4_cleaning' else
                wamv(store,args.download) if args.robot=='wamv' else
                bluerov2_heavy(store,args.download) if args.robot=='bluerov2_heavy' else
                octopod(store,args.download) if args.robot=='octopod' else summit_steel(store,args.download))
        print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
