"""Build source-backed TurtleBot3 control derivatives on the SSD.

The original ROBOTIS URDF/SDF files are preserved. This connects the lab's
existing controllers, not OpenCR firmware. A controller derivative is separate
from the recorded evidence needed to enable mapping and navigation modes.
"""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def turtlebot3_drive(derived, name, simulation_root):
    root = ET.parse(derived).getroot()
    variant = name.removeprefix('asset_turtlebot3_')
    if variant not in ('burger', 'waffle', 'waffle_pi'):
        raise ValueError('Unknown TurtleBot3 variant: ' + variant)
    sdf = Path(simulation_root)/'turtlebot3_gazebo/models'/('turtlebot3_'+variant)/'model.sdf'
    model = ET.parse(sdf).getroot().find('model')
    plugin = next(p for p in model.findall('plugin')
                  if p.get('name') == 'turtlebot3_diff_drive')
    wheels = (plugin.findtext('left_joint'), plugin.findtext('right_joint'))
    joints = {j.get('name'): j for j in root.findall('joint')}
    links = {link.get('name'): link for link in root.findall('link')}
    for wheel in wheels:
        if joints[wheel].get('type') != 'continuous':
            raise ValueError('Expected continuous source wheel: ' + wheel)
    radii = [float(links[joints[j].find('child').get('link')].find(
        'collision/geometry/cylinder').get('radius')) for j in wheels]
    radius = float(plugin.findtext('wheel_diameter'))/2
    if any(abs(r-radius) > 1e-6 for r in radii):
        raise ValueError('Source wheel geometry/diameter mismatch')
    centers = [list(map(float, joints[j].find('origin').get('xyz').split())) for j in wheels]
    separation = centers[0][1]-centers[1][1]
    if (separation <= 0 or abs(centers[0][0]-centers[1][0]) > 1e-6
            or abs(separation-float(plugin.findtext('wheel_separation'))) > .002):
        raise ValueError('Source wheel center/separation mismatch')
    # Waffle's source SDF rounds the track to .287; the actual URDF wheel
    # centers are .288 apart. Control follows the executed geometry.
    drive = dict(type='diff', left_wheel_joint=wheels[0], right_wheel_joint=wheels[1],
                 wheel_radius=radius, wheel_separation=separation, max_speed=.18,
                 max_accel=.3, max_angular_speed=1.2, max_angular_accel=2.,
                 mujoco_wheel_armature=.0002, mujoco_wheel_velocity_gain=.1,
                 mujoco_ramp_watchdog_stop=True)
    for wheel in wheels:
        if joints[wheel].find('limit') is not None:
            raise ValueError('Unexpected existing wheel limit')
        # Match the non-Gazebo bridges' existing 5 Nm simulation motor cap.
        # The untouched Classic plugin records 20 Nm; no firmware claim.
        ET.SubElement(joints[wheel], 'limit', effort='5', velocity='15')
    virtual_links = []
    for link in root.findall('link'):
        if link.find('inertial') is not None:
            continue
        # Bullet otherwise invents mass=1 / inertia=(1,1,1) for each frame.
        # A tiny explicit inertia keeps the free root mobile; exactly zero
        # base mass would anchor it in Bullet. Original authored inertia stays;
        # source camera mounts also omit inertia and receive this regularizer.
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass', value='0.000001')
        ET.SubElement(inertial, 'inertia', ixx='0.000000001', iyy='0.000000001',
                      izz='0.000000001', ixy='0', ixz='0', iyz='0')
        virtual_links.append(link.get('name'))

    scan = model.find("link[@name='base_scan']/sensor")
    imu = model.find("link[@name='imu_link']/sensor")
    if scan is None or scan.get('type') != 'ray' or imu is None:
        raise ValueError('Missing original TurtleBot3 scan/IMU definitions')
    scan_range = scan.find('ray/range')
    sensor_config = dict(laser_link_name='base_scan', scan_rate=5.,
        scan_samples=int(scan.findtext('ray/scan/horizontal/samples')),
        scan_range_min=float(scan_range.findtext('min')),
        scan_range_max=float(scan_range.findtext('max')), camera_rate=0.)
    # The pinned Humble SDF has RGB-only cameras, including Waffle. Keep
    # camera geometry in the URDF, but do not invent a depth stream or allow
    # 3D SLAM through the lab's RGB-D-only renderer. RGB integration is separate.
    for frame, original in [('base_scan', scan), ('imu_link', imu)]:
        if frame not in links:
            raise ValueError('Original mounted sensor link missing: ' + frame)
        sensor = copy.deepcopy(original)
        for child in list(sensor):
            if child.tag in ('plugin', 'pose'):
                sensor.remove(child)
        # Source Classic sensor poses are model-relative; the executed URDF
        # already contains the complete mount. Attach at its local origin.
        sensor.find('update_rate').text = '5' if frame == 'base_scan' else '50'
        sensor.set('type', 'gpu_lidar' if frame == 'base_scan' else 'imu')
        if frame == 'base_scan':
            sensor.find('ray').tag = 'lidar'
        ET.SubElement(sensor, 'topic').text = '/scan' if frame == 'base_scan' else '/imu'
        ET.SubElement(sensor, 'gz_frame_id').text = frame
        ET.SubElement(root, 'gazebo', reference=frame).append(sensor)
        parent_joint = next(j.get('name') for j in root.findall('joint')
                            if j.find('child').get('link') == frame)
        block = ET.SubElement(root, 'gazebo', reference=parent_joint)
        ET.SubElement(block, 'preserveFixedJoint').text = 'true'
    from robot_lab_utils.sensor_config import sensor_parameters
    sensor_config = sensor_parameters(sensor_config, ET.tostring(root, encoding='unicode'))

    params = dict(use_sim_time=True, use_stamped_vel=False,
        left_wheel_names=[wheels[0]], right_wheel_names=[wheels[1]],
        wheel_radius=radius, wheel_separation=separation,
        base_frame_id='base_footprint', odom_frame_id='odom',
        enable_odom_tf=False, open_loop=False, cmd_vel_timeout=.5,
        publish_rate=50., publish_limited_velocity=True)
    for axis, key, speed, accel in [('linear','x','max_speed','max_accel'),
                                  ('angular','z','max_angular_speed','max_angular_accel')]:
        params[axis] = {key: dict(has_velocity_limits=True, max_velocity=drive[speed],
                                  has_acceleration_limits=True, max_acceleration=drive[accel])}
    config = {'controller_manager': {'ros__parameters': {'update_rate':100, 'use_sim_time':True,
        'robot_lab_controller': {'type':'diff_drive_controller/DiffDriveController'},
        'joint_state_broadcaster': {'type':'joint_state_broadcaster/JointStateBroadcaster'}}},
        'joint_state_broadcaster': {'ros__parameters': {'use_sim_time':True}},
        'robot_lab_controller': {'ros__parameters':params}}
    config_path = derived.parent/'drive-controllers.yaml'
    config_path.write_text(yaml.safe_dump(config, sort_keys=False))
    control = ET.SubElement(root, 'ros2_control', name='TurtleBot3System', type='system')
    ET.SubElement(ET.SubElement(control, 'hardware'), 'plugin').text = 'ign_ros2_control/IgnitionSystem'
    for wheel in wheels:
        joint = ET.SubElement(control, 'joint', name=wheel)
        command = ET.SubElement(joint, 'command_interface', name='velocity')
        ET.SubElement(command, 'param', name='min').text = '-15'
        ET.SubElement(command, 'param', name='max').text = '15'
        for interface in ('position', 'velocity'):
            ET.SubElement(joint, 'state_interface', name=interface)
    gazebo = ET.SubElement(root, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin', filename='ign_ros2_control-system',
                           name='ign_ros2_control::IgnitionROS2ControlPlugin')
    ET.SubElement(plugin, 'parameters').text = str(config_path)
    ET.SubElement(gazebo, 'plugin', filename='ignition-gazebo-imu-system',
                  name='ignition::gazebo::systems::Imu')
    sensor_system = ET.SubElement(gazebo, 'plugin', filename='ignition-gazebo-sensors-system',
                                  name='ignition::gazebo::systems::Sensors')
    ET.SubElement(sensor_system, 'render_engine').text = 'ogre2'
    truth = ET.SubElement(gazebo, 'plugin', filename='ignition-gazebo-odometry-publisher-system',
                          name='ignition::gazebo::systems::OdometryPublisher')
    for key, value in {'odom_frame':'world', 'robot_base_frame':'base_footprint',
                      'odom_topic':'/model/'+name+'/odometry_truth',
                      'tf_topic':'/model/'+name+'/truth_pose',
                      'odom_publish_frequency':'50', 'dimensions':'3'}.items():
        ET.SubElement(truth, key).text = value
    for link in links:
        if 'wheel_' not in link and 'caster_' not in link:
            continue
        block = ET.SubElement(root, 'gazebo', reference=link)
        for key in ('mu1', 'mu2'):
            ET.SubElement(block, key).text = '0.1' if 'caster_' in link else '1.0'
    path = derived.parent/'drive.urdf'
    ET.ElementTree(root).write(path, encoding='unicode', xml_declaration=True)
    return path, drive, config_path, sensor_config, dict(
        source_sdf=str(sdf), source_track_m=float(model.find(
            "plugin[@name='turtlebot3_diff_drive']/wheel_separation").text),
        executed_urdf_track_m=separation, mounted_sensor_geometry='original URDF frames',
        cameras='source RGB-only cameras retained as geometry; renderer disabled',
        virtual_frame_inertias=dict(links=virtual_links, mass_per_link_kg=1e-6,
                                   diagonal_per_link_kg_m2=1e-9),
        rate_note='lab lidar 5 Hz / IMU 50 Hz; original SDF unchanged')
