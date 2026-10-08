"""Physical four-wheel Husky simulation derivative with an explicit lab sensor kit.

Original vendor geometry/inertia and input URDF stay intact on SSD. The lab
kit is declared here with real sensor mounts, not inferred vendor hardware.
Control integration alone does not qualify SLAM/navigation modes.
"""
import copy
import xml.etree.ElementTree as ET

import yaml


def husky_drive(derived, name):
    root = ET.parse(derived).getroot()
    upstream = ET.parse(derived.parent/'input.urdf').getroot()
    joints = {j.get('name'): j for j in root.findall('joint')}
    links = {l.get('name'): l for l in root.findall('link')}
    left = ['front_left_wheel_joint', 'rear_left_wheel_joint']
    right = ['front_right_wheel_joint', 'rear_right_wheel_joint']
    radii, centers = [], {}
    for wheel in left+right:
        joint = joints[wheel]
        if (joint.get('type') != 'continuous' or joint.find('axis').get('xyz') != '0 1 0'
                or joint.find('parent').get('link') != 'base_link'):
            raise ValueError('Unexpected source Husky wheel topology: '+wheel)
        centers[wheel] = list(map(float, joint.find('origin').get('xyz').split()))
        radii.append(float(links[joint.find('child').get('link')].find(
            'collision/geometry/cylinder').get('radius')))
        if joint.find('limit') is not None:
            raise ValueError('Unexpected existing Husky wheel limit')
        # Lab simulation motor envelope, not a vendor hardware torque claim.
        ET.SubElement(joint, 'limit', effort='50', velocity='10')
    if max(radii)-min(radii) > 1e-8:
        raise ValueError('Husky wheels have different radii')
    track = centers[left[0]][1]-centers[right[0]][1]
    if track <= 0 or any(abs(centers[a][1]-centers[b][1]) > 1e-8 for a,b in
            ((left[0],left[1]), (right[0],right[1]))) or any(
            abs(centers[a][0]-centers[b][0]) > 1e-8 for a,b in zip(left,right)):
        raise ValueError('Unexpected source Husky axle/track geometry')
    drive = dict(type='skid_steer', left_wheel_joints=left, right_wheel_joints=right,
        left_wheel_joint=left[0], right_wheel_joint=right[0], wheel_radius=radii[0],
        wheel_separation=track, max_speed=.4, max_accel=.5,
        max_angular_speed=1., max_angular_accel=1.5, wheel_force_limit=50.,
        mujoco_wheel_velocity_gain=25.,mujoco_wheel_armature=.05)
    lateral_ratios = []
    for block in upstream.findall('gazebo'):
        if block.get('reference') in {joints[w].find('child').get('link') for w in left+right}:
            retained = copy.deepcopy(block)
            for value in retained:
                if value.get('value') is not None:
                    value.text = value.attrib.pop('value')
            rolling = float(retained.findtext('mu1'))
            lateral = float(retained.findtext('mu2'))
            if rolling <= 0 or lateral <= 0:
                raise ValueError('Invalid original Husky tire friction')
            lateral_ratios.append(lateral/rolling)
            root.append(retained)
    if len(lateral_ratios)!=4 or max(lateral_ratios)-min(lateral_ratios)>1e-8:
        raise ValueError('Husky tire friction differs between wheels')
    # Joint axes are the source wheels' local Y. Preserve the lower axial
    # friction instead of replacing both coefficients with Bullet's mu1.
    drive['pybullet_anisotropic_wheel_friction'] = [1.,lateral_ratios[0],1.]
    # Independent tangential constraints preserve source longitudinal/axial
    # coefficients during skid turns. Bullet's coupled cone caused asymmetric
    # sideways drift in the recorded native contact/GUI trials. This is a
    # declared lab solver choice; geometry, gravity and motors stay physical.
    drive['pybullet_enable_cone_friction'] = False
    # Four fixed axles scrub laterally in MuJoCo/Isaac; encoder yaw is not body yaw.
    # A bounded PI servo uses the plant's measured angular rate (the IMU rate),
    # commanding only physical wheel velocity actuators. Neutral clears its
    # integral. These are lab controller gains, not altered geometry/odometry.
    drive['skid_yaw_rate_feedback'] = dict(kp=1.,ki=4.,max_correction=3.,max_wheel_speed=10.,
        linear_kp=1.,linear_ki=2.,max_linear_correction=.5)
    virtual = []
    for link in root.findall('link'):
        if link.find('inertial') is None:
            _frame_inertia(link)
            virtual.append(link.get('name'))
    # The default source description has no enabled lidar/camera. Add a named
    # optional lab kit at explicit base-frame poses; never label it OEM sensing.
    _fixed_frame(root, 'lab_laser_link', 'base_link', '.30 0 .40')
    # Existing RGB-D bridge frame names; calibration remains this lab kit.
    _fixed_frame(root, 'oakd_rgb_camera_frame', 'base_link', '.35 0 .45')
    _fixed_frame(root, 'oakd_rgb_camera_optical_frame', 'oakd_rgb_camera_frame', '0 0 0', '-1.57079632679 0 -1.57079632679')
    if 'imu_link' not in links:
        _fixed_frame(root, 'imu_link', 'base_link', '0 0 0')
    lidar = ET.SubElement(ET.SubElement(root, 'gazebo', reference='lab_laser_link'),
        'sensor', name='lab_lidar', type='gpu_lidar')
    for key, value in dict(always_on='true', update_rate='5', topic='/scan',
                           gz_frame_id='lab_laser_link').items():
        ET.SubElement(lidar, key).text = value
    ray = ET.SubElement(lidar, 'lidar')
    horizontal = ET.SubElement(ET.SubElement(ray, 'scan'), 'horizontal')
    for key, value in dict(samples='360', resolution='1', min_angle='-3.14159265359',
                           max_angle='3.14159265359').items():
        ET.SubElement(horizontal, key).text = value
    distance = ET.SubElement(ray, 'range')
    for key, value in dict(min='.10', max='12', resolution='.01').items():
        ET.SubElement(distance, key).text = value
    rgbd = ET.SubElement(ET.SubElement(root, 'gazebo', reference='oakd_rgb_camera_frame'),
                         'sensor', name='rgbd_camera', type='rgbd_camera')
    for key, value in dict(always_on='true', update_rate='5',
                           gz_frame_id='oakd_rgb_camera_optical_frame').items():
        ET.SubElement(rgbd, key).text = value
    camera = ET.SubElement(rgbd, 'camera')
    ET.SubElement(camera, 'horizontal_fov').text = '1.0471975512'
    ET.SubElement(camera, 'optical_frame_id').text = 'oakd_rgb_camera_optical_frame'
    image = ET.SubElement(camera, 'image')
    for key, value in dict(width='320', height='240', format='R8G8B8').items():
        ET.SubElement(image, key).text = value
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '.05'
    ET.SubElement(clip, 'far').text = '12'
    imu = ET.SubElement(ET.SubElement(root, 'gazebo', reference='imu_link'),
                        'sensor', name='lab_imu', type='imu')
    for key, value in dict(always_on='true', update_rate='50', topic='/imu', gz_frame_id='imu_link').items():
        ET.SubElement(imu, key).text = value
    for frame in ('lab_laser_link','oakd_rgb_camera_frame','oakd_rgb_camera_optical_frame','imu_link'):
        joint = next(j for j in root.findall('joint') if j.find('child').get('link') == frame)
        ET.SubElement(ET.SubElement(root,'gazebo',reference=joint.get('name')), 'preserveFixedJoint').text = 'true'
    params = dict(use_sim_time=True, use_stamped_vel=False, left_wheel_names=left,
        right_wheel_names=right, wheels_per_side=2, wheel_radius=radii[0], wheel_separation=track,
        base_frame_id='base_link', odom_frame_id='odom', enable_odom_tf=False, open_loop=False,
        cmd_vel_timeout=.5, publish_rate=50., publish_limited_velocity=True)
    for axis,key,speed,accel in [('linear','x','max_speed','max_accel'),
                                ('angular','z','max_angular_speed','max_angular_accel')]:
        params[axis] = {key: dict(has_velocity_limits=True,max_velocity=drive[speed],
                                has_acceleration_limits=True,max_acceleration=drive[accel])}
    config = {'controller_manager': {'ros__parameters': dict(update_rate=100,use_sim_time=True,
        robot_lab_controller={'type':'diff_drive_controller/DiffDriveController'},
        joint_state_broadcaster={'type':'joint_state_broadcaster/JointStateBroadcaster'})},
        'joint_state_broadcaster': {'ros__parameters': {'use_sim_time':True}},
        'robot_lab_controller': {'ros__parameters':params}}
    config_path = derived.parent/'drive-controllers.yaml'
    config_path.write_text(yaml.safe_dump(config,sort_keys=False))
    control = ET.SubElement(root,'ros2_control',name='HuskyLabSystem',type='system')
    ET.SubElement(ET.SubElement(control,'hardware'),'plugin').text = 'ign_ros2_control/IgnitionSystem'
    for wheel in left+right:
        joint = ET.SubElement(control,'joint',name=wheel)
        command = ET.SubElement(joint,'command_interface',name='velocity')
        ET.SubElement(command,'param',name='min').text = '-10'
        ET.SubElement(command,'param',name='max').text = '10'
        for interface in ('position','velocity'):
            ET.SubElement(joint,'state_interface',name=interface)
    gazebo = ET.SubElement(root,'gazebo')
    plugin = ET.SubElement(gazebo,'plugin',filename='ign_ros2_control-system',
                           name='ign_ros2_control::IgnitionROS2ControlPlugin')
    ET.SubElement(plugin,'parameters').text = str(config_path)
    ET.SubElement(gazebo,'plugin',filename='ignition-gazebo-imu-system',name='ignition::gazebo::systems::Imu')
    render = ET.SubElement(gazebo,'plugin',filename='ignition-gazebo-sensors-system',name='ignition::gazebo::systems::Sensors')
    ET.SubElement(render,'render_engine').text = 'ogre2'
    truth = ET.SubElement(gazebo,'plugin',filename='ignition-gazebo-odometry-publisher-system',
                         name='ignition::gazebo::systems::OdometryPublisher')
    for key,value in dict(odom_frame='world',robot_base_frame='base_link',
        odom_topic='/model/'+name+'/odometry_truth',tf_topic='/model/'+name+'/truth_pose',
        odom_publish_frequency='50',dimensions='3').items():
        ET.SubElement(truth,key).text = value
    path = derived.parent/'drive.urdf'
    ET.ElementTree(root).write(path,encoding='unicode',xml_declaration=True)
    sensor = dict(laser_link_name='lab_laser_link',scan_rate=5.,scan_samples=360,
        scan_range_min=.1,scan_range_max=12.,camera_link_name='oakd_rgb_camera_frame',
        camera_optical_frame='oakd_rgb_camera_optical_frame',camera_rate=5.,camera_width=320,
        camera_height=240,camera_horizontal_fov=1.0471975512,camera_near=.05,camera_far=12.)
    from robot_lab_utils.sensor_config import sensor_parameters
    sensor = sensor_parameters(sensor, ET.tostring(root,encoding='unicode'))
    return path,drive,config_path,sensor,dict(wheels=left+right,wheel_centers_base_m=centers,
        lab_motor_effort_cap_nm=50.,virtual_frame_regularization=virtual,
        sensor_kit='Explicit lab simulation lidar/RGB-D/IMU mounts, not source vendor hardware',
        source_geometry_and_inertia='retained; no pose writing or base attachment')


def _frame_inertia(link):
    inertia = ET.SubElement(link,'inertial')
    ET.SubElement(inertia,'mass',value='.000001')
    ET.SubElement(inertia,'inertia',ixx='.000000001',iyy='.000000001',izz='.000000001',ixy='0',ixz='0',iyz='0')


def _fixed_frame(root,name,parent,xyz,rpy='0 0 0'):
    if root.find("link[@name='"+name+"']") is not None:
        raise ValueError('Sensor kit would overwrite an existing source frame: '+name)
    link = ET.SubElement(root,'link',name=name)
    _frame_inertia(link)
    joint = ET.SubElement(root,'joint',name=name+'_joint',type='fixed')
    ET.SubElement(joint,'parent',link=parent)
    ET.SubElement(joint,'child',link=name)
    ET.SubElement(joint,'origin',xyz=xyz,rpy=rpy)
