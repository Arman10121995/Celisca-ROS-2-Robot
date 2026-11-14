"""Official PAL TIAGo PMB2/pal-gripper contracts and declared lab adapters.

Source kinematics, collision meshes, inertials, independent fingers and fixed
public-simulation casters stay intact. Position-servo gains and reduced rates
are lab simulation choices; they are not PAL hardware tuning.
"""
import math
import xml.etree.ElementTree as ET

from .sensor_config import sensor_parameters

WHEELS = ('wheel_left_joint', 'wheel_right_joint')
POSITION_JOINTS = ('torso_lift_joint', *(f'arm_{i}_joint' for i in range(1, 8)),
                   'gripper_left_finger_joint', 'gripper_right_finger_joint',
                   'head_1_joint', 'head_2_joint')
# Last point of the official tiago_motions_general.yaml home motion. This is
# an explicit GUI Home target, never an automatic runtime posture overwrite.
HOME = dict(zip(POSITION_JOINTS, (.15, .50, -1.34, -.48, 1.94, -1.49, 1.37, 0.,
                                  .04, .04, 0., 0.)))


def contracts(root):
    """Check the source interface, derive position limits and bounded servos."""
    joints = {j.get('name'): j for j in root.findall('joint')}
    controls = root.findall('ros2_control/joint')
    commanded = {j.get('name'): [c.get('name') for c in j.findall('command_interface')]
                 for j in controls if j.findall('command_interface')}
    expected = {**{n: ['velocity'] for n in WHEELS},
                **{n: ['position'] for n in POSITION_JOINTS}}
    if commanded != expected:
        raise ValueError('Unexpected official TIAGo command interfaces')
    for name, side in zip(WHEELS, (1, -1)):
        j = joints[name]
        xyz = [float(v) for v in j.find('origin').get('xyz').split()]
        rpy = [float(v) for v in j.find('origin').get('rpy').split()]
        child = j.find('child').get('link')
        cylinder = root.find(f"link[@name='{child}']/collision/geometry/cylinder")
        if (j.get('type') != 'continuous' or xyz != [0., side*.2022, 0.]
                or not math.isclose(rpy[0], -math.pi/2) or rpy[1:] != [0., 0.]
                or j.find('axis').get('xyz') != '0 0 1'
                or cylinder is None or not math.isclose(float(cylinder.get('radius')), .0985)
                or float(j.find('limit').get('effort')) != 6.):
            raise ValueError('Unexpected official TIAGo wheel geometry/limits')
    articulation = dict(schema='robot_lab.urdf_articulation.v1', panel='arm',
                        home=dict(HOME), joints={}, couplings={},
                        isaac_native_gravity_compensation=True)
    for name in POSITION_JOINTS:
        joint = joints[name]
        if joint.find('mimic') is not None:
            raise ValueError('TIAGo fingers/arm must remain independent')
        limit = joint.find('limit')
        low, high, effort, rate = (float(limit.get(k)) for k in ('lower', 'upper', 'effort', 'velocity'))
        if not low <= HOME[name] <= high or not 0 < effort or not 0 < rate:
            raise ValueError('Invalid TIAGo source envelope: '+name)
        # Bounded low-speed lab servos with plant gravity feedforward. Original
        # source gains are ideal position interfaces, not a physical PD law.
        if name == 'torso_lift_joint':
            kp, kd, cap = 15000., 1800., .04
        elif name.startswith('gripper_'):
            kp, kd, cap = 800., 20., .025
        else:
            kp, kd, cap = 40.*effort, 8.*math.sqrt(effort), .35
        articulation['joints'][name] = dict(mode='position', lower=low, upper=high,
            effort=effort, max_velocity=min(rate, cap), kp=kp, kd=kd)
    mass = sum(float(m.get('value')) for m in root.findall('link/inertial/mass'))
    drive = dict(type='diff', left_wheel_joint=WHEELS[0], right_wheel_joint=WHEELS[1],
        wheel_radius=.0985, wheel_separation=.4044, max_speed=.2, max_accel=.2,
        max_angular_speed=.4, max_angular_accel=.4, wheel_force_limit=6.,
        mujoco_wheel_velocity_gain=10., mujoco_wheel_armature=.005,
        mujoco_implicit_wheel_control=True, mujoco_preserve_fixed_root_inertia=True,
        mujoco_expected_mass_kg=mass, pybullet_explicit_massless_frames=True,
        pybullet_preserve_passive_springs=True, pybullet_expected_mass_kg=mass+1e-6,
        isaac_source_friction=True)
    return drive, articulation


def portable_description(root):
    """Remove source runtime systems, retaining mechanics/contact properties.

    Our three portable backends already supply joint/sensor publishers. The
    vendor's modern Gazebo plugins cannot be loaded in Fortress. Keep Gazebo
    unavailable for this variant until its separate source port exists.
    """
    for control in root.findall('ros2_control'):
        root.remove(control)
    for block in root.findall('gazebo'):
        for item in list(block):
            if item.tag in ('plugin', 'sensor'):
                block.remove(item)
        if not len(block):
            root.remove(block)
    # A declared panoramic lab lidar at the source laser frame. Source SICK
    # field of view/sonars/wrist FT are not claimed by the generic lab kit.
    camera_frame = 'head_front_camera_color_frame'
    sensor = ET.SubElement(ET.SubElement(root, 'gazebo', reference=camera_frame),
                           'sensor', name='rgbd_camera', type='rgbd_camera')
    ET.SubElement(sensor, 'update_rate').text = '5'
    camera = ET.SubElement(sensor, 'camera')
    ET.SubElement(camera, 'horizontal_fov').text = '1.0995574287564276'
    image = ET.SubElement(camera, 'image')
    for key, value in dict(width='320', height='240', format='R8G8B8').items():
        ET.SubElement(image, key).text = value
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '.3'
    ET.SubElement(clip, 'far').text = '8'
    return sensor_parameters(dict(laser_link_name='base_laser_link',
        scan_samples=640, scan_rate=5., scan_range_min=.1, scan_range_max=15.,
        camera_link_name=camera_frame, camera_optical_frame='head_front_camera_color_optical_frame',
        camera_width=320, camera_height=240, camera_rate=5.,
        camera_horizontal_fov=1.0995574287564276, camera_near=.3, camera_far=8.,
        imu_link_name='base_footprint'), ET.tostring(root, encoding='unicode'))
