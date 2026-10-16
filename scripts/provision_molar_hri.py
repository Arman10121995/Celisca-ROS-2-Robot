"""Run only on molar1's native Robot Lab checkout; preserve HRI source assets."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
import yaml

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source', type=Path, default=Path('/workspace/molar_voice_super_gui_redesign/simulation/robot'))
parser.add_argument('--root', type=Path, default=Path('/workspace/molar/ros_ws/bumperbot_ws'))
args = parser.parse_args()
ROOT = args.root.resolve()
SOURCE = args.source.resolve(strict=True)
assert (ROOT / 'AGENTS.md').exists()

def save(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')

def insert_once(path, anchor, addition):
    text = path.read_text()
    if addition in text:
        return
    assert text.count(anchor) == 1, (str(path), anchor)
    save(path, text.replace(anchor, anchor + addition, 1))

articulation = {
    'schema': 'robot_lab.urdf_articulation.v1',
    'joints': {
        'conveyor_lift_joint': dict(mode='position', lower=0., upper=.68, max_velocity=.035, effort=2000., kp=20000., kd=200.),
        'telescope_stage1_joint': dict(mode='position', lower=0., upper=.14, max_velocity=.1, effort=250., kp=4000., kd=80.),
        'transfer_roller_joint': dict(mode='velocity', max_velocity=8., effort=2., kp=0., kd=.2),
    },
    'couplings': {'telescope_stage2_joint': dict(joint='telescope_stage1_joint', multiplier=1., offset=0.)},
}
profiles = {}
registry = []
for variant in ('probot_l', 'molar_hri'):
    dest = ROOT / 'src/robot_lab_robots' / variant
    orig = SOURCE / (variant + '.urdf')
    (dest / 'source').mkdir(parents=True, exist_ok=True)
    shutil.copyfile(orig, dest / 'source/robot.urdf')
    shutil.copytree(SOURCE / 'meshes' / variant, dest / 'meshes', dirs_exist_ok=True)
    robot = ET.fromstring(orig.read_text())
    robot.set('xmlns:xacro', 'http://ros.org/wiki/xacro')
    for mesh in robot.findall('.//mesh'):
        mesh.set('filename', 'package://robot_lab_robots/' + variant + '/meshes/' + Path(mesh.get('filename')).name)
    # Coupling belongs to the common owned controller in every backend.
    # Retain the exact original mimic URDF above; do not ask two controllers to own it.
    for joint in robot.findall('joint'):
        mimic = joint.find('mimic')
        if mimic is not None:
            joint.remove(mimic)
    control = ET.SubElement(robot, 'ros2_control', name='MolarSimulation', type='system')
    hw = ET.SubElement(control, 'hardware')
    ET.SubElement(hw, 'plugin').text = 'ign_ros2_control/IgnitionSystem'
    for joint in robot.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        name = joint.get('name')
        spec = ET.SubElement(control, 'joint', name=name)
        if name in ('drive_left_joint', 'drive_right_joint', 'transfer_roller_joint'):
            ET.SubElement(spec, 'command_interface', name='velocity')
        elif name in ('conveyor_lift_joint', 'telescope_stage1_joint', 'telescope_stage2_joint'):
            ET.SubElement(spec, 'command_interface', name='position')
        for interface in ('position', 'velocity', 'effort'):
            ET.SubElement(spec, 'state_interface', name=interface)
    gz = ET.SubElement(robot, 'gazebo')
    plugin = ET.SubElement(gz, 'plugin', filename='ign_ros2_control-system', name='ign_ros2_control::IgnitionROS2ControlPlugin')
    ET.SubElement(plugin, 'parameters').text = '$(find robot_lab_robots)/' + variant + '/config/controllers.yaml'
    odom = ET.SubElement(gz, 'plugin', filename='ignition-gazebo-odometry-publisher-system', name='ignition::gazebo::systems::OdometryPublisher')
    for tag, value in dict(odom_frame='world', robot_base_frame='base_link', odom_topic='/model/' + variant + '/odometry_truth', odom_publish_frequency='50', dimensions='3').items():
        ET.SubElement(odom, tag).text = value
    for link in robot.findall('link'):
        settings = ET.SubElement(robot, 'gazebo', reference=link.get('name'))
        ET.SubElement(settings, 'selfCollide').text = 'false'
    ET.indent(robot, space='  ')
    save(dest / ('urdf/' + variant + '.urdf.xacro'), '<?xml version="1.0"?>\n' + ET.tostring(robot, encoding='unicode') + '\n')
    controllers = {
        'controller_manager': {'ros__parameters': {'update_rate': 100, 'use_sim_time': True,
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            'molar_base_controller': {'type': 'diff_drive_controller/DiffDriveController'},
            'molar_position_controller': {'type': 'position_controllers/JointGroupPositionController'},
            'molar_roller_controller': {'type': 'velocity_controllers/JointGroupVelocityController'}}},
        'molar_base_controller': {'ros__parameters': dict(
            left_wheel_names=['drive_left_joint'], right_wheel_names=['drive_right_joint'],
            wheel_separation=.573, wheel_radius=.09, use_stamped_vel=False, cmd_vel_timeout=.3,
            base_frame_id='base_link', odom_frame_id='odom', enable_odom_tf=True, publish_rate=50.,
            linear={'x': {'has_velocity_limits': True, 'max_velocity': .25, 'has_acceleration_limits': True, 'max_acceleration': .3}},
            angular={'z': {'has_velocity_limits': True, 'max_velocity': .5, 'has_acceleration_limits': True, 'max_acceleration': .5}})},
        'molar_position_controller': {'ros__parameters': {'joints': ['conveyor_lift_joint', 'telescope_stage1_joint', 'telescope_stage2_joint']}},
        'molar_roller_controller': {'ros__parameters': {'joints': ['transfer_roller_joint']}},
    }
    save(dest / 'config/controllers.yaml', yaml.safe_dump(controllers, sort_keys=False))
    save(dest / 'config/articulation.json', json.dumps(articulation, indent=2) + '\n')
    manifest = {'source': str(orig), 'source_sha256': hashlib.sha256(orig.read_bytes()).hexdigest(),
                'source_repository': '/workspace/molar_voice_super_gui_redesign', 'source_commit': 'a98ff3c2bd015b46edf382e84353d70042728c5b',
                'variant': variant, 'license': 'Project-provided CAD; redistribution permission not established by this adapter',
                'changes': ['package mesh URIs', 'simulation-only Fortress ros2_control and truth odometry',
                            'manual controller coupling replaces URDF mimic in executed derivative only'],
                'dynamics': 'Estimated, not hardware identification; no hardware controller included',
                'assets': {str(p.relative_to(dest)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(dest.rglob('*.stl'))}}
    save(dest / 'provenance.json', json.dumps(manifest, indent=2) + '\n')
    profiles[variant] = dict(package='robot_lab_robots', xacro=variant + '/urdf/' + variant + '.urdf.xacro',
        name=variant, type='mobile_manipulator', supported_modes=['display'], default_mode='display',
        default_map='nav_empty', supported_simulators=['gazebo', 'mujoco', 'pybullet', 'isaac'],
        drive_in_display=True, controllers=['joint_state_broadcaster', 'molar_base_controller', 'molar_position_controller', 'molar_roller_controller'],
        features=['velocity_base', 'articulated_conveyor'], articulation=articulation,
        drive=dict(type='diff', left_wheel_joint='drive_left_joint', right_wheel_joint='drive_right_joint', wheel_radius=.09,
                   wheel_separation=.573, max_speed=.25, max_accel=.3, max_angular_speed=.5, max_angular_accel=.5,
                   mujoco_wheel_velocity_gain=20., mujoco_wheel_armature=.1),
        notes='Experimental simulation-only CAD derivative: differential drive, passive casters, lift, coupled telescope and roller. Estimated dynamics. Display/Drive controls; sensors, SLAM/Nav2 and cross-backend missions not qualified.')
    registry.append(dict(id=variant, version='0.1.0', name='MOLAR HRI' if variant == 'molar_hri' else 'ASTI ProBOT-L',
        ros_package='robot_lab_robots', status='available', robot_class='mobile_manipulator', maturity='prototype',
        supported_simulators=['gazebo', 'mujoco', 'pybullet', 'isaac'], supported_modes=['display'],
        locomotion=dict(type='differential', dof=2, max_velocity=.25), capabilities=['control'],
        command_interfaces=['geometry_msgs/Twist', 'std_msgs/String'], state_interfaces=['sensor_msgs/JointState', 'nav_msgs/Odometry'],
        frames=['odom', 'base_link', 'conveyor_carriage', 'telescope_stage1', 'telescope_stage2'],
        assets={'urdf': 'robot_lab_robots/' + variant + '/urdf/' + variant + '.urdf.xacro'},
        tags=['complete_robot', 'differential_drive', 'articulated_conveyor', 'estimated_dynamics', 'experimental_control']))

insert_once(ROOT / 'src/robot_lab_robots/config/robots.yaml', 'robots:\n', yaml.safe_dump(profiles, sort_keys=False, indent=2).replace('\n', '\n  ').join(['  ', '']).rstrip() + '\n')
path = ROOT / 'src/robot_lab/robot_lab_registry/config/robots.yaml'
current = path.read_text()
if '\n- id: molar_hri\n' not in current:
    save(path, current.rstrip() + '\n' + yaml.safe_dump(registry, sort_keys=False))
insert_once(ROOT / 'src/robot_lab_bringup/config/asset_groups.yaml', 'robots:\n',
    '  - id: molar_family\n    name: MOLAR / ProBOT-L\n    preferred: molar_hri\n    members:\n      - {id: molar_hri, label: Complete MOLAR HRI assembly}\n      - {id: probot_l, label: Complete ProBOT-L carrier}\n')
insert_once(ROOT / 'src/robot_lab_bringup/config/robot_taxonomy.yaml', 'classes:\n',
    '  - category: Mobile manipulators\n    subtype: Differential-drive conveyor\n    tags: [Lift, Coupled telescope, Passive casters, Estimated dynamics]\n    members: [molar_family, molar_hri, probot_l]\n')

path = ROOT / 'src/robot_lab_bringup/launch/simulated_robot.launch.py'
insert_once(path, '    drive_args["drive_config"] = json.dumps(drive_config, sort_keys=True)\n',
    '    drive_args["articulation_config"] = json.dumps(robot_config.get("articulation", {}), sort_keys=True)\n')
insert_once(path, '            display_controllers = robot_config.get("controllers", [])\n', '''            if robot_config.get('articulation'):
                actions.append(Node(package='robot_lab_bringup', executable='molar_gazebo_control.py',
                    parameters=[{'use_sim_time': _as_bool(use_sim_time, True),
                                 'model_path': model_path,
                                 'articulation_config': json.dumps(robot_config['articulation'], sort_keys=True)}],
                    output='screen'))
''')
path = ROOT / 'src/robot_lab_bringup/CMakeLists.txt'
insert_once(path, '  PROGRAMS scripts/export_3d_map.py scripts/native_arm_scene.py\n', '    scripts/molar_gazebo_control.py\n')
print(json.dumps({'profiles': list(profiles), 'native_root': str(ROOT), 'source_assets': str(SOURCE)}, indent=2))
