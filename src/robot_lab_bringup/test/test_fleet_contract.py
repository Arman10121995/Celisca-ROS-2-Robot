"""Focused identity, spawn and bounded group-command contracts (no mission claims)."""
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.fleet_session import SpawnGrid, bridge_entries, choose_spawns, namespace_description, validate_session
from robot_lab_utils.fleet_motion import Neighbor, scan_guard, separation_guard, swarm_velocities


def test_namespace_keeps_source_physics_and_separates_sensors_controllers():
    source = '''<robot name="source"><link name="base_link"><inertial><mass value="4"/></inertial></link>
      <joint name="drive_joint" type="continuous"><limit effort="5" velocity="10"/></joint>
      <ros2_control name="source_hardware"><joint name="drive_joint"/></ros2_control>
      <gazebo reference="base_link"><sensor name="lidar" type="gpu_lidar"><gz_frame_id>laser_link</gz_frame_id><topic>scan</topic></sensor></gazebo>
      <gazebo><plugin name="ign_ros2_control::IgnitionROS2ControlPlugin"><parameters>source.yaml</parameters></plugin>
       <plugin name="ignition::gazebo::systems::OdometryPublisher"><odom_frame>world</odom_frame></plugin></gazebo></robot>'''
    original = ET.fromstring(source)
    members = [dict(id='alpha', model='bumperbot'), dict(id='beta', model='bumperbot')]
    prepared = [ET.fromstring(namespace_description(source, member, member['id'] + '.yaml')) for member in members]
    for root, member in zip(prepared, members):
        for tag in ('link', 'joint', 'ros2_control'):
            assert ET.tostring(root.find(tag)) == ET.tostring(original.find(tag))
        assert root.find('.//gz_frame_id').text == member['id'] + '/laser_link'
        assert root.find('.//ros/namespace').text == '/' + member['id']
        assert root.find('.//robot_param_node').text == 'robot_state_publisher'
    entries = bridge_entries(members)
    assert sum(e['ros_topic_name'] == '/clock' for e in entries) == 1
    assert len({e['ros_topic_name'] for e in entries}) == len(entries)
    assert len({e['gz_topic_name'] for e in entries}) == len(entries)


def test_labbot_fleet_sensor_moves_physically_without_changing_drive_geometry():
    source = '''<robot name="labbot"><link name="base_link"/>
      <joint name="drive_joint" type="continuous"><origin xyz="0 .15 0"/><limit effort="5" velocity="10"/></joint>
      <joint name="laser_joint" type="fixed"><origin xyz="0 0 .08"/></joint>
      <gazebo><plugin name="ign_ros2_control::IgnitionROS2ControlPlugin"/></gazebo></robot>'''
    root = ET.fromstring(namespace_description(source, dict(id='beta', model='labbot'), 'beta.yaml'))
    original = ET.fromstring(source)
    assert ET.tostring(root.find("joint[@name='drive_joint']")) == ET.tostring(original.find("joint[@name='drive_joint']"))
    assert original.find("joint[@name='laser_joint']/origin").get('xyz') == '0 0 .08'
    assert root.find("joint[@name='laser_joint']/origin").get('xyz') == '0 0 0.13'


@pytest.mark.parametrize('members', [[], [{'id':'alpha','model':'bumperbot'}]*2,
    [{'id':'tf','model':'bumperbot'}], [{'id':'other','model':'px4_x500'}],
    [{'id':'alpha','model':'labbot','spawn':{'x':0}}],
    [{'id':'alpha','model':'labbot','spawn':{'x':True,'y':0}}],
    [{'id':'alpha','model':'labbot','spawn':{'x':math.nan,'y':0}}]])
def test_invalid_session_cannot_launch(members):
    with pytest.raises(ValueError):
        validate_session(dict(schema_version=1, map='nav_empty', members=members))


def test_rotated_occupancy_unknown_and_exterior_are_not_free_spawns(tmp_path):
    image = np.full((40, 40), 255, dtype=np.uint8)
    image[10:14, 10:14] = 205  # unknown
    image[:, 0] = 0
    Image.fromarray(image).save(tmp_path / 'map.pgm')
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=.1,
        origin=[2., 3., math.pi/2], free_thresh=.196, occupied_thresh=.65, negate=0)))
    grid = SpawnGrid(tmp_path / 'map.yaml')
    assert grid.clear(0., 5., .1)
    assert not grid.clear(20., 20., .1)
    assert not grid.clear(2.-2.8, 3.+1.2, .1)
    members = [dict(id='alpha', radius=.18, spawn=None), dict(id='beta', radius=.18, spawn=None)]
    poses = choose_spawns(members, dict(x=0.,y=5.,z=.1,yaw=0.), grid)
    assert math.dist([poses[0]['spawn'][k] for k in ('x','y')], [poses[1]['spawn'][k] for k in ('x','y')]) > .66
    assert poses[0]['spawn'] != poses[1]['spawn']
    with pytest.raises(ValueError, match='explicit'):
        choose_spawns(members, dict(x=0., y=0.))


def test_group_controller_equilibrium_and_real_twist_only():
    robots = [Neighbor('alpha',0.,-.5,0.,0.,.18), Neighbor('beta',0.,.5,0.,0.,.18)]
    result = swarm_velocities('formation', robots, .25,.6, spacing=1., anchor=(0.,0.))
    assert result == {'alpha':(0.,0.), 'beta':(0.,0.)}
    shifted = swarm_velocities('formation', robots, .25,.6, spacing=1., anchor=(1.,0.))
    assert all(command == (.25,0.) for command in shifted.values())
    assert robots[0].x == robots[1].x == 0.  # kernel never mutates physical/estimated poses
    for mode in ('rendezvous','dispersion','flocking'):
        values = swarm_velocities(mode, robots, .25,.6)
        assert all(0 <= v <= .25 and abs(w) <= .6 for v,w in values.values())


def test_scan_and_predicted_peer_stop_prevent_command_progression():
    robot = Neighbor('alpha',0.,0.,0.,0.,.18)
    peer = Neighbor('beta',.6,0.,math.pi,.25,.18)
    assert separation_guard(robot,[peer],.3,0.)[:2] == (0.,0.)
    assert scan_guard(.3,0.,[.25,1.,1.],0.,.1,.18)[:2] == (0.,0.)
    assert scan_guard(.3,0.,[math.nan,1.],0.,.1,.18)[:2] == (0.,0.)
    assert scan_guard(.3,0.,[math.inf]*10,0.,.1,.18)[:2] == (.3,0.)


def test_overlapping_explicit_members_are_refused_before_physics():
    members=[dict(id='alpha',radius=.18,spawn=dict(x=0.,y=0.)),dict(id='beta',radius=.118,spawn=dict(x=.1,y=0.))]
    with pytest.raises(ValueError,match='overlaps'):
        choose_spawns(members,dict(x=0.,y=0.))
