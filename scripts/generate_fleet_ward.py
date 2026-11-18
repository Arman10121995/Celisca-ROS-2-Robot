#!/usr/bin/env python3
"""A one-floor fleet world with a physical door and two chargers, and no lift (R10.2).

Two 5 x 7 m rooms share one 1.6 m doorway. Its sliding door is the
fleet_facility main door (same panel, travel, gains and path-zone rule).
Each room has an authored charger dock. Not mission results.
"""
import json
import xml.etree.ElementTree as ET

import yaml
from PIL import Image, ImageDraw
from generate_marine_assets import ROOT, child, model, write

NAME = 'fleet_ward'
HALF_X, HALF_Y = 5.0, 3.5
DOORWAY = 0.8                      # half width of the opening in the partition
WALLS = [((-HALF_X, 0, 1.25), (.2, 2 * HALF_Y, 2.5)), ((HALF_X, 0, 1.25), (.2, 2 * HALF_Y, 2.5)),
         ((0, HALF_Y, 1.25), (2 * HALF_X, .2, 2.5)), ((0, -HALF_Y, 1.25), (2 * HALF_X, .2, 2.5)),
         # The partition at x = 0, open between y = -0.8 and 0.8.
         ((0, -(HALF_Y + DOORWAY) / 2, 1.25), (.15, HALF_Y - DOORWAY, 2.5)),
         ((0, (HALF_Y + DOORWAY) / 2, 1.25), (.15, HALF_Y - DOORWAY, 2.5))]
DOCKS = {'west_charger': dict(x=-3.8, y=-2.3, z=0., yaw=0.), 'east_charger': dict(x=3.8, y=2.3, z=0., yaw=3.14159)}


def generate_map(target):
    resolution = .05
    maps = target / 'maps'
    maps.mkdir(parents=True, exist_ok=True)
    width, height = int(2 * HALF_X / resolution) + 8, int(2 * HALF_Y / resolution) + 8
    origin = [-HALF_X - 4 * resolution, -HALF_Y - 4 * resolution]
    image = Image.new('L', (width, height), 254)
    draw = ImageDraw.Draw(image)
    for (x, y, _), (dx, dy, _) in WALLS:
        draw.rectangle(((x - dx / 2 - origin[0]) / resolution, (height * resolution + origin[1] - y - dy / 2) / resolution,
                        (x + dx / 2 - origin[0]) / resolution, (height * resolution + origin[1] - y + dy / 2) / resolution),
                       fill=0)
    image.save(maps / 'map.pgm')
    (maps / 'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=resolution, origin=origin + [0.],
                                                       negate=0, occupied_thresh=.65, free_thresh=.25)))


def generate():
    doc = ET.Element('sdf', version='1.7')
    w = child(doc, 'world', name=NAME)
    physics = child(w, 'physics', name='physics', type='ode')
    child(physics, 'max_step_size', '.002')
    child(physics, 'real_time_factor', '1')
    child(w, 'gravity', '0 0 -9.81')
    for system, short in [('Physics', 'physics'), ('UserCommands', 'user-commands'),
                          ('SceneBroadcaster', 'scene-broadcaster'), ('Sensors', 'sensors'), ('Imu', 'imu')]:
        plugin = child(w, 'plugin', name='ignition::gazebo::systems::' + system,
                       filename='ignition-gazebo-' + short + '-system')
        if system == 'Sensors':
            child(plugin, 'render_engine', 'ogre2')
    light = child(w, 'light', name='sun', type='directional')
    child(light, 'pose', '0 0 10 0 0 0')
    child(light, 'diffuse', '.9 .9 .9 1')
    child(light, 'direction', '-.3 .2 -1')
    model(w, 'floor', (0, 0, -.15), (2 * HALF_X + .2, 2 * HALF_Y + .2, .3), '.55 .55 .55 1')
    for index, (pose, size) in enumerate(WALLS):
        model(w, 'wall_' + str(index), pose, size, '.65 .7 .75 1')
    for name, dock in DOCKS.items():
        # A painted pad only: docking is judged from the robot's pose.
        model(w, name + '_pad', (dock['x'], dock['y'], .001), (.5, .5, .002), '.2 .6 .3 1', collision=False)
    hardware = child(w, 'model', name='facility')
    child(hardware, 'static', 'false')
    anchor = child(hardware, 'link', name='anchor')
    inertial = child(anchor, 'inertial')
    child(inertial, 'mass', '1')
    child(inertial, 'inertia')
    fixed = child(hardware, 'joint', name='anchor_fixed', type='fixed')
    child(fixed, 'parent', 'world')
    child(fixed, 'child', 'anchor')
    # fleet_facility's main door: panel, travel, mass, force and gains.
    link_name, joint_name, pose, size, maximum, mass, limit, kp, kd = (
        'ward_door', 'door_ward', (.16, 0, 1.1), (.08, 1.6, 2.2), 1.8, 12., 300., 300., 80.)
    link = child(hardware, 'link', name=link_name)
    child(link, 'pose', ' '.join(map(str, pose)) + ' 0 0 0')
    inertial = child(link, 'inertial')
    child(inertial, 'mass', mass)
    tensor = child(inertial, 'inertia')
    for key, a, b in [('ixx', size[1], size[2]), ('iyy', size[0], size[2]), ('izz', size[0], size[1])]:
        child(tensor, key, mass * (a * a + b * b) / 12)
    for kind in ('collision', 'visual'):
        part = child(link, kind, name=link_name + '_' + kind)
        child(part, 'pose', '0 0 .005 0 0 0')     # 10 mm floor clearance, as in fleet_facility
        child(child(child(part, 'geometry'), 'box'), 'size', ' '.join(map(str, (size[0], size[1], size[2] - .01))))
    joint = child(hardware, 'joint', name=joint_name, type='prismatic')
    child(joint, 'parent', 'anchor')
    child(joint, 'child', link_name)
    axis = child(joint, 'axis')
    child(axis, 'xyz', '0 1 0')
    limits = child(axis, 'limit')
    for key, value in dict(lower=0, upper=maximum, effort=limit, velocity=.25).items():
        child(limits, key, value)
    plugin = child(hardware, 'plugin', name='ignition::gazebo::systems::JointPositionController',
                   filename='ignition-gazebo-joint-position-controller-system')
    for key, value in dict(joint_name=joint_name, topic='/facility/' + joint_name + '/target', p_gain=kp, d_gain=kd,
                           i_gain=20, i_min=-8, i_max=8, cmd_offset=0., cmd_min=-limit, cmd_max=limit,
                           use_velocity_commands='false', initial_position=0).items():
        child(plugin, key, value)
    publisher = child(hardware, 'plugin', name='ignition::gazebo::systems::JointStatePublisher',
                      filename='ignition-gazebo-joint-state-publisher-system')
    child(publisher, 'topic', '/facility/joint_states')
    target = ROOT / 'src/robot_lab_maps/maps' / NAME
    write(doc, target / 'worlds' / (NAME + '.world'))
    facility = yaml.safe_load((ROOT / 'src/robot_lab_maps/maps/fleet_facility/infrastructure.yaml').read_text())
    config = dict(
        schema_version=1,
        doors={'ward': dict(joint=joint_name, closed=0., opened=maximum, sweep=[-.5, .5, -1.1, DOORWAY + maximum + .3],
                            approach=[[-1.2, 0.], [1.2, 0.]], path_zone=[-1.5, 1.5, -1., 1.])},
        floor_maps=dict(L0='maps/%s/maps/map.yaml' % NAME),
        docks={name: dict(dock, position_tolerance=.12, height_tolerance=.12, yaw_tolerance=.15)
               for name, dock in DOCKS.items()},
        # The facility's battery model and drivetrain calibrations: they are
        # per robot model and description, not per world.
        energy=facility['energy'])
    (target / 'infrastructure.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    (target / 'SOURCE.json').write_text(json.dumps(dict(
        author='Robot Lab contributors', license='Apache-2.0',
        assets='Two rooms, one physical sliding door (fleet_facility main door design), two authored charger docks; no lift',
        scope='Uncalibrated first-party infrastructure; door and docking missions need actual feedback receipts'),
        indent=2) + '\n')
    generate_map(target)


if __name__ == '__main__':
    generate()
