#!/usr/bin/env python3
"""A 5 x 5 m walled arena for distributed swarm behaviours (R10.3).

ROS2swarm's aggregation finds robots within 3.5 m (its burger parameters)
and explores straight ahead otherwise; in a 14 m world dispersed members
lose each other (measured 2026-11-18). This arena is sized to that range.
"""
import json
import xml.etree.ElementTree as ET

import yaml
from PIL import Image, ImageDraw
from generate_marine_assets import ROOT, child, model, write

NAME = 'swarm_arena'
HALF = 2.5
WALLS = [((-HALF, 0, .5), (.2, 2 * HALF + .2, 1.)), ((HALF, 0, .5), (.2, 2 * HALF + .2, 1.)),
         ((0, HALF, .5), (2 * HALF + .2, .2, 1.)), ((0, -HALF, .5), (2 * HALF + .2, .2, 1.))]


def generate_map(target):
    resolution = .05
    maps = target / 'maps'
    maps.mkdir(parents=True, exist_ok=True)
    size = int(2 * (HALF + .3) / resolution)
    origin = [-(HALF + .3), -(HALF + .3)]
    image = Image.new('L', (size, size), 254)
    draw = ImageDraw.Draw(image)
    for (x, y, _), (dx, dy, _) in WALLS:
        draw.rectangle(((x - dx / 2 - origin[0]) / resolution, (size * resolution + origin[1] - y - dy / 2) / resolution,
                        (x + dx / 2 - origin[0]) / resolution, (size * resolution + origin[1] - y + dy / 2) / resolution),
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
    model(w, 'floor', (0, 0, -.15), (2 * HALF + 1., 2 * HALF + 1., .3), '.55 .55 .55 1')
    for index, (pose, size) in enumerate(WALLS):
        model(w, 'wall_' + str(index), pose, size, '.65 .7 .75 1')
    target = ROOT / 'src/robot_lab_maps/maps' / NAME
    write(doc, target / 'worlds' / (NAME + '.world'))
    (target / 'SOURCE.json').write_text(json.dumps(dict(
        author='Robot Lab contributors', license='Apache-2.0', assets='5 x 5 m walled arena, 1 m walls',
        scope='Swarm behaviour arena sized to ROS2swarm burger sensing (3.5 m); not a mission result'), indent=2) + '\n')
    generate_map(target)


if __name__ == '__main__':
    generate()
