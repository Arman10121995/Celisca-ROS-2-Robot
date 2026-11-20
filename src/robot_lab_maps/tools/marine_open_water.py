#!/usr/bin/env python3
"""Author ``marine_open_water``: a 60 x 40 m open bay for long AUV routes (R10.5).

The tank is 12 x 10 m. With the planner's 1.3 m vehicle radius and 3 m
terminal braking corridor, a 6.5 m straight route is the longest it holds,
and Tethys' 10-30 m Dubins turn radius cannot turn in it (2026-11-20). This
bay keeps the tank's water, seafloor depth, buoyancy and physics, without
walls or the reference board, and widens the seafloor and surface to 60 x
40 m. Visuals and collisions only; no localization input.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

SIZE = (60., 40.)   # seafloor and water surface extent (m)


def generate(root=None):
    root = Path(root or Path(__file__).resolve().parents[1])
    tree = ET.parse(root/'maps/marine_tank/worlds/marine_tank.world')
    world = tree.getroot().find('world')
    world.set('name', 'marine_open_water')
    for model in list(world.findall('model')):
        name = model.get('name')
        if name.startswith('tank_wall_') or name == 'visual_reference_board':
            world.remove(model)
        elif name in ('seafloor', 'water_surface'):
            for size in model.iter('size'):
                values = size.text.split()
                size.text = '%g %g %s' % (*SIZE, values[2])
    folder = root/'maps/marine_open_water/worlds'
    folder.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(folder/'marine_open_water.world', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    generate()
