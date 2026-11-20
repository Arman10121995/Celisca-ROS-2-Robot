#!/usr/bin/env python3
"""Author ``terrain_stairs_flight``: the terrain_stairs flight with a run-up (R10.7 multi-step flight).

In ``terrain_stairs`` the five-step flight (0.18 m rises, 1.1 m treads, up to
0.9 m) starts 0.8 m from the arena wall, too close for the 0.5 m tracked hull
and its flippers to approach the first riser head on; the measured 11-19
climbs took the first box from its side. This variant moves the same five
platforms 2 m along +x, leaving a 2.6 m run-up. Everything else (arena walls,
the descending flight, ground) is the source world unchanged; the original
stays as measured.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

FLIGHT = ('platform_04', 'platform_05', 'platform_06', 'platform_07', 'platform_08')
SHIFT_X = 2.


def generate(root=None):
    root = Path(root or Path(__file__).resolve().parents[1])
    tree = ET.parse(root/'maps/terrain_stairs/worlds/terrain_stairs.world')
    world = tree.getroot().find('world')
    world.set('name', 'terrain_stairs_flight')
    moved = 0
    for model in world.findall('model'):
        if model.get('name') in FLIGHT:
            pose = model.find('pose')
            values = pose.text.split()
            values[0] = '%.3f' % (float(values[0])+SHIFT_X)
            pose.text = ' '.join(values)
            moved += 1
    if moved != len(FLIGHT):
        raise ValueError('terrain_stairs flight platforms changed in the source world')
    folder = root/'maps/terrain_stairs_flight/worlds'
    folder.mkdir(parents=True, exist_ok=True)
    tree.write(folder/'terrain_stairs_flight.world', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    generate()
