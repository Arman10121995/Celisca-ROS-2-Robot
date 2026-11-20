#!/usr/bin/env python3
"""Author ``terrain_step_wide``: one wide 0.18 m step for angled tracked approaches (R10.7).

The terrain_stairs steps are 1.1 m wide or meet other steps beside them, so
an approach at an angle reaches a corner or a double face (2026-11-20). This
variant removes the terrain_stairs flight and places one slab of the same
0.18 m rise, material and friction (``platform_08``): 3 m deep along +x from
x 0 and 6 m wide (y -7..-1). A vehicle can meet its face at up to 45 deg
and stay on it. The arena walls and the far descending flight are unchanged.
"""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

SOURCE_FLIGHT = ('platform_04', 'platform_05', 'platform_06', 'platform_07', 'platform_08')
STEP = dict(x=1.5, y=-4., depth=3., width=6., rise=.18)


def generate(root=None):
    root = Path(root or Path(__file__).resolve().parents[1])
    tree = ET.parse(root/'maps/terrain_stairs/worlds/terrain_stairs.world')
    world = tree.getroot().find('world')
    world.set('name', 'terrain_step_wide')
    template = copy.deepcopy(world.find("model[@name='platform_08']"))
    for name in SOURCE_FLIGHT:
        world.remove(world.find("model[@name='%s']" % name))
    template.set('name', 'wide_step')
    template.find('pose').text = '%.3f %.3f %.3f 0 0 0' % (STEP['x'], STEP['y'], STEP['rise']/2)
    for size in template.iter('size'):
        size.text = '%.3f %.3f %.3f' % (STEP['depth'], STEP['width'], STEP['rise'])
    world.append(template)
    folder = root/'maps/terrain_step_wide/worlds'
    folder.mkdir(parents=True, exist_ok=True)
    tree.write(folder/'terrain_step_wide.world', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    generate()
