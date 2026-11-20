#!/usr/bin/env python3
"""Author ``terrain_stairs_over``: up a five-step flight and down the other side (R10.7 descents).

``terrain_stairs_flight`` (2026-11-20) has the source flight with a run-up,
but its 4.4 m top tread ends in a 0.9 m drop: no way down. This variant
rebuilds the flight from 1.1 x 1.1 m treads of the source's 0.18 m slabs
(same rises, treads, material and friction as ``platform_08``): five steps
up from x -5.2, a 1.1 m top tread at 0.9 m, and five steps down to the floor
at x 4.7, leaving 3.3 m to the arena wall. The rest of terrain_stairs is
unchanged.
"""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

RISE, TREAD, START_X, Y = .18, 1.1, -5.2, -4.
SOURCE_FLIGHT = ('platform_04', 'platform_05', 'platform_06', 'platform_07', 'platform_08')


def treads():
    """(name, centre x, top z) of every tread, up then down."""
    up = [('flight_up_%d' % k, START_X+TREAD*(k+.5), RISE*(k+1)) for k in range(5)]
    down = [('flight_down_%d' % j, START_X+TREAD*(4+j+.5), RISE*(5-j)) for j in range(1, 5)]
    return up+down


def generate(root=None):
    root = Path(root or Path(__file__).resolve().parents[1])
    tree = ET.parse(root/'maps/terrain_stairs/worlds/terrain_stairs.world')
    world = tree.getroot().find('world')
    world.set('name', 'terrain_stairs_over')
    template = world.find("model[@name='platform_08']")
    if template is None or template.find('.//collision//box/size').text.split()[2] != '0.180':
        raise ValueError('terrain_stairs slab template changed in the source world')
    for name in SOURCE_FLIGHT:
        world.remove(world.find("model[@name='%s']" % name))
    for name, x, top in treads():
        model = copy.deepcopy(template)
        model.set('name', name)
        model.find('pose').text = '%.3f %.3f %.3f 0 0 0' % (x, Y, top-RISE/2)
        for size in model.iter('size'):
            size.text = '%.3f %.3f %.3f' % (TREAD, TREAD, RISE)
        world.append(model)
    folder = root/'maps/terrain_stairs_over/worlds'
    folder.mkdir(parents=True, exist_ok=True)
    tree.write(folder/'terrain_stairs_over.world', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    generate()
