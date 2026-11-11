#!/usr/bin/env python3
"""Write a copy of a Gazebo world with labelled floor objects for the
robot-vacuum camera (R13.3): a cable, a sock, a shoe and pet mess.

Each object is a static primitive with the Gazebo Label system, so the
oomwoo semantic segmentation camera sees its class id (20 cable, 21 sock,
22 shoe, 23 pet_waste; vacuum_objects.DEFAULT_LABELS).  The saved occupancy
map does not contain them.  Every object must lie on free floor of that map
with at least --clearance metres to the nearest wall or furniture.

  scripts/vacuum_object_world.py --world <in.world> --map <map.yaml> --out <out.world> \\
      [--objects objects.json]

The default layout suits the small_house keep-out zone (4.0, -3.5)-(7.5, -0.5).
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

LABELS = {'cable': 20, 'sock': 21, 'shoe': 22, 'pet_waste': 23}
# class: (geometry SDF, half-extents for the clearance check, z)
SHAPES = {
    'cable': ('<cylinder><radius>0.006</radius><length>0.8</length></cylinder>', (0.4, 0.006), 0.006,
              '0 1.5708 0', '0.1 0.1 0.1'),
    'sock': ('<box><size>0.22 0.09 0.03</size></box>', (0.11, 0.045), 0.015, '0 0 0', '0.85 0.85 0.9'),
    'shoe': ('<box><size>0.28 0.11 0.10</size></box>', (0.14, 0.055), 0.05, '0 0 0', '0.35 0.2 0.1'),
    'pet_waste': ('<sphere><radius>0.025</radius></sphere>', (0.025, 0.025), 0.02, '0 0 0', '0.4 0.25 0.1'),
}
DEFAULT_OBJECTS = [
    {'class': 'cable', 'x': 4.9, 'y': -1.1, 'yaw': 0.0},
    {'class': 'sock', 'x': 6.6, 'y': -2.9, 'yaw': 0.6},
    {'class': 'shoe', 'x': 5.0, 'y': -2.9, 'yaw': -0.4},
    {'class': 'pet_waste', 'x': 6.1, 'y': -1.0, 'yaw': 0.0},
]


def free_clearance(map_yaml: Path, x: float, y: float, half=(0.1, 0.1), yaw=0.0) -> float:
    """Distance from the object's footprint to the nearest occupied or unknown cell."""
    meta = yaml.safe_load(map_yaml.read_text())
    img = np.asarray(Image.open(map_yaml.parent / meta['image']).convert('L'), dtype=float) / 255.0
    occ = (1.0 - img) if not meta.get('negate', 0) else img
    res, (ox, oy) = float(meta['resolution']), meta['origin'][:2]
    blocked = (occ > float(meta.get('occupied_thresh', 0.65))) | \
        ((occ > float(meta.get('free_thresh', 0.25))) & (occ <= float(meta.get('occupied_thresh', 0.65))))
    h = img.shape[0]
    ys, xs = np.nonzero(blocked)
    wx = ox + (xs + 0.5) * res
    wy = oy + (h - 1 - ys + 0.5) * res
    # Distance to an oriented rectangle footprint.
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx = c * (wx - x) - s * (wy - y)
    ly = s * (wx - x) + c * (wy - y)
    dx = np.maximum(np.abs(lx) - half[0], 0.0)
    dy = np.maximum(np.abs(ly) - half[1], 0.0)
    return float(np.hypot(dx, dy).min()) if len(wx) else math.inf


def model_sdf(i: int, obj: dict) -> str:
    geom, _, z, rpy, colour = SHAPES[obj['class']]
    return f'''    <model name="floor_{obj['class']}_{i}">
      <static>true</static>
      <pose>{obj['x']} {obj['y']} {z} 0 0 {obj.get('yaw', 0.0)}</pose>
      <link name="link">
        <collision name="collision"><pose>0 0 0 {rpy}</pose><geometry>{geom}</geometry></collision>
        <visual name="visual"><pose>0 0 0 {rpy}</pose><geometry>{geom}</geometry>
          <material><ambient>{colour} 1</ambient><diffuse>{colour} 1</diffuse></material></visual>
      </link>
      <plugin filename="ignition-gazebo-label-system" name="ignition::gazebo::systems::Label">
        <label>{LABELS[obj['class']]}</label>
      </plugin>
    </model>
'''


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', type=Path, required=True)
    ap.add_argument('--map', type=Path, required=True, help='occupancy map YAML of that world')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--objects', type=Path, help='JSON list of {class, x, y, yaw}')
    ap.add_argument('--clearance', type=float, default=0.25)
    args = ap.parse_args()
    objects = json.loads(args.objects.read_text()) if args.objects else DEFAULT_OBJECTS
    bad = []
    for obj in objects:
        d = free_clearance(args.map, obj['x'], obj['y'], SHAPES[obj['class']][1], obj.get('yaw', 0.0))
        obj['clearance_m'] = round(d, 3)
        if d < args.clearance:
            bad.append(obj)
    if bad:
        sys.exit('objects too close to mapped obstacles: %s' % json.dumps(bad))
    world = args.world.read_text()
    i = world.rindex('</world>')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(world[:i] + ''.join(model_sdf(n, o) for n, o in enumerate(objects)) + world[i:])
    args.out.with_suffix('.objects.json').write_text(json.dumps(
        [dict(o, label=LABELS[o['class']], footprint=(
            {'kind': 'circle', 'radius': 0.025} if o['class'] == 'pet_waste' else
            {'kind': 'box', 'half_extents': list(SHAPES[o['class']][1]), 'yaw': o.get('yaw', 0.0)}))
         for o in objects], indent=1) + '\n')
    print('wrote %s with %d objects' % (args.out, len(objects)))


if __name__ == '__main__':
    main()
