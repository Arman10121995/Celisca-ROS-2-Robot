#!/usr/bin/env python3
"""Author ``marine_tank_textured``: the tank with visual and depth structure for AUV relocalization.

The plain tank gave RTAB-Map no visual features and no ICP structure, so a
saved map never relocalized (R10.5, 2026-11-19). This variant keeps the tank
(walls, board, water, buoyancy) and adds, in the view of the Tethys rear
camera (yaw π) at the 2–4 m working depth:

- two 2 x 3 m panels of distinct ArUco markers, facing the Tethys tank
  start from either side of its route;
- a marker-textured 0.8 m box standing 0.5 m proud of each panel, which
  gives depth structure for ICP.

Each surface uses its own marker IDs, so no texture repeats. The original
``marine_tank`` stays unchanged for its measured runs. Visuals and
collisions only; no localization input.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

# The Tethys RGB-D camera looks along the heading (72 deg field of view, 8 m
# depth clip). From the measured route start (-4, -3, heading 0.5 rad; goal
# (-0.93, -1.32)) two marker panels stand 6 m away at +-30 deg: in view at
# the start, where relocalization must happen, and more than 2.1 m from the
# route line and its 3 m terminal braking corridor (planner radius 1.3 m).
START, HEADING = (-4., -3.), .5
# (model name, bearing from the heading (rad), distance (m), first marker id)
PANELS = [('relocalization_panel_left', math.radians(30.), 6., 40),
          ('relocalization_panel_right', -math.radians(30.), 6., 60)]


def marker_sheet(first_id, rows, cols, cell=512, marker=384):
    import cv2
    import numpy as np
    image = np.full((rows*cell, cols*cell), 240, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    margin = (cell-marker)//2
    for row in range(rows):
        for col in range(cols):
            image[row*cell+margin:row*cell+margin+marker,
                  col*cell+margin:col*cell+margin+marker] = draw(dictionary, first_id+row*cols+col, marker)
    return image


def _textured(visual, texture):
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = '.8 .8 .8 1'
    ET.SubElement(material, 'diffuse').text = '1 1 1 1'
    metal = ET.SubElement(ET.SubElement(material, 'pbr'), 'metal')
    ET.SubElement(metal, 'albedo_map').text = '../materials/textures/'+texture
    ET.SubElement(metal, 'metalness').text = '0'
    ET.SubElement(metal, 'roughness').text = '1'


def generate(root=None):
    import cv2
    root = Path(root or Path(__file__).resolve().parents[1])
    source = root/'maps/marine_tank/worlds/marine_tank.world'
    folder = root/'maps/marine_tank_textured'
    textures = folder/'materials/textures'
    textures.mkdir(parents=True, exist_ok=True)
    (folder/'worlds').mkdir(exist_ok=True)
    sheets = {}
    for name, _, _, first in PANELS:
        sheets[name+'.png'] = marker_sheet(first, 3, 2)
        sheets[name+'_box.png'] = marker_sheet(first+6, 2, 2)
    for name, image in sheets.items():
        if not cv2.imwrite(str(textures/name), image):
            raise OSError('Could not write texture '+name)
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    world.set('name', 'marine_tank_textured')
    # The original board's texture lives with the source tank.
    board = world.find("model[@name='visual_reference_board']//albedo_map")
    if board is not None:
        board.text = '../../marine_tank/materials/textures/tank_reference.png'
    def box(name, centre, yaw, size, texture):
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = '%.4f %.4f %.4f 0 0 %.4f' % (*centre, yaw)
        link = ET.SubElement(model, 'link', name='link')
        for tag in ('collision', 'visual'):
            element = ET.SubElement(link, tag, name=tag)
            ET.SubElement(ET.SubElement(ET.SubElement(element, 'geometry'), 'box'), 'size').text = '%g %g %g' % size
            if tag == 'visual':
                _textured(element, texture)
    for name, bearing, distance, _ in PANELS:
        angle = HEADING+bearing
        x, y = START[0]+distance*math.cos(angle), START[1]+distance*math.sin(angle)
        box(name, (x, y, -3.), angle, (.2, 2., 3.), name+'.png')        # faces the start
        box(name+'_box', (x-.5*math.cos(angle), y-.5*math.sin(angle), -3.4), angle, (.8, .8, .8), name+'_box.png')
    ET.indent(tree, space='  ')
    tree.write(folder/'worlds/marine_tank_textured.world', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    generate()
