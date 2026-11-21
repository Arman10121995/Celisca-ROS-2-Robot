"""mobile_fetch_lab's declared RGB-D box matches its world (R5.9 camera object poses)."""
import colorsys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from robot_lab_utils.rgbd_boxes import bindings

MAP = Path(__file__).resolve().parents[2] / 'robot_lab_maps' / 'maps' / 'mobile_fetch_lab'


def _world_boxes():
    root = ET.parse(MAP / 'worlds' / 'mobile_fetch_lab.world').getroot()
    result = {}
    for model in root.iter('model'):
        visual = next(model.iter('visual'))
        size = [float(v) for v in visual.find('geometry/box/size').text.split()]
        colour = [float(v) for v in visual.find('material/diffuse').text.split()[:3]]
        result[model.get('name')] = (size, colour)
    return result


def _opencv_hsv(rgb):
    h, s, v = colorsys.rgb_to_hsv(*rgb)
    return [round(h * 179), round(s * 255), round(v * 255)]


def _inside(hsv, binding):
    return all(lo <= value <= hi for value, lo, hi in zip(hsv, binding['hsv_low'], binding['hsv_high']))


def test_binding_names_a_loose_world_box_of_its_size_and_colour():
    boxes = _world_boxes()
    declared = bindings(yaml.safe_load((MAP / 'rgbd_boxes.yaml').read_text()))
    assert set(declared) == {'fetch_block'}
    size, colour = boxes['fetch_block']
    assert declared['fetch_block']['size_m'] == size
    assert _inside(_opencv_hsv(colour), declared['fetch_block'])
    # Shading darkens the faces; the hue and saturation stay in range.
    assert _inside(_opencv_hsv([0.5 * c for c in colour]), declared['fetch_block'])


def test_no_other_world_model_falls_in_the_colour_range():
    declared = bindings(yaml.safe_load((MAP / 'rgbd_boxes.yaml').read_text()))['fetch_block']
    for name, (_size, colour) in _world_boxes().items():
        if name != 'fetch_block':
            assert not _inside(_opencv_hsv(colour), declared), name


def test_cylinder_fixture_reuses_static_world_and_declares_the_loose_lid_target():
    directory = MAP.with_name('mobile_fetch_lab_can')
    world = ET.parse(directory / 'worlds' / 'mobile_fetch_lab_can.world').getroot()
    original = ET.parse(MAP / 'worlds' / 'mobile_fetch_lab.world').getroot()
    assert [ET.tostring(m) for m in world.iter('model') if m.findtext('static') == 'true'] == [
        ET.tostring(m) for m in original.iter('model') if m.findtext('static') == 'true']
    loose, = [m for m in world.iter('model') if m.findtext('static') == 'false']
    target, = bindings(yaml.safe_load((directory / 'rgbd_targets.yaml').read_text()),
                       allow_primitives=True).values()
    cylinder = loose.find('link/visual/geometry/cylinder')
    diameter = 2 * float(cylinder.findtext('radius'))
    colour = [float(v) for v in loose.findtext('link/visual/material/diffuse').split()[:3]]
    assert target['object'] == loose.get('name')
    assert target['shape'] == 'cylinder' and target['size_m'] == [diameter, diameter, float(cylinder.findtext('length'))]
    assert _inside(_opencv_hsv(colour), target)
