"""Gazebo example world review and Fuel licence gating (R6.6)."""
import importlib.util
from pathlib import Path

import pytest


def load(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).resolve().parents[3] / 'scripts' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


review = load('review_gazebo_examples')
fuel = load('fuel_models')


def world(tmp_path, name, body):
    path = tmp_path / name
    path.write_text('<?xml version="1.0"?>\n<!-- ign service --reqtype x -->\n<sdf version="1.6"><world name="w">'
                    '<plugin filename="ignition-gazebo-physics-system" name="x"/>%s</world></sdf>' % body)
    return path


def test_tolerant_parse_and_classes(tmp_path):
    env = world(tmp_path, 'shapes.sdf', '<model name="box"><static>true</static><link name="l"/></model>')
    assert review.review(env, [])['kind'] == 'environment'
    robot = world(tmp_path, 'robot.sdf', '<model name="car"><link name="a"/><link name="b"/><joint name="j" type="revolute">'
                  '<parent>a</parent><child>b</child></joint><plugin filename="ignition-gazebo-diff-drive-system" name="d"/>'
                  '</model>')
    assert review.review(robot, [])['kind'] == 'robot_demo'
    prefixed = world(tmp_path, 'mecanum.sdf', '<model name="m"><link name="l"><collision name="c"><surface><friction><ode>'
                     '<fdir1 ignition:expressed_in="l">1 0 0</fdir1></ode></friction></surface></collision></link></model>')
    assert review.review(prefixed, [])['kind'] == 'environment'
    fueled = world(tmp_path, 'fuel.sdf', '<include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Cone</uri></include>')
    row = review.review(fueled, [])
    assert row['needs_fuel'] and row['kind'] == 'environment'
    actor = world(tmp_path, 'actor.sdf', '<actor name="walker"/>')
    assert review.review(actor, [])['kind'] == 'actor_demo'
    (tmp_path / 'blank.sdf').write_text('')
    with pytest.raises(ValueError):
        review.parse_sdf(tmp_path / 'blank.sdf')


def test_fuel_licences_gate_each_world():
    assert fuel.model_key('https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction%20Cone/3') == \
        ('OpenRobotics', 'Construction Cone', '3')
    with pytest.raises(ValueError):
        fuel.model_key('https://example.com/model')
    licences = {'Cone': 'Creative Commons Zero v1.0 Universal', 'Secret': 'Proprietary'}

    def fetch(url):
        name = url.rsplit('/', 1)[1].replace('%20', ' ')
        return dict(owner='OpenRobotics', name=name, version=2, license_name=licences[name])
    data = dict(worlds=[
        dict(file='a.sdf', kind='environment', fuel_uris=['https://fuel.gazebosim.org/1.0/OpenRobotics/models/Cone']),
        dict(file='b.sdf', kind='environment', fuel_uris=['https://fuel.gazebosim.org/1.0/OpenRobotics/models/Cone',
                                                          'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Secret']),
        dict(file='c.sdf', kind='environment')])
    cache = {}
    models, worlds = fuel.collect(data, cache, fetch=fetch, pause=0)
    assert models['OpenRobotics/Cone']['allowed'] and not models['OpenRobotics/Secret']['allowed']
    by_file = {w['file']: w for w in worlds}
    assert by_file['a.sdf']['installable_licences'] and not by_file['b.sdf']['installable_licences']
    assert by_file['b.sdf']['blocked'] == ['OpenRobotics/Secret'] and 'c.sdf' not in by_file
    assert set(cache) == {'OpenRobotics/Cone', 'OpenRobotics/Secret'}      # queried once each


def test_fuel_uris_resolve_to_gazebos_cache_without_downloading(tmp_path):
    from robot_lab_utils.sdf_world import fuel_cache_path, uri_resolver
    model = tmp_path / 'fuel.ignitionrobotics.org/openrobotics/models/construction cone'
    (model / '2/meshes').mkdir(parents=True)
    (model / '3/meshes').mkdir(parents=True)
    (model / '3/model.sdf').write_text('<sdf/>')
    (model / '3/meshes/cone.dae').write_text('<COLLADA/>')
    uri = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction%20Cone'
    assert fuel_cache_path(uri, tmp_path) == str(model / '3')                   # newest version, legacy host
    assert fuel_cache_path(uri + '/2', tmp_path) == str(model / '2')
    assert fuel_cache_path(uri + '/3/files/meshes/cone.dae', tmp_path) == str(model / '3/meshes/cone.dae')
    assert fuel_cache_path('https://fuel.gazebosim.org/1.0/OpenRobotics/models/Missing', tmp_path) == ''
    assert uri_resolver()('https://fuel.gazebosim.org/1.0/nobody/models/none', '.') == ''
