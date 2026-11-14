"""The temporary Gazebo world copy keeps relative meshes and actor skins resolvable (R6.8)."""
import importlib.util
from pathlib import Path


def load_launch():
    path = Path(__file__).resolve().parents[2] / 'robot_lab_description' / 'launch' / 'gazebo.launch.py'
    spec = importlib.util.spec_from_file_location('robot_lab_gazebo_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_relative_uris_and_actor_filenames_become_absolute(tmp_path):
    world = tmp_path / 'city.world'
    world.write_text('<sdf version="1.9"><world name="city">'
                     '<model name="b"><link name="l"><visual name="v"><geometry><mesh><uri>buildings/b1.obj</uri>'
                     '</mesh></geometry></visual></link></model>'
                     '<actor name="car"><skin><filename>vehicle_box.dae</filename></skin>'
                     '<animation name="walk"><filename>package://robot_lab_description/meshes/walk.dae</filename></animation>'
                     '</actor></world></sdf>')
    text = Path(load_launch().world_with_contact_system(str(world))).read_text()
    assert '<uri>%s</uri>' % (tmp_path / 'buildings/b1.obj') in text
    assert '<filename>%s</filename>' % (tmp_path / 'vehicle_box.dae') in text
    assert '<filename>package://robot_lab_description/meshes/walk.dae</filename>' in text   # URIs stay


def _fake_seed_library(tmp_path, monkeypatch):
    import ament_index_python.packages as packages
    (tmp_path / 'prefix/lib').mkdir(parents=True)
    (tmp_path / 'prefix/lib/librobot_lab_simulation_seed.so').write_text('')
    monkeypatch.setattr(packages, 'get_package_prefix', lambda name: str(tmp_path / 'prefix'))


def test_seed_system_is_added_once_and_keeps_declared_systems(tmp_path, monkeypatch):
    """R7: sim_seed adds the seed system; worlds without systems also get Gazebo's defaults."""
    import pytest
    _fake_seed_library(tmp_path, monkeypatch)
    launch = load_launch()
    bare = tmp_path / 'bare.world'
    bare.write_text('<sdf version="1.9"><world name="w"><model name="m"><plugin filename="x" name="y"/></model>'
                    '</world></sdf>')
    text = Path(launch.world_with_seed(str(bare), 42)).read_text()
    assert text.count('robot_lab::SimulationSeed') == 1 and '<seed>42</seed>' in text
    assert 'ignition::gazebo::systems::Physics' in text          # model plugins do not count as world systems
    declared = tmp_path / 'declared.world'
    declared.write_text('<sdf version="1.9"><world name="w"><plugin filename="p" name="ignition::gazebo::systems::Physics"/>'
                        '</world></sdf>')
    text = Path(launch.world_with_seed(str(declared), 7)).read_text()
    assert text.count('ignition::gazebo::systems::Physics') == 1 and '<seed>7</seed>' in text
    with pytest.raises(ValueError):
        launch.world_with_seed(str(declared), 2 ** 32)


def test_seed_copy_replaces_old_seed_before_sensors_and_preserves_asset_schemes(tmp_path, monkeypatch):
    import xml.etree.ElementTree as ET
    _fake_seed_library(tmp_path, monkeypatch)
    world=tmp_path/'assets.world'
    original='''<sdf version="1.9"><world name="w">
      <plugin filename="sensors" name="ignition::gazebo::systems::Sensors"/>
      <plugin filename="old" name="robot_lab::SimulationSeed"><seed>9</seed></plugin>
      <model name="assets"><link name="l">
        <visual name="relative"><geometry><mesh><uri> meshes/a.obj </uri></mesh></geometry></visual>
        <visual name="package"><geometry><mesh><uri> package://demo/a.obj </uri></mesh></geometry></visual>
        <visual name="web"><geometry><mesh><uri> https://example.org/a.obj </uri></mesh></geometry></visual>
      </link></model></world></sdf>'''
    world.write_text(original)
    launch=load_launch()
    copy=Path(launch.world_with_seed(str(world),42))
    root=ET.parse(copy).getroot();plugins=root.find('world').findall('plugin')
    assert [p.get('name') for p in plugins]==['robot_lab::SimulationSeed','ignition::gazebo::systems::Sensors']
    assert plugins[0].findtext('seed')=='42'
    assert [e.text.strip() for e in root.iter('uri')]==[
        str(tmp_path/'meshes/a.obj'),'package://demo/a.obj','https://example.org/a.obj']
    assert world.read_text()==original and copy!=world
    assert launch.world_with_seed(str(world),42)==str(copy)
