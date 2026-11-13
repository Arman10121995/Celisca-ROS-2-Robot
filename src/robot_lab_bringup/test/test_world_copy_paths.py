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
