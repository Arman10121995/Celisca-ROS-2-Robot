"""The shared SDF reader must describe the same world the MJCF generator does.

Both read the Gazebo source worlds; if they disagree, two backends show two
different maps for the same map name.
"""
import os
import sys
import unittest
import xml.etree.ElementTree as ET

_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_SRC, "robot_lab_utils"))
sys.path.insert(0, os.path.join(_SRC, "robot_lab_maps", "tools"))

from robot_lab_utils import sdf_world  # noqa: E402
import gen_mjcf_worlds  # noqa: E402

_MAPS = os.path.join(_SRC, "robot_lab_maps", "maps")
_MJCF = os.path.join(_SRC, "robot_lab_maps", "mjcf")


def _world(name):
    return os.path.join(_MAPS, name, "worlds", name + ".world")


def _mjcf_geoms(name):
    """Generated MJCF geoms, minus the ground plane the generator synthesizes."""
    geoms = ET.parse(os.path.join(_MJCF, name + ".xml")).getroot().find("worldbody").findall("geom")
    return [g for g in geoms
            if not (g.get("type") == "plane" and g.get("pos") is None)]


class SdfWorldTests(unittest.TestCase):

    def _shapes(self, name):
        return sdf_world.extract_static_shapes(
            _world(name), lambda uri, base: gen_mjcf_worlds._resolve_uri(uri, base))

    def test_box_arena_matches_generated_mjcf_shape_for_shape(self):
        for name in ("nav_maze", "nav_warehouse", "terrain_stairs", "aerial_indoor"):
            with self.subTest(world=name):
                shapes, _ = self._shapes(name)
                geoms = _mjcf_geoms(name)
                self.assertEqual(len(geoms), len(shapes))
                expected = sorted(tuple(round(float(v), 3) for v in g.get("pos").split())
                                  for g in geoms)
                actual = sorted(tuple(round(v, 3) for v in s["position"])
                                for s in shapes)
                self.assertEqual(expected, actual)

    def test_include_worlds_resolve_their_models(self):
        shapes, skipped = self._shapes("small_house")
        self.assertEqual([], [n for n in skipped if "unresolved" in n])
        meshes = [s for s in shapes if s["type"] == "mesh"]
        self.assertTrue(meshes)
        self.assertTrue(all(os.path.isfile(s["mesh"]) for s in meshes))
        self.assertEqual(len(_mjcf_geoms("small_house")), len(shapes))

    def test_box_sizes_are_full_extents(self):
        shapes, _ = self._shapes("nav_maze")
        geoms = {tuple(round(float(v), 3) for v in g.get("pos").split()): g
                 for g in _mjcf_geoms("nav_maze")}
        for shape in shapes:
            geom = geoms[tuple(round(v, 3) for v in shape["position"])]
            half = [float(v) for v in geom.get("size").split()]
            for full, half_extent in zip(shape["size"], half):
                self.assertAlmostEqual(full / 2.0, half_extent, places=3)

    def test_actors_are_reported_not_dropped(self):
        _, skipped = self._shapes("nav_dynamic")
        self.assertTrue(any("actor" in note for note in skipped))

    def test_orientation_is_a_unit_quaternion(self):
        shapes, _ = self._shapes("aerial_indoor")
        for shape in shapes:
            norm = sum(c * c for c in shape["orientation"]) ** 0.5
            self.assertAlmostEqual(1.0, norm, places=6)


if __name__ == "__main__":
    unittest.main()


def test_occupancy_uses_collision_geometry_without_display_fallback(tmp_path):
    world=tmp_path/'world.sdf'
    world.write_text('<sdf version="1.7"><world name="test"><model name="model">'
        '<link name="decorative"><visual name="v"><geometry><box><size>2 2 2</size></box></geometry></visual></link>'
        '<link name="solid"><collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision></link>'
        '</model></world></sdf>')
    visible,_=sdf_world.extract_static_shapes(str(world),lambda uri,base:uri)
    collision,_=sdf_world.extract_static_shapes(str(world),lambda uri,base:uri,collision_only=True)
    assert len(visible)==2
    assert len(collision)==1 and collision[0]['size']==[1,1,1]


def test_loose_primitive_objects_are_dynamic_bodies_not_scenery(tmp_path):
    from robot_lab_utils.sdf_world import extract_dynamic_bodies, extract_static_shapes
    world = tmp_path / 'w.world'
    world.write_text('''<sdf version="1.7"><world name="w">
      <model name="table"><static>true</static><pose>1 0 0.25 0 0 0</pose><link name="l">
        <collision name="c"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision></link></model>
      <model name="cube"><static>false</static><pose>1 0 0.52 0 0 0</pose><link name="l">
        <inertial><mass>0.05</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz></inertia></inertial>
        <collision name="c"><geometry><box><size>0.04 0.04 0.04</size></box></geometry>
          <surface><friction><ode><mu>0.9</mu></ode></friction></surface></collision></link></model>
      <model name="massless"><static>false</static><link name="l">
        <collision name="c"><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link></model>
    </world></sdf>''')
    bodies = extract_dynamic_bodies(str(world))
    assert [b['name'] for b in bodies] == ['cube']              # no mass: not a free body
    cube = bodies[0]
    assert cube['position'] == [1.0, 0.0, 0.52] and cube['mass'] == 0.05 and cube['mu'] == 0.9
    assert cube['shape']['type'] == 'box' and cube['shape']['size'] == [0.04, 0.04, 0.04]
    every, _ = extract_static_shapes(str(world), lambda uri, base: '')
    scenery, _ = extract_static_shapes(str(world), lambda uri, base: '', exclude_dynamic=True)
    assert {s['model'] for s in every} == {'table', 'cube', 'massless'}          # unchanged default
    assert {s['model'] for s in scenery} == {'table', 'massless'}


def test_static_collision_friction_is_reported(tmp_path):
    """PyBullet builds its own static bodies and needs the declared mu (MOLAR station, 2026-11-17)."""
    world = tmp_path / "w.world"
    world.write_text(
        '<sdf version="1.7"><world name="w"><model name="station"><static>true</static><link name="l">'
        '<collision name="c"><geometry><box><size>1 1 1</size></box></geometry>'
        '<surface><friction><ode><mu>0.05</mu></ode></friction></surface></collision>'
        '<visual name="v"><geometry><box><size>1 1 1</size></box></geometry></visual></link></model>'
        '<model name="wall"><static>true</static><link name="l"><collision name="c">'
        '<geometry><box><size>1 1 1</size></box></geometry></collision></link></model></world></sdf>')
    shapes, _ = sdf_world.extract_static_shapes(str(world), lambda uri, base: "")
    friction = {shape["model"]: shape.get("mu") for shape in shapes}
    assert friction == {"station": 0.05, "wall": None}
