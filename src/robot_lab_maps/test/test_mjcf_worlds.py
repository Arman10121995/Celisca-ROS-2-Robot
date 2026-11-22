"""Every registered map must exist for the MuJoCo backend.

MuJoCo needs an MJCF description of the environment.  When one is missing the
backend falls back to an empty stage, which looks exactly like a world that
loaded but is empty - so the generated worlds are checked in and their
agreement with the Gazebo sources is asserted here rather than discovered at
runtime.
"""
import os
import sys
import unittest
import xml.etree.ElementTree as ET

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PACKAGE = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_PACKAGE, "tools"))

import gen_mjcf_worlds  # noqa: E402

_MAPS_DIR = os.path.join(_PACKAGE, "maps")
_MJCF_DIR = os.path.join(_PACKAGE, "mjcf")


def _source_worlds():
    worlds = {}
    for name in sorted(os.listdir(_MAPS_DIR)):
        path = os.path.join(_MAPS_DIR, name, "worlds", name + ".world")
        if os.path.isfile(path):
            worlds[name] = path
    return worlds


def test_compound_has_one_free_joint_and_keeps_all_collision_frames(tmp_path):
    from robot_lab_utils.sdf_world import extract_dynamic_bodies, extract_static_shapes

    world = tmp_path / "compound.world"
    world.write_text("""<sdf version="1.7"><world name="fixture">
        <model name="tool"><static>false</static><pose>1 2 3 0 0 0</pose>
          <link name="body"><pose>0.1 0 0 0 0 0</pose>
            <inertial><mass>0.45</mass></inertial>
            <collision name="block">
              <geometry><box><size>0.2 0.3 0.4</size></box></geometry>
              <surface><friction><ode><mu>0.6</mu></ode></friction></surface>
            </collision>
            <collision name="knob"><pose>0.3 0 0.1 0 0 0</pose>
              <geometry><sphere><radius>0.04</radius></sphere></geometry>
            </collision>
            <collision name="handle"><pose>0 0.2 0 0 1.5707963267948966 0</pose>
              <geometry><cylinder><radius>0.05</radius><length>0.3</length></cylinder></geometry>
            </collision>
          </link>
        </model>
        <model name="mixed"><static>false</static><link name="body">
          <inertial><mass>1</mass></inertial>
          <collision name="base"><geometry><box><size>1 1 1</size></box></geometry></collision>
          <collision name="unsupported"><geometry><plane><size>1 1</size></plane></geometry></collision>
        </link></model>
    </world></sdf>""")
    body, = extract_dynamic_bodies(world)
    assert body["name"] == "tool" and body["shape"] is None
    assert body["mu"] == 0.6 and body["inertia"] is None
    shapes, _ = extract_static_shapes(world, lambda _uri, _base: "", exclude_dynamic=True)
    assert {shape["model"] for shape in shapes} == {"mixed"}

    generated, _ = gen_mjcf_worlds.convert_world(world)
    compiled = ET.fromstring(generated).find("worldbody/body")
    assert compiled.get("pos") == "1 2 3"
    assert compiled.find("freejoint").get("name") == "tool_free"
    assert compiled.find("inertial") is None
    geoms = compiled.findall("geom")
    assert [geom.get("type") for geom in geoms] == ["box", "sphere", "cylinder"]
    assert [geom.get("pos") for geom in geoms] == ["0.1 0 0", "0.4 0 0.1", "0.1 0.2 0"]
    assert geoms[2].get("quat") == "0.707107 0 0.707107 0"
    assert sum(float(geom.get("mass")) for geom in geoms) == pytest.approx(0.45)
    assert geoms[0].get("friction") == "0.6 0.005 0.0001"


class MjcfWorldTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.worlds = _source_worlds()

    def test_every_gazebo_world_has_an_mjcf_world(self):
        missing = [name for name in self.worlds
                   if not os.path.isfile(os.path.join(_MJCF_DIR, name + ".xml"))]
        self.assertEqual([], missing,
                         "run robot_lab_maps/tools/gen_mjcf_worlds.py")

    def test_checked_in_mjcf_matches_the_generator(self):
        """The committed MJCF is what the Gazebo source currently produces."""
        stale = []
        for name, path in self.worlds.items():
            generated, _ = gen_mjcf_worlds.convert_world(path, name)
            with open(os.path.join(_MJCF_DIR, name + ".xml")) as handle:
                if handle.read() != generated:
                    stale.append(name)
        self.assertEqual([], stale, "regenerate with gen_mjcf_worlds.py")

    def test_every_mjcf_world_is_well_formed_and_has_geometry(self):
        for name in self.worlds:
            with self.subTest(world=name):
                root = ET.parse(os.path.join(_MJCF_DIR, name + ".xml")).getroot()
                self.assertEqual("mujoco", root.tag)
                worldbody = root.find("worldbody")
                self.assertIsNotNone(worldbody)
                self.assertTrue(worldbody.findall("geom"),
                                "%s has no geometry" % name)

    def test_mesh_assets_resolve_to_real_files(self):
        """A referenced mesh that does not exist becomes an invisible gap."""
        missing = []
        for name in self.worlds:
            root = ET.parse(os.path.join(_MJCF_DIR, name + ".xml")).getroot()
            for mesh in root.iter("mesh"):
                path = mesh.get("file") or ""
                if path and not os.path.isfile(path):
                    missing.append("%s: %s" % (name, path))
        self.assertEqual([], missing)

    def test_obstacle_geometry_is_carried_over_from_the_source_world(self):
        """A box arena must not silently convert to a bare ground plane."""
        for name in ("nav_maze", "nav_obstacle", "nav_warehouse",
                     "terrain_stairs", "aerial_indoor"):
            with self.subTest(world=name):
                source = ET.parse(self.worlds[name]).getroot()
                boxes = sum(1 for el in source.iter()
                            if el.tag.rsplit("}", 1)[-1] == "box")
                root = ET.parse(os.path.join(_MJCF_DIR, name + ".xml")).getroot()
                geoms = root.find("worldbody").findall("geom")
                # Collision and visual describe the same box, so the MJCF
                # carries at most one geom per source box, and at least a
                # meaningful fraction of them.
                self.assertGreaterEqual(len(geoms), max(1, boxes // 2),
                                        "%s lost obstacle geometry" % name)

    def test_fallback_plane_is_visual_only_when_ground_box_exists(self):
        """Do not duplicate a source ground collision with a plane contact."""
        generated, _ = gen_mjcf_worlds.convert_world(
            self.worlds["nav_empty"], "nav_empty")
        root = ET.fromstring(generated)
        planes = [geom for geom in root.find("worldbody").findall("geom")
                  if geom.get("type") == "plane"]
        self.assertEqual(1, len(planes))
        self.assertEqual("0", planes[0].get("contype"))
        self.assertEqual("0", planes[0].get("conaffinity"))

    def test_visual_fallback_never_replaces_all_collision_geometry(self):
        for name, path in self.worlds.items():
            generated, _ = gen_mjcf_worlds.convert_world(path, name)
            root = ET.fromstring(generated)
            geoms = root.find("worldbody").findall("geom")
            fallbacks = [geom for geom in geoms
                         if geom.get("type") == "plane"
                         and geom.get("contype") == "0"
                         and geom.get("conaffinity") == "0"]
            if fallbacks:
                self.assertTrue(
                    any(geom.get("type") != "plane" for geom in geoms),
                    f"{name}: visual fallback removed all collision geometry")

    def test_actors_are_skipped_and_reported(self):
        """Scripted movers are not static geometry; they must not be silent."""
        _, skipped = gen_mjcf_worlds.convert_world(
            self.worlds["nav_dynamic"], "nav_dynamic")
        self.assertTrue(any("actor" in note for note in skipped),
                        "moving actors were dropped without a note")


if __name__ == "__main__":
    unittest.main()
