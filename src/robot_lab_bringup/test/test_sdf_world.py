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
