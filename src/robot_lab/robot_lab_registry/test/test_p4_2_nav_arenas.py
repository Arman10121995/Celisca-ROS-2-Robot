"""
P4.2 - Deterministic Navigation Arena Qualification.

Validates the five newly-added deterministic navigation arenas
(nav_empty, nav_obstacle, nav_maze, nav_narrow_passage, nav_warehouse):

  1. Each is registered and qualified as `integrated` in environments.yaml.
  2. Each `world_file` resolves to an on-disk, well-formed `.world` (XML/SDF).
  3. Each ships a real occupancy PGM plus a companion map.yaml (provenance).
  4. The world geometry and the occupancy map agree: every obstacle center
     declared in the SDF rasterizes to an occupied pixel in the PGM, and the
     arena's reference spawn point is free space. This guarantees a Nav2 plan
     loaded from map.pgm is consistent with the simulated walls.
  5. Each arena is registered in sim_maps.yaml (launchable) with has_2d_map.
  6. Cross-reference validation still passes with the enlarged catalog.
"""

import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

# Prefer the source package over any stale installed copy.
_SRC_PACKAGE_DIR = Path(__file__).resolve().parent.parent
if _SRC_PACKAGE_DIR.name == "robot_lab_registry":
    sys.path[:] = [
        p for p in sys.path
        if not (p.endswith("dist-packages") and "robot_lab_registry" in p)
    ]
    if str(_SRC_PACKAGE_DIR) not in sys.path:
        sys.path.insert(0, str(_SRC_PACKAGE_DIR))
    for _stale in [m for m in list(sys.modules)
                   if m == "robot_lab_registry" or m.startswith("robot_lab_registry.")]:
        del sys.modules[_stale]

from robot_lab_registry.catalog import Registry
from robot_lab_registry.validation import validate_cross_references

# The five P4.2 deterministic navigation arenas.
ARENAS = [
    "nav_empty",
    "nav_obstacle",
    "nav_maze",
    "nav_narrow_passage",
    "nav_warehouse",
]

# Reference spawn point (x, y) for each arena (must be free space).
SPAWN = {
    "nav_empty": (-0.5, -0.5),
    "nav_obstacle": (-7.0, -7.0),
    "nav_maze": (-7.0, -7.0),
    "nav_narrow_passage": (-6.0, -6.0),
    "nav_warehouse": (-8.0, -8.0),
}


def _read_pgm(path: Path):
    """Read a binary P5 PGM; return (cols, rows, bytearray body)."""
    data = path.read_bytes()
    assert data[:2] == b"P5", "not a binary P5 PGM"
    idx = 2
    parts = []
    while len(parts) < 3:
        while data[idx] in (0x20, 0x09, 0x0A, 0x0D):
            idx += 1
        if data[idx] == 0x23:  # comment
            while data[idx] not in (0x0A, 0x0D):
                idx += 1
            continue
        tok = []
        while data[idx] not in (0x20, 0x09, 0x0A, 0x0D):
            tok.append(data[idx])
            idx += 1
        parts.append(int(bytes(tok)))
    cols, rows, _maxv = parts
    return cols, rows, bytearray(data[idx + 1:])


class ArenaRegistrationTests(unittest.TestCase):
    """All five arenas are registered and qualified as integrated."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)
        cls.src_root = Path(__file__).resolve().parents[4] / "src"

    def test_all_arenas_registered(self):
        ids = {e["id"] for e in self.registry.environments.get_all().values()}
        for arena in ARENAS:
            self.assertIn(arena, ids, f"arena '{arena}' missing from registry")

    def test_all_arenas_integrated(self):
        registered = {
            e["id"]: e for e in self.registry.environments.get_all().values()
        }
        for arena in ARENAS:
            self.assertEqual(
                registered[arena]["status"], "integrated",
                f"arena '{arena}' should be integrated after P4.2",
            )
            self.assertIn(
                registered[arena].get("ros_package"), ("maps",),
                f"arena '{arena}' needs ros_package maps",
            )
            self.assertEqual(registered[arena]["dimension"], "2D")


class ArenaAssetTests(unittest.TestCase):
    """World files and occupancy-map provenance for each arena."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)
        cls.src_root = Path(__file__).resolve().parents[4] / "src"

    def test_world_file_exists_and_well_formed(self):
        envs = {e["id"]: e for e in self.registry.environments.get_all().values()}
        for arena in ARENAS:
            env = envs[arena]
            world = self.src_root / env["world_file"]
            self.assertTrue(world.is_file(), f"missing world for '{arena}': {world}")
            self.assertTrue(env["world_file"].endswith(".world"))
            try:
                ET.parse(world)
            except ET.ParseError as exc:
                self.fail(f"'{arena}' world is malformed XML: {exc}")

    def test_occupancy_map_provenance(self):
        envs = {e["id"]: e for e in self.registry.environments.get_all().values()}
        for arena in ARENAS:
            env = envs[arena]
            occ = env["occupancy_map"]
            self.assertTrue(occ.endswith(".pgm"), f"'{arena}' must be a .pgm")
            pgm = self.src_root / occ
            self.assertTrue(pgm.is_file(), f"missing PGM for '{arena}': {pgm}")
            map_yaml = Path(str(pgm).replace(".pgm", ".yaml"))
            self.assertTrue(map_yaml.is_file(), f"missing map.yaml for '{arena}'")
            content = map_yaml.read_text()
            self.assertIn(
                occ.rsplit("/", 1)[-1], content,
                f"map.yaml for '{arena}' must reference the PGM image",
            )

class ArenaGeometryMapConsistencyTests(unittest.TestCase):
    """World obstacles rasterize to occupied map cells; spawn is free."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)
        cls.src_root = Path(__file__).resolve().parents[4] / "src"
        cls.envs = {e["id"]: e for e in cls.registry.environments.get_all().values()}

    def _map_dims(self, pgm_path):
        map_yaml = Path(str(pgm_path).replace(".pgm", ".yaml")).read_text()
        origin_line = [l for l in map_yaml.splitlines() if l.startswith("origin")][0]
        origin_x = float(origin_line.split("[")[1].split(",")[0])
        origin_y = float(origin_line.split("[")[1].split(",")[1])
        res = float(
            [l for l in map_yaml.splitlines() if l.startswith("resolution")][0]
            .split(":")[1]
        )
        return origin_x, origin_y, res

    def test_every_obstacle_center_is_occupied(self):
        for arena in ARENAS:
            env = self.envs[arena]
            world = self.src_root / env["world_file"]
            pgm_path = self.src_root / env["occupancy_map"]
            origin_x, origin_y, res = self._map_dims(pgm_path)
            cols, rows, body = _read_pgm(pgm_path)
            map_hi_y = origin_y + rows * res

            root = ET.parse(world).getroot()
            obstacles = [
                m for m in root.iter("model")
                if m.get("name", "").startswith("obstacle_")
            ]
            self.assertGreaterEqual(
                len(obstacles), 6,
                f"'{arena}' should contain wall/obstacle primitives, "
                f"found {len(obstacles)}",
            )
            for model in obstacles:
                cx, cy, *_ = (float(v) for v in model.find("pose").text.split())
                c = int(round((cx - origin_x) / res))
                r = int(round((map_hi_y - cy) / res))
                self.assertTrue(
                    body[r * cols + c] == 0,
                    f"'{arena}' obstacle at ({cx},{cy}) not occupied in map",
                )

    def test_spawn_point_is_free(self):
        for arena in ARENAS:
            env = self.envs[arena]
            pgm_path = self.src_root / env["occupancy_map"]
            origin_x, origin_y, res = self._map_dims(pgm_path)
            cols, rows, body = _read_pgm(pgm_path)
            map_hi_y = origin_y + rows * res
            sx, sy = SPAWN[arena]
            c = int(round((sx - origin_x) / res))
            r = int(round((map_hi_y - sy) / res))
            self.assertNotEqual(
                body[r * cols + c], 0,
                f"'{arena}' spawn point ({sx},{sy}) is inside an obstacle",
            )


class LaunchRegistrationTests(unittest.TestCase):
    """Each arena is launchable via sim_maps.yaml."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)
        cls.src_root = Path(__file__).resolve().parents[4] / "src"
        cls.sim_maps = (
            cls.src_root / "bumperbot_bringup" / "config" / "sim_maps.yaml"
        )

    def test_all_arenas_in_sim_maps(self):
        import yaml
        data = yaml.safe_load(self.sim_maps.read_text())
        maps_ = data["maps"]
        for arena in ARENAS:
            self.assertIn(arena, maps_, f"arena '{arena}' missing from sim_maps.yaml")
            profile = maps_[arena]
            self.assertTrue(
                profile["map"].get("has_2d_map", False),
                f"'{arena}' should declare has_2d_map: true",
            )
            self.assertEqual(profile["gazebo"]["world_package"], "maps")
            world_rel = profile["gazebo"]["world_path"]
            self.assertTrue(
                (self.src_root / "maps" / world_rel).is_file(),
                f"'{arena}' world_path not found: {world_rel}",
            )
            map_rel = profile["map"]["path"]
            self.assertTrue(
                (self.src_root / "maps" / map_rel).is_file(),
                f"'{arena}' map path not found: {map_rel}",
            )


class CrossReferenceTests(unittest.TestCase):
    """Cross-reference validation still passes with the enlarged catalog."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)

    def test_cross_reference_validation_passes(self):
        result = validate_cross_references(self.registry)
        self.assertTrue(
            result.valid,
            f"cross-reference validation fails after P4.2: {result.errors}",
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)

