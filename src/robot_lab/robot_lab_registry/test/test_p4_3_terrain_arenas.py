"""
P4.3 - Rough terrain / stairs / stepping-stone arena qualification.
Validates the three new P4.3 terrain arenas for legged/humanoid robots:
  1. Registered and qualified as `integrated` in environments.yaml.
  2. Each `world_file` resolves to an on-disk, well-formed `.world` (XML/SDF).
  3. Each ships a real occupancy PGM plus a companion map.yaml.
  4. World geometry and occupancy map agree (platforms occupied, spawn free).
  5. Each arena is registered in sim_maps.yaml with has_2d_map.
  6. Cross-reference validation passes.
"""

import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

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

TERRAIN = [
    "outdoor_terrain",
    "terrain_stairs",
    "terrain_stepping_stones",
]

SPAWN = {
    "outdoor_terrain": (-9.0, -9.0),
    "terrain_stairs": (-7.0, -5.0),
    "terrain_stepping_stones": (-8.0, -6.0),
}


RES = 0.05


def _read_pgm(path):
    data = path.read_bytes()
    assert data[:2] == b"P5", "not a binary P5 PGM"
    idx = 2
    parts = []
    while len(parts) < 3:
        while data[idx] in (0x20, 0x09, 0x0A, 0x0D):
            idx += 1
        if data[idx] == 0x23:
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


def _grid_for(envs, src_root, arena):
    pgm = src_root / envs[arena]["occupancy_map"]
    cols, rows, body = _read_pgm(pgm)
    grid = [list(body[r * cols:(r + 1) * cols]) for r in range(rows)]
    return grid, cols, rows


def _origin_for(envs, src_root, arena):
    map_yaml = src_root / envs[arena]["occupancy_map"]
    map_yaml = map_yaml.with_suffix(".yaml")
    for line in map_yaml.read_text().splitlines():
        if line.startswith("origin:"):
            val = line.split(":", 1)[1].strip()
            return [float(x) for x in val.strip("[]").split(",")]
    raise AssertionError(f"no origin in {map_yaml}")


class TerrainRegistrationTests(unittest.TestCase):
    """All three arenas registered and integrated."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)

    def test_all_terrains_registered(self):
        ids = {e["id"] for e in self.registry.environments.get_all().values()}
        for a in TERRAIN:
            self.assertIn(a, ids, f"'{a}' missing")

    def test_all_terrains_integrated(self):
        reg = {e["id"]: e for e in self.registry.environments.get_all().values()}
        for a in TERRAIN:
            self.assertEqual(reg[a]["status"], "integrated")
            self.assertEqual(reg[a].get("ros_package"), "maps")
            self.assertEqual(reg[a]["dimension"], "3D")
            self.assertTrue(reg[a]["world_file"])
            self.assertTrue(reg[a]["occupancy_map"])


class TerrainAssetTests(unittest.TestCase):
    """World files and occupancy-map provenance."""

    def setUp(self):
        config_dir = Path(__file__).parent.parent / "config"
        self.registry = Registry(config_dir)
        self.registry.load(config_dir)
        self.src = Path(__file__).resolve().parents[4] / "src"

    def test_world_files_exist(self):
        envs = {e["id"]: e for e in self.registry.environments.get_all().values()}
        for a in TERRAIN:
            w = self.src / envs[a]["world_file"]
            self.assertTrue(w.is_file(), f"missing world: {w}")
            ET.parse(w)  # well-formed XML

    def test_occupancy_maps_exist(self):
        envs = {e["id"]: e for e in self.registry.environments.get_all().values()}
        for a in TERRAIN:
            pgm = self.src / envs[a]["occupancy_map"]
            self.assertTrue(pgm.is_file(), f"missing map: {pgm}")
            self.assertEqual(pgm.read_bytes()[:2], b"P5")
            self.assertTrue(pgm.with_suffix(".yaml").is_file())


class TerrainGeometryMapConsistencyTests(unittest.TestCase):
    """Platforms occupied, spawn free."""

    def setUp(self):
        config_dir = Path(__file__).parent.parent / "config"
        self.registry = Registry(config_dir)
        self.registry.load(config_dir)
        self.src = Path(__file__).resolve().parents[4] / "src"
        self.envs = {e["id"]: e for e in self.registry.environments.get_all().values()}

    def test_platforms_occupied(self):
        for a in TERRAIN:
            root = ET.parse(self.src / self.envs[a]["world_file"]).getroot()
            grid, cols, rows = _grid_for(self.envs, self.src, a)
            origin = _origin_for(self.envs, self.src, a); ox, oy = origin[0], origin[1]
            mhy = oy + rows * RES
            n = 0
            for m in root.iter("model"):
                nm = m.attrib.get("name", "")
                if not nm.startswith("platform_"):
                    continue
                pose = m.findtext("./pose").split()
                cx, cy = float(pose[0]), float(pose[1])
                c = int(round((cx - ox) / RES))
                r = int(round((mhy - cy) / RES))
                self.assertTrue(0 <= c < cols and 0 <= r < rows,
                                f"{nm} OOB in {a}")
                self.assertEqual(grid[r][c], 0,
                                 f"platform {nm} center not occupied in {a}")
                n += 1
            self.assertGreater(n, 0, f"no platforms in {a}")

    def test_spawn_free(self):
        for a in TERRAIN:
            grid, cols, rows = _grid_for(self.envs, self.src, a)
            origin = _origin_for(self.envs, self.src, a); ox, oy = origin[0], origin[1]
            mhy = oy + rows * RES
            sx, sy = SPAWN[a]
            c = int(round((sx - ox) / RES))
            r = int(round((mhy - sy) / RES))
            self.assertTrue(0 <= c < cols and 0 <= r < rows,
                            f"spawn OOB in {a}")
            self.assertNotEqual(grid[r][c], 0,
                                f"spawn occupied in {a}")


class LaunchRegistrationTests(unittest.TestCase):
    """sim_maps.yaml registration."""

    def setUp(self):
        self.path = Path(__file__).resolve().parents[4] / \
            "src/robot_lab_bringup/config/sim_maps.yaml"
        import yaml
        self.data = yaml.safe_load(self.path.read_text())
        self.maps = self.data.get("maps", {})

    def test_all_in_sim_maps(self):
        for a in TERRAIN:
            self.assertIn(a, self.maps)
            self.assertTrue(self.maps[a]["map"]["has_2d_map"])
            self.assertEqual(self.maps[a]["gazebo"]["world_package"], "maps")

    def test_world_paths_resolve(self):
        src = Path(__file__).resolve().parents[4] / "src"
        for a in TERRAIN:
            wp = self.maps[a]["gazebo"]["world_path"]
            self.assertTrue((src / "maps" / wp).is_file(),
                            f"world missing: {wp}")


class CrossReferenceTests(unittest.TestCase):
    """Cross-reference validation passes."""

    def test_cross_reference_passes(self):
        config_dir = Path(__file__).parent.parent / "config"
        reg = Registry(config_dir)
        reg.load(config_dir)
        result = validate_cross_references(reg)
        self.assertTrue(result.valid, f"errors: {result.errors}")


if __name__ == "__main__":
    unittest.main()
