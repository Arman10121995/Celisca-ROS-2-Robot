"""
P4.6 - Seeds, reset services, spawn zones, goals, and reference paths.

The deterministic P4.2-P4.5 arenas must expose reproducible navigation metadata:
  1. A central `arena_navigation.yaml` covers every such arena.
  2. Each arena declares seed, reset_service, goals, and reference_paths.
  3. Every goal and reference-path waypoint is free space in the arena's own
     occupancy map (i.e. Nav2 can actually navigate there).
  4. The registry environment entries carry matching seed / goals /
     reference_paths metadata.
  5. Cross-reference validation passes.
"""

import sys
import unittest
from pathlib import Path

import yaml

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

RES = 0.05

# The 12 deterministic arenas introduced in P4.2-P4.5.
ARENAS = [
    "nav_empty", "nav_obstacle", "nav_maze", "nav_narrow_passage",
    "nav_warehouse", "outdoor_terrain", "terrain_stairs",
    "terrain_stepping_stones", "aerial_course", "aerial_indoor",
    "nav_dynamic", "nav_sensor_degraded",
]


def _src():
    return Path(__file__).resolve().parents[4] / "src"


def _registry():
    cfg = Path(__file__).parent.parent / "config"
    r = Registry(cfg)
    r.load(cfg)
    return r


def _envs():
    return {e["id"]: e for e in _registry().environments.get_all().values()}


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


def _grid(envs, arena):
    pgm = _src() / envs[arena]["occupancy_map"]
    cols, rows, body = _read_pgm(pgm)
    return [list(body[r * cols:(r + 1) * cols]) for r in range(rows)], cols, rows


def _origin(envs, arena):
    my = (_src() / envs[arena]["occupancy_map"]).with_suffix(".yaml")
    for line in my.read_text().splitlines():
        if line.startswith("origin:"):
            val = line.split(":", 1)[1].strip()
            return [float(x) for x in val.strip("[]").split(",")]
    raise AssertionError(f"no origin in {my}")


def _is_free(grid, cols, rows, origin, x, y):
    ox, oy = origin[0], origin[1]
    c = int(round((x - ox) / RES))
    r = int(round((oy + rows * RES - y) / RES))
    if not (0 <= c < cols and 0 <= r < rows):
        return False
    return grid[r][c] != 0


class MetadataCoverageTests(unittest.TestCase):
    def setUp(self):
        self.path = _src() / "maps/config/arena_navigation.yaml"
        self.data = yaml.safe_load(self.path.read_text())
        self.arenas = self.data["arenas"]

    def test_file_exists(self):
        self.assertTrue(self.path.is_file(), f"missing {self.path}")

    def test_covers_all_deterministic_arenas(self):
        for a in ARENAS:
            self.assertIn(a, self.arenas, f"arena '{a}' missing from metadata")

    def test_every_arena_has_all_fields(self):
        for a in ARENAS:
            m = self.arenas[a]
            self.assertIn("seed", m, f"{a} missing seed")
            self.assertIn("reset_service", m, f"{a} missing reset_service")
            self.assertTrue(m.get("reset_service"), f"{a} empty reset_service")
            self.assertTrue(m.get("goals"), f"{a} missing goals")
            self.assertTrue(m.get("reference_paths"), f"{a} missing reference_paths")
            for g in m["goals"]:
                self.assertIn("pose", g, f"{a} goal missing pose")
            for rp in m["reference_paths"]:
                self.assertTrue(rp.get("waypoints"), f"{a} path missing waypoints")

    def test_goals_and_waypoints_free(self):
        envs = _envs()
        for a in ARENAS:
            grid, cols, rows = _grid(envs, a)
            o = _origin(envs, a)
            m = self.arenas[a]
            for g in m["goals"]:
                p = g["pose"]
                self.assertTrue(
                    _is_free(grid, cols, rows, o, p["x"], p["y"]),
                    f"goal {g['id']} in '{a}' is occupied/out-of-bounds")
            for rp in m["reference_paths"]:
                for w in rp["waypoints"]:
                    self.assertTrue(
                        _is_free(grid, cols, rows, o, w["x"], w["y"]),
                        f"waypoint {w} in '{a}' path '{rp['id']}' not free")


class RegistryMetadataTests(unittest.TestCase):
    def test_registry_entries_carry_p4_6_fields(self):
        meta = yaml.safe_load((_src() / "maps/config/arena_navigation.yaml").read_text())
        arena_meta = meta["arenas"]
        envs = _envs()
        for a in ARENAS:
            e = envs[a]
            self.assertEqual(e.get("seed"), arena_meta[a]["seed"],
                             f"registry seed mismatch for '{a}'")
            self.assertEqual(e.get("reset_service"), arena_meta[a]["reset_service"],
                             f"registry reset_service mismatch for '{a}'")
            self.assertEqual(e.get("goals"), arena_meta[a]["goals"],
                             f"registry goals mismatch for '{a}'")
            self.assertEqual(e.get("reference_paths"), arena_meta[a]["reference_paths"],
                             f"registry reference_paths mismatch for '{a}'")

    def test_spawn_zones_present(self):
        envs = _envs()
        for a in ARENAS:
            self.assertTrue(envs[a].get("spawn_zones"), f"'{a}' missing spawn_zones")


class CrossReferenceTests(unittest.TestCase):
    def test_cross_reference_passes(self):
        result = validate_cross_references(_registry())
        self.assertTrue(result.valid, f"errors: {result.errors}")


if __name__ == "__main__":
    unittest.main()
