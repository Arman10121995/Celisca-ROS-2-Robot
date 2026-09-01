"""
P4.5 - Dynamic-obstacle and sensor-degradation variant qualification.

Validates the two P4.5 arenas (nav_dynamic with scripted moving actors,
nav_sensor_degraded with blind-corner occluding walls):
  1. Registered and qualified as integrated in environments.yaml.
  2. Dynamic metadata matches the world: nav_dynamic declares
     dynamics.dynamic_obstacles true with max_dynamic_count >= 1 and its SDF
     world contains <actor> elements; nav_sensor_degraded declares no dynamic
     obstacles and has no actors.
  3. World files are well-formed and morph into a Nav2 occupancy map (static
     geometry only, matching world<->map consistency, spawn free).
  4. Registered in sim_maps.yaml with has_2d_map.
  5. Cross-reference validation passes.
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

VARIANTS = ["nav_dynamic", "nav_sensor_degraded"]
SPAWN = {
    "nav_dynamic": (-7.0, -7.0),
    "nav_sensor_degraded": (-7.0, -7.0),
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


def _registry():
    cfg = Path(__file__).parent.parent / "config"
    r = Registry(cfg)
    r.load(cfg)
    return r


def _src():
    return Path(__file__).resolve().parents[4] / "src"


def _envs():
    return {e["id"]: e for e in _registry().environments.get_all().values()}


def _grid(envs, src, arena):
    pgm = src / envs[arena]["occupancy_map"]
    cols, rows, body = _read_pgm(pgm)
    return [list(body[r * cols:(r + 1) * cols]) for r in range(rows)], cols, rows


def _origin(envs, src, arena):
    my = (src / envs[arena]["occupancy_map"]).with_suffix(".yaml")
    for line in my.read_text().splitlines():
        if line.startswith("origin:"):
            val = line.split(":", 1)[1].strip()
            return [float(x) for x in val.strip("[]").split(",")]
    raise AssertionError(f"no origin in {my}")


def _actors_in_world(world_path):
    root = ET.parse(world_path).getroot()
    return root.findall(".//actor")


class VariantRegistrationTests(unittest.TestCase):
    def test_registered_integrated(self):
        envs = _envs()
        for a in VARIANTS:
            self.assertIn(a, envs)
            self.assertEqual(envs[a]["status"], "integrated")
            self.assertEqual(envs[a].get("ros_package"), "maps")
            self.assertTrue(envs[a]["world_file"])
            self.assertTrue(envs[a]["occupancy_map"])

    def test_dynamic_metadata_matches_world(self):
        envs = _envs()
        for a in VARIANTS:
            dyn = envs[a]["dynamics"]
            world = _src() / envs[a]["world_file"]
            n_actors = len(_actors_in_world(world))
            if a == "nav_dynamic":
                self.assertTrue(dyn["dynamic_obstacles"])
                self.assertGreaterEqual(dyn["max_dynamic_count"], 1)
                self.assertGreater(n_actors, 0, f"{a} should have actors")
            else:
                self.assertFalse(dyn["dynamic_obstacles"])
                self.assertEqual(dyn["max_dynamic_count"], 0)
                self.assertEqual(n_actors, 0, f"{a} should have no actors")


class VariantAssetTests(unittest.TestCase):
    def test_worlds_well_formed(self):
        envs = _envs()
        for a in VARIANTS:
            w = _src() / envs[a]["world_file"]
            self.assertTrue(w.is_file(), f"missing world: {w}")
            ET.parse(w)

    def test_maps_present(self):
        envs = _envs()
        for a in VARIANTS:
            pgm = _src() / envs[a]["occupancy_map"]
            self.assertTrue(pgm.is_file(), f"missing map: {pgm}")
            self.assertEqual(pgm.read_bytes()[:2], b"P5")
            self.assertTrue(pgm.with_suffix(".yaml").is_file())


class VariantConsistencyTests(unittest.TestCase):
    def test_platforms_occupied(self):
        envs = _envs()
        for a in VARIANTS:
            world = _src() / envs[a]["world_file"]
            grid, cols, rows = _grid(envs, _src(), a)
            o = _origin(envs, _src(), a)
            mhy = o[1] + rows * RES
            root = ET.parse(world).getroot()
            n = 0
            for m in root.iter("model"):
                nm = m.attrib.get("name", "")
                if not nm.startswith("obstacle_"):
                    continue
                pose = m.findtext("./pose").split()
                cx, cy = float(pose[0]), float(pose[1])
                c = int(round((cx - o[0]) / RES))
                r = int(round((mhy - cy) / RES))
                self.assertTrue(0 <= c < cols and 0 <= r < rows,
                                f"{nm} OOB in {a}")
                self.assertEqual(grid[r][c], 0,
                                 f"obstacle {nm} not occupied in {a}")
                n += 1
            self.assertGreater(n, 0, f"no obstacles in {a}")

    def test_spawn_free(self):
        envs = _envs()
        for a in VARIANTS:
            grid, cols, rows = _grid(envs, _src(), a)
            o = _origin(envs, _src(), a)
            mhy = o[1] + rows * RES
            sx, sy = SPAWN[a]
            c = int(round((sx - o[0]) / RES))
            r = int(round((mhy - sy) / RES))
            self.assertTrue(0 <= c < cols and 0 <= r < rows, f"spawn OOB {a}")
            self.assertNotEqual(grid[r][c], 0, f"spawn occupied in {a}")


class LaunchRegistrationTests(unittest.TestCase):
    def setUp(self):
        self.path = Path(__file__).resolve().parents[4] / \
            "src/robot_lab_bringup/config/sim_maps.yaml"
        import yaml
        self.maps = yaml.safe_load(self.path.read_text()).get("maps", {})

    def test_in_sim_maps(self):
        for a in VARIANTS:
            self.assertIn(a, self.maps)
            self.assertTrue(self.maps[a]["map"]["has_2d_map"])
            self.assertEqual(self.maps[a]["gazebo"]["world_package"], "maps")
            wp = self.maps[a]["gazebo"]["world_path"]
            self.assertTrue((_src() / "maps" / wp).is_file(), f"world missing {wp}")


class CrossReferenceTests(unittest.TestCase):
    def test_cross_reference_passes(self):
        result = validate_cross_references(_registry())
        self.assertTrue(result.valid, f"errors: {result.errors}")


if __name__ == "__main__":
    unittest.main()
