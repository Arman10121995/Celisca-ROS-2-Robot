"""
P4.4 - 3D / aerial course qualification.

Validates the two P4.4 3D aerial courses (aerial_course promoted from the
placeholder, aerial_indoor): registration as integrated, world well-formedness,
occupancy-map provenance, world<->map geometry consistency, sim_maps
launch registration with has_2d_map, and cross-reference validation.
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

AERIAL = ["aerial_course", "aerial_indoor"]
SPAWN = {
    "aerial_course": (-40.0, -40.0),
    "aerial_indoor": (-18.0, -18.0),
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


def _registry():
    cfg = Path(__file__).parent.parent / "config"
    r = Registry(cfg)
    r.load(cfg)
    return r


def _src():
    return Path(__file__).resolve().parents[4] / "src"


class AerialRegistrationTests(unittest.TestCase):
    def test_registered(self):
        ids = {e["id"] for e in _registry().environments.get_all().values()}
        for a in AERIAL:
            self.assertIn(a, ids)

    def test_integrated_3d(self):
        envs = {e["id"]: e for e in _registry().environments.get_all().values()}
        for a in AERIAL:
            self.assertEqual(envs[a]["status"], "integrated")
            self.assertEqual(envs[a].get("ros_package"), "robot_lab_maps")
            self.assertEqual(envs[a]["dimension"], "3D")
            self.assertTrue(envs[a]["world_file"])
            self.assertTrue(envs[a]["occupancy_map"])


class AerialAssetTests(unittest.TestCase):
    def test_worlds_well_formed(self):
        envs = {e["id"]: e for e in _registry().environments.get_all().values()}
        for a in AERIAL:
            w = _src() / envs[a]["world_file"]
            self.assertTrue(w.is_file(), f"missing world: {w}")
            ET.parse(w)

    def test_maps_present(self):
        envs = {e["id"]: e for e in _registry().environments.get_all().values()}
        for a in AERIAL:
            pgm = _src() / envs[a]["occupancy_map"]
            self.assertTrue(pgm.is_file(), f"missing map: {pgm}")
            self.assertEqual(pgm.read_bytes()[:2], b"P5")
            self.assertTrue(pgm.with_suffix(".yaml").is_file())


class AerialConsistencyTests(unittest.TestCase):
    def test_platforms_occupied(self):
        envs = {e["id"]: e for e in _registry().environments.get_all().values()}
        for a in AERIAL:
            root = ET.parse(_src() / envs[a]["world_file"]).getroot()
            grid, cols, rows = _grid(envs, _src(), a)
            o = _origin(envs, _src(), a)
            mhy = o[1] + rows * RES
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
                                 f"obstacle {nm} center not occupied in {a}")
                n += 1
            self.assertGreater(n, 0, f"no obstacles in {a}")

    def test_spawn_free(self):
        envs = {e["id"]: e for e in _registry().environments.get_all().values()}
        for a in AERIAL:
            grid, cols, rows = _grid(envs, _src(), a)
            o = _origin(envs, _src(), a)
            mhy = o[1] + rows * RES
            sx, sy = SPAWN[a]
            c = int(round((sx - o[0]) / RES))
            r = int(round((mhy - sy) / RES))
            self.assertTrue(0 <= c < cols and 0 <= r < rows,
                            f"spawn OOB in {a}")
            self.assertNotEqual(grid[r][c], 0, f"spawn occupied in {a}")


class LaunchRegistrationTests(unittest.TestCase):
    def setUp(self):
        self.path = Path(__file__).resolve().parents[4] / \
            "src/robot_lab_bringup/config/sim_maps.yaml"
        import yaml
        self.maps = yaml.safe_load(self.path.read_text()).get("maps", {})

    def test_in_sim_maps(self):
        for a in AERIAL:
            self.assertIn(a, self.maps)
            self.assertTrue(self.maps[a]["map"]["has_2d_map"])
            self.assertEqual(self.maps[a]["gazebo"]["world_package"], "robot_lab_maps")

    def test_world_paths_resolve(self):
        for a in AERIAL:
            wp = self.maps[a]["gazebo"]["world_path"]
            self.assertTrue((_src() / "robot_lab_maps" / wp).is_file(),
                            f"world missing: {wp}")


class CrossReferenceTests(unittest.TestCase):
    def test_cross_reference_passes(self):
        result = validate_cross_references(_registry())
        self.assertTrue(result.valid, f"errors: {result.errors}")


if __name__ == "__main__":
    unittest.main()
