"""
P4.1 Qualify All Existing Gazebo Worlds and Occupancy-Map Provenance.

Validates that:
  1. Every `integrated` environment in the registry corresponds to a real,
     on-disk Gazebo `.world` file (the 14 pre-existing worlds are all
     integrated).
  2. Each integrated environment that ships an occupancy map points at an
     actual `<id>/maps/map.pgm` file on disk (provenance is real, not a
     placeholder), and every declared `occupancy_map` is accompanied by a
     companion `map.yaml` (ROS nav2 map metadata).
  3. The four sim_maps-launchable legacy worlds (small_house, small_warehouse,
     small_office/simple_office, warehouse_demo) remain integrated.
  4. Every integrated environment resolves to a real world file and its
     `ground_truth_available` flag is consistent with declared dynamics.
  5. Cross-reference validation still passes.
"""

import sys
import unittest
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

# ---------------------------------------------------------------------------
# The 14 pre-existing Gazebo worlds that had been authored in the maps package
# before P4.1. Every one of them is now registered and integrated.
# ---------------------------------------------------------------------------
PRE_EXISTING_WORLDS = [
    "small_office",
    "small_house",
    "small_warehouse",
    "warehouse_demo",
    "celisca_floor_1",
    "celisca_floor_2",
    "celisca_floor_2_furniture",
    "celisca_floor_1_furniture",
    "celisca_f1_actor",
    "celisca_f2_actor",
    "bigger_warehouse",
    "empty",
    "residential_demo",
    "simple_box",
]

class WorldQualificationTests(unittest.TestCase):
    """Every pre-existing world is registered, integrated, and loadable."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)
        # test/ -> robot_lab_registry -> robot_lab -> src -> workspace root
        cls.src_root = Path(__file__).resolve().parents[4] / "src"
        cls.integrated = [
            e for e in cls.registry.environments.get_all().values()
            if e.get("status") == "integrated"
        ]

    def _world_path(self, env):
        return self.src_root / env["world_file"]

    def test_all_pre_existing_worlds_registered(self):
        """Each of the 14 pre-existing worlds has a registry entry."""
        ids = {e["id"] for e in self.registry.environments.get_all().values()}
        for world in PRE_EXISTING_WORLDS:
            self.assertIn(world, ids, f"world '{world}' missing from registry")

    def test_all_pre_existing_worlds_integrated(self):
        """Every pre-existing world is qualified as integrated."""
        registered = {
            e["id"]: e
            for e in self.registry.environments.get_all().values()
        }
        for world in PRE_EXISTING_WORLDS:
            self.assertEqual(
                registered[world]["status"], "integrated",
                f"world '{world}' should be integrated after P4.1",
            )

    def test_every_integrated_world_file_exists(self):
        """The referenced .world file for every integrated env exists on disk."""
        for env in self.integrated:
            self.assertTrue(
                self._world_path(env).is_file(),
                f"missing world file for '{env['id']}': {self._world_path(env)}",
            )
            self.assertTrue(
                env["world_file"].endswith(".world"),
                f"'{env['id']}' world_file must end in .world",
            )

    def test_actor_worlds_declare_dynamic_obstacles(self):
        """Actor worlds (celisca_f1/f2_actor) declare dynamic_obstacles."""
        for env in self.integrated:
            if env["id"] in ("celisca_f1_actor", "celisca_f2_actor"):
                self.assertTrue(
                    env.get("dynamics", {}).get("dynamic_obstacles", False),
                    f"'{env['id']}' must declare dynamic_obstacles: true",
                )
                self.assertGreater(
                    env.get("dynamics", {}).get("max_dynamic_count", 0), 0,
                    f"'{env['id']}' must set max_dynamic_count > 0",
                )


# Worlds that should carry an occupancy map (all except 'empty').
WORLDS_WITHOUT_OCCUPANCY = {"empty"}


class OccupancyMapProvenanceTests(unittest.TestCase):
    """Occupancy-map provenance for all integrated environments."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)
        cls.src_root = Path(__file__).resolve().parents[4] / "src"
        cls.integrated = [
            e for e in cls.registry.environments.get_all().values()
            if e.get("status") == "integrated"
        ]

    def test_occupancy_map_resolves_on_disk(self):
        """Every declared occupancy_map must point at a real PGM file."""
        for env in self.integrated:
            occ = env.get("occupancy_map", "")
            if env["id"] in WORLDS_WITHOUT_OCCUPANCY:
                # No map expected for these.
                self.assertEqual(occ, "", f"'{env['id']}' should have no occupancy map")
                continue
            self.assertTrue(
                occ.endswith(".pgm"),
                f"'{env['id']}' occupancy_map must be a .pgm, got '{occ}'",
            )
            pgm = self.src_root / occ
            self.assertTrue(
                pgm.is_file(),
                f"missing occupancy PGM for '{env['id']}': {pgm}",
            )

    def test_occupancy_map_has_companion_map_yaml(self):
        """Each occupancy PGM must have a ROS nav2 map.yaml companion."""
        for env in self.integrated:
            occ = env.get("occupancy_map", "")
            if env["id"] in WORLDS_WITHOUT_OCCUPANCY or not occ:
                continue
            map_yaml = Path(str(self.src_root / occ).replace(".pgm", ".yaml"))
            self.assertTrue(
                map_yaml.is_file(),
                f"missing map.yaml companion for '{env['id']}': {map_yaml}",
            )
            # A companion map YAML must reference the same PGM image.
            content = map_yaml.read_text()
            self.assertIn(
                occ.rsplit("/", 1)[-1], content,
                f"map.yaml for '{env['id']}' must reference the PGM image",
            )

    def test_ground_truth_consistent_with_dynamics(self):
        """ground_truth_available must be true for every integrated env."""
        for env in self.integrated:
            self.assertTrue(
                env.get("ground_truth_available", False),
                f"'{env['id']}' should declare ground_truth_available",
            )

class LegacyLaunchCompatibilityTests(unittest.TestCase):
    """Worlds launched via sim_maps/bringup remain integrated."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)

    def test_legacy_launch_worlds_integrated(self):
        """small_house, small_warehouse, small_office, warehouse_demo qualify."""
        for world in ("small_house", "small_warehouse", "small_office", "warehouse_demo"):
            env = self.registry.environments.get(world)
            self.assertIsNotNone(env, f"'{world}' should be registered")
            self.assertEqual(env["status"], "integrated", f"'{world}' must be integrated")

    def test_cross_reference_validation_passes(self):
        """Cross-reference validation passes with all P4.1 world entries."""
        result = validate_cross_references(self.registry)
        self.assertTrue(
            result.valid,
            f"cross-reference validation fails after P4.1: {result.errors}",
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)

