"""
R3.2 Typed Composition Compatibility Tests

Acceptance criteria (ROADMAP.md R3.2):
- Unknown simulator, AMCL in global_planning and missing required LiDAR are
  rejected; valid combinations pass with actionable diagnostics; 2D planners
  cannot gain flight capability by a metadata label.

These tests exercise the single shared validator (validation.check_composition)
used by the CLI and the adapter/GUI composition flows.
"""

import sys
import unittest
from pathlib import Path

# Prefer the source package over any stale installed copy (e.g. a colcon
# install injected through PYTHONPATH) when this file is executed directly.
_SRC_PACKAGE_DIR = Path(__file__).resolve().parent.parent
if _SRC_PACKAGE_DIR.name == "robot_lab_registry":
    sys.path[:] = [
        p for p in sys.path
        if not (p.endswith("dist-packages") and "robot_lab_registry" in p)
    ]
    if str(_SRC_PACKAGE_DIR) not in sys.path:
        sys.path.insert(0, str(_SRC_PACKAGE_DIR))
    for _stale in [m for m in list(sys.modules) if m == "robot_lab_registry" or m.startswith("robot_lab_registry.")]:
        del sys.modules[_stale]

from robot_lab_registry.catalog import Registry
from robot_lab_registry.validation import (
    Composition,
    check_capabilities,
    check_composition,
    check_dimension_compatibility,
    validate_experiment,
)


def make_composition(robot_id="bumperbot", environment_id="small_office",
                     simulator="gazebo", **algorithm_ids):
    """Build a Composition with sensible defaults for R3.2 tests."""
    return Composition(
        robot_id=robot_id,
        environment_id=environment_id,
        simulator=simulator,
        algorithm_ids=algorithm_ids,
    )


class R32TypedCompositionTests(unittest.TestCase):
    """R3.2: typed composition compatibility acceptance tests."""

    @classmethod
    def setUpClass(cls):
        """Load registry once for all tests."""
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)

    # ------------------------------------------------------------------
    # Simulator typing
    # ------------------------------------------------------------------

    def test_unknown_simulator_rejected(self):
        """A simulator outside the known set must be rejected."""
        result = check_composition(
            self.registry,
            make_composition(simulator="holodeck", global_planning="navfn_planner"),
        )
        self.assertFalse(result.valid)
        self.assertTrue(
            any("Unknown simulator 'holodeck'" in err for err in result.errors),
            f"expected an unknown-simulator diagnostic, got: {result.errors}",
        )

    def test_missing_simulator_rejected(self):
        """A composition must declare an explicit simulator."""
        result = check_composition(
            self.registry,
            make_composition(simulator=""),
        )
        self.assertFalse(result.valid)
        self.assertTrue(
            any("explicit simulator" in err for err in result.errors),
            f"expected an explicit-simulator diagnostic, got: {result.errors}",
        )

    def test_simulator_environment_mismatch_rejected(self):
        """A composition simulator that contradicts the environment is rejected."""
        result = check_composition(
            self.registry,
            make_composition(simulator="pybullet"),
        )
        self.assertFalse(result.valid)
        self.assertTrue(
            any(
                "authored for simulator 'gazebo'" in err and "'pybullet'" in err
                for err in result.errors
            ),
            f"expected a simulator-mismatch diagnostic, got: {result.errors}",
        )

    def test_matching_simulator_accepted(self):
        """The environment's own simulator passes the simulator check."""
        result = check_composition(self.registry, make_composition())
        self.assertFalse(
            any("simulator" in err for err in result.errors),
            f"no simulator errors expected, got: {result.errors}",
        )

    # ------------------------------------------------------------------
    # Algorithm category slots
    # ------------------------------------------------------------------

    def test_amcl_in_global_planning_rejected(self):
        """AMCL (localization) assigned to the global_planning slot is rejected."""
        result = check_composition(
            self.registry,
            make_composition(global_planning="amcl"),
        )
        self.assertFalse(result.valid)
        self.assertTrue(
            any(
                "amcl" in err
                and "localization" in err
                and "global_planning" in err
                for err in result.errors
            ),
            f"expected a category-mismatch diagnostic, got: {result.errors}",
        )

    def test_category_match_passes(self):
        """AMCL in its own localization slot produces no category error."""
        result = check_composition(
            self.registry,
            make_composition(localization="amcl"),
        )
        self.assertFalse(
            any("category" in err for err in result.errors),
            f"no category errors expected, got: {result.errors}",
        )

    # ------------------------------------------------------------------
    # Sensor requirements
    # ------------------------------------------------------------------

    def test_missing_required_lidar_rejected(self):
        """A scan-based algorithm on a robot without a LiDAR is rejected."""
        # go2 carries camera/imu/odometry but no lidar.
        result = check_composition(
            self.registry,
            make_composition(robot_id="go2", environment_id="empty",
                             localization="amcl"),
        )
        self.assertFalse(result.valid)
        self.assertTrue(
            any(
                "go2" in err and "lidar" in err and "amcl" in err
                for err in result.errors
            ),
            f"expected a missing-LiDAR diagnostic, got: {result.errors}",
        )

    def test_lidar_equipped_robot_passes_scan_algorithm(self):
        """A robot with a LiDAR satisfies scan-based algorithm inputs."""
        result = check_composition(
            self.registry,
            make_composition(localization="amcl", global_planning="navfn_planner"),
        )
        self.assertFalse(
            any("lidar" in err for err in result.errors),
            f"no lidar errors expected, got: {result.errors}",
        )

    def test_algorithm_output_satisfies_downstream_input(self):
        """A topic provided by another selected algorithm satisfies consumers."""
        # The quadrotor has no odometry sensor, but the EKF in the same
        # composition provides /odom, so consumers must not hard-fail.
        result = check_composition(
            self.registry,
            make_composition(robot_id="quadrotor_sitl", environment_id="empty",
                             state_estimation="ekf_localization_node"),
        )
        self.assertFalse(
            any("no odometry sensor" in err for err in result.errors),
            f"ekf-provided /odom should satisfy consumers, got: {result.errors}",
        )

    # ------------------------------------------------------------------
    # Dimension compatibility (2D planners cannot gain flight capability)
    # ------------------------------------------------------------------

    def _inject_algorithm(self, algorithm):
        """Register a synthetic algorithm for the duration of a test."""
        self.registry.algorithms.entities[algorithm["id"]] = algorithm
        self.addCleanup(self.registry.algorithms.entities.pop, algorithm["id"], None)

    def test_2d_planner_cannot_gain_flight_capability_by_label(self):
        """An 'aerial' metadata label does not make a 2D-only planner fly."""
        self._inject_algorithm({
            "id": "r32_2d_only_planner",
            "name": "R3.2 2D-only planner",
            "category": "global_planning",
            "input_contract": {"required_topics": []},
            "output_contract": {"provided_topics": []},
            "required_capabilities": [],
            # The metadata label claims aerial support...
            "supported_robot_classes": ["aerial"],
            # ...but the typed dimension contract says ground-only.
            "supported_dimensions": ["2D"],
        })
        result = check_composition(
            self.registry,
            make_composition(robot_id="quadrotor_sitl", environment_id="aerial_course",
                             global_planning="r32_2d_only_planner"),
        )
        self.assertFalse(result.valid)
        self.assertTrue(
            any(
                "r32_2d_only_planner" in err and "aerial_course" in err
                for err in result.errors
            ),
            f"expected a dimension-mismatch diagnostic, got: {result.errors}",
        )

    def test_dimension_declared_match_passes(self):
        """Declaring the environment dimension makes the same planner valid."""
        self._inject_algorithm({
            "id": "r32_2d3d_planner",
            "name": "R3.2 2D/3D planner",
            "category": "global_planning",
            "input_contract": {"required_topics": []},
            "output_contract": {"provided_topics": []},
            "required_capabilities": [],
            "supported_robot_classes": ["aerial"],
            "supported_dimensions": ["2D", "3D"],
        })
        result = check_composition(
            self.registry,
            make_composition(robot_id="quadrotor_sitl", environment_id="aerial_course",
                             global_planning="r32_2d3d_planner"),
        )
        self.assertFalse(
            any("dimension" in err for err in result.errors),
            f"no dimension errors expected, got: {result.errors}",
        )

    def test_dimension_check_unit_rejects_mismatch(self):
        """check_dimension_compatibility rejects a 2D algorithm in a 3D world."""
        result = check_dimension_compatibility(
            {"id": "planner_2d", "supported_dimensions": ["2D"]},
            {"id": "world_3d", "dimension": "3D"},
        )
        self.assertFalse(result.valid)
        self.assertEqual(len(result.errors), 1)
        self.assertIn("planner_2d", result.errors[0])

    def test_dimension_check_unit_passes_undeclared(self):
        """Algorithms without a dimension declaration stay unconstrained."""
        result = check_dimension_compatibility(
            {"id": "legacy_planner"},
            {"id": "world_3d", "dimension": "3D"},
        )
        self.assertTrue(result.valid)

    # ------------------------------------------------------------------
    # Restored capability checks
    # ------------------------------------------------------------------

    def test_capability_check_restored_unit(self):
        """check_capabilities enforces algorithm required_capabilities."""
        robot = {"id": "r", "robot_class": "mobile", "capabilities": ["navigation"]}
        algorithm = {"id": "a", "required_capabilities": ["standing"]}
        result = check_capabilities(robot, algorithm)
        self.assertFalse(result.valid)
        self.assertIn("standing", result.errors[0])

        robot["capabilities"].append("standing")
        self.assertTrue(check_capabilities(robot, algorithm).valid)

    def test_humanoid_controller_on_humanoid_passes_capability(self):
        """The standing controller composes onto the robot that has 'standing'."""
        result = check_composition(
            self.registry,
            make_composition(robot_id="berkeley_humanoid_lite", environment_id="empty",
                             control="humanoid_standing_controller"),
        )
        self.assertFalse(
            any("capabilities" in err for err in result.errors),
            f"no capability errors expected, got: {result.errors}",
        )

    # ------------------------------------------------------------------
    # Valid combinations pass with actionable diagnostics
    # ------------------------------------------------------------------

    def test_valid_bumperbot_stack_passes(self):
        """The canonical bumperbot stack validates with zero errors."""
        result = check_composition(
            self.registry,
            make_composition(localization="amcl", global_planning="navfn_planner"),
        )
        self.assertTrue(result.valid, f"unexpected errors: {result.errors}")

    def test_all_registry_experiments_pass(self):
        """The new typed checks must not break any registered experiment."""
        failures = []
        for exp_id, experiment in self.registry.experiments.get_all().items():
            result = validate_experiment(self.registry, experiment)
            if not result.valid:
                failures.append((exp_id, result.errors))
        self.assertEqual(failures, [])

    def test_diagnostics_are_actionable(self):
        """Rejections name the robot, the algorithm and the missing thing."""
        result = check_composition(
            self.registry,
            make_composition(robot_id="go2", environment_id="empty",
                             localization="amcl"),
        )
        self.assertFalse(result.valid)
        message = "\n".join(result.errors)
        # Robot, algorithm, sensor type and topic are all identified.
        for token in ("go2", "amcl", "lidar", "/scan"):
            self.assertIn(token, message)

if __name__ == "__main__":
    unittest.main()
