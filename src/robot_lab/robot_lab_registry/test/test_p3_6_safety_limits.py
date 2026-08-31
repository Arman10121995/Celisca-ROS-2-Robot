"""
P3.6 Per-class Smoke Scenarios and Safety/Compute Limits Tests.

Validates that:
  1. Every `integrated` robot in the registry declares a documented safety
     envelope (`safety_limits`) and a compute budget (`compute_limits`).
  2. Each of the four commandable robot classes (mobile, legged, humanoid,
     aerial) has a registered, integrated class-level smoke scenario.
  3. Each class-level smoke experiment references a real robot, environment,
     and scenario, and pages the correct robot class.
  4. The cross-reference validation still passes with the new entries.
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

# The four commandable robot classes.
ROBOT_CLASSES = ("mobile", "legged", "humanoid", "aerial")

# Class -> reference robot used by the class-level smoke experiment.
CLASS_REFERENCE_ROBOT = {
    "mobile": "bumperbot",
    "legged": "go2",
    "humanoid": "berkeley_humanoid_lite",
    "aerial": "quadrotor_sitl",
}

# Required keys inside each safety_limits block.
SAFETY_KEYS = (
    "max_velocity",
    "max_acceleration",
    "command_rate_hz",
    "obstacle_clearance_m",
    "collision_stop_time_s",
    "max_tilt_rad",
    "restricted_modes",
)

# Required keys inside each compute_limits block.
COMPUTE_KEYS = (
    "cpu_cores",
    "memory_mb",
    "min_real_time_factor",
    "notes",
)


class SafetyComputeLimitTests(unittest.TestCase):
    """Every integrated robot must document safety and compute limits."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)
        cls.integrated = [
            r for r in cls.registry.robots.get_all().values()
            if r.get("status") == "integrated"
        ]

    def test_every_robot_class_has_an_integrated_robot(self):
        """All four commandable classes must have at least one integrated robot."""
        classes = {r["robot_class"] for r in self.integrated}
        for robot_class in ROBOT_CLASSES:
            self.assertIn(
                robot_class, classes,
                f"No integrated robot exists for class '{robot_class}'",
            )

    def test_all_integrated_robots_declare_safety_limits(self):
        """Each integrated robot must declare a safety_limits block."""
        for robot in self.integrated:
            self.assertIn(
                "safety_limits", robot,
                f"integrated robot '{robot['id']}' is missing safety_limits",
            )

    def test_all_integrated_robots_declare_compute_limits(self):
        """Each integrated robot must declare a compute_limits block."""
        for robot in self.integrated:
            self.assertIn(
                "compute_limits", robot,
                f"integrated robot '{robot['id']}' is missing compute_limits",
            )

    def test_safety_limits_have_required_fields(self):
        """safety_limits must contain every documented safety key."""
        for robot in self.integrated:
            safety = robot.get("safety_limits", {})
            for key in SAFETY_KEYS:
                self.assertIn(
                    key, safety,
                    f"robot '{robot['id']}' safety_limits missing '{key}'",
                )
            self.assertIsInstance(
                safety["restricted_modes"], list,
                f"robot '{robot['id']}' restricted_modes must be a list",
            )

    def test_compute_limits_have_required_fields(self):
        """compute_limits must contain every documented compute key."""
        for robot in self.integrated:
            compute = robot.get("compute_limits", {})
            for key in COMPUTE_KEYS:
                self.assertIn(
                    key, compute,
                    f"robot '{robot['id']}' compute_limits missing '{key}'",
                )
            self.assertGreater(
                compute["cpu_cores"], 0,
                f"robot '{robot['id']}' cpu_cores must be positive",
            )
            self.assertGreater(
                compute["memory_mb"], 0,
                f"robot '{robot['id']}' memory_mb must be positive",
            )
            self.assertFalse(
                compute["min_real_time_factor"] > 1.0,
                f"robot '{robot['id']}' min_real_time_factor cannot exceed 1.0",
            )

    def test_safety_limits_exceed_locomotion_caps(self):
        """Safety velocity/acceleration caps must not exceed locomotion caps."""
        for robot in self.integrated:
            safety = robot.get("safety_limits", {})
            locomotion = robot.get("locomotion", {})
            if "max_velocity" in locomotion:
                self.assertGreaterEqual(
                    safety["max_velocity"], locomotion["max_velocity"],
                    f"robot '{robot['id']}': safety max_velocity below locomotion cap",
                )
            if "max_acceleration" in locomotion:
                self.assertGreaterEqual(
                    safety["max_acceleration"], locomotion["max_acceleration"],
                    f"robot '{robot['id']}': safety max_acceleration below locomotion cap",
                )


class ClassSmokeScenarioTests(unittest.TestCase):
    """Each robot class must have a registered, integrated class smoke scenario."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)

    def test_class_smoke_scenarios_registered(self):
        """There must be a <class>_class_smoke scenario for each class."""
        for robot_class in ROBOT_CLASSES:
            scenario = self.registry.scenarios.get(f"{robot_class}_class_smoke")
            self.assertIsNotNone(
                scenario,
                f"Missing scenario '{robot_class}_class_smoke'",
            )
            self.assertEqual(
                scenario["status"], "integrated",
                f"scenario '{robot_class}_class_smoke' must be integrated",
            )
            self.assertEqual(
                scenario["task_type"], "smoke_test",
                f"scenario '{robot_class}_class_smoke' must be task_type smoke_test",
            )

    def test_class_smoke_scenario_requires_correct_class(self):
        """The scenario's required_robot_classes must list exactly its class."""
        for robot_class in ROBOT_CLASSES:
            scenario = self.registry.scenarios.get(f"{robot_class}_class_smoke")
            required = scenario["required_robot_classes"]
            self.assertIn(
                robot_class, required,
                f"scenario '{robot_class}_class_smoke' must require '{robot_class}'",
            )


class ClassSmokeExperimentTests(unittest.TestCase):
    """Each class smoke experiment must reference valid entities and resolve."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)

    def test_class_smoke_experiments_registered(self):
        """There must be a <class>_class_smoke experiment for each class."""
        for robot_class in ROBOT_CLASSES:
            experiment = self.registry.experiments.get(f"{robot_class}_class_smoke")
            self.assertIsNotNone(
                experiment,
                f"Missing experiment '{robot_class}_class_smoke'",
            )
            self.assertEqual(
                experiment["status"], "integrated",
                f"experiment '{robot_class}_class_smoke' must be integrated",
            )

    def test_class_smoke_experiments_reference_valid_entities(self):
        """robot_id/env/scenario must all resolve and robot class must match."""
        for robot_class in ROBOT_CLASSES:
            experiment = self.registry.experiments.get(
                f"{robot_class}_class_smoke"
            )
            robot = self.registry.robots.get(experiment["robot_id"])
            env = self.registry.environments.get(experiment["environment_id"])
            scenario = self.registry.scenarios.get(experiment["scenario_id"])
            self.assertIsNotNone(
                robot, f"{robot_class}_class_smoke references unknown robot"
            )
            self.assertIsNotNone(
                env, f"{robot_class}_class_smoke references unknown environment"
            )
            self.assertIsNotNone(
                scenario, f"{robot_class}_class_smoke references unknown scenario"
            )
            self.assertEqual(
                robot["robot_class"], robot_class,
                f"experiment '{robot_class}_class_smoke' robot class mismatch",
            )
            self.assertEqual(
                experiment["scenario_id"], f"{robot_class}_class_smoke",
                f"experiment '{robot_class}_class_smoke' must pin its class scenario",
            )

    def test_class_smoke_experiments_page_reference_robot(self):
        """Each class smoke experiment uses the designated reference robot."""
        for robot_class, ref_robot in CLASS_REFERENCE_ROBOT.items():
            experiment = self.registry.experiments.get(
                f"{robot_class}_class_smoke"
            )
            self.assertEqual(
                experiment["robot_id"], ref_robot,
                f"experiment '{robot_class}_class_smoke' should use '{ref_robot}'",
            )

    def test_cross_reference_validation_passes(self):
        """Cross-reference validation must pass with all new entries."""
        result = validate_cross_references(self.registry)
        self.assertTrue(
            result.valid,
            f"cross-reference validation fails after P3.6 entries: {result.errors}",
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)

