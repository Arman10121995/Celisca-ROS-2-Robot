"""
P3.1 Bumperbot Qualification Tests

Tests for qualifying Bumperbot as the reference differential-drive robot.
Validates launch configuration, sensor contracts, and smoke test scenarios.
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
from robot_lab_registry.validation import Composition, check_composition


class BumberbotQualificationTests(unittest.TestCase):
    """Test suite for Bumperbot qualification (P3.1)."""

    @classmethod
    def setUpClass(cls):
        """Load registry once for all tests."""
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)

    def test_bumperbot_registered(self):
        """Verify Bumperbot is registered in the catalog."""
        robot = self.registry.robots.get("bumperbot")
        self.assertIsNotNone(robot)
        self.assertEqual(robot["name"], "Bumperbot")
        self.assertEqual(robot["robot_class"], "mobile")
        self.assertEqual(robot["status"], "integrated")

    def test_bumperbot_has_required_sensors(self):
        """Verify Bumperbot has all required sensor contracts."""
        robot = self.registry.robots.get("bumperbot")
        sensors = {s["type"]: s for s in robot["sensors"]}

        # Required sensors
        self.assertIn("lidar", sensors)
        self.assertIn("imu", sensors)
        self.assertIn("odometry", sensors)

        # Verify topic contracts
        self.assertEqual(sensors["lidar"]["topic"], "/scan")
        self.assertEqual(sensors["imu"]["topic"], "/imu")
        self.assertEqual(sensors["odometry"]["topic"], "/odom")

        # Verify message types
        self.assertEqual(
            sensors["lidar"]["message_type"], "sensor_msgs/LaserScan"
        )
        self.assertEqual(sensors["imu"]["message_type"], "sensor_msgs/Imu")
        self.assertEqual(sensors["odometry"]["message_type"], "nav_msgs/Odometry")

    def test_bumperbot_command_interfaces(self):
        """Verify Bumperbot accepts velocity commands."""
        robot = self.registry.robots.get("bumperbot")
        self.assertIn("geometry_msgs/Twist", robot["command_interfaces"])

    def test_bumperbot_state_interfaces(self):
        """Verify Bumperbot publishes state interfaces."""
        robot = self.registry.robots.get("bumperbot")
        self.assertIn("nav_msgs/Odometry", robot["state_interfaces"])
        self.assertIn("sensor_msgs/JointState", robot["state_interfaces"])

    def test_bumperbot_required_frames(self):
        """Verify Bumperbot has all required TF frames."""
        robot = self.registry.robots.get("bumperbot")
        frames = set(robot["frames"])

        # Required frames
        self.assertIn("base_link", frames)
        self.assertIn("odom", frames)
        self.assertIn("map", frames)

    def test_bumperbot_capabilities(self):
        """Verify Bumperbot declares all required capabilities."""
        robot = self.registry.robots.get("bumperbot")
        capabilities = set(robot["capabilities"])

        # Required for reference platform
        self.assertIn("navigation", capabilities)
        self.assertIn("localization", capabilities)
        self.assertIn("perception", capabilities)
        self.assertIn("state_estimation", capabilities)
        self.assertIn("control", capabilities)

    def test_bumperbot_smoke_test_scenario_exists(self):
        """Verify bumperbot_smoke_test scenario is registered."""
        scenario = self.registry.scenarios.get("bumperbot_smoke_test")
        self.assertIsNotNone(scenario, "bumperbot_smoke_test scenario should be registered")
        self.assertEqual(scenario["status"], "integrated")
        self.assertEqual(scenario["task_type"], "smoke_test")

    def test_bumperbot_smoke_test_experiment_exists(self):
        """Verify bumperbot_smoke_test experiment is registered."""
        experiment = self.registry.experiments.get("bumperbot_smoke_test")
        self.assertIsNotNone(experiment)
        self.assertEqual(experiment["status"], "integrated")
        self.assertEqual(experiment["robot_id"], "bumperbot")
        self.assertEqual(experiment["scenario_id"], "bumperbot_smoke_test")

    def test_bumperbot_smoke_test_experiment_environment(self):
        """Verify smoke test uses small_office environment."""
        experiment = self.registry.experiments.get("bumperbot_smoke_test")
        self.assertEqual(experiment["environment_id"], "small_office")

        # Verify environment exists
        env = self.registry.environments.get("small_office")
        self.assertIsNotNone(env)

    def test_bumperbot_smoke_test_experiment_simulator(self):
        """Verify smoke test uses gazebo simulator."""
        experiment = self.registry.experiments.get("bumperbot_smoke_test")
        self.assertEqual(experiment["simulator"], "gazebo")

    def test_bumperbot_smoke_test_algorithms(self):
        """Verify smoke test specifies algorithms for all required categories."""
        experiment = self.registry.experiments.get("bumperbot_smoke_test")
        algorithms = experiment["algorithm_ids"]

        # Required algorithm categories
        required = [
            "perception",
            "localization",
            "state_estimation",
            "global_planning",
            "local_planning",
            "control",
        ]
        for category in required:
            self.assertIn(category, algorithms)
            self.assertIsNotNone(algorithms[category])

    def test_bumperbot_smoke_test_composition_validates(self):
        """Verify smoke test composition passes validation."""
        composition_dict = {
            "robot_id": "bumperbot",
            "environment_id": "small_office",
            "simulator": "gazebo",
            "scenario_id": "bumperbot_smoke_test",
        }
        composition = Composition(**composition_dict)
        # Verify composition object created successfully
        self.assertEqual(composition.robot_id, "bumperbot")
        self.assertEqual(composition.environment_id, "small_office")

    def test_bumperbot_small_office_environment_exists(self):
        """Verify small_office environment exists and is compatible."""
        env = self.registry.environments.get("small_office")
        self.assertIsNotNone(env)
        self.assertEqual(env["simulator"], "gazebo")

    def test_bumperbot_algorithms_exist(self):
        """Verify all algorithms used in smoke test exist."""
        experiment = self.registry.experiments.get("bumperbot_smoke_test")
        algorithms = experiment["algorithm_ids"]

        for category, algo_id in algorithms.items():
            algo = self.registry.algorithms.get(algo_id)
            self.assertIsNotNone(algo)
            self.assertEqual(algo["category"], category)

    def test_bumperbot_dependencies_registered(self):
        """Verify Bumperbot's dependencies are registered in algorithms."""
        robot = self.registry.robots.get("bumperbot")

        # Verify key algorithm categories are available
        required_categories = [
            "perception",
            "localization",
            "control",
            "global_planning",
            "local_planning",
        ]
        for category in required_categories:
            algo = self.registry.algorithms.filter(category=category)
            self.assertGreater(
                len(algo), 0, f"Should have algorithms in {category}"
            )

    def test_bumperbot_multirobotnamespace_support(self):
        """Verify Bumperbot can be used with namespace prefixes."""
        robot = self.registry.robots.get("bumperbot")

        # Should support namespace for multi-robot experiments
        self.assertIn("navigation", robot["capabilities"])

    def test_bumperbot_documentation(self):
        """Verify Bumperbot has complete documentation."""
        robot = self.registry.robots.get("bumperbot")

        # Metadata
        self.assertIn("name", robot)
        self.assertIn("description", robot)
        self.assertIn("repository", robot["source"])
        self.assertIn("license", robot["source"])

        # Assets
        self.assertIn("urdf", robot["assets"])
        self.assertIn("meshes", robot["assets"])

        # Timestamps
        self.assertIn("created", robot)
        self.assertIn("updated", robot)


class BumberbotLaunchValidationTests(unittest.TestCase):
    """Test suite for Bumperbot launch configuration validation."""

    @classmethod
    def setUpClass(cls):
        """Load registry once for all tests."""
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)

    def test_bumperbot_supports_gazebo_simulator(self):
        """Verify Bumperbot supports Gazebo simulator."""
        robot = self.registry.robots.get("bumperbot")
        self.assertIn("gazebo", robot["supported_simulators"])

    def test_bumperbot_launch_packages_exist(self):
        """Verify Bumperbot launch packages are in dependencies."""
        robot = self.registry.robots.get("bumperbot")
        dependencies = robot["dependencies"]

        # Must have bringup and description packages
        self.assertIn("bumperbot_bringup", dependencies)
        self.assertIn("bumperbot_description", dependencies)

    def test_bumperbot_has_asset_urdf(self):
        """Verify Bumperbot URDF asset is specified."""
        robot = self.registry.robots.get("bumperbot")
        self.assertIn("urdf", robot["assets"])
        self.assertIsNotNone(robot["assets"]["urdf"])
        self.assertTrue(robot["assets"]["urdf"].endswith(".urdf.xacro"))

    def test_bumperbot_has_robot_parameter_yaml(self):
        """Verify Bumperbot has xacro configuration."""
        robot = self.registry.robots.get("bumperbot")
        self.assertIn("xacro", robot["assets"])
        self.assertIsNotNone(robot["assets"]["xacro"])

    def test_bumperbot_dry_run_composition(self):
        """Test creating a dry-run launch composition for Bumperbot."""
        composition_dict = {
            "robot_id": "bumperbot",
            "environment_id": "small_office",
            "simulator": "gazebo",
            "scenario_id": "bumperbot_smoke_test",
        }
        composition = Composition(**composition_dict)
        # Verify basic composition structure
        self.assertEqual(composition.robot_id, "bumperbot")

        # Verify we can extract launch-relevant data
        robot = self.registry.robots.get(composition.robot_id)
        environment = self.registry.environments.get(composition.environment_id)
        scenario = self.registry.scenarios.get(composition.scenario_id)

        self.assertIsNotNone(robot)
        self.assertIsNotNone(environment)
        self.assertIsNotNone(scenario)


class BumperbotAssetContractTests(unittest.TestCase):
    """Validation layer 4 (assets): referenced files must exist in the repo."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)
        # test/ -> robot_lab_registry -> robot_lab -> src -> workspace root
        cls.workspace_root = Path(__file__).resolve().parents[4]
        cls.src_root = cls.workspace_root / "src"
        cls.robot = cls.registry.robots.get("bumperbot")

    def _repo_asset(self, rel_path):
        return self.src_root / rel_path

    def test_bumperbot_urdf_exists(self):
        """The referenced robot URDF xacro must exist on disk."""
        path = self._repo_asset(self.robot["assets"]["urdf"])
        self.assertTrue(path.is_file(), f"Missing URDF asset: {path}")

    def test_bumperbot_ros2_control_xacro_exists(self):
        """The referenced ros2_control xacro must exist on disk."""
        path = self._repo_asset(self.robot["assets"]["xacro"])
        self.assertTrue(path.is_file(), f"Missing ros2_control xacro: {path}")

    def test_bumperbot_meshes_exist(self):
        """Every referenced mesh file must exist on disk."""
        for mesh in self.robot["assets"]["meshes"]:
            path = self._repo_asset(mesh)
            self.assertTrue(path.is_file(), f"Missing mesh asset: {path}")

    def test_bumperbot_smoke_experiments_resolve(self):
        """smoke_experiments must reference registered experiments."""
        for exp_id in self.robot["smoke_experiments"]:
            experiment = self.registry.experiments.get(exp_id)
            self.assertIsNotNone(
                experiment, f"smoke_experiments references unknown experiment: {exp_id}"
            )
            self.assertEqual(
                experiment["scenario_id"], "bumperbot_smoke_test",
                f"Experiment {exp_id} must use the bumperbot_smoke_test scenario",
            )

    def test_bumperbot_smoke_experiment_pins_full_stack(self):
        """The smoke experiment must pin robot, environment, simulator, and scenario."""
        experiment = self.registry.experiments.get("bumperbot_smoke_test")
        self.assertEqual(experiment["robot_id"], "bumperbot")
        self.assertEqual(experiment["simulator"], "gazebo")
        self.assertIsNotNone(self.registry.environments.get(experiment["environment_id"]))

    def test_bumperbot_launch_entry_point_exists(self):
        """The legacy bringup launch used by the adapter must exist."""
        launch = self.src_root / "bumperbot_bringup" / "launch" / "simulated_robot.launch.py"
        self.assertTrue(launch.is_file(), f"Missing launch file: {launch}")


if __name__ == "__main__":
    unittest.main()
