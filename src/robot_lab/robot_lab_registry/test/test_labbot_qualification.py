"""
P3.2 Labbot Qualification Tests

Tests for qualifying Labbot as the second integrated differential-drive
mobile robot. Validates registry contracts, smoke-test composition, and
the on-disk asset contract (URDF, ros2_control, gazebo xacros, and the
diff-drive controller configuration).

Labbot is a first-party robot whose description uses primitive geometry
only (no mesh dependencies), unlike the mesh-based Bumperbot reference.
"""

import shutil
import subprocess
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
from robot_lab_registry.validation import Composition


class LabbotQualificationTests(unittest.TestCase):
    """Test suite for Labbot qualification (P3.2)."""

    @classmethod
    def setUpClass(cls):
        """Load registry once for all tests."""
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)

    def test_labbot_registered(self):
        """Verify Labbot is registered in the catalog."""
        robot = self.registry.robots.get("labbot")
        self.assertIsNotNone(robot)
        self.assertEqual(robot["name"], "Labbot")
        self.assertEqual(robot["robot_class"], "mobile")
        self.assertEqual(robot["status"], "integrated")

    def test_labbot_is_differential_drive(self):
        """Verify Labbot is a 2-DOF differential-drive robot."""
        robot = self.registry.robots.get("labbot")
        self.assertEqual(robot["locomotion"]["type"], "differential_drive")
        self.assertEqual(robot["locomotion"]["dof"], 2)

    def test_labbot_has_required_sensors(self):
        """Verify Labbot has all required sensor contracts."""
        robot = self.registry.robots.get("labbot")
        sensors = {s["type"]: s for s in robot["sensors"]}

        # Required sensors
        self.assertIn("lidar", sensors)
        self.assertIn("imu", sensors)
        self.assertIn("odometry", sensors)

        # Verify topic contracts
        self.assertEqual(sensors["lidar"]["topic"], "/scan")
        self.assertEqual(sensors["imu"]["topic"], "/imu")
        self.assertEqual(sensors["odometry"]["topic"], "/odom")

        # Verify frame contracts
        self.assertEqual(sensors["lidar"]["frame"], "laser_link")
        self.assertEqual(sensors["imu"]["frame"], "imu_link")

    def test_labbot_actuator_joints(self):
        """Verify Labbot declares its wheel actuator joints."""
        robot = self.registry.robots.get("labbot")
        joints = {a["name"]: a["joint"] for a in robot["actuators"]}
        self.assertEqual(joints["left_wheel"], "labbot_left_wheel_joint")
        self.assertEqual(joints["right_wheel"], "labbot_right_wheel_joint")

    def test_labbot_supports_gazebo_simulator(self):
        """Verify Labbot supports Gazebo simulator."""
        robot = self.registry.robots.get("labbot")
        self.assertIn("gazebo", robot["supported_simulators"])

    def test_labbot_dependencies_exist(self):
        """Verify Labbot's declared dependency packages exist in src/."""
        robot = self.registry.robots.get("labbot")
        self.assertIn("robots", robot["dependencies"])
        src_root = Path(__file__).resolve().parents[4] / "src"
        for dep in robot["dependencies"]:
            self.assertTrue(
                (src_root / dep).is_dir(),
                f"Dependency package missing in src/: {dep}",
            )

    def test_labbot_has_asset_urdf(self):
        """Verify Labbot URDF asset is specified."""
        robot = self.registry.robots.get("labbot")
        self.assertIn("urdf", robot["assets"])
        self.assertTrue(robot["assets"]["urdf"].endswith(".urdf.xacro"))

    def test_labbot_has_mesh_free_description(self):
        """Labbot's description must not depend on mesh files."""
        robot = self.registry.robots.get("labbot")
        meshes = robot["assets"].get("meshes", [])
        self.assertEqual(
            meshes, [],
            "Labbot is primitive-geometry only; no meshes may be declared",
        )

    def test_labbot_has_required_capabilities(self):
        """Verify Labbot declares navigation capability for smoke scenarios."""
        robot = self.registry.robots.get("labbot")
        self.assertIn("navigation", robot["capabilities"])

    def test_labbot_dry_run_composition(self):
        """Test creating a dry-run launch composition for Labbot."""
        composition_dict = {
            "robot_id": "labbot",
            "environment_id": "small_office",
            "simulator": "gazebo",
            "scenario_id": "labbot_smoke_test",
        }
        composition = Composition(**composition_dict)
        self.assertEqual(composition.robot_id, "labbot")

        # Verify we can extract launch-relevant data
        robot = self.registry.robots.get(composition.robot_id)
        environment = self.registry.environments.get(composition.environment_id)
        scenario = self.registry.scenarios.get(composition.scenario_id)

        self.assertIsNotNone(robot)
        self.assertIsNotNone(environment)
        self.assertIsNotNone(scenario)

    def test_labbot_scenario_registered(self):
        """Verify the Labbot smoke scenario is registered and well-formed."""
        scenario = self.registry.scenarios.get("labbot_smoke_test")
        self.assertIsNotNone(scenario)
        self.assertEqual(scenario["task_type"], "smoke_test")
        self.assertIn("mobile", scenario["required_robot_classes"])

    def test_labbot_experiment_registered(self):
        """Verify the Labbot smoke experiment is registered and pinned."""
        experiment = self.registry.experiments.get("labbot_smoke_test")
        self.assertIsNotNone(experiment)
        self.assertEqual(experiment["robot_id"], "labbot")
        self.assertEqual(experiment["simulator"], "gazebo")
        self.assertEqual(experiment["scenario_id"], "labbot_smoke_test")
        self.assertIsNotNone(
            self.registry.environments.get(experiment["environment_id"])
        )

    def test_labbot_experiment_algorithms_resolve(self):
        """Every algorithm pinned by the Labbot experiment must be registered."""
        experiment = self.registry.experiments.get("labbot_smoke_test")
        algorithm_ids = experiment["algorithm_ids"]
        expected_categories = {
            "perception", "localization", "state_estimation", "sensor_fusion",
            "global_planning", "local_planning", "control",
        }
        self.assertEqual(set(algorithm_ids.keys()), expected_categories)
        for category, algorithm_id in algorithm_ids.items():
            algorithm = self.registry.algorithms.get(algorithm_id)
            self.assertIsNotNone(
                algorithm,
                f"Experiment pins unknown {category} algorithm: {algorithm_id}",
            )
            self.assertEqual(algorithm["category"], category)


class LabbotAssetContractTests(unittest.TestCase):
    """Validation layer 4 (assets): referenced files must exist and be consistent."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)
        # test/ -> robot_lab_registry -> robot_lab -> src -> workspace root
        cls.workspace_root = Path(__file__).resolve().parents[4]
        cls.src_root = cls.workspace_root / "src"
        cls.robot = cls.registry.robots.get("labbot")
        cls.robot_dir = cls.src_root / "robots" / "labbot"

    def _repo_asset(self, rel_path):
        return self.src_root / rel_path

    def test_labbot_urdf_exists(self):
        """The referenced robot URDF xacro must exist on disk."""
        path = self._repo_asset(self.robot["assets"]["urdf"])
        self.assertTrue(path.is_file(), f"Missing URDF asset: {path}")

    def test_labbot_ros2_control_xacro_exists(self):
        """The referenced ros2_control xacro must exist on disk."""
        path = self._repo_asset(self.robot["assets"]["xacro"])
        self.assertTrue(path.is_file(), f"Missing ros2_control xacro: {path}")

    def test_labbot_gazebo_xacro_exists(self):
        """The gazebo xacro included by the URDF must exist on disk."""
        path = self.robot_dir / "urdf" / "labbot_gazebo.xacro"
        self.assertTrue(path.is_file(), f"Missing gazebo xacro: {path}")

    def test_labbot_controllers_config_exists(self):
        """The diff-drive controller configuration must exist on disk."""
        path = self.robot_dir / "config" / "labbot_controllers.yaml"
        self.assertTrue(path.is_file(), f"Missing controllers config: {path}")

    def test_labbot_urdf_only_references_robots_package(self):
        """All package:// URIs in the description must resolve to the robots package."""
        urdf = self._repo_asset(self.robot["assets"]["urdf"]).read_text()
        for token in urdf.split("package://")[1:]:
            package = token.split("/")[0]
            self.assertEqual(
                package, "robots",
                f"URDF references unknown package: {package}",
            )

    def test_labbot_joint_names_consistent(self):
        """Wheel joints must agree across actuator registry, control xacro, and controllers."""
        registry_joints = {a["joint"] for a in self.robot["actuators"]}

        control = (
            self._repo_asset(self.robot["assets"]["xacro"]).read_text()
        )
        for joint in registry_joints:
            self.assertIn(
                f'<joint name="{joint}">', control,
                f"ros2_control xacro is missing joint: {joint}",
            )

        controllers = (
            self.robot_dir / "config" / "labbot_controllers.yaml"
        ).read_text()
        for joint in registry_joints:
            self.assertIn(joint, controllers, f"controllers yaml missing joint: {joint}")

    def test_labbot_gazebo_plugin_uses_labbot_controllers(self):
        """The gazebo xacro must load Labbot's own controller parameters."""
        gazebo = (self.robot_dir / "urdf" / "labbot_gazebo.xacro").read_text()
        self.assertIn("labbot/config/labbot_controllers.yaml", gazebo)

    def test_labbot_gazebo_sensors_declared(self):
        """The gazebo xacro must declare the IMU and LIDAR sensors."""
        gazebo = (self.robot_dir / "urdf" / "labbot_gazebo.xacro").read_text()
        self.assertIn('type="imu"', gazebo)
        self.assertIn('type="gpu_lidar"', gazebo)

    def test_labbot_smoke_experiments_resolve(self):
        """smoke_experiments must reference registered experiments."""
        for exp_id in self.robot["smoke_experiments"]:
            experiment = self.registry.experiments.get(exp_id)
            self.assertIsNotNone(
                experiment, f"smoke_experiments references unknown experiment: {exp_id}"
            )
            self.assertEqual(
                experiment["scenario_id"], "labbot_smoke_test",
                f"Experiment {exp_id} must use the labbot_smoke_test scenario",
            )

    def test_labbot_description_processes_with_xacro(self):
        """The full description must expand cleanly with xacro when available."""
        xacro_bin = shutil.which("xacro")
        if xacro_bin is None:
            self.skipTest("xacro executable not available")
        result = subprocess.run(
            [xacro_bin, str(self._repo_asset(self.robot["assets"]["urdf"]))],
            capture_output=True, text=True, timeout=60,
        )
        self.assertEqual(
            result.returncode, 0,
            f"xacro processing failed:\n{result.stderr}",
        )
        self.assertIn('<robot name="labbot">', result.stdout)
        self.assertIn("labbot_left_wheel_joint", result.stdout)
        self.assertIn("labbot_right_wheel_joint", result.stdout)


if __name__ == "__main__":
    unittest.main()
