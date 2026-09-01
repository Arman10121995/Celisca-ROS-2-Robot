"""
P3.4 Berkeley Humanoid Lite Qualification Tests

Tests for qualifying the Berkeley Humanoid Lite (BHL) biped as the first
integrated simulated commandable humanoid robot. Validates registry contracts,
the berkeley_humanoid_lite_smoke_test composition, and the on-disk asset
contract (bhl_sim.xacro wrapper, bhl_ros2_control.xacro, and the standing
controller configuration).

BHL is a vendored static URDF (22-DOF: 12 leg + 10 arm joints) qualified as a
simulated commandable humanoid profile.
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


class BhlQualificationTests(unittest.TestCase):
    """Test suite for BHL registry qualification (P3.4)."""

    @classmethod
    def setUpClass(cls):
        """Load registry once for all tests."""
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)

    def test_bhl_registered(self):
        """Verify BHL is registered and integrated."""
        robot = self.registry.robots.get("berkeley_humanoid_lite")
        self.assertIsNotNone(robot)
        self.assertEqual(robot["name"], "Berkeley Humanoid Lite")
        self.assertEqual(robot["robot_class"], "humanoid")
        self.assertEqual(robot["status"], "integrated")
        self.assertEqual(robot["maturity"], "simulated")

    def test_bhl_is_biped(self):
        """Verify BHL is a 22-DOF biped."""
        robot = self.registry.robots.get("berkeley_humanoid_lite")
        self.assertEqual(robot["locomotion"]["type"], "biped")
        self.assertEqual(robot["locomotion"]["dof"], 22)


# __CHUNK2__

    def test_bhl_sensor_contracts(self):
        robot = self.registry.robots.get("berkeley_humanoid_lite")
        sensors = {s["type"]: s for s in robot["sensors"]}
        self.assertIn("imu", sensors)
        self.assertIn("odometry", sensors)
        self.assertEqual(sensors["imu"]["topic"], "/imu")
        self.assertEqual(sensors["imu"]["frame"], "imu")
        self.assertEqual(sensors["odometry"]["topic"], "/odom")

    def test_bhl_has_twenty_two_actuators(self):
        robot = self.registry.robots.get("berkeley_humanoid_lite")
        act = {a["name"]: a["joint"] for a in robot["actuators"]}
        self.assertEqual(len(act), 22)
        for leg in ("left", "right"):
            for part in ("hip_roll", "hip_yaw", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll"):
                self.assertEqual(act[f"leg_{leg}_{part}"], f"leg_{leg}_{part}_joint")
        for side in ("left", "right"):
            for part in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_roll", "elbow_pitch"):
                self.assertEqual(act[f"arm_{side}_{part}"], f"arm_{side}_{part}_joint")

    def test_bhl_is_position_commandable(self):
        robot = self.registry.robots.get("berkeley_humanoid_lite")
        self.assertIn("std_msgs/Float64MultiArray", robot["command_interfaces"])
        for si in ("sensor_msgs/JointState", "sensor_msgs/Imu", "nav_msgs/Odometry"):
            self.assertIn(si, robot["state_interfaces"])

    def test_bhl_has_standing_capability(self):
        robot = self.registry.robots.get("berkeley_humanoid_lite")
        self.assertIn("standing", robot["capabilities"])

    def test_bhl_dependencies_exist(self):
        robot = self.registry.robots.get("berkeley_humanoid_lite")
        self.assertIn("robots", robot["dependencies"])
        src_root = Path(__file__).resolve().parents[4] / "src"
        for dep in robot["dependencies"]:
            self.assertTrue((src_root / dep).is_dir(), f"Missing dep in src/: {dep}")

    def test_bhl_asset_urdf_points_to_sim_xacro(self):
        robot = self.registry.robots.get("berkeley_humanoid_lite")
        self.assertEqual(
            robot["assets"]["urdf"],
            "robots/berkeley_humanoid_lite/xacro/bhl_sim.xacro",
        )
        self.assertEqual(
            robot["assets"]["xacro"],
            "robots/berkeley_humanoid_lite/xacro/bhl_ros2_control.xacro",
        )

    def test_bhl_smoke_composition(self):
        composition = Composition(
            robot_id="berkeley_humanoid_lite",
            environment_id="empty",
            simulator="gazebo",
            scenario_id="berkeley_humanoid_lite_smoke_test",
        )
        self.assertEqual(composition.robot_id, "berkeley_humanoid_lite")
        robot = self.registry.robots.get(composition.robot_id)
        environment = self.registry.environments.get(composition.environment_id)
        scenario = self.registry.scenarios.get(composition.scenario_id)
        self.assertIsNotNone(robot)
        self.assertIsNotNone(environment)
        self.assertIsNotNone(scenario)

    def test_bhl_scenario_registered(self):
        scenario = self.registry.scenarios.get("berkeley_humanoid_lite_smoke_test")
        self.assertIsNotNone(scenario)
        self.assertEqual(scenario["task_type"], "smoke_test")
        self.assertIn("humanoid", scenario["required_robot_classes"])
        self.assertIn("standing", scenario["required_capabilities"])

    def test_bhl_experiment_registered(self):
        experiment = self.registry.experiments.get("berkeley_humanoid_lite_smoke_test")
        self.assertIsNotNone(experiment)
        self.assertEqual(experiment["robot_id"], "berkeley_humanoid_lite")
        self.assertEqual(experiment["simulator"], "gazebo")
        self.assertEqual(experiment["scenario_id"], "berkeley_humanoid_lite_smoke_test")
        self.assertIsNotNone(
            self.registry.environments.get(experiment["environment_id"])
        )

# __CHUNK3__

    def test_bhl_experiment_pins_standing_controller(self):
        experiment = self.registry.experiments.get("berkeley_humanoid_lite_smoke_test")
        self.assertEqual(
            experiment["algorithm_ids"]["control"], "humanoid_standing_controller"
        )

    def test_bhl_experiment_algorithms_resolve(self):
        experiment = self.registry.experiments.get("berkeley_humanoid_lite_smoke_test")
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
            self.assertIn(
                "humanoid", algorithm["supported_robot_classes"],
                f"{algorithm_id} must support humanoid class",
            )

    def test_humanoid_standing_controller_algorithm_registered(self):
        """The new control algorithm must exist with a standing capability."""
        algorithm = self.registry.algorithms.get("humanoid_standing_controller")
        self.assertIsNotNone(algorithm)
        self.assertEqual(algorithm["category"], "control")
        self.assertIn("standing", algorithm["required_capabilities"])
        self.assertIn("humanoid", algorithm["supported_robot_classes"])


class BhlAssetContractTests(unittest.TestCase):
    """Validation layer 4 (assets): referenced files must exist and be consistent."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)
        cls.workspace_root = Path(__file__).resolve().parents[4]
        cls.src_root = cls.workspace_root / "src"
        cls.robot = cls.registry.robots.get("berkeley_humanoid_lite")
        cls.robot_dir = cls.src_root / "robots" / "berkeley_humanoid_lite"

    def _repo_asset(self, rel_path):
        return self.src_root / rel_path

    def test_bhl_urdf_exists(self):
        path = self._repo_asset(self.robot["assets"]["urdf"])
        self.assertTrue(path.is_file(), f"Missing bhl_sim.xacro: {path}")
        xacro = self._repo_asset(self.robot["assets"]["xacro"])
        self.assertTrue(xacro.is_file(), f"Missing bhl_ros2_control.xacro: {xacro}")

    def test_bhl_controllers_config_exists(self):
        path = self.robot_dir / "config" / "bhl_controllers.yaml"
        self.assertTrue(path.is_file(), f"Missing controllers config: {path}")

    def test_bhl_sim_includes_static_urdf_and_ros2control(self):
        sim = self._repo_asset(self.robot["assets"]["urdf"]).read_text()
        self.assertIn("berkeley_humanoid_lite.urdf", sim)
        self.assertIn("bhl_ros2_control.xacro", sim)
        self.assertIn("bhl_ros2_control", sim)

    def test_bhl_joint_names_consistent(self):
        """All 22 registry joints must be commanded by the controllers config."""
        registry_joints = {a["joint"] for a in self.robot["actuators"]}
        self.assertEqual(len(registry_joints), 22)
        import yaml
        controllers = yaml.safe_load(
            (self.robot_dir / "config" / "bhl_controllers.yaml").read_text()
        )
        commanded = controllers["bhl_standing_controller"]["ros__parameters"]["joints"]
        self.assertEqual(set(commanded), registry_joints)

    def test_bhl_gazebo_plugin_uses_bhl_controllers(self):
        gazebo = self._repo_asset(self.robot["assets"]["xacro"]).read_text()
        self.assertIn("berkeley_humanoid_lite/config/bhl_controllers.yaml", gazebo)

    def test_bhl_gazebo_imu_declared(self):
        gazebo = self._repo_asset(self.robot["assets"]["xacro"]).read_text()
        self.assertIn('type="imu"', gazebo)
        self.assertIn('>imu</topic>', gazebo)

    def test_bhl_smoke_experiments_resolve(self):
        for exp_id in self.robot["smoke_experiments"]:
            experiment = self.registry.experiments.get(exp_id)
            self.assertIsNotNone(experiment, f"Unknown smoke experiment: {exp_id}")
            self.assertEqual(
                experiment["scenario_id"], "berkeley_humanoid_lite_smoke_test"
            )

    def test_bhl_all_meshes_resolve(self):
        """Every package:// mesh URI in the static URDF must exist on disk."""
        import re
        urdf = (self.robot_dir / "urdf" / "berkeley_humanoid_lite.urdf").read_text()
        refs = re.findall(
            r"package://robots/berkeley_humanoid_lite/([\w.\-/]+)", urdf
        )
        self.assertGreater(len(refs), 0, "No mesh refs found in BHL URDF")
        for rel in refs:
            self.assertTrue(
                (self.src_root / "robots" / "berkeley_humanoid_lite" / rel).exists(),
                f"BHL URDF references missing file: {rel}",
            )

    def test_bhl_description_processes_with_xacro(self):
        xacro_bin = shutil.which("xacro")
        if xacro_bin is None:
            self.skipTest("xacro executable not available")
        urdf_path = str(self._repo_asset(self.robot["assets"]["urdf"]))
        result = subprocess.run(
            ["bash", "-c", f"source /opt/ros/humble/setup.bash && {xacro_bin} {urdf_path}"],
            capture_output=True, text=True, timeout=90,
        )
        self.assertEqual(
            result.returncode, 0,
            f"xacro processing failed:\n{result.stderr}",
        )
        self.assertIn('<robot name="berkeley_humanoid_lite">', result.stdout)
        for joint in (
            "leg_left_hip_roll_joint", "leg_left_knee_pitch_joint",
            "leg_right_ankle_roll_joint", "arm_right_shoulder_pitch_joint",
            "arm_left_elbow_pitch_joint",
        ):
            self.assertIn(joint, result.stdout)
        self.assertIn('type="imu"', result.stdout)


if __name__ == "__main__":
    unittest.main()