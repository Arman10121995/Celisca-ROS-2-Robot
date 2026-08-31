"""
P3.3 Go2 Qualification Tests

Tests for qualifying the Unitree Go2 quadruped as a simulated,
commandable legged profile (P3.3). Validates registry contracts,
smoke-test composition, and the on-disk asset contract (sim wrapper
xacro, ros2_control xacro, and diff-drive-equivalent leg controllers).

Go2 uses the vendored upstream Unitree description (12 leg joints:
FL/FR/RL/RR hip/thigh/calf) combined with first-party ros2_control
and gz-sim wiring in go2_ros2_control.xacro and go2_sim.xacro.
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


class Go2QualificationTests(unittest.TestCase):
    """Test suite for Go2 qualification (P3.3)."""

    @classmethod
    def setUpClass(cls):
        """Load registry once for all tests."""
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)

    def test_go2_registered(self):
        """Verify Go2 is registered in the catalog as integrated."""
        robot = self.registry.robots.get("go2")
        self.assertIsNotNone(robot)
        self.assertEqual(robot["name"], "Unitree Go2")
        self.assertEqual(robot["robot_class"], "legged")
        self.assertEqual(robot["status"], "integrated")

    def test_go2_is_quadruped(self):
        """Verify Go2 is a 12-DOF quadruped."""
        robot = self.registry.robots.get("go2")
        self.assertEqual(robot["locomotion"]["type"], "quadruped")
        self.assertEqual(robot["locomotion"]["dof"], 12)

    def test_go2_has_required_sensors(self):
        """Verify Go2 has all required sensor contracts."""
        robot = self.registry.robots.get("go2")
        sensors = {s["type"]: s for s in robot["sensors"]}

        # Required sensors
        self.assertIn("imu", sensors)
        self.assertIn("camera", sensors)
        self.assertIn("odometry", sensors)

        # Verify topic contracts
        self.assertEqual(sensors["imu"]["topic"], "/imu")
        self.assertEqual(sensors["camera"]["topic"], "/camera/rgb/image_raw")
        self.assertEqual(sensors["odometry"]["topic"], "/odom")

        # Verify frame contracts
        self.assertEqual(sensors["imu"]["frame"], "imu_link")
        self.assertEqual(sensors["camera"]["frame"], "front_camera")

    def test_go2_twelve_leg_joints(self):
        """Verify Go2 declares exactly the 12 Unitree leg joints."""
        robot = self.registry.robots.get("go2")
        joints = [a["joint"] for a in robot["actuators"]]
        expected = []
        for leg in ("FL", "FR", "RL", "RR"):
            for part in ("hip", "thigh", "calf"):
                expected.append(f"{leg}_{part}_joint")
        self.assertEqual(joints, expected)

    def test_go2_supports_gazebo(self):
        """Verify Go2 supports Gazebo simulator."""
        robot = self.registry.robots.get("go2")
        self.assertIn("gazebo", robot["supported_simulators"])

    def test_go2_dependencies_exist(self):
        """Verify Go2's declared dependency packages exist in src/."""
        robot = self.registry.robots.get("go2")
        self.assertIn("robots", robot["dependencies"])
        src_root = Path(__file__).resolve().parents[4] / "src"
        for dep in robot["dependencies"]:
            self.assertTrue(
                (src_root / dep).is_dir(),
                f"Dependency package missing in src/: {dep}",
            )

    def test_go2_has_asset_urdf(self):
        """Verify Go2 URDF asset is specified and is the sim wrapper."""
        robot = self.registry.robots.get("go2")
        self.assertIn("urdf", robot["assets"])
        self.assertTrue(robot["assets"]["urdf"].endswith("go2_sim.xacro"))

    def test_go2_commandable_profile(self):
        """Verify the commandable profile declares effort command interfaces."""
        robot = self.registry.robots.get("go2")
        self.assertIn("std_msgs/Float64MultiArray", robot["command_interfaces"])

    def test_go2_state_interfaces(self):
        """Verify state interfaces cover joint states, IMU, and odometry."""
        robot = self.registry.robots.get("go2")
        for iface in ("sensor_msgs/JointState", "sensor_msgs/Imu", "nav_msgs/Odometry"):
            self.assertIn(iface, robot["state_interfaces"])

    def test_go2_scenario_registered(self):
        """Verify the Go2 smoke scenario is registered and well-formed."""
        scenario = self.registry.scenarios.get("go2_smoke_test")
        self.assertIsNotNone(scenario)
        self.assertEqual(scenario["task_type"], "smoke_test")
        self.assertIn("legged", scenario["required_robot_classes"])

    def test_go2_experiment_registered(self):
        """Verify the Go2 smoke experiment is registered and pinned."""
        experiment = self.registry.experiments.get("go2_smoke_test")
        self.assertIsNotNone(experiment)
        self.assertEqual(experiment["robot_id"], "go2")
        self.assertEqual(experiment["simulator"], "gazebo")
        self.assertEqual(experiment["scenario_id"], "go2_smoke_test")
        self.assertIsNotNone(
            self.registry.environments.get(experiment["environment_id"])
        )

    def test_go2_experiment_algorithms_resolve(self):
        """Every algorithm pinned by the Go2 experiment must be registered."""
        experiment = self.registry.experiments.get("go2_smoke_test")
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

    def test_go2_smoke_experiments_resolve(self):
        """smoke_experiments must reference registered Go2 experiments."""
        robot = self.registry.robots.get("go2")
        for exp_id in robot["smoke_experiments"]:
            experiment = self.registry.experiments.get(exp_id)
            self.assertIsNotNone(
                experiment, f"smoke_experiments references unknown experiment: {exp_id}"
            )
            self.assertEqual(
                experiment["scenario_id"], "go2_smoke_test",
                f"Experiment {exp_id} must use the go2_smoke_test scenario",
            )


class Go2AssetContractTests(unittest.TestCase):
    """Validation layer 4 (assets): referenced files must exist and be consistent."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)
        # test/ -> robot_lab_registry -> robot_lab -> src -> workspace root
        cls.workspace_root = Path(__file__).resolve().parents[4]
        cls.src_root = cls.workspace_root / "src"
        cls.robot = cls.registry.robots.get("go2")
        cls.robot_dir = cls.src_root / "robots" / "unitree" / "go2_description"

    def _repo_asset(self, rel_path):
        return self.src_root / rel_path

    def test_go2_sim_xacro_exists(self):
        """The referenced sim wrapper xacro must exist on disk."""
        path = self._repo_asset(self.robot["assets"]["urdf"])
        self.assertTrue(path.is_file(), f"Missing URDF asset: {path}")

    def test_go2_ros2_control_xacro_exists(self):
        """The referenced ros2_control xacro must exist on disk."""
        path = self._repo_asset(self.robot["assets"]["xacro"])
        self.assertTrue(path.is_file(), f"Missing ros2_control xacro: {path}")

    def test_go2_upstream_description_exists(self):
        """The vendored upstream robot.xacro included by the wrapper must exist."""
        path = self.robot_dir / "xacro" / "robot.xacro"
        self.assertTrue(path.is_file(), f"Missing upstream description: {path}")

    def test_go2_controllers_config_exists(self):
        """The leg controller configuration must exist on disk."""
        path = self.robot_dir / "config" / "go2_controllers.yaml"
        self.assertTrue(path.is_file(), f"Missing controllers config: {path}")

    def test_go2_sim_xacro_includes_robot_and_control(self):
        """The sim wrapper must include the upstream description and ros2_control."""
        sim = self._repo_asset(self.robot["assets"]["urdf"]).read_text()
        self.assertIn("robot.xacro", sim)
        self.assertIn("go2_ros2_control.xacro", sim)

    def test_go2_joint_names_consistent(self):
        """Leg joints must agree across actuator registry and control xacro."""
        registry_joints = [a["joint"] for a in self.robot["actuators"]]
        self.assertEqual(len(registry_joints), 12)

        control = self._repo_asset(self.robot["assets"]["xacro"]).read_text()
        # The xacro declares joints via a per-leg macro using ${prefix}
        for part in ("hip", "thigh", "calf"):
            self.assertIn(
                f'name="${{prefix}}_{part}_joint"', control,
                f"ros2_control xacro macro is missing the {part} joint",
            )
        # Every registry joint must be produced by a macro instantiation
        for joint in registry_joints:
            prefix = joint.split("_")[0]
            self.assertIn(
                f'go2_leg_ros2_control prefix="{prefix}"', control,
                f"ros2_control xacro does not instantiate leg: {prefix}",
            )

    def test_go2_controllers_list_all_joints(self):
        """The controllers YAML must command exactly the 12 registry joints."""
        registry_joints = [a["joint"] for a in self.robot["actuators"]]
        import yaml
        controllers = yaml.safe_load(
            (self.robot_dir / "config" / "go2_controllers.yaml").read_text()
        )
        commanded = controllers["go2_group_effort_controller"]["ros__parameters"]["joints"]
        self.assertEqual(commanded, registry_joints)

    def test_go2_control_xacro_declares_ignition_plugin(self):
        """The ros2_control xacro must support the Ignition hardware plugin."""
        control = self._repo_asset(self.robot["assets"]["xacro"]).read_text()
        self.assertIn("ign_ros2_control/IgnitionSystem", control)

    def test_go2_upstream_meshes_exist(self):
        """Mesh references in the upstream xacros must resolve on disk."""
        import re
        refs = set()
        for xacro_path in ("xacro/robot.xacro", "xacro/leg.xacro"):
            text = (self.robot_dir / xacro_path).read_text()
            refs.update(
                re.findall(
                    r"package://robots/unitree/go2_description/meshes/([\w./\-]+)",
                    text,
                )
            )
        self.assertGreater(len(refs), 0, "No mesh references found in go2 xacros")
        for rel in refs:
            self.assertTrue(
                (self.robot_dir / "meshes" / rel).exists(),
                f"go2 xacro references missing mesh: {rel}",
            )

    def test_go2_description_processes_with_xacro(self):
        """The full sim description must expand cleanly with xacro when available."""
        xacro_bin = shutil.which("xacro")
        if xacro_bin is None:
            self.skipTest("xacro executable not available")
        result = subprocess.run(
            [xacro_bin, str(self._repo_asset(self.robot["assets"]["urdf"]))],
            capture_output=True, text=True, timeout=120,
        )
        self.assertEqual(
            result.returncode, 0,
            f"xacro processing failed:\n{result.stderr}",
        )
        self.assertIn('<robot name="go2">', result.stdout)
        self.assertIn("FL_hip_joint", result.stdout)
        self.assertIn("RR_calf_joint", result.stdout)


if __name__ == "__main__":
    unittest.main()
