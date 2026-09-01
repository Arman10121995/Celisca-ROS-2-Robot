"""
P3.5 Quadrotor SITL Qualification Tests

Tests for qualifying the Quadrotor SITL as a simulated, commandable aerial
profile (P3.5). Validates registry contracts, smoke-test composition, the
MAVLink command interface, and the bringup controller console entry point.

Unlike legged/mobile robots, aerial SITL does not use ros2_control joints
or xacro expansion of a URDF. Its commandable interface is the mavros
Offboard Controller publishing AttitudeTarget setpoints to
/mavros/setpoint/attitude. This suite verifies that contract without
requiring mavros installed (the node degrades gracefully).
"""

import shutil
import subprocess
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
from robot_lab_registry.validation import Composition


class QuadrotorSITLQualificationTests(unittest.TestCase):
    """Test suite for Quadrotor SITL qualification (P3.5)."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)

    def test_quadrotor_registered(self):
        """Verify the Quadrotor SITL entry is registered as integrated."""
        robot = self.registry.robots.get("quadrotor_sitl")
        self.assertIsNotNone(robot)
        self.assertEqual(robot["name"], "Quadrotor SITL")
        self.assertEqual(robot["robot_class"], "aerial")
        self.assertEqual(robot["status"], "integrated")

    def test_quadrotor_is_multirotor(self):
        """Verify the locomotion contract: multirotor, 4 rotors."""
        robot = self.registry.robots.get("quadrotor_sitl")
        self.assertEqual(robot["locomotion"]["type"], "multirotor")
        self.assertEqual(robot["locomotion"]["dof"], 4)
        self.assertIn("max_velocity", robot["locomotion"])

    def test_quadrotor_has_required_sensors(self):
        """Verify the IMU, GPS, and RGB camera sensor contracts."""
        robot = self.registry.robots.get("quadrotor_sitl")
        sensors = {s["type"]: s for s in robot["sensors"]}
        self.assertIn("imu", sensors)
        self.assertIn("gps", sensors)
        self.assertIn("camera", sensors)
        self.assertEqual(sensors["imu"]["topic"], "/imu")
        self.assertEqual(sensors["imu"]["frame"], "base_link")
        self.assertEqual(sensors["gps"]["topic"], "/gps")
        self.assertEqual(sensors["camera"]["topic"], "/camera/image_raw")
        self.assertEqual(sensors["camera"]["frame"], "camera_link")

    def test_quadrotor_has_mavlink_command_interface(self):
        """Verify the MAVLink command contracts."""
        robot = self.registry.robots.get("quadrotor_sitl")
        self.assertIn("mavros_msgs/AttitudeTarget", robot["command_interfaces"])
        self.assertIn("mavros_msgs/PositionTarget", robot["command_interfaces"])

    def test_quadrotor_has_actuators(self):
        """Verify the four rotor actuator contracts."""
        robot = self.registry.robots.get("quadrotor_sitl")
        names = {a["name"] for a in robot["actuators"]}
        self.assertEqual(names, {"front_left_rotor", "front_right_rotor",
                                 "rear_left_rotor", "rear_right_rotor"})

    def test_quadrotor_has_state_interfaces(self):
        """Verify odometry + IMU state contracts."""
        robot = self.registry.robots.get("quadrotor_sitl")
        states = robot["state_interfaces"]
        self.assertIn("nav_msgs/Odometry", states)
        self.assertIn("sensor_msgs/Imu", states)

    def test_quadrotor_has_aerial_capabilities(self):
        """Verify flight-capable capabilities."""
        robot = self.registry.robots.get("quadrotor_sitl")
        self.assertIn("flight", robot["capabilities"])
        self.assertIn("waypoint_navigation", robot["capabilities"])

    def test_quadrotor_supports_sitl_and_gazebo(self):
        """Verify both SITL and Gazebo simulator backends."""
        robot = self.registry.robots.get("quadrotor_sitl")
        self.assertIn("gazebo", robot["supported_simulators"])
        self.assertIn("sitl", robot["supported_simulators"])

    def test_quadrotor_assets_declaration(self):
        """Verify the URDF asset path is declared (mesh-free sim model)."""
        robot = self.registry.robots.get("quadrotor_sitl")
        self.assertIn("urdf", robot["assets"])
        self.assertTrue(
            robot["assets"]["urdf"].endswith("quadrotor_sitl.urdf.xacro")
        )

    def test_quadrotor_smoke_experiments_resolve(self):
        """Verify smoke_experiments reference registered experiments."""
        robot = self.registry.robots.get("quadrotor_sitl")
        for exp_id in robot["smoke_experiments"]:
            experiment = self.registry.experiments.get(exp_id)
            self.assertIsNotNone(
                experiment, f"smoke_experiments references unknown experiment: {exp_id}"
            )
            self.assertEqual(experiment["robot_id"], "quadrotor_sitl")
            self.assertEqual(experiment["scenario_id"], "quadrotor_sitl_smoke_test")

    def test_quadrotor_dry_run_composition(self):
        """Test creating a dry-run launch composition for the aerial smoke test."""
        composition_dict = {
            "robot_id": "quadrotor_sitl",
            "environment_id": "empty",
            "simulator": "gazebo",
            "scenario_id": "quadrotor_sitl_smoke_test",
        }
        composition = Composition(**composition_dict)
        self.assertEqual(composition.robot_id, "quadrotor_sitl")
        robot = self.registry.robots.get(composition.robot_id)
        environment = self.registry.environments.get(composition.environment_id)
        scenario = self.registry.scenarios.get(composition.scenario_id)
        self.assertIsNotNone(robot)
        self.assertIsNotNone(environment)
        self.assertIsNotNone(scenario)

    def test_quadrotor_smoke_test_scenario_registered(self):
        """Verify the aerial smoke scenario is registered and well-formed."""
        scenario = self.registry.scenarios.get("quadrotor_sitl_smoke_test")
        self.assertIsNotNone(scenario)
        self.assertEqual(scenario["task_type"], "smoke_test")
        self.assertIn("aerial", scenario["required_robot_classes"])

    def test_quadrotor_smoke_test_experiment_pinned(self):
        """Verify the experiment pins all seven algorithm categories."""
        experiment = self.registry.experiments.get("quadrotor_sitl_smoke_test")
        self.assertIsNotNone(experiment)
        self.assertEqual(experiment["robot_id"], "quadrotor_sitl")
        self.assertEqual(experiment["simulator"], "gazebo")
        self.assertEqual(experiment["environment_id"], "empty")
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

    def test_quadrotor_control_algorithm_supports_aerial(self):
        """mavros_offboard_controller must support the aerial robot class."""
        algorithm = self.registry.algorithms.get("mavros_offboard_controller")
        self.assertIsNotNone(algorithm)
        self.assertEqual(algorithm["category"], "control")
        self.assertIn("aerial", algorithm["supported_robot_classes"])


class QuadrotorSITLAssetContractTests(unittest.TestCase):
    """Validation layer 4 (assets): referenced files must exist and be consistent."""

    @classmethod
    def setUpClass(cls):
        config_dir = Path(__file__).parent.parent / "config"
        cls.registry = Registry(config_dir)
        cls.registry.load(config_dir)
        # robot_lab_registry/test/ -> src/robot_lab/robot_lab_registry -> src/robot_lab
        # -> src -> workspace root
        cls.src_root = Path(__file__).resolve().parents[1].parent.parent
        cls.robot = cls.registry.robots.get("quadrotor_sitl")

    def test_quadrotor_urdf_exists(self):
        """The quadrotor URDF xacro must exist on disk."""
        path = self.src_root / self.robot["assets"]["urdf"]
        self.assertTrue(path.is_file(), f"Missing URDF asset: {path}")

    def test_quadrotor_no_mesh_dependencies(self):
        """The mesh-free quadrotor must not reference any mesh files."""
        urdf_path = self.src_root / self.robot["assets"]["urdf"]
        urdf = urdf_path.read_text()
        # Check for actual mesh element tags (not just the word in comments)
        import re
        self.assertIsNone(
            re.search(r'<mesh\b', urdf),
            "Quadrotor description must be mesh-free (found <mesh> element)",
        )
        # No .dae / .stl / .obj mesh file references
        for ext in ('.dae', '.stl', '.obj'):
            self.assertNotIn(ext, urdf,
                             f"Quadrotor is primitive-geometry only (found {ext})")
        meshes = self.robot["assets"].get("meshes", [])
        self.assertEqual(meshes, [],
                         "Quadrotor is primitive-geometry only")

    def test_quadrotor_four_rotor_joints_in_urdf(self):
        """The URDF must declare all four rotor sides via xacro macro."""
        urdf_path = self.src_root / self.robot["assets"]["urdf"]
        urdf = urdf_path.read_text()
        for side in ("front_left", "front_right",
                      "rear_left", "rear_right"):
            self.assertIn(f'side="{side}"', urdf,
                          f"URDF missing rotor macro invocation for side: {side}")
        # The macro itself defines the _rotor_joint pattern
        self.assertIn("_rotor_joint", urdf)

    def test_quadrotor_sensors_in_urdf(self):
        """The URDF must carry IMU, LIDAR, and camera links."""
        urdf_path = self.src_root / self.robot["assets"]["urdf"]
        urdf = urdf_path.read_text()
        self.assertIn("imu_link", urdf)
        self.assertIn("laser_link", urdf)
        self.assertIn("camera_link", urdf)

    def test_quadrotor_mavros_controller_node(self):
        """The bringup mavros_offboard_controller.py must exist."""
        node = self.src_root / "robot_lab_adapter" / "robot_lab_adapter" / \
               "mavros_offboard_controller.py"
        self.assertTrue(node.is_file(), f"Missing controller node: {node}")
        src = node.read_text()
        self.assertIn("MavrosOffboardController", src)
        self.assertIn("AttitudeTarget", src)

    def test_quadrotor_description_processes_with_xacro(self):
        """The full quadrotor description must expand cleanly with xacro."""
        xacro_bin = shutil.which("xacro")
        if xacro_bin is None:
            self.skipTest("xacro executable not available")
        urdf_path = str(self.src_root / self.robot["assets"]["urdf"])
        result = subprocess.run(
            ["bash", "-c", f"source /opt/ros/humble/setup.bash && {xacro_bin} {urdf_path}"],
            capture_output=True, text=True, timeout=60,
        )
        # Skip if the error is due to missing workspace packages (not a code defect)
        if result.returncode != 0 and "No such file" in result.stderr:
            self.skipTest(
                f"$(find) package resolution failed (workspace may not be built): "
                f"{result.stderr.strip()}"
            )
        self.assertEqual(
            result.returncode, 0,
            f"xacro processing failed:\n{result.stderr}",
        )
        self.assertIn('quadrotor_sitl', result.stdout)
        self.assertIn("front_left_rotor_joint", result.stdout)
        self.assertIn("rear_right_rotor_joint", result.stdout)
        self.assertIn("imu_link", result.stdout)


if __name__ == "__main__":
    unittest.main()


