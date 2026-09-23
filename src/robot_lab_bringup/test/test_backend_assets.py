"""Regression tests for simulator-backend asset handling.

Each test pins a defect that made robots fail to open in a backend:

* MuJoCo dropped the robot's <asset> block while merging it into a world, so
  every mesh-based robot failed ("mesh 'base_link' not found in geom 1")
  and silently became the fallback box; imported robots were also welded to
  the world with no free joint.
* MuJoCo stepped the Unitree H1-2 into NaN on the first step because its
  thumb links start interpenetrating the wrist.
* PyBullet could not load descriptions with relative mesh paths once the
  prepared URDF was written to a temporary directory.
* The Isaac runtime exited during Kit startup because the host CUDA
  libnvJitLink was bound instead of the one Isaac's torch needs.
"""
import os
import re
import sys
import tempfile
import unittest

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_HERE))
for _path in (os.path.join(_SRC, "robot_lab_mujoco", "python"),
              os.path.join(_SRC, "robot_lab_pybullet", "python"),
              os.path.join(_SRC, "robot_lab_utils")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

_ROBOTS = os.path.join(_SRC, "robot_lab_robots", "config", "robots.yaml")


def _mujoco_available():
    try:
        import mujoco  # noqa: F401
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory("robot_lab_robots")
        get_package_share_directory("robot_lab_maps")
    except Exception:
        return False
    return True


def _robot_mjcf(spawner, robot_id, logger=None):
    from ament_index_python.packages import get_package_share_directory
    config = yaml.safe_load(open(_ROBOTS))["robots"][robot_id]
    path = os.path.join(get_package_share_directory("robot_lab_robots"),
                        config["xacro"])
    urdf = spawner._xacro_to_urdf(path)
    packages = {}
    for package in set(re.findall(r"package://([^/]+)/", urdf)):
        try:
            packages[package] = get_package_share_directory(package)
        except Exception:
            pass
    return spawner._build_mjcf_from_urdf(
        urdf, packages, logger=logger,
        robot_name=robot_id, base_dir=os.path.dirname(os.path.abspath(path)))


@unittest.skipUnless(_mujoco_available(), "mujoco / workspace shares unavailable")
class MujocoRobotImportTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        import mujoco
        from ament_index_python.packages import get_package_share_directory
        from robot_lab_mujoco import mujoco_spawner
        cls.mujoco = mujoco
        cls.spawner = mujoco_spawner
        cls.world = os.path.join(get_package_share_directory("robot_lab_maps"),
                                 "mjcf", "nav_empty.xml")

    def _model(self, robot_id):
        mjcf = _robot_mjcf(self.spawner, robot_id)
        self.assertNotEqual(self.spawner._FALLBACK_MJCF, mjcf,
                            "%s fell back to the generic box" % robot_id)
        merged = self.spawner.MuJoCoSpawner._merge_mjcf(self.world, mjcf)
        return self.mujoco.MjModel.from_xml_string(merged)

    def test_merge_keeps_robot_meshes(self):
        model = self._model("bumperbot")
        self.assertGreater(model.nmesh, 0, "robot mesh assets were dropped")

    def test_imported_robot_has_a_floating_base(self):
        model = self._model("bumperbot")
        free = [joint for joint in range(model.njnt)
                if model.jnt_type[joint] == self.mujoco.mjtJoint.mjJNT_FREE]
        self.assertEqual(1, len(free), "robot is welded to the world")

    def test_h1_2_steps_without_numerical_blow_up(self):
        mujoco = self.mujoco
        model = self._model("unitree_h1_2")
        data = mujoco.MjData(model)
        free = [joint for joint in range(model.njnt)
                if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE][0]
        data.qpos[model.jnt_qposadr[free] + 2] = 1.1
        bad = mujoco.mjtWarning.mjWARN_BADQACC
        for _ in range(int(0.5 / model.opt.timestep)):
            mujoco.mj_step(model, data)
            self.assertEqual(0, data.warning[bad].number,
                             "QACC blew up at t=%.3f" % data.time)

    def test_imported_wheels_are_actuated_and_drive_forward(self):
        """The URDF importer creates no actuators, so /cmd_vel moved nothing."""
        mujoco = self.mujoco
        mjcf = self.spawner._add_wheel_velocity_actuators(
            _robot_mjcf(self.spawner, "bumperbot"),
            ("wheel_left_joint", "wheel_right_joint"))
        model = mujoco.MjModel.from_xml_string(
            self.spawner.MuJoCoSpawner._merge_mjcf(self.world, mjcf))
        self.assertEqual(2, model.nu)
        self.assertEqual(2, mjcf.count('armature="0.005"'),
                         "unstable velocity servo on a bare 53 g wheel")
        data = mujoco.MjData(model)
        free = [joint for joint in range(model.njnt)
                if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE][0]
        base = model.jnt_bodyid[free]
        mujoco.mj_forward(model, data)
        start = data.xpos[base].copy()
        data.ctrl[:] = 0.3 / 0.033
        for _ in range(int(2.0 / model.opt.timestep)):
            mujoco.mj_step(model, data)
        moved = data.xpos[base] - start
        self.assertGreater(moved[0], 0.3, "robot did not drive forward")
        self.assertLess(abs(moved[1]), 0.1)

    def test_turn_command_turns_at_the_commanded_rate(self):
        mujoco = self.mujoco
        mjcf = self.spawner._add_wheel_velocity_actuators(
            _robot_mjcf(self.spawner, "bumperbot"),
            ("wheel_left_joint", "wheel_right_joint"))
        model = mujoco.MjModel.from_xml_string(
            self.spawner.MuJoCoSpawner._merge_mjcf(self.world, mjcf))
        data = mujoco.MjData(model)
        free = [joint for joint in range(model.njnt)
                if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE][0]
        quat = model.jnt_qposadr[free] + 3
        for _ in range(int(0.5 / model.opt.timestep)):
            mujoco.mj_step(model, data)
        # 0.15 m/s and 0.6 rad/s over the 0.17 m contact track, r = 0.033 m.
        data.ctrl[0], data.ctrl[1] = 3.0, 6.09
        start = None
        for step in range(int(3.0 / model.opt.timestep)):
            mujoco.mj_step(model, data)
            if step == int(1.0 / model.opt.timestep):
                w, _, _, z = data.qpos[quat:quat + 4]
                start = 2.0 * __import__("math").atan2(z, w)
        w, _, _, z = data.qpos[quat:quat + 4]
        rate = (2.0 * __import__("math").atan2(z, w) - start) / 2.0
        self.assertAlmostEqual(0.6, rate, delta=0.06)

    def test_physics_time_keeps_pace_with_the_tick(self):
        self.assertEqual(2, self.spawner._physics_substeps(1.0 / 240.0, 0.002))
        self.assertEqual(1, self.spawner._physics_substeps(0.001, 0.002))

    def test_existing_actuators_are_not_duplicated(self):
        mjcf = self.spawner._add_wheel_velocity_actuators(
            self.spawner._FALLBACK_MJCF, ("wheel_left_joint", "wheel_right_joint"))
        self.assertEqual(2, mjcf.count("<velocity"))

    def test_rest_pose_exclusion_leaves_non_overlapping_robots_alone(self):
        mjcf = _robot_mjcf(self.spawner, "bumperbot")
        self.assertNotIn("<exclude", mjcf)


@unittest.skipUnless(_mujoco_available(), "mujoco / workspace shares unavailable")
class MujocoRclpyLoggingTests(unittest.TestCase):
    """Log through a real rclpy logger, which enforces one severity per call
    site. A permissive fake logger hid that every Berkeley Humanoid Lite
    spawn aborted with "Logger severity cannot be changed between calls"."""

    @classmethod
    def setUpClass(cls):
        import rclpy
        from robot_lab_mujoco import mujoco_spawner
        cls.rclpy = rclpy
        cls.spawner = mujoco_spawner
        cls.owns_context = not rclpy.ok()
        if cls.owns_context:
            rclpy.init()
        cls.node = rclpy.create_node("backend_assets_logging")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if cls.owns_context and cls.rclpy.ok():
            cls.rclpy.shutdown()

    def test_warning_then_info_does_not_abort_the_import(self):
        mjcf = _robot_mjcf(self.spawner, "berkeley_humanoid_lite",
                           logger=self.node.get_logger())
        self.assertNotEqual(self.spawner._FALLBACK_MJCF, mjcf)

    def test_every_severity_can_follow_any_other(self):
        logger = self.node.get_logger()
        for level in ("warning", "info", "error", "info", "warning", "debug"):
            self.spawner._emit_log(logger, level, "severity switch: " + level)


class PybulletMeshPathTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        try:
            from robot_lab_pybullet import pybullet_spawner
        except Exception as exc:  # pragma: no cover - environment dependent
            raise unittest.SkipTest("pybullet spawner unavailable: %s" % exc)
        cls.spawner = pybullet_spawner

    def test_relative_mesh_paths_are_anchored_to_the_description(self):
        urdf = '<mesh filename="meshes/pelvis.STL"/>'
        result = self.spawner._absolutize_mesh_paths(urdf, "/robots/h1_2")
        self.assertIn('filename="/robots/h1_2/meshes/pelvis.STL"', result)

    def test_shared_reader_quaternions_are_reordered_for_pybullet(self):
        # sdf_world is scalar-first (w, x, y, z); PyBullet is (x, y, z, w).
        self.assertEqual([0.0, 0.0, 0.0, 1.0], self.spawner._xyzw([1, 0, 0, 0]))
        self.assertEqual([0.1, 0.2, 0.3, 0.9],
                         self.spawner._xyzw([0.9, 0.1, 0.2, 0.3]))

    def test_resolved_and_uri_mesh_paths_are_untouched(self):
        for filename in ("/abs/a.stl", "package://pkg/a.stl",
                         "file:///abs/a.stl", "model://m/a.dae"):
            urdf = '<mesh filename="%s" scale="1 1 1"/>' % filename
            self.assertEqual(
                urdf, self.spawner._absolutize_mesh_paths(urdf, "/robots/x"))


class IsaacDriveKinematicsTests(unittest.TestCase):
    """/cmd_vel reaches the Isaac runtime as [linear_x, angular_z]; it must be
    converted to wheel speeds, not applied to the wheels as-is."""

    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, os.path.join(_SRC, "robot_lab_isaac", "python"))
        from robot_lab_isaac.isaac_runtime import _wheel_velocities
        cls.wheels = staticmethod(_wheel_velocities)

    def test_forward_command_drives_both_wheels_equally(self):
        left, right = self.wheels(0.3, 0.0, 0.033, 0.17)
        self.assertAlmostEqual(0.3 / 0.033, left)
        self.assertAlmostEqual(left, right)

    def test_pure_rotation_spins_wheels_in_opposite_directions(self):
        left, right = self.wheels(0.0, 1.0, 0.033, 0.17)
        self.assertAlmostEqual(-left, right)
        self.assertAlmostEqual(0.085 / 0.033, right)

    def test_zero_radius_does_not_divide_by_zero(self):
        left, right = self.wheels(0.1, 0.0, 0.0, 0.17)
        self.assertTrue(left > 0 and right > 0)


class IsaacQuaternionOrderTests(unittest.TestCase):
    """Isaac Sim is scalar-first (w, x, y, z); ROS messages are (x, y, z, w).
    Mixing them spawned robots rotated 180 degrees and scrambled /odom."""

    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, os.path.join(_SRC, "robot_lab_isaac", "python"))
        from robot_lab_isaac import isaac_runtime
        cls.runtime = isaac_runtime

    def test_zero_yaw_is_the_identity_rotation_in_isaac_order(self):
        self.assertEqual((1.0, 0.0, 0.0, 0.0),
                         self.runtime._isaac_quat_from_yaw(0.0))

    def test_yaw_survives_the_round_trip_to_ros_order(self):
        import math
        for yaw in (0.0, 0.7, -2.5, math.pi / 2):
            x, y, z, w = self.runtime._ros_quat_from_isaac(
                self.runtime._isaac_quat_from_yaw(yaw))
            self.assertAlmostEqual(0.0, x)
            self.assertAlmostEqual(0.0, y)
            self.assertAlmostEqual(yaw, math.atan2(2 * w * z, 1 - 2 * z * z))


class IsaacRuntimeEnvironmentTests(unittest.TestCase):

    def setUp(self):
        from robot_lab_utils import isaac_env
        self.isaac_env = isaac_env
        self.root = tempfile.mkdtemp()
        self.interpreter = os.path.join(self.root, "python.sh")
        with open(self.interpreter, "w") as handle:
            handle.write("#!/bin/sh\n")
        os.chmod(self.interpreter, 0o755)

    def _add_bundled_nvjitlink(self):
        path = os.path.join(self.root, self.isaac_env.BUNDLED_PRELOADS[0])
        os.makedirs(os.path.dirname(path))
        open(path, "w").close()
        return path

    def test_bundled_nvjitlink_is_preloaded_first(self):
        bundled = self._add_bundled_nvjitlink()
        value = self.isaac_env.isaac_preload_libraries(
            self.interpreter, "/opt/extra.so")
        entries = value.split()
        self.assertEqual(bundled, entries[0])
        self.assertIn("/opt/extra.so", entries)

    def test_preload_has_no_duplicates(self):
        bundled = self._add_bundled_nvjitlink()
        value = self.isaac_env.isaac_preload_libraries(
            self.interpreter, bundled + ":" + bundled)
        self.assertEqual(1, value.split().count(bundled))

    def test_install_without_bundled_library_keeps_caller_preloads(self):
        value = self.isaac_env.isaac_preload_libraries(
            self.interpreter, "/opt/extra.so")
        self.assertIn("/opt/extra.so", value.split())
        self.assertNotIn("nvJitLink", value)

    def test_explicit_isaac_python_always_wins(self):
        env = {"ISAAC_PYTHON": "/does/not/exist/python.sh"}
        self.assertEqual("/does/not/exist/python.sh",
                         self.isaac_env.find_isaac_python(env))
        available, detail = self.isaac_env.isaac_status(env)
        self.assertFalse(available)
        self.assertIn("does not exist", detail)

    def test_isaac_sim_path_is_searched(self):
        env = {"ISAAC_SIM_PATH": self.root}
        self.assertEqual(self.interpreter,
                         self.isaac_env.find_isaac_python(env))


if __name__ == "__main__":
    unittest.main()
