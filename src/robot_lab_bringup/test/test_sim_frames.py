"""Frame conventions of the PyBullet, MuJoCo and Isaac Sim bridges.

Each test pins a defect that made localization, mapping or navigation wrong
on a non-Gazebo backend:

* MuJoCo published ``xquat`` (w, x, y, z) as ROS (x, y, z, w), so a level
  robot was reported rolled 180 degrees.
* All three bridges published world-frame twists on /odom/ground_truth, while
  robot_localization fuses vx / vy / vyaw in the child frame.
* MuJoCo cast its scan from 0.12 m above the base along the base heading;
  Bumperbot's laser is 0.154 m up and faces backwards.
* Every LaserScan claimed ``angle_max = pi`` for 360 samples starting at
  -pi, i.e. one more sample than it carried.
* Isaac Sim had no scan at all, and its reset command was silently dropped.
"""
import io
import math
import os
import sys
import types
import unittest
from unittest import mock

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.dirname(os.path.dirname(_HERE))
for _path in (os.path.join(_SRC, "robot_lab_utils"),
              os.path.join(_SRC, "robot_lab_mujoco", "python"),
              os.path.join(_SRC, "robot_lab_pybullet", "python"),
              os.path.join(_SRC, "robot_lab_isaac", "python")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from robot_lab_utils import sim_frames  # noqa: E402

# Bumperbot's laser mount, reduced to the joints that place it.
_BUMPERBOT_LASER_URDF = """
<robot name="bumperbot">
  <link name="base_footprint"/>
  <link name="base_link"/>
  <link name="laser_link"/>
  <link name="wheel_left_link"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/><child link="base_link"/>
    <origin xyz="0 0 0.033" rpy="0 0 0"/>
  </joint>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/><child link="laser_link"/>
    <origin xyz="-0.0050526 -0.0023221 0.1208" rpy="0 0 3.14"/>
  </joint>
  <joint name="wheel_left_joint" type="continuous">
    <parent link="base_link"/><child link="wheel_left_link"/>
    <origin xyz="0 0.0701 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def _yaw_quaternion(yaw):
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


class SimFramesTests(unittest.TestCase):

    def assertVectorAlmostEqual(self, expected, actual, places=6):
        for e, a in zip(expected, actual):
            self.assertAlmostEqual(e, a, places=places)

    def test_quaternion_orders_round_trip(self):
        wxyz = (0.9, 0.1, 0.2, 0.3)
        xyzw = sim_frames.xyzw_from_wxyz(wxyz)
        self.assertEqual([0.1, 0.2, 0.3, 0.9], xyzw)
        self.assertEqual(wxyz, sim_frames.wxyz_from_xyzw(xyzw))

    def test_level_robot_is_identity_in_ros_order(self):
        self.assertEqual([0.0, 0.0, 0.0, 1.0],
                         sim_frames.xyzw_from_wxyz((1.0, 0.0, 0.0, 0.0)))

    def test_world_velocity_is_expressed_in_the_body_frame(self):
        # Facing +y and driving forward: world (0, 1, 0) is body (1, 0, 0).
        heading = _yaw_quaternion(math.pi / 2)
        self.assertVectorAlmostEqual(
            (1.0, 0.0, 0.0), sim_frames.world_to_body((0.0, 1.0, 0.0), heading))
        self.assertVectorAlmostEqual(
            (0.0, 0.0, 0.4), sim_frames.world_to_body((0.0, 0.0, 0.4), heading))

    def test_laser_offset_comes_from_the_description(self):
        position, quaternion = sim_frames.offset_from_root(
            _BUMPERBOT_LASER_URDF, "laser_link")
        self.assertVectorAlmostEqual((-0.0050526, -0.0023221, 0.1538), position)
        self.assertAlmostEqual(3.14, sim_frames.yaw_of(quaternion), places=6)

    def test_offsets_are_available_from_every_carrying_link(self):
        offsets = sim_frames.mounted_sensor_offsets(
            _BUMPERBOT_LASER_URDF, "laser_link")
        self.assertAlmostEqual(0.1538, offsets["base_footprint"][0][2])
        self.assertAlmostEqual(0.1208, offsets["base_link"][0][2])

    def test_unknown_sensor_link_has_no_offset(self):
        self.assertIsNone(
            sim_frames.offset_from_root(_BUMPERBOT_LASER_URDF, "camera_link"))
        self.assertEqual({}, sim_frames.mounted_sensor_offsets(
            _BUMPERBOT_LASER_URDF, "camera_link"))
        self.assertEqual({}, sim_frames.mounted_sensor_offsets("<robot", "x"))

    def test_odometry_is_re_expressed_at_the_base_when_another_body_is_reported(self):
        # Isaac reports base_link, 0.033 m above base_footprint, and a point
        # 0.1 m ahead so the lever arm matters while turning.
        offset = ((0.1, 0.0, 0.033), (1.0, 0.0, 0.0, 0.0))
        yaw = math.pi / 2
        reported = (1.0, 2.1, 0.033)  # base at (1, 2, 0) facing +y
        q = sim_frames.xyzw_from_wxyz(_yaw_quaternion(yaw))
        # Base drives forward at 0.3 m/s while turning at 1 rad/s: the
        # reported point moves at v + w x r = (0, 0.3, 0) + (-0.1, 0, 0).
        position, orientation, linear, angular = sim_frames.body_odometry(
            reported, q, (-0.1, 0.3, 0.0), (0.0, 0.0, 1.0), offset)
        self.assertVectorAlmostEqual((1.0, 2.0, 0.0), position)
        self.assertVectorAlmostEqual(q, orientation)
        self.assertVectorAlmostEqual((0.3, 0.0, 0.0), linear)
        self.assertVectorAlmostEqual((0.0, 0.0, 1.0), angular)

    def test_odometry_without_offset_only_rotates_the_twist(self):
        q = sim_frames.xyzw_from_wxyz(_yaw_quaternion(math.pi))
        position, orientation, linear, _ = sim_frames.body_odometry(
            (3.0, 4.0, 0.0), q, (-0.2, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.assertVectorAlmostEqual((3.0, 4.0, 0.0), position)
        self.assertVectorAlmostEqual((0.2, 0.0, 0.0), linear)

    def test_mounted_pose_follows_the_base_heading(self):
        offset = ((1.0, 0.0, 0.5), _yaw_quaternion(math.pi))
        position, quaternion = sim_frames.mounted_pose(
            (1.0, 2.0, 0.0), _yaw_quaternion(math.pi / 2), offset)
        self.assertVectorAlmostEqual((1.0, 3.0, 0.5), position)
        self.assertAlmostEqual(-math.pi / 2, sim_frames.yaw_of(quaternion))


def _mujoco_available():
    try:
        import mujoco  # noqa: F401
        from robot_lab_mujoco import mujoco_spawner  # noqa: F401
    except Exception:
        return False
    return True


@unittest.skipUnless(_mujoco_available(), "mujoco spawner unavailable")
class MujocoScanTests(unittest.TestCase):
    """The ray starts inside the robot, including a sensor body that was not
    fused into the base; only the wall may be reported."""

    MJCF = """
    <mujoco>
      <worldbody>
        <geom type="plane" size="10 10 0.1"/>
        <geom name="wall" type="box" pos="2 0 0.5" size="0.1 2 0.5"/>
        <body name="robot" pos="0 0 0.2">
          <freejoint/>
          <geom type="box" size="0.15 0.15 0.1"/>
          <body name="laser" pos="0 0 0">
            <geom type="cylinder" size="0.05 0.05"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """

    @classmethod
    def setUpClass(cls):
        import mujoco
        import numpy as np
        from robot_lab_mujoco import mujoco_spawner
        cls.np = np
        cls.spawner = mujoco_spawner
        cls.model = mujoco.MjModel.from_xml_string(cls.MJCF)
        cls.data = mujoco.MjData(cls.model)
        mujoco.mj_forward(cls.model, cls.data)
        cls.robot = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_BODY, "robot")

    def _ray(self, direction):
        geomid = self.np.zeros(1, dtype=self.np.int32)
        return self.spawner._ray_skipping_robot(
            self.model, self.data, [0.0, 0.0, 0.2],
            self.np.array(direction, dtype=float), self.robot, 12.0, geomid)

    def test_ray_reports_the_wall_not_the_robot(self):
        self.assertAlmostEqual(1.9, self._ray([1.0, 0.0, 0.0]), places=3)

    def test_ray_with_nothing_beyond_the_robot_reports_no_return(self):
        self.assertLess(self._ray([-1.0, 0.0, 0.0]), 0.0)

    def test_body_velocity_is_in_the_body_frame_not_the_inertial_frame(self):
        import mujoco
        model = mujoco.MjModel.from_xml_string("""
        <mujoco><option gravity="0 0 0"/><worldbody>
          <body name="b" euler="0 0 90">
            <freejoint/>
            <inertial pos="0.1 0 0" euler="0 14 17" mass="1" diaginertia="1 2 3"/>
            <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
          </body>
        </worldbody></mujoco>""")
        data = mujoco.MjData(model)
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "b")
        # Facing +y, moving +y and spinning about z: forward in its own frame.
        data.qvel[:6] = [0.0, 0.3, 0.0, 0.0, 0.0, 0.5]
        mujoco.mj_forward(model, data)
        angular, linear = self.spawner._body_frame_velocity(model, data, body)
        for expected, actual in zip((0.3, 0.0, 0.0), linear):
            self.assertAlmostEqual(expected, actual)
        self.assertAlmostEqual(0.5, angular[2])

    def test_mujoco_quaternion_of_a_level_body_is_identity_in_ros_order(self):
        self.assertEqual(
            [0.0, 0.0, 0.0, 1.0],
            sim_frames.xyzw_from_wxyz(self.data.xquat[self.robot]))


class IsaacScanTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from robot_lab_isaac import isaac_runtime
        cls.runtime = isaac_runtime

    @staticmethod
    def _cast(hits_by_direction):
        """Fake raycast_all: reports (rigid_body, distance) hits per ray."""
        calls = []

        def cast(origin, direction, distance, report):
            calls.append((origin, direction, distance))
            key = (round(direction[0]), round(direction[1]))
            for body, hit_distance in hits_by_direction.get(key, ()):
                report(types.SimpleNamespace(rigid_body=body,
                                             distance=hit_distance))
        return cast, calls

    def test_robot_bodies_are_ignored_and_nearest_world_hit_wins(self):
        cast, _ = self._cast({(-1, 0): [("/World/base_link", 0.0),
                                        ("/World/map/far", 3.0),
                                        ("/World/map/near", 1.5)]})
        ranges = self.runtime._scan_ranges(
            cast, (0, 0, 0.15), 0.0, 4, 0.12, 12.0, {"/World/base_link"})
        # Four rays from -pi: -x, -y, +x, +y.
        self.assertAlmostEqual(1.5, ranges[0])
        self.assertTrue(all(math.isinf(r) for r in ranges[1:]))

    def test_hits_are_clamped_to_range_min_and_beyond_range_max_is_inf(self):
        cast, calls = self._cast({(-1, 0): [("/World/map/wall", 0.05)],
                                  (1, 0): [("/World/map/wall", 12.0)]})
        ranges = self.runtime._scan_ranges(cast, (0, 0, 0), 0.0, 4, 0.12,
                                           12.0, set())
        self.assertAlmostEqual(0.12, ranges[0])
        self.assertTrue(math.isinf(ranges[2]))
        self.assertEqual(12.0, calls[0][2])

    def test_scan_heading_is_the_laser_heading(self):
        cast, calls = self._cast({})
        self.runtime._scan_ranges(cast, (0, 0, 0), math.pi / 2, 4, 0.1, 5.0,
                                  set())
        first = calls[0][1]  # yaw - pi = -pi/2 -> -y
        self.assertAlmostEqual(0.0, first[0])
        self.assertAlmostEqual(-1.0, first[1])

    def test_runtime_frame_math_matches_the_shared_helper(self):
        offset = [[-0.0050526, -0.0023221, 0.1538],
                  list(_yaw_quaternion(3.14))]
        for yaw in (0.0, 0.8, -2.2):
            base_q = _yaw_quaternion(yaw)
            origin, scan_yaw = self.runtime._mounted_origin_and_yaw(
                (1.0, -2.0, 0.0), base_q, offset)
            expected, quaternion = sim_frames.mounted_pose(
                (1.0, -2.0, 0.0), base_q, offset)
            for e, a in zip(expected, origin):
                self.assertAlmostEqual(e, a)
            self.assertAlmostEqual(
                math.cos(sim_frames.yaw_of(quaternion)), math.cos(scan_yaw))
            self.assertAlmostEqual(
                math.sin(sim_frames.yaw_of(quaternion)), math.sin(scan_yaw))

    def test_reset_command_reaches_the_runtime_loop(self):
        reader = self.runtime._StdinReader()
        stdin = io.StringIO('{"cmd_vel": [0.2, 0.1]}\n{"reset": true}\n')
        with mock.patch.object(self.runtime.sys, "stdin", stdin):
            reader.run()
        self.assertTrue(reader.reset_requested)
        self.assertEqual([0.2, 0.1], reader.cmd)
        self.assertTrue(reader.stop)  # EOF


class PybulletScanTests(unittest.TestCase):
    """rayTestBatch hits are (body, link, fraction, position, normal); the
    bridge read the position as the fraction, so every range was inf."""

    @classmethod
    def setUpClass(cls):
        try:
            from robot_lab_pybullet import pybullet_spawner
        except Exception as exc:  # pragma: no cover - environment dependent
            raise unittest.SkipTest("pybullet spawner unavailable: %s" % exc)
        cls.scan = staticmethod(pybullet_spawner._planar_scan)

    def test_hit_fraction_becomes_a_range(self):
        def batch(froms, tos):
            return [(7, -1, 0.25, (9, 9, 9), (0, 0, 1)) for _ in froms]
        ranges = self.scan(batch, (0, 0, 0.15), 0.0, 8, 0.12, 12.0, robot_id=1)
        self.assertEqual([3.0] * 8, ranges)

    def test_robot_self_hits_are_cast_again_from_beyond(self):
        calls = []

        def batch(froms, tos):
            calls.append(froms)
            if len(calls) == 1:  # inside the laser housing
                return [(1, 3, 0.01, (0, 0, 0), (0, 0, 1)) for _ in froms]
            return [(-1, -1, 1.0, (0, 0, 0), (0, 0, 0)) if i % 2 else
                    (5, -1, 0.5, (0, 0, 0), (0, 0, 1))
                    for i, _ in enumerate(froms)]
        ranges = self.scan(batch, (0, 0, 0), 0.0, 4, 0.12, 10.0, robot_id=1)
        self.assertEqual(2, len(calls))
        self.assertAlmostEqual(0.101, calls[1][0][0] * -1, places=6)
        self.assertAlmostEqual(0.101 + 0.5 * (10.0 - 0.101), ranges[0])
        self.assertTrue(math.isinf(ranges[1]))


class UrdfContactTests(unittest.TestCase):

    def test_gazebo_friction_is_read_per_link(self):
        from robot_lab_utils.urdf_contact import gazebo_link_friction
        urdf = ('<robot name="r"><link name="caster"/>'
                '<gazebo reference="caster"><mu1>0.1</mu1><mu2>0.3</mu2></gazebo>'
                '<gazebo reference="wheel"><mu2>2</mu2></gazebo>'
                '<gazebo><plugin filename="x"/></gazebo>'
                '<gazebo reference="bad"><mu1>high</mu1></gazebo></robot>')
        self.assertEqual({"caster": 0.1, "wheel": 2.0}, gazebo_link_friction(urdf))
        self.assertEqual({}, gazebo_link_friction("<robot"))


class PybulletDriveTests(unittest.TestCase):
    """Bumperbot turns at the commanded rate once its casters slide."""

    @classmethod
    def setUpClass(cls):
        try:
            import pybullet  # noqa: F401
            from ament_index_python.packages import get_package_share_directory
            from robot_lab_pybullet import pybullet_spawner
            get_package_share_directory("robot_lab_robots")
        except Exception as exc:  # pragma: no cover - environment dependent
            raise unittest.SkipTest("pybullet / workspace unavailable: %s" % exc)
        cls.spawner = pybullet_spawner

    def _turn_rate(self, use_description_friction):
        import re
        import tempfile
        import pybullet as p
        import pybullet_data
        from ament_index_python.packages import get_package_share_directory as share
        from robot_lab_utils.urdf_contact import gazebo_link_friction
        sp = self.spawner
        text = sp._xacro_to_urdf(os.path.join(
            share("robot_lab_robots"), "bumperbot/urdf/bumperbot.urdf.xacro"))
        text = sp._rewrite_package_uris(text, {
            name: share(name) for name in set(re.findall(r"package://([^/]+)/", text))})
        friction = gazebo_link_friction(text)
        with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as handle:
            handle.write(sp._strip_gazebo_tags(text))
        client = p.connect(p.DIRECT)
        try:
            p.setGravity(0, 0, -9.81)
            p.setTimeStep(1.0 / 240.0)
            p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
            robot = p.loadURDF(handle.name, [0, 0, 0],
                               flags=p.URDF_USE_INERTIA_FROM_FILE)
            links, joints = {}, {}
            for index in range(p.getNumJoints(robot)):
                info = p.getJointInfo(robot, index)
                joints[info[1].decode()] = index
                links[info[12].decode()] = index
            if use_description_friction:
                self.assertIn("caster_front_link",
                              sp._apply_link_friction(robot, links, friction))
            for _ in range(240):
                p.stepSimulation()
            _, start = p.getBasePositionAndOrientation(robot)
            for _ in range(720):
                p.setJointMotorControl2(robot, joints["wheel_left_joint"],
                                        p.VELOCITY_CONTROL, targetVelocity=3.0, force=5.0)
                p.setJointMotorControl2(robot, joints["wheel_right_joint"],
                                        p.VELOCITY_CONTROL, targetVelocity=6.09, force=5.0)
                p.stepSimulation()
            _, end = p.getBasePositionAndOrientation(robot)
        finally:
            p.disconnect(client)
            os.unlink(handle.name)
        yaw = lambda q: p.getEulerFromQuaternion(q)[2]  # noqa: E731
        return math.remainder(yaw(end) - yaw(start), 2 * math.pi) / 3.0

    def test_description_friction_brings_turn_rate_close_to_command(self):
        default_rate = self._turn_rate(False)
        rate = self._turn_rate(True)
        self.assertGreater(rate, default_rate)
        self.assertGreater(rate, 0.5)  # command 0.6 rad/s


class CameraModelTests(unittest.TestCase):
    """The simulated RGB-D cameras must match the Gazebo OAK-D sensor."""

    def setUp(self):
        from robot_lab_utils import camera_model
        self.cm = camera_model

    def test_intrinsics_follow_the_horizontal_field_of_view(self):
        fx, fy, cx, cy = self.cm.intrinsics(320, 240, 1.25)
        self.assertAlmostEqual(160.0 / math.tan(0.625), fx)
        self.assertEqual((fx, 160.0, 120.0), (fy, cx, cy))
        vertical = self.cm.vertical_fov(1.25, 320, 240)
        self.assertAlmostEqual(120.0, math.tan(vertical / 2.0) * fy)

    def test_opengl_camera_looks_along_the_link_x_axis(self):
        rotation = self.cm.LINK_TO_OPENGL_CAMERA
        forward = sim_frames.rotate((0.0, 0.0, -1.0), rotation)
        up = sim_frames.rotate((0.0, 1.0, 0.0), rotation)
        for expected, actual in zip((1.0, 0.0, 0.0), forward):
            self.assertAlmostEqual(expected, actual)
        for expected, actual in zip((0.0, 0.0, 1.0), up):
            self.assertAlmostEqual(expected, actual)

    def test_depth_buffer_is_linearised_to_metres(self):
        near, far = 0.3, 100.0
        self.assertAlmostEqual(near, self.cm.linear_depth(0.0, near, far))
        self.assertAlmostEqual(far, self.cm.linear_depth(1.0, near, far))
        z_buffer = (far - far * near / 2.0) / (far - near)
        self.assertAlmostEqual(2.0, self.cm.linear_depth(z_buffer, near, far))


class PybulletCameraTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        try:
            import pybullet  # noqa: F401
            from robot_lab_pybullet import pybullet_spawner
        except Exception as exc:  # pragma: no cover - environment dependent
            raise unittest.SkipTest("pybullet spawner unavailable: %s" % exc)
        cls.spawner = pybullet_spawner

    def test_rgbd_render_has_metric_depth_and_upright_rows(self):
        import pybullet as p
        import pybullet_data
        camera = {"width": 160, "height": 120, "horizontal_fov": 1.25,
                  "near": 0.3, "far": 100.0}
        client = p.connect(p.DIRECT)
        try:
            p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
            rgb, depth = self.spawner._render_rgbd((0, 0, 0.5), (0, 0, 0, 1), camera)
            self.assertEqual((120, 160, 3), rgb.shape)
            self.assertEqual("float32", str(depth.dtype))
            # Level camera over a floor: sky in the top row, floor in the bottom.
            self.assertTrue(all(math.isinf(v) for v in depth[0]))
            self.assertTrue(all(math.isfinite(v) for v in depth[-1]))
            wall = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 3.0, 2.0])
            p.createMultiBody(0, wall, p.createVisualShape(
                p.GEOM_BOX, halfExtents=[0.1, 3.0, 2.0]), [2.1, 0.0, 2.0])
            _, depth = self.spawner._render_rgbd((0, 0, 0.5), (0, 0, 0, 1), camera)
            self.assertAlmostEqual(2.0, float(depth[60, 80]), delta=0.02)
        finally:
            p.disconnect(client)


@unittest.skipUnless(_mujoco_available(), "mujoco spawner unavailable")
class MujocoCameraTests(unittest.TestCase):

    MJCF = """
    <mujoco>
      <worldbody>
        <light pos="0 0 3"/>
        <geom type="plane" size="10 10 0.1"/>
        <geom type="box" pos="2.1 0 1" size="0.1 3 1"/>
        <body name="robot" pos="0 0 0">
          <freejoint/>
          <geom type="box" size="0.05 0.05 0.05" pos="0 0 0.05"/>
        </body>
      </worldbody>
    </mujoco>
    """

    @classmethod
    def setUpClass(cls):
        from robot_lab_mujoco import mujoco_spawner
        cls.spawner = mujoco_spawner

    def _model(self):
        import mujoco
        offset = ((0.06, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0))
        mjcf = self.spawner._add_camera_to_base(self.MJCF, "cam", offset, 43.0)
        self.assertEqual(mjcf, self.spawner._add_camera_to_base(mjcf, "cam", offset, 43.0))
        model = mujoco.MjModel.from_xml_string(mjcf)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        return mujoco, model, data

    def test_camera_rides_the_base_and_looks_forward(self):
        mujoco, model, data = self._model()
        cam = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "cam")
        self.assertGreaterEqual(cam, 0)
        for expected, actual in zip((0.06, 0.0, 0.5), data.cam_xpos[cam]):
            self.assertAlmostEqual(expected, actual)
        forward = -data.cam_xmat[cam].reshape(3, 3)[:, 2]
        for expected, actual in zip((1.0, 0.0, 0.0), forward):
            self.assertAlmostEqual(expected, actual)

    def test_depth_render_is_metric_when_opengl_is_available(self):
        mujoco, model, data = self._model()
        model.vis.map.znear = 0.3 / model.stat.extent
        try:
            renderer = mujoco.Renderer(model, 120, 160)
        except Exception as exc:  # pragma: no cover - host dependent
            self.skipTest("no OpenGL context: %s" % exc)
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera="cam")
        depth = renderer.render()
        renderer.close()
        self.assertAlmostEqual(1.94, float(depth[60, 80]), delta=0.02)


class LaserScanGeometryTests(unittest.TestCase):
    """angle_min + (n - 1) * increment must equal angle_max."""

    def test_isaac_scan_message_describes_its_own_samples(self):
        try:
            from robot_lab_isaac.isaac_spawner import IsaacSpawner
        except Exception as exc:  # pragma: no cover - ROS environment only
            self.skipTest("isaac spawner unavailable: %s" % exc)
        parameters = {"laser_link_name": "laser_link", "scan_rate": 5.0,
                      "scan_range_min": 0.12, "scan_range_max": 12.0}
        published = []
        node = types.SimpleNamespace(
            get_parameter=lambda name: types.SimpleNamespace(
                value=parameters[name]),
            _scan_pub=types.SimpleNamespace(publish=published.append))
        IsaacSpawner._publish_scan(node, {"t": 2.25, "ranges": [1.0] * 360})
        msg = published[0]
        self.assertEqual("laser_link", msg.header.frame_id)
        self.assertEqual(2, msg.header.stamp.sec)
        samples = round((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        self.assertEqual(360, samples)
        self.assertEqual(360, len(msg.ranges))


if __name__ == "__main__":
    unittest.main()
