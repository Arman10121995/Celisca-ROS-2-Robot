"""PyBullet robot spawner node.

Opens a PyBullet GUI window, loads the robot URDF, runs the physics
step loop, and publishes all ROS 2 topics required by the rest of the
stack (joint_states, TF, odom, scan, imu, clock).
"""
import math
import os
import re
import subprocess
import tempfile
import threading
import time

try:
    import pybullet as p
    import pybullet_data
except ImportError:
    p = None
    pybullet_data = None

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import TransformBroadcaster


def _xacro_to_urdf(xacro_path):
    """Run xacro and return the URDF string."""
    result = subprocess.run(
        ["xacro", xacro_path],
        capture_output=True, text=True, timeout=30,
    )
    if result.returncode != 0:
        raise RuntimeError(f"xacro failed: {result.stderr[:500]}")
    return result.stdout


def _rewrite_package_uris(urdf_text, pkg_map):
    """Replace package://pkg/... with absolute file:// paths."""
    def _repl(m):
        pkg, rest = m.group(1), m.group(2)
        base = pkg_map.get(pkg, "")
        return ("file://" + os.path.join(base, rest)) if base else m.group(0)
    return re.sub(r"package://([^/]+)/(.+?)(?=[\"'\s<]|$)", _repl, urdf_text)


def _strip_gazebo_tags(urdf_text):
    """Remove XML tags that PyBullet cannot parse."""
    for tag in ("ros2_control", "transmission", "gazebo"):
        urdf_text = re.sub(
            rf"<{tag}[^>]*>.*?</{tag}>", "", urdf_text, flags=re.DOTALL)
    return urdf_text


def _yaw_quaternion(yaw):
    return Quaternion(x=0.0, y=0.0,
                      z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _rpy_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return Quaternion(
        x=sr*cp*cy - cr*sp*sy, y=cr*sp*cy + sr*cp*sy,
        z=cr*cp*sy - sr*sp*cy, w=cr*cp*cy + sr*sp*sy)


class PyBulletSpawner(Node):
    """Spawn robot URDF into a PyBullet physics world and publish
    joint_states, TF, odom, scan, imu, and clock."""

    def __init__(self):
        super().__init__("pybullet_spawner")
        # Parameters
        self.declare_parameter("robot_name", "bumperbot")
        self.declare_parameter("robot_package", "robot_lab_robots")
        self.declare_parameter("robot_xacro", "")
        self.declare_parameter("model", "")
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_z", 0.0)
        self.declare_parameter("spawn_yaw", 0.0)
        # use_sim_time is auto-declared by rclpy when passed via launch
        # overrides — only declare it if not already present.
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("world_name", "empty")
        self.declare_parameter("world_path", "")
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)
        self.declare_parameter("left_wheel_joint", "wheel_left_joint")
        self.declare_parameter("right_wheel_joint", "wheel_right_joint")
        self.declare_parameter("gui", True)
        self.declare_parameter("physics_rate", 240.0)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("scan_rate", 5.0)
        self.declare_parameter("laser_link_name", "laser_link")
        self.declare_parameter("scan_samples", 360)
        self.declare_parameter("scan_range_min", 0.12)
        self.declare_parameter("scan_range_max", 12.0)

        # State
        self._robot_id = -1
        self._link_idx = {}
        self._joint_idx = {}
        self._joint_names = []
        self._lw = -1
        self._rw = -1
        self._twist = Twist()
        self._twist_lock = threading.Lock()
        self._sim_t = 0.0
        self._sim_step = 0
        self._bpos = [0.0, 0.0, 0.0]
        self._born = [0.0, 0.0, 0.0, 1.0]
        self._blin = [0.0, 0.0, 0.0]
        self._bang = [0.0, 0.0, 0.0]
        self._jpos = []
        self._jvel = []
        self._running = True
        self._dt = 1.0 / max(self.get_parameter("physics_rate").value, 1.0)

        # Publishers
        self._pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self._imu_pub = self.create_publisher(Imu, "/imu/out", 10)
        self._clock_pub = self.create_publisher(Time, "/clock", 10)
        self._tf_br = TransformBroadcaster(self)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self._timer = self.create_timer(0.5, self._try_spawn)
        self._thread = None

    def _load_sdf_meshes(self, world_path: str) -> None:
        """Load static mesh geometries from an SDF .world file into PyBullet.


        Parses ``<uri>package://<pkg>/<path></uri>`` mesh references (and optional
        sibling ``<scale>n n n</scale>``), resolves them via ament package share
        directories,and creates a fixed-base multi-body per mesh."""
        import re as _re
        import xml.etree.ElementTree as ET
        try:
            tree = ET.parse(world_path)
        except Exception as e:
            self.get_logger().warn(f"SDF world parse error: {e}")
            return
        for mesh_el in tree.getroot().iter():
            if mesh_el.tag.rsplit("}", 1)[-1] != "mesh":
                continue
            uri_el = scale_el = None
            for child in mesh_el:
                tag = child.tag.rsplit("}", 1)[-1]
                if tag == "uri":
                    uri_el = child
                elif tag == "scale":
                    scale_el = child
            if uri_el is None:
                continue
            uri = (uri_el.text or "").strip()
            if not uri.startswith("package://"):
                continue
            rest = uri[len("package://"):]
            pkg, _, rel = rest.partition("/")
            try:
                from ament_index_python.packages import get_package_share_directory
                abs_path = os.path.join(get_package_share_directory(pkg), rel)
            except Exception:
                self.get_logger().warn(f"SDF mesh package not found: {pkg}")
                continue
            if not os.path.isfile(abs_path):
                continue
            scale = [1.0, 1.0, 1.0]
            if scale_el is not None and scale_el.text:
                try:
                    scale = [float(v) for v in scale_el.text.split()]
                except (ValueError, TypeError):
                    pass
            try:
                cid = p.createCollisionShape(p.GEOM_MESH, fileName=abs_path,
                                              meshScale=scale)
                vid = p.createVisualShape(p.GEOM_MESH, fileName=abs_path,
                                          meshScale=scale,
                                          rgbaColor=[0.6, 0.6, 0.6, 1.0])
                p.createMultiBody(baseCollisionShapeIndex=cid,
                                  baseVisualShapeIndex=vid, baseMass=0)
                self.get_logger().info(f"Loaded SDF mesh: {abs_path} (scale {scale})")
            except Exception as e:
                self.get_logger().warn(f"SDF mesh load error {abs_path}: {e}")
    def _on_cmd(self, msg):
        with self._twist_lock:
            self._twist = msg

    def _try_spawn(self):
        if self._robot_id >= 0:
            return
        self._timer.cancel()
        if p is None:
            self.get_logger().error("pybullet not importable")
            return
        try:
            self._spawn()
        except Exception as e:
            self.get_logger().error(f"Spawn failed: {e}")
            self._timer = self.create_timer(2.0, self._try_spawn)

    def _spawn(self):
        from ament_index_python.packages import get_package_share_directory
        # Resolve xacro
        model = self.get_parameter("model").value
        if not model:
            pkg = self.get_parameter("robot_package").value
            xacro = self.get_parameter("robot_xacro").value
            if pkg and xacro:
                model = os.path.join(get_package_share_directory(pkg), xacro)
        if not model or not os.path.isfile(str(model)):
            self.get_logger().error(f"URDF not found: {model}")
            return
        urdf = _xacro_to_urdf(str(model))
        pkg_map = {}
        for pkg_name in re.findall(r"\$\(find\s+([^)]+)\)", urdf):
            try:
                pkg_map[pkg_name] = get_package_share_directory(pkg_name)
            except Exception:
                pass
        rp = self.get_parameter("robot_package").value
        if rp and rp not in pkg_map:
            try:
                pkg_map[rp] = get_package_share_directory(rp)
            except Exception:
                pass
        urdf = _rewrite_package_uris(urdf, pkg_map)
        urdf = _strip_gazebo_tags(urdf)

        tmp = tempfile.NamedTemporaryFile(suffix=".urdf", delete=False, mode="w")
        tmp.write(urdf)
        tmp.close()

        # Connect
        gui = bool(self.get_parameter("gui").value and os.environ.get("DISPLAY"))
        p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self._dt)
        p.setRealTimeSimulation(0)

        # Ground
        p.loadURDF("plane.urdf")

        # World mesh (optional)
        wp = self.get_parameter("world_path").value
        if wp and os.path.isfile(str(wp)):
            try:
                ext = os.path.splitext(str(wp))[1].lower()
                if ext in (".stl", ".obj"):
                    cid = p.createCollisionShape(p.GEOM_MESH, fileName=str(wp))
                    vid = p.createVisualShape(p.GEOM_MESH, fileName=str(wp),
                                              rgbaColor=[0.6, 0.6, 0.6, 1.0])
                    p.createMultiBody(baseCollisionShapeIndex=cid,
                                      baseVisualShapeIndex=vid, baseMass=0)
                elif ext in (".world", ".sdf"):
                    self._load_sdf_meshes(str(wp))
                else:
                    p.loadURDF(str(wp), useFixedBase=True)
            except Exception as e:
                self.get_logger().warn(f"World load error: {e}")

        # Robot
        sx = self.get_parameter("spawn_x").value
        sy = self.get_parameter("spawn_y").value
        sz = self.get_parameter("spawn_z").value
        syaw = self.get_parameter("spawn_yaw").value
        orn = _rpy_quaternion(0, 0, syaw)
        self._robot_id = p.loadURDF(
            tmp.name, [sx, sy, sz], [orn.x, orn.y, orn.z, orn.w],
            useFixedBase=False,
            flags=p.URDF_USE_INERTIA_FROM_FILE)
        os.unlink(tmp.name)
        self.get_logger().info(f"Loaded robot id={self._robot_id}"
                               f" ({p.getNumJoints(self._robot_id)} joints)")

        # Index maps
        for i in range(p.getNumJoints(self._robot_id)):
            info = p.getJointInfo(self._robot_id, i)
            jn = info[1].decode()
            ln = info[12].decode()
            self._link_idx[ln] = i
            self._joint_idx[jn] = i
            if info[2] != p.JOINT_FIXED:
                self._joint_names.append(jn)
        lw = self.get_parameter("left_wheel_joint").value
        rw = self.get_parameter("right_wheel_joint").value
        self._lw = self._joint_idx.get(lw, -1)
        self._rw = self._joint_idx.get(rw, -1)

        for i in range(p.getNumJoints(self._robot_id)):
            if p.getJointInfo(self._robot_id, i)[2] != p.JOINT_FIXED:
                p.setJointMotorControl2(
                    self._robot_id, i, p.VELOCITY_CONTROL,
                    targetVelocity=0, force=0)

        self.get_logger().info(
            f"Joints: {self._joint_names}  lw={lw} rw={rw}")

        # Start physics thread
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.get_logger().info("PyBullet spawner running")

    def _loop(self):
        pub_dt = 1.0 / max(self.get_parameter("publish_rate").value, 1.0)
        scan_dt = 1.0 / max(self.get_parameter("scan_rate").value, 0.1)
        last_pub = 0.0
        last_scan = 0.0
        t0 = time.monotonic()
        wr = self.get_parameter("wheel_radius").value
        ws = self.get_parameter("wheel_separation").value

        while self._running and rclpy.ok():
            now = time.monotonic()
            elapsed = now - t0

            # cmd_vel
            with self._twist_lock:
                t = self._twist
            vl = (t.linear.x - t.angular.z * ws / 2.0) / wr
            vr = (t.linear.x + t.angular.z * ws / 2.0) / wr
            clamp = 50.0
            vl = max(-clamp, min(clamp, vl))
            vr = max(-clamp, min(clamp, vr))
            if self._lw >= 0:
                p.setJointMotorControl2(
                    self._robot_id, self._lw, p.VELOCITY_CONTROL,
                    targetVelocity=vl, force=5.0)
            if self._rw >= 0:
                p.setJointMotorControl2(
                    self._robot_id, self._rw, p.VELOCITY_CONTROL,
                    targetVelocity=vr, force=5.0)

            p.stepSimulation()
            self._sim_step += 1
            self._sim_t = self._sim_step * self._dt

            pos, orn = p.getBasePositionAndOrientation(self._robot_id)
            lv, av = p.getBaseVelocity(self._robot_id)
            self._bpos = list(pos)
            self._born = list(orn)
            self._blin = list(lv)
            self._bang = list(av)
            self._jpos, self._jvel = [], []
            for jn in self._joint_names:
                s = p.getJointState(self._robot_id, self._joint_idx[jn])
                self._jpos.append(s[0])
                self._jvel.append(s[1])

            if elapsed - last_pub >= pub_dt:
                last_pub = elapsed
                self._pub_joint_states()
                self._pub_odom()
                self._pub_imu()
                self._pub_clock()
                self._pub_tf()
            if elapsed - last_scan >= scan_dt:
                last_scan = elapsed
                self._pub_scan()

            dt = time.monotonic() - now
            if dt < self._dt:
                time.sleep(self._dt - dt)

    def _stamp(self):
        s = int(self._sim_t)
        ns = int((self._sim_t - s) * 1e9)
        return Time(sec=s, nanosec=ns)

    def _pub_joint_states(self):
        m = JointState()
        m.header.stamp = self._stamp()
        m.name = list(self._joint_names)
        m.position = list(self._jpos)
        m.velocity = list(self._jvel)
        m.effort = [0.0] * len(self._joint_names)
        self._pub_js.publish(m)

    def _pub_odom(self):
        m = Odometry()
        m.header.stamp = self._stamp()
        m.header.frame_id = "odom"
        m.child_frame_id = "base_footprint"
        m.pose.pose.position = Point(x=self._bpos[0], y=self._bpos[1], z=self._bpos[2])
        m.pose.pose.orientation = Quaternion(x=self._born[0], y=self._born[1], z=self._born[2], w=self._born[3])
        m.twist.twist.linear = Vector3(x=self._blin[0], y=self._blin[1], z=self._blin[2])
        m.twist.twist.angular = Vector3(x=self._bang[0], y=self._bang[1], z=self._bang[2])
        self._odom_pub.publish(m)

    def _pub_tf(self):
        t = TransformStamped()
        t.header.stamp = self._stamp()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation = Vector3(x=self._bpos[0], y=self._bpos[1], z=self._bpos[2])
        t.transform.rotation = Quaternion(x=self._born[0], y=self._born[1], z=self._born[2], w=self._born[3])
        self._tf_br.sendTransform(t)

    def _pub_imu(self):
        m = Imu()
        m.header.stamp = self._stamp()
        m.header.frame_id = "imu_link"
        m.orientation = Quaternion(x=self._born[0], y=self._born[1], z=self._born[2], w=self._born[3])
        m.orientation_covariance = [0.001,0.0,0.0, 0.0,0.001,0.0, 0.0,0.0,0.001]
        m.angular_velocity = Vector3(x=self._bang[0], y=self._bang[1], z=self._bang[2])
        m.angular_velocity_covariance = [0.01,0.0,0.0, 0.0,0.01,0.0, 0.0,0.0,0.01]
        m.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)
        m.linear_acceleration_covariance = [0.1,0.0,0.0, 0.0,0.1,0.0, 0.0,0.0,0.1]
        self._imu_pub.publish(m)

    def _pub_clock(self):
        self._clock_pub.publish(self._stamp())

    def _pub_scan(self):
        ln = self.get_parameter("laser_link_name").value
        li = self._link_idx.get(ln, -1)
        if li >= 0:
            st = p.getLinkState(self._robot_id, li)
            lp = list(st[0])
            lo = list(st[1])
        else:
            lp = [self._bpos[0], self._bpos[1], self._bpos[2] + 0.12]
            lo = self._born
        siny = 2.0*(lo[3]*lo[2] + lo[0]*lo[1])
        cosy = 1.0 - 2.0*(lo[1]**2 + lo[2]**2)
        byaw = math.atan2(siny, cosy)
        n = int(self.get_parameter("scan_samples").value)
        am = self.get_parameter("scan_range_min").value
        ax = self.get_parameter("scan_range_max").value
        rng = ax - am
        ai = 2.0 * math.pi / n
        ranges = []
        for i in range(n):
            a = byaw - math.pi + i * ai
            rt = [lp[0]+math.cos(a)*ax, lp[1]+math.sin(a)*ax, lp[2]]
            res = p.rayTest(lp, rt)
            if res and res[0][0] >= 0:
                hf = res[0][3]
                if isinstance(hf, (int, float)) and hf < 1.0:
                    d = hf * ax
                    ranges.append(max(am, float(d)))
                else:
                    ranges.append(float("inf"))
            else:
                ranges.append(float("inf"))
        msg = LaserScan()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = ln
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = ai
        msg.scan_time = 1.0 / max(self.get_parameter("scan_rate").value, 0.1)
        msg.range_min = am
        msg.range_max = ax
        msg.ranges = ranges
        self._scan_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if p is not None and self._robot_id >= 0:
            try:
                p.disconnect()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PyBulletSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
