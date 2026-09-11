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
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as RosClock
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# TF is published by the EKF (odom→base_footprint), not by the simulator spawner.


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


def _absolutize_mesh_paths(urdf_text, base_dir):
    """Make relative <mesh filename="..."> paths absolute.

    Some vendored descriptions (Unitree G1-29DOF, H1-2) reference their
    meshes relatively, e.g. ``meshes/pelvis.STL``.  The prepared URDF is
    written to a temporary directory, so PyBullet resolves those paths
    against the wrong base and refuses to load the model
    ("cannot find 'meshes/...' in any directory in urdf path").  Anchoring
    them to the source description's directory fixes it without touching
    the upstream files.
    """
    if not base_dir:
        return urdf_text

    def _repl(match):
        prefix, path, suffix = match.group(1), match.group(2), match.group(3)
        if not path or os.path.isabs(path):
            return match.group(0)
        if path.startswith(("package://", "model://", "file://", "http://",
                            "https://")):
            return match.group(0)
        return prefix + os.path.normpath(os.path.join(base_dir, path)) + suffix

    return re.sub(r'(<mesh\b[^>]*?\bfilename=")([^"]*)(")', _repl, urdf_text)


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
        self.get_logger().info("PyBulletSpawner __init__ started")
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
        self.get_logger().info("Parameters declared")
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
        self._last_cmd_time = time.monotonic()
        self._watchdog_timeout = 0.5  # stop if no cmd_vel for 500ms
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
        # Ground-truth odometry on /odom/ground_truth (perfect, from physics).
        # The estimated /odom is owned by the controller/EKF stack.
        self._odom_pub = self.create_publisher(Odometry, "/odom/ground_truth", 10)
        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self._imu_pub = self.create_publisher(Imu, "/imu/out", 10)
        self._clock_pub = self.create_publisher(RosClock, "/clock", 10)
        # TF published by the EKF, not the spawner.

        # Readiness / health / reset contracts (R2.3).
        try:
            self._ready_pub = self.create_publisher(Bool, "/robot_lab/ready", 10)
            self.get_logger().info("Created /robot_lab/ready publisher")
        except Exception as e:
            self.get_logger().error("Failed to create /robot_lab/ready: %s" % e)
        try:
            self._health_pub = self.create_publisher(
                DiagnosticArray, "/robot_lab/health", 10)
            self.get_logger().info("Created /robot_lab/health publisher")
        except Exception as e:
            self.get_logger().error("Failed to create /robot_lab/health: %s" % e)
        try:
            self._reset_srv = self.create_service(
                Trigger, "/robot_lab/reset", self._on_reset)
            self.get_logger().info("Created /robot_lab/reset service")
        except Exception as e:
            self.get_logger().error("Failed to create /robot_lab/reset: %s" % e)
        self._ready = False
        # Health timer created after init to avoid blocking
        self._health_timer = None

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        # Wall-clock timer — see note in mujoco_spawner / above comment about
        # the use_sim_time deadlock (this node publishes /clock itself).
        self._timer = self.create_timer(
            0.5, self._try_spawn,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )
        self._thread = None

    # ------------------------------------------------------------------
    # SDF world loading
    # ------------------------------------------------------------------
    _MODEL_DIRS = ("robot_lab_models",)

    def _resolve_sdf_uri(self, uri, base_dir):
        """Resolve a model://, package:// or relative SDF URI to a real path."""
        from ament_index_python.packages import get_package_share_directory
        uri = (uri or "").strip()
        if not uri:
            return ""
        if uri.startswith("model://"):
            rest = uri[len("model://"):]
            for package in self._MODEL_DIRS:
                try:
                    share = get_package_share_directory(package)
                except Exception:
                    continue
                candidate = os.path.join(share, "models", rest)
                if os.path.exists(candidate):
                    return candidate
            return ""
        if uri.startswith("package://"):
            package, _, rest = uri[len("package://"):].partition("/")
            try:
                candidate = os.path.join(
                    get_package_share_directory(package), rest)
            except Exception:
                return ""
            return candidate if os.path.exists(candidate) else ""
        if uri.startswith("file://"):
            uri = uri[len("file://"):]
        if os.path.isabs(uri):
            return uri if os.path.exists(uri) else ""
        candidate = os.path.normpath(os.path.join(base_dir, uri))
        return candidate if os.path.exists(candidate) else ""

    @staticmethod
    def _sdf_pose(element):
        """(x, y, z, roll, pitch, yaw) from an SDF <pose>; zeros when absent."""
        import xml.etree.ElementTree as ET
        if element is None:
            return [0.0] * 6
        for child in element:
            if child.tag.rsplit("}", 1)[-1] == "pose":
                values = [float(v) for v in (child.text or "").split()]
                return (values + [0.0] * 6)[:6]
        return [0.0] * 6

    @staticmethod
    def _sdf_child(element, name):
        if element is None:
            return None
        for child in element:
            if child.tag.rsplit("}", 1)[-1] == name:
                return child
        return None

    @staticmethod
    def _sdf_children(element, name):
        if element is None:
            return []
        return [c for c in element if c.tag.rsplit("}", 1)[-1] == name]

    @staticmethod
    def _sdf_text(element, name, default=""):
        child = PyBulletSpawner._sdf_child(element, name)
        if child is None or child.text is None:
            return default
        return child.text

    @staticmethod
    def _compose_pose(parent, child):
        """Compose two (position, quaternion) frames (PyBullet xyzw order)."""
        position, orientation = p.multiplyTransforms(
            parent[0], parent[1], child[0], child[1])
        return (list(position), list(orientation))

    @staticmethod
    def _frame_from_pose(pose):
        return ([pose[0], pose[1], pose[2]],
                list(p.getQuaternionFromEuler([pose[3], pose[4], pose[5]])))

    def _load_sdf_world(self, world_path: str) -> None:
        """Materialise an SDF world's static geometry in PyBullet.

        Previously only ``<mesh>`` geometry was loaded, so every arena built
        from box primitives (all twelve deterministic nav/terrain/aerial
        arenas) rendered as an empty plane.  Boxes, spheres, cylinders,
        planes and meshes are now created, with the full model -> link ->
        geometry pose chain applied, and ``model://`` includes are resolved
        against the vendored model library.
        """
        import xml.etree.ElementTree as ET
        try:
            root = ET.parse(world_path).getroot()
        except Exception as exc:
            self.get_logger().warn(f"SDF world parse error: {exc}")
            return
        world = self._sdf_child(root, "world") or root
        base_dir = os.path.dirname(os.path.abspath(world_path))
        identity = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])

        counts = {"created": 0, "skipped": 0}
        for model in self._sdf_children(world, "model"):
            self._load_sdf_model(model, identity, base_dir, counts)
        for include in self._sdf_children(world, "include"):
            wrapper = ET.Element("model")
            wrapper.append(include)
            self._load_sdf_model(wrapper, identity, base_dir, counts)
        self.get_logger().info(
            "SDF world '%s': %d static shape(s) created, %d skipped"
            % (os.path.basename(world_path), counts["created"],
               counts["skipped"]))

    def _load_sdf_model(self, model, frame, base_dir, counts, depth=0):
        import xml.etree.ElementTree as ET
        if depth > 8:  # pragma: no cover - defensive against cyclic includes
            return
        frame = self._compose_pose(
            frame, self._frame_from_pose(self._sdf_pose(model)))

        include = self._sdf_child(model, "include")
        if include is not None:
            uri = self._sdf_text(include, "uri")
            directory = self._resolve_sdf_uri(uri, base_dir)
            include_frame = self._compose_pose(
                frame, self._frame_from_pose(self._sdf_pose(include)))
            sdf_path = os.path.join(directory, "model.sdf") if directory else ""
            if sdf_path and os.path.isfile(sdf_path):
                try:
                    nested_root = ET.parse(sdf_path).getroot()
                except Exception:
                    counts["skipped"] += 1
                else:
                    for nested in self._sdf_children(nested_root, "model"):
                        self._load_sdf_model(
                            nested, include_frame, os.path.dirname(sdf_path),
                            counts, depth + 1)
            else:
                counts["skipped"] += 1

        for link in self._sdf_children(model, "link"):
            link_frame = self._compose_pose(
                frame, self._frame_from_pose(self._sdf_pose(link)))
            sources = (self._sdf_children(link, "collision")
                       or self._sdf_children(link, "visual"))
            for source in sources:
                geometry = self._sdf_child(source, "geometry")
                if geometry is None:
                    continue
                pose = self._compose_pose(
                    link_frame, self._frame_from_pose(self._sdf_pose(source)))
                if self._create_sdf_shape(geometry, pose, base_dir):
                    counts["created"] += 1
                else:
                    counts["skipped"] += 1

        for nested in self._sdf_children(model, "model"):
            self._load_sdf_model(nested, frame, base_dir, counts, depth + 1)

    def _stage_mesh(self, path):
        """Return a loadable mesh path, converting Collada when needed."""
        extension = os.path.splitext(path)[1].lower()
        if extension in (".obj", ".stl"):
            return path
        try:
            from robot_lab_utils.mesh_assets import (
                mesh_staging_dir, stage_mesh_file)
        except ImportError:
            self.get_logger().warn(
                "robot_lab_utils.mesh_assets unavailable; "
                "skipping mesh %s" % os.path.basename(path))
            return ""
        cache_dir = mesh_staging_dir("pybullet_world", os.path.dirname(path))
        staged = stage_mesh_file(path, cache_dir)
        if not staged:
            self.get_logger().warn(
                "could not convert mesh %s" % os.path.basename(path))
        return staged

    def _create_sdf_shape(self, geometry, pose, base_dir):
        """Create one static PyBullet body for an SDF <geometry>."""
        position, orientation = pose
        colour = [0.55, 0.57, 0.6, 1.0]
        collision = visual = -1

        box = self._sdf_child(geometry, "box")
        sphere = self._sdf_child(geometry, "sphere")
        cylinder = self._sdf_child(geometry, "cylinder")
        plane = self._sdf_child(geometry, "plane")
        mesh = self._sdf_child(geometry, "mesh")
        try:
            if box is not None:
                size = [float(v) for v in
                        (self._sdf_text(box, "size", "1 1 1")).split()]
                half = [max(v, 1e-6) / 2.0 for v in (size + [1.0, 1.0, 1.0])[:3]]
                collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
                visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half,
                                             rgbaColor=colour)
            elif sphere is not None:
                radius = float(self._sdf_text(sphere, "radius", "0.5"))
                collision = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
                visual = p.createVisualShape(p.GEOM_SPHERE, radius=radius,
                                             rgbaColor=colour)
            elif cylinder is not None:
                radius = float(self._sdf_text(cylinder, "radius", "0.5"))
                length = float(self._sdf_text(cylinder, "length", "1.0"))
                collision = p.createCollisionShape(
                    p.GEOM_CYLINDER, radius=radius, height=length)
                visual = p.createVisualShape(
                    p.GEOM_CYLINDER, radius=radius, length=length,
                    rgbaColor=colour)
            elif plane is not None:
                # The ground plane is already loaded; an SDF plane at a
                # non-zero pose is modelled as a thin box so its offset and
                # orientation are preserved.
                size = [float(v) for v in
                        (self._sdf_text(plane, "size", "100 100")).split()]
                half = [max(size[0], 1e-6) / 2.0,
                        max(size[1] if len(size) > 1 else size[0], 1e-6) / 2.0,
                        0.005]
                collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
                visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half,
                                             rgbaColor=colour)
            elif mesh is not None:
                uri = self._sdf_text(mesh, "uri")
                path = self._resolve_sdf_uri(uri, base_dir)
                if not path or not os.path.isfile(path):
                    return False
                # PyBullet cannot build a shape from Collada, which is the
                # only format the vendored Gazebo model library ships, so
                # meshes go through the shared staging/conversion cache.
                path = self._stage_mesh(path)
                if not path:
                    return False
                scale = [float(v) for v in
                         (self._sdf_text(mesh, "scale", "1 1 1")).split()]
                scale = (scale + [1.0, 1.0, 1.0])[:3]
                collision = p.createCollisionShape(
                    p.GEOM_MESH, fileName=path, meshScale=scale)
                visual = p.createVisualShape(
                    p.GEOM_MESH, fileName=path, meshScale=scale,
                    rgbaColor=colour)
            else:
                return False
        except Exception as exc:
            self.get_logger().warn(f"SDF shape load error: {exc}")
            return False

        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=position,
            baseOrientation=orientation,
        )
        return True

    def _on_cmd(self, msg):
        with self._twist_lock:
            self._twist = msg
            self._last_cmd_time = time.monotonic()

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
            self._timer = self.create_timer(
                2.0, self._try_spawn,
                clock=Clock(clock_type=ClockType.SYSTEM_TIME),
            )

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
        urdf = _absolutize_mesh_paths(
            urdf, os.path.dirname(os.path.abspath(str(model))))
        urdf = _strip_gazebo_tags(urdf)

        tmp = tempfile.NamedTemporaryFile(suffix=".urdf", delete=False, mode="w")
        tmp.write(urdf)
        tmp.close()
        self.get_logger().info(f"URDF written to {tmp.name}")

        # Determine GUI mode: handle both boolean and string values from launch
        gui_param = self.get_parameter("gui").value
        # Convert string "true"/"false" to boolean if needed
        if isinstance(gui_param, str):
            gui_param = gui_param.lower() in ("true", "1", "yes")
        display = os.environ.get("DISPLAY")
        gui = bool(gui_param and display)
        self.get_logger().info(f"PyBullet gui param={self.get_parameter('gui').value} (resolved={gui_param}), DISPLAY={display} -> mode={'GUI' if gui else 'DIRECT'}")
        self.get_logger().info("Connecting to PyBullet...")
        try:
            client_id = p.connect(p.GUI if gui else p.DIRECT)
            self.get_logger().info(f"PyBullet connected with client_id={client_id}")
        except Exception as e:
            self.get_logger().warn(f"PyBullet GUI connection failed: {e}. Falling back to DIRECT mode.")
            client_id = p.connect(p.DIRECT)
            self.get_logger().info(f"PyBullet connected in DIRECT mode with client_id={client_id}")
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self._dt)
        self.get_logger().info("PyBullet physics engine configured")
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
                    self._load_sdf_world(str(wp))
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

        # Signal readiness (R2.3).
        self._ready = True
        self._ready_pub.publish(Bool(data=True))

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

            # cmd_vel — apply watchdog: stop if no recent command
            with self._twist_lock:
                t = self._twist
                stale = (time.monotonic() - self._last_cmd_time) > self._watchdog_timeout
            if stale:
                t = Twist()
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

    def _on_reset(self, request, response):
        """Reset the simulation (R2.3)."""
        try:
            if self._robot_id >= 0 and p is not None:
                # Reset robot position to spawn point.
                sx = self.get_parameter("spawn_x").value
                sy = self.get_parameter("spawn_y").value
                sz = self.get_parameter("spawn_z").value
                syaw = self.get_parameter("spawn_yaw").value
                orn = _rpy_quaternion(0, 0, syaw)
                p.resetBasePositionAndOrientation(
                    self._robot_id, [sx, sy, sz], [orn.x, orn.y, orn.z, orn.w])
                # Reset velocity.
                p.resetBaseVelocity(self._robot_id, [0, 0, 0], [0, 0, 0])
                # Reset simulation time.
                self._sim_t = 0.0
                self._sim_step = 0
                self.get_logger().info("Simulation reset")
                response.success = True
                response.message = "Simulation reset successfully"
            else:
                response.success = False
                response.message = "No robot loaded"
        except Exception as e:
            response.success = False
            response.message = f"Reset failed: {e}"
            self.get_logger().error(f"Reset failed: {e}")
        return response

    def _publish_health(self):
        """Publish health status (R2.3)."""
        msg = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = "pybullet_spawner"
        status.hardware_id = "pybullet"

        if self._ready and self._robot_id >= 0:
            status.level = DiagnosticStatus.OK
            status.message = "running"
        elif self._robot_id < 0:
            status.level = DiagnosticStatus.WARN
            status.message = "no robot loaded"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = "not ready"

        status.values.append(KeyValue(key="ready", value=str(self._ready)))
        status.values.append(
            KeyValue(key="robot_id", value=str(self._robot_id)))
        status.values.append(
            KeyValue(key="sim_t", value=f"{self._sim_t:.3f}"))
        msg.status.append(status)
        self._health_pub.publish(msg)

    def _pub_clock(self):
        clock_msg = RosClock()
        clock_msg.clock = self._stamp()
        self._clock_pub.publish(clock_msg)

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
