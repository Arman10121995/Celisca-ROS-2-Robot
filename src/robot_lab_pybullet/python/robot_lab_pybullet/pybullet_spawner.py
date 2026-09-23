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
from rclpy.executors import ExternalShutdownException
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as RosClock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

import numpy as np

from robot_lab_utils import camera_model
from robot_lab_utils.camera_msgs import camera_info_msg, image_msg
from robot_lab_utils.sim_frames import rotate, world_to_body, wxyz_from_xyzw
from robot_lab_utils.urdf_contact import gazebo_link_friction

# Display-hold motor force for joints whose URDF declares no effort limit.
_HOLD_FORCE = 200.0

# Upper bound for friction taken from <gazebo><mu1>: descriptions use values
# like 1e15 to mean "no slip" in Gazebo's solver.
_MAX_LATERAL_FRICTION = 1.0


def _apply_link_friction(robot_id, link_indices, friction):
    """Apply per-link lateral friction; returns the links that were set.

    *link_indices* maps link names to PyBullet link indices; the base link
    (index -1) must be included by the caller if it is named in *friction*.
    """
    applied = []
    for link, mu in sorted(friction.items()):
        if link not in link_indices:
            continue
        p.changeDynamics(robot_id, link_indices[link],
                         lateralFriction=min(float(mu), _MAX_LATERAL_FRICTION))
        applied.append(link)
    return applied


def _render_rgbd(position, orientation_xyzw, camera, client=0):
    """Render RGB and metric depth from a camera link frame.

    The link looks along its +x axis with +z up, as the OAK-D link does.
    Returns ``(rgb, depth)``: uint8 HxWx3 and float32 HxW metres along the
    optical axis, ``inf`` where nothing is within the far clip.  The tiny
    software renderer needs no display or GPU.  ``client`` selects the
    PyBullet client to render from, so the camera can use a dedicated
    mirror world instead of stalling the physics client.
    """
    q = wxyz_from_xyzw(orientation_xyzw)
    eye = [float(v) for v in position]
    forward = rotate((1.0, 0.0, 0.0), q)
    up = rotate((0.0, 0.0, 1.0), q)
    width, height = int(camera["width"]), int(camera["height"])
    view = p.computeViewMatrix(
        eye, [e + f for e, f in zip(eye, forward)], list(up),
        physicsClientId=client)
    projection = p.computeProjectionMatrixFOV(
        fov=math.degrees(camera_model.vertical_fov(
            camera["horizontal_fov"], width, height)),
        aspect=float(width) / height,
        nearVal=camera["near"], farVal=camera["far"],
        physicsClientId=client)
    _, _, rgba, z_buffer, _ = p.getCameraImage(
        width, height, view, projection, renderer=p.ER_TINY_RENDERER,
        flags=p.ER_NO_SEGMENTATION_MASK, physicsClientId=client)
    rgb = np.asarray(rgba, dtype=np.uint8).reshape(height, width, 4)[:, :, :3]
    z_buffer = np.asarray(z_buffer, dtype=np.float64).reshape(height, width)
    depth = camera_model.linear_depth(
        z_buffer, camera["near"], camera["far"]).astype(np.float32)
    depth[z_buffer >= 1.0] = np.inf
    return rgb, depth


def _planar_scan(ray_batch, origin, yaw, samples, range_min, range_max,
                 robot_id, max_passes=4):
    """Ranges of a 360-degree planar scan; angles run from -pi about *yaw*.

    *ray_batch* is ``pybullet.rayTestBatch``, whose hits are
    ``(body, link, hit_fraction, hit_position, hit_normal)``.  The scan used
    to read ``hit_position`` as the fraction, so every ray came back ``inf``.
    Rays that hit the robot itself are cast again from just past that hit.
    """
    samples = max(int(samples), 1)
    increment = 2.0 * math.pi / samples
    directions = [(math.cos(yaw - math.pi + i * increment),
                   math.sin(yaw - math.pi + i * increment))
                  for i in range(samples)]
    ranges = [float("inf")] * samples
    starts = [0.0] * samples
    pending = list(range(samples))
    for _ in range(max_passes):
        if not pending:
            break
        froms = [[origin[0] + directions[i][0] * starts[i],
                  origin[1] + directions[i][1] * starts[i], origin[2]]
                 for i in pending]
        tos = [[origin[0] + directions[i][0] * range_max,
                origin[1] + directions[i][1] * range_max, origin[2]]
               for i in pending]
        retry = []
        for i, hit in zip(pending, ray_batch(froms, tos)):
            body, fraction = hit[0], float(hit[2])
            if body < 0:
                continue
            distance = starts[i] + fraction * (range_max - starts[i])
            if body == robot_id:
                starts[i] = distance + 1e-3
                if starts[i] < range_max:
                    retry.append(i)
                continue
            if distance < range_max:
                ranges[i] = max(float(range_min), distance)
        pending = retry
    return ranges

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


def _xyzw(orientation_wxyz):
    """Scalar-first (w, x, y, z) as the (x, y, z, w) PyBullet expects.

    The shared SDF reader is scalar-first; handing its quaternions to PyBullet
    unconverted would rotate every non-axis-aligned obstacle.
    """
    w, x, y, z = (float(v) for v in list(orientation_wxyz)[:4])
    return [x, y, z, w]


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
        # Display-mode hold: 'auto' lets the spawner hold joints only for the
        # map-free display case, 'true' always holds joints at their spawn
        # pose (passive visualization for legged/humanoid robots that would
        # otherwise collapse/jump under gravity), 'false' runs full physics.
        # Bringup forwards mode:=display automatically.
        self.declare_parameter("hold_position", "false")
        # RGB-D camera, rendered from the description's camera link with the
        # Gazebo sensor's intrinsics; 0 Hz disables it.
        self.declare_parameter("camera_rate", 5.0)
        self.declare_parameter("camera_link_name", camera_model.OAKD["link"])
        self.declare_parameter("camera_optical_frame",
                               camera_model.OAKD["optical_frame"])
        self.declare_parameter("camera_width", camera_model.OAKD["width"])
        self.declare_parameter("camera_height", camera_model.OAKD["height"])
        self.declare_parameter("camera_horizontal_fov",
                               camera_model.OAKD["horizontal_fov"])
        self.declare_parameter("camera_near", camera_model.OAKD["near"])
        self.declare_parameter("camera_far", camera_model.OAKD["far"])

        # State
        self._robot_id = -1
        self._link_idx = {}
        self._camera = None
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
        # Display-hold state (set in _spawn once joints are known).
        self._hold_joints = False
        self._hold_pose = {}
        self._hold_base = None

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
        # Wall-clock health timer - a default-clock timer would deadlock on
        # sim time because this node is itself the /clock publisher (same
        # contract and cadence as mujoco_spawner).
        self._health_timer = self.create_timer(
            1.0, self._publish_health,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME))

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
    def _load_sdf_world(self, world_path: str, client: int = 0) -> None:
        """Materialise an SDF world's static geometry in a PyBullet client.

        Parsing is the shared ``robot_lab_utils.sdf_world`` reader that the
        Isaac backend also uses (primitives, the full model -> link ->
        geometry pose chain, ``model://`` includes), so both backends build
        the same world from the same source instead of each keeping a copy.
        ``client`` targets the physics client (default) or the mirror
        render client, which must hold the same geometry for faithful
        RGB-D rendering (R8.1).
        """
        from ament_index_python.packages import get_package_share_directory
        try:
            from robot_lab_utils.sdf_world import (
                extract_static_shapes, uri_resolver)
        except ImportError as exc:
            self.get_logger().warn(
                f"World geometry helpers unavailable ({exc}); world not loaded")
            return
        model_dirs = []
        try:
            model_dirs.append(os.path.join(
                get_package_share_directory("robot_lab_models"), "models"))
        except Exception:
            pass
        try:
            shapes, skipped = extract_static_shapes(
                world_path,
                uri_resolver(model_dirs, get_package_share_directory))
        except Exception as exc:
            self.get_logger().warn(f"SDF world parse error: {exc}")
            return
        created = 0
        for shape in shapes:
            if self._create_shape(shape, client=client):
                created += 1
            else:
                skipped.append("%s shape not created" % shape.get("type"))
        self.get_logger().info(
            "SDF world '%s': %d static shape(s) created, %d skipped"
            % (os.path.basename(world_path), created, len(skipped)))

    def _stage_mesh(self, path):
        """Return a loadable mesh path, converted and face-capped if needed.

        STL/OBJ are natively loadable, but a map mesh can carry millions of
        facets: PyBullet builds a collision BVH *and* a renderer mesh from
        the file, which stalls the GUI for minutes on a furniture map.
        Oversized STLs are decimated to the world facet budget, small ones
        (and the other native formats) are used straight from the source.
        """
        extension = os.path.splitext(path)[1].lower()
        try:
            from robot_lab_utils.mesh_assets import (
                WORLD_MAX_STL_FACES, binary_stl_face_count, mesh_staging_dir,
                stage_mesh_file)
        except ImportError:
            self.get_logger().warn(
                "robot_lab_utils.mesh_assets unavailable; "
                "skipping mesh %s" % os.path.basename(path))
            return ""
        if extension in (".obj", ".msh"):
            return path
        if extension == ".stl" and os.path.isfile(path):
            faces = binary_stl_face_count(path)
            if 0 < faces <= WORLD_MAX_STL_FACES:
                return path
        cache_dir = mesh_staging_dir("pybullet_world", os.path.dirname(path))
        staged = stage_mesh_file(path, cache_dir, max_faces=WORLD_MAX_STL_FACES)
        if not staged:
            self.get_logger().warn(
                "could not convert mesh %s" % os.path.basename(path))
            return ""
        self.get_logger().info(
            "World mesh '%s': staged %d face(s) for PyBullet"
            % (os.path.basename(path), binary_stl_face_count(staged)))
        return staged

    def _create_shape(self, shape, client: int = 0):
        """Create one static PyBullet body from a shared shape record."""
        kind = shape.get("type")
        size = [float(v) for v in (shape.get("size") or [])]
        colour = [0.55, 0.57, 0.6, 1.0]
        try:
            if kind in ("box", "plane"):
                if kind == "box":
                    half = [max(v, 1e-6) / 2.0 for v in (size + [1.0] * 3)[:3]]
                else:  # a thin box keeps the plane's own pose and extent
                    full = (size + [100.0, 100.0])[:2]
                    half = [max(full[0], 1e-6) / 2.0,
                            max(full[1], 1e-6) / 2.0, 0.005]
                collision = p.createCollisionShape(
                    p.GEOM_BOX, halfExtents=half, physicsClientId=client)
                visual = p.createVisualShape(
                    p.GEOM_BOX, halfExtents=half, rgbaColor=colour,
                    physicsClientId=client)
            elif kind == "sphere":
                radius = (size + [0.5])[0]
                collision = p.createCollisionShape(
                    p.GEOM_SPHERE, radius=radius, physicsClientId=client)
                visual = p.createVisualShape(
                    p.GEOM_SPHERE, radius=radius, rgbaColor=colour,
                    physicsClientId=client)
            elif kind == "cylinder":
                radius, length = (size + [0.5, 1.0])[:2]
                collision = p.createCollisionShape(
                    p.GEOM_CYLINDER, radius=radius, height=length,
                    physicsClientId=client)
                visual = p.createVisualShape(
                    p.GEOM_CYLINDER, radius=radius, length=length,
                    rgbaColor=colour, physicsClientId=client)
            elif kind == "mesh":
                path = self._stage_mesh(shape.get("mesh", ""))
                if not path:
                    return False
                scale = [float(v) for v in
                         (shape.get("scale") or [1.0, 1.0, 1.0])[:3]]
                collision = p.createCollisionShape(
                    p.GEOM_MESH, fileName=path, meshScale=scale,
                    flags=p.GEOM_FORCE_CONCAVE_TRIMESH, physicsClientId=client)
                visual = p.createVisualShape(
                    p.GEOM_MESH, fileName=path, meshScale=scale,
                    rgbaColor=colour, physicsClientId=client)
            else:
                return False
        except Exception as exc:
            self.get_logger().warn(f"SDF shape load error: {exc}")
            return False
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=[float(v) for v in shape.get("position", [0, 0, 0])],
            baseOrientation=_xyzw(shape.get("orientation", [1, 0, 0, 0])),
            physicsClientId=client,
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
        # Resolve xacro; an explicitly empty model means robot-free display
        # (robot_model:=none): load only the world, no robot.  Treating that
        # as an error was the "map-only display does nothing" bug in the
        # non-Gazebo backends.
        model = self.get_parameter("model").value
        if not model:
            pkg = self.get_parameter("robot_package").value
            xacro = self.get_parameter("robot_xacro").value
            if pkg and xacro:
                model = os.path.join(get_package_share_directory(pkg), xacro)
        robot_free = (not model) or str(model).strip().lower() == "none"
        urdf = ""
        tmp = None
        link_friction = {}
        if not robot_free:
            if not os.path.isfile(str(model)):
                self.get_logger().error(f"URDF not found: {model}")
                return
            urdf = _xacro_to_urdf(str(model))
        if not robot_free:
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
            # Per-link friction lives in the <gazebo> blocks that are removed next.
            link_friction = gazebo_link_friction(urdf)
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

        if robot_free:
            self._ready = True
            self._thread = threading.Thread(target=self._world_loop, daemon=True)
            self._thread.start()
            self._ready_pub.publish(Bool(data=True))
            self.get_logger().info("PyBullet world-only display running")
            return

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
        # The URDF file is kept until the mirror render client (below) has
        # loaded its own copy, then removed.
        self._robot_urdf_path = tmp.name
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
        # URDF roots are not uniformly at sole level (dog roots are at the
        # trunk). Lift only an intersecting spawn, using actual collision
        # bounds, before freezing a display pose or taking the first step.
        min_z = min(p.getAABB(self._robot_id, link)[0][2]
                    for link in range(-1, p.getNumJoints(self._robot_id)))
        base_position, base_orientation = p.getBasePositionAndOrientation(self._robot_id)
        lift = max(0.0, .002 - min_z)
        if lift > 0:
            base_position = (base_position[0], base_position[1], base_position[2] + lift)
            p.resetBasePositionAndOrientation(self._robot_id, base_position, base_orientation)
            self.get_logger().info("Spawn ground clearance: raised root by %.4f m" % lift)
        self._spawn_base_pose = (base_position, base_orientation)
        friction_links = dict(self._link_idx)
        friction_links[p.getBodyInfo(self._robot_id)[0].decode()] = -1
        applied = _apply_link_friction(
            self._robot_id, friction_links, link_friction)
        if applied:
            self.get_logger().info(
                "Friction from the description's <gazebo> blocks: %s"
                % ", ".join("%s=%g" % (link, min(link_friction[link],
                                                 _MAX_LATERAL_FRICTION))
                            for link in applied))
        self._camera = self._camera_setup()
        # A dedicated DIRECT client renders the RGB-D camera: the tiny
        # software renderer costs hundreds of ms per frame on this host,
        # which would otherwise stall the physics loop below its 50 Hz
        # publish contract.  The mirror world holds the same plane + robot
        # and is posed from the physics loop's latest state snapshot before
        # each render, so the image is geometrically faithful.
        self._render_client = -1
        self._camera_thread = None
        self._cam_pose = None
        if self._camera is not None:
            try:
                self._render_client = p.connect(p.DIRECT)
                p.setAdditionalSearchPath(
                    pybullet_data.getDataPath(),
                    physicsClientId=self._render_client)
                p.loadURDF("plane.urdf", physicsClientId=self._render_client)
                self._render_robot_id = p.loadURDF(
                    self._robot_urdf_path, [sx, sy, sz],
                    [orn.x, orn.y, orn.z, orn.w],
                    useFixedBase=False,
                    flags=p.URDF_USE_INERTIA_FROM_FILE,
                    physicsClientId=self._render_client)
                self._render_joint_idx = {
                    p.getJointInfo(self._render_robot_id, i,
                                   physicsClientId=self._render_client)[1]
                    .decode(): i
                    for i in range(p.getNumJoints(
                        self._render_robot_id,
                        physicsClientId=self._render_client))}
                # Mirror the same static world geometry so RGB-D frames see
                # the environment, not just the ground plane.
                wp = self.get_parameter("world_path").value
                if wp and os.path.isfile(str(wp)):
                    ext = os.path.splitext(str(wp))[1].lower()
                    if ext in (".world", ".sdf"):
                        self._load_sdf_world(
                            str(wp), client=self._render_client)
                    elif ext in (".stl", ".obj"):
                        cid = p.createCollisionShape(
                            p.GEOM_MESH, fileName=str(wp),
                            physicsClientId=self._render_client)
                        vid = p.createVisualShape(
                            p.GEOM_MESH, fileName=str(wp),
                            rgbaColor=[0.6, 0.6, 0.6, 1.0],
                            physicsClientId=self._render_client)
                        p.createMultiBody(
                            baseCollisionShapeIndex=cid,
                            baseVisualShapeIndex=vid, baseMass=0,
                            physicsClientId=self._render_client)
                    else:
                        p.loadURDF(str(wp), useFixedBase=True,
                                   physicsClientId=self._render_client)
            except Exception as e:
                self.get_logger().warn(
                    f"Mirror render client unavailable ({e}); "
                    "RGB-D camera disabled")
                self._camera = None
                if self._render_client >= 0:
                    try:
                        p.disconnect(physicsClientId=self._render_client)
                    except Exception:
                        pass
                    self._render_client = -1
            finally:
                if os.path.exists(self._robot_urdf_path):
                    os.unlink(self._robot_urdf_path)
        lw = self.get_parameter("left_wheel_joint").value
        rw = self.get_parameter("right_wheel_joint").value
        self._lw = self._joint_idx.get(lw, -1)
        self._rw = self._joint_idx.get(rw, -1)

        # Display-mode hold: joints stay at their spawn pose so RViz reflects
        # stable joint_states instead of a collapse/jump under gravity.  This
        # restores the pre-regression display behaviour for legged/humanoid
        # robots (e.g. unitree_h1_2, berkeley_humanoid_lite) while wheeled
        # robots still drive on /cmd_vel.  'auto' holds only when the robot
        # has no drive joints configured (the map-free display case);
        # 'true' always holds; 'false' keeps full physics.
        hold_mode = str(self.get_parameter("hold_position").value or "auto").lower()
        has_drive = self._lw >= 0 or self._rw >= 0
        self._hold_joints = (hold_mode == "true") or (hold_mode == "auto" and not has_drive)
        if self._hold_joints:
            self._hold_pose = {}
            for jn in self._joint_names:
                index = self._joint_idx[jn]
                try:
                    angle = float(p.getJointState(self._robot_id, index)[0])
                except Exception:
                    angle = 0.0
                # URDF <limit effort>; unlimited or missing -> a generous cap.
                effort = float(p.getJointInfo(self._robot_id, index)[10])
                self._hold_pose[jn] = (angle, effort if effort > 0 else _HOLD_FORCE)
            self.get_logger().info(
                "Display hold active (hold_position=%s): %d joint(s) held at spawn pose by position motors"
                % (hold_mode, len(self._hold_pose)))

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
        # Start the RGB-D render thread (mirror client, decoupled from
        # physics so the software renderer cannot stall the publish rate).
        if self._camera is not None:
            self._camera_thread = threading.Thread(
                target=self._camera_loop, daemon=True)
            self._camera_thread.start()
        self.get_logger().info("PyBullet spawner running")

        # Signal readiness (R2.3).
        self._ready = True
        self._ready_pub.publish(Bool(data=True))

    def _world_loop(self):
        """Step a world without publishing fictitious robot state."""
        while self._running and rclpy.ok():
            start = time.monotonic()
            p.stepSimulation()
            self._sim_step += 1
            self._sim_t = self._sim_step * self._dt
            try:
                self._pub_clock()
            except Exception:
                if not rclpy.ok():
                    break
                raise
            time.sleep(max(0.0, self._dt - (time.monotonic() - start)))

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
            if getattr(self, "_hold_joints", False):
                # Display hold: every non-drive joint servos to its spawn
                # angle through the physics engine's position motor, within
                # the joint's own effort limit.  The base is free, so the
                # robot stands on the floor and still moves physically
                # (settling, sagging, pushes), and /joint_states reflects
                # that; pinning the base and forcing 200 N*m froze it.
                for jn, (q0, force) in getattr(self, "_hold_pose", {}).items():
                    idx = self._joint_idx.get(jn, -1)
                    if idx < 0 or idx == self._lw or idx == self._rw:
                        continue
                    try:
                        p.setJointMotorControl2(
                            self._robot_id, idx, p.POSITION_CONTROL,
                            targetPosition=q0, force=force)
                    except Exception:
                        pass
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
            # PyBullet reports world-frame velocities; the odometry twist and
            # the IMU rates are expressed in the robot's own frame, which is
            # how robot_localization fuses vx / vy / vyaw.
            base_q = wxyz_from_xyzw(orn)
            self._blin = list(world_to_body(lv, base_q))
            self._bang = list(world_to_body(av, base_q))
            self._jpos, self._jvel = [], []
            for jn in self._joint_names:
                s = p.getJointState(self._robot_id, self._joint_idx[jn])
                self._jpos.append(s[0])
                self._jvel.append(s[1])
            if self._camera is not None and self._camera.get("link", -1) >= 0:
                # Snapshot the camera link pose for the render thread; the
                # tiny renderer itself runs on the mirror client (R8.1).
                st = p.getLinkState(self._robot_id, self._camera["link"],
                                    computeForwardKinematics=True)
                self._cam_pose = (list(st[4]), list(st[5]))

            try:
                # A reset rewinds sim time; re-arm the publish gates so the
                # reset stamp flows immediately (R2.3 reset contract).
                last_pub = min(last_pub, self._sim_t)
                last_scan = min(last_scan, self._sim_t)
                # Publish gates use sim time so the stamped 50 Hz odom /
                # 5 Hz scan contracts hold in sim seconds regardless of the
                # wall real-time factor.
                if self._sim_t - last_pub >= pub_dt:
                    last_pub = self._sim_t
                    self._pub_joint_states()
                    self._pub_odom()
                    self._pub_imu()
                    self._pub_clock()
                if self._sim_t - last_scan >= scan_dt:
                    last_scan = self._sim_t
                    self._pub_scan()
            except Exception:
                # Launch's SIGINT shuts the ROS context down before
                # destroy_node() stops this thread; publishing then raises.
                if not rclpy.ok():
                    break
                raise

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
                pose = getattr(self, "_spawn_base_pose", ([sx, sy, sz], [orn.x, orn.y, orn.z, orn.w]))
                p.resetBasePositionAndOrientation(self._robot_id, *pose)
                # Reset velocity.
                p.resetBaseVelocity(self._robot_id, [0, 0, 0], [0, 0, 0])
                # Zero actuated joint motion so the robot does not coast
                # away from the spawn pose on pre-reset wheel spin (R6.1
                # reset contract: velocities restored, not just the pose).
                for i in range(p.getNumJoints(self._robot_id)):
                    if p.getJointInfo(self._robot_id, i)[2] != p.JOINT_FIXED:
                        p.resetJointState(self._robot_id, i, 0.0, 0.0)
                # A still-fresh cmd_vel would re-accelerate the wheels on
                # the next physics tick and drag the robot out of its reset
                # pose, so the pending command is cleared as well.
                with self._twist_lock:
                    self._twist = Twist()
                    self._last_cmd_time = 0.0
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

        if self._ready:
            status.level = DiagnosticStatus.OK
            status.message = "running" if self._robot_id >= 0 else "world-only display"
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
            # Indices 4/5 are the URDF link frame; 0/1 are the link's centre
            # of mass, which for Bumperbot's laser is 1.2 cm off the sensor.
            st = p.getLinkState(self._robot_id, li,
                                computeForwardKinematics=True)
            lp = list(st[4])
            lo = list(st[5])
        else:
            lp = [self._bpos[0], self._bpos[1], self._bpos[2] + 0.12]
            lo = self._born
        siny = 2.0*(lo[3]*lo[2] + lo[0]*lo[1])
        cosy = 1.0 - 2.0*(lo[1]**2 + lo[2]**2)
        byaw = math.atan2(siny, cosy)
        n = int(self.get_parameter("scan_samples").value)
        am = self.get_parameter("scan_range_min").value
        ax = self.get_parameter("scan_range_max").value
        ai = 2.0 * math.pi / n
        ranges = _planar_scan(p.rayTestBatch, lp, byaw, n, am, ax,
                              self._robot_id)
        msg = LaserScan()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = ln
        msg.angle_min = -math.pi
        # n samples span n - 1 increments; angle_max = pi claimed n + 1.
        msg.angle_max = -math.pi + (n - 1) * ai
        msg.angle_increment = ai
        msg.scan_time = 1.0 / max(self.get_parameter("scan_rate").value, 0.1)
        msg.range_min = am
        msg.range_max = ax
        msg.ranges = ranges
        self._scan_pub.publish(msg)

    def _camera_setup(self):
        """Camera settings and publishers, or None when not applicable."""
        rate = float(self.get_parameter("camera_rate").value)
        link = self.get_parameter("camera_link_name").value
        if rate <= 0.0:
            return None
        if link not in self._link_idx:
            self.get_logger().info(
                "Robot has no '%s' link; no RGB-D camera is published." % link)
            return None
        camera = {
            "link": self._link_idx[link],
            "rate": rate,
            "frame": self.get_parameter("camera_optical_frame").value,
            "width": int(self.get_parameter("camera_width").value),
            "height": int(self.get_parameter("camera_height").value),
            "horizontal_fov": float(
                self.get_parameter("camera_horizontal_fov").value),
            "near": float(self.get_parameter("camera_near").value),
            "far": float(self.get_parameter("camera_far").value),
        }
        self._rgb_pub = self.create_publisher(
            Image, camera_model.OAKD["rgb_topic"], 5)
        self._depth_pub = self.create_publisher(
            Image, camera_model.OAKD["depth_topic"], 5)
        self._camera_info_pub = self.create_publisher(
            CameraInfo, camera_model.OAKD["info_topic"], 5)
        self.get_logger().info(
            "RGB-D camera on '%s': %dx%d at %.1f Hz (software renderer)"
            % (link, camera["width"], camera["height"], rate))
        return camera

    def _camera_loop(self):
        """Render and publish RGB-D from the mirror client (R8.1).

        Runs on its own thread: the tiny software renderer takes hundreds
        of ms per frame on this host, so rendering inside the physics loop
        collapsed the publish rate.  Each frame mirrors the latest physics
        state snapshot into the dedicated render client, so the image is
        geometrically faithful to the simulated world.  The publish gate
        uses sim time so the stamped camera_rate contract holds in sim
        seconds regardless of render duration.
        """
        camera = self._camera
        camera_dt = 1.0 / max(camera["rate"], 0.1)
        last_sim = -camera_dt
        while self._running and rclpy.ok():
            # A reset rewinds sim time; re-arm so the first post-reset frame
            # flows immediately.
            last_sim = min(last_sim, self._sim_t)
            if (self._cam_pose is None or self._render_client < 0
                    or self._sim_t - last_sim < camera_dt):
                time.sleep(0.002)
                continue
            last_sim = self._sim_t
            try:
                # Stamp at the snapshot moment: the image reflects this sim
                # time, and consecutive stamps then honour the camera_rate
                # interval exactly regardless of render duration.
                stamp = self._stamp()
                p.resetBasePositionAndOrientation(
                    self._render_robot_id, self._bpos, self._born,
                    physicsClientId=self._render_client)
                for jn, jp, jv in zip(self._joint_names, self._jpos,
                                      self._jvel):
                    ji = self._render_joint_idx.get(jn)
                    if ji is not None:
                        p.resetJointState(self._render_robot_id, ji, jp, jv,
                                          physicsClientId=self._render_client)
                rgb, depth = _render_rgbd(
                    self._cam_pose[0], self._cam_pose[1], camera,
                    client=self._render_client)
                self._rgb_pub.publish(
                    image_msg(stamp, camera["frame"], rgb, "rgb8"))
                self._depth_pub.publish(
                    image_msg(stamp, camera["frame"], depth, "32FC1"))
                self._camera_info_pub.publish(camera_info_msg(
                    stamp, camera["frame"], camera["width"], camera["height"],
                    camera["horizontal_fov"]))
            except Exception:
                # Shutting down: the physics client may already be gone.
                if not rclpy.ok() or not self._running:
                    break
                raise

    def destroy_node(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if getattr(self, "_camera_thread", None) and self._camera_thread.is_alive():
            self._camera_thread.join(timeout=2.0)
        if p is not None and getattr(self, "_render_client", -1) >= 0:
            try:
                p.disconnect(physicsClientId=self._render_client)
            except Exception:
                pass
        if p is not None and p.isConnected():
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
