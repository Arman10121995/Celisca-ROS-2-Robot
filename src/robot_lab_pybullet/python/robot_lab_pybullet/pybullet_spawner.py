"""PyBullet robot spawner node.

Opens a PyBullet GUI window, loads the robot URDF, runs the physics
step loop, and publishes all ROS 2 topics required by the rest of the
stack (joint_states, TF, odom, scan, imu, clock).
"""
import json
import hashlib
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

from robot_lab_utils.drive_kinematics import drive_from_config, parse_drive_config
from robot_lab_utils.process_lifetime import exit_with_parent
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as RosClock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

import numpy as np

from robot_lab_utils import camera_model
from robot_lab_utils.camera_msgs import camera_info_msg, image_msg
from robot_lab_utils.sim_frames import rotate, urdf_link_frames, world_to_body, wxyz_from_xyzw
from robot_lab_utils.urdf_contact import gazebo_link_friction
from robot_lab_utils.urdf_joints import driven_assembly_joints
from robot_lab_utils.ros_frames import publish_fallback_scan_frame

# Display-hold motor force for joints whose URDF declares no effort limit.
_HOLD_FORCE = 200.0

# Upper bound for friction taken from <gazebo><mu1>: descriptions use values
# like 1e15 to mean "no slip" in Gazebo's solver.
_MAX_LATERAL_FRICTION = 1.0
# Declared static-surface friction at or above this keeps Bullet's default (see _create_shape).
_STATIC_DEFAULT_FRICTION_ABOVE = 1.0
# Loose world objects are drawn in this colour, in the physics client and in
# the camera's mirror client alike.
DYNAMIC_BODY_COLOUR = [0.85, 0.2, 0.15, 1.0]


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

    Robot camera links look along +X with +Z up. A workcell optical frame
    looks along +Z with -Y up.
    Returns ``(rgb, depth)``: uint8 HxWx3 and float32 HxW metres along the
    optical axis, ``inf`` where nothing is within the far clip.  The tiny
    software renderer needs no display or GPU.  ``client`` selects the
    PyBullet client to render from, so the camera can use a dedicated
    mirror world instead of stalling the physics client.
    """
    q = wxyz_from_xyzw(orientation_xyzw)
    eye = [float(v) for v in position]
    axes = ((0., 0., 1.), (0., -1., 0.)) if camera.get('workcell_pose') else ((1., 0., 0.), (0., 0., 1.))
    forward, up = (rotate(axis, q) for axis in axes)
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


# PyBullet applies URDF joint damping as an explicit force, stable only
# while damping * dt / I stays below about 1 (I: the joint's effective
# inertia).  Berkeley Humanoid Lite's elbow roll links weigh 0.8 g with
# 1e-8 kg*m^2 of inertia under damping 5, so its arms diverged on the first
# steps and the whole robot was thrown across the world at up to 110 m/s.
_DAMPING_STABILITY = 0.5


def _cap_joint_damping(robot_id, dt):
    """Lower each joint's damping to its stability bound; returns the joints.

    A joint's effective inertia is 1 / (M^-1)_ii, the inertia it shows when
    every other joint and the floating base are free to move (M: the
    joint-space mass matrix at the current pose, base rows first).  The
    diagonal M_ii overstates it for a light link on a heavy chain, and a
    bound built from it still let the humanoid's arms diverge.
    """
    movable = [i for i in range(p.getNumJoints(robot_id))
               if p.getJointInfo(robot_id, i)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)]
    if not movable:
        return []
    positions = [p.getJointState(robot_id, i)[0] for i in movable]
    try:
        matrix = p.calculateMassMatrix(robot_id, positions)
    except Exception:
        return []
    offset = len(matrix) - len(movable)
    try:
        inverse = np.linalg.inv(np.asarray(matrix, dtype=float))
    except np.linalg.LinAlgError:
        return []
    capped = []
    for row, joint in enumerate(movable):
        info = p.getJointInfo(robot_id, joint)
        damping = float(info[6])
        inertia = 1.0 / max(float(inverse[offset + row][offset + row]), 1e-12)
        limit = _DAMPING_STABILITY * inertia / dt
        if damping > limit:
            p.changeDynamics(robot_id, joint, jointDamping=limit)
            capped.append(info[1].decode())
    return capped


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


def _is_water_world(world_path):
    """True for an SDF world declaring a Buoyancy system (it has its own seabed)."""
    if not world_path or os.path.splitext(str(world_path))[1].lower() not in (".world", ".sdf") \
            or not os.path.isfile(str(world_path)):
        return False
    import xml.etree.ElementTree as ET
    try:
        world = ET.parse(str(world_path)).getroot().find("world")
    except ET.ParseError:
        return False
    return world is not None and any(plugin.get("name", "").endswith("::Buoyancy")
                                     for plugin in world.findall("plugin"))


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
        # The robot's whole drive block as JSON (drive_kinematics); empty
        # for the differential-drive parameters above.
        self.declare_parameter("drive_config", "")
        self.declare_parameter("articulation_config", "")
        self.declare_parameter('joint_pd_config','')
        self.declare_parameter('gait_probe_config','')
        self.declare_parameter('gait_contact_config','')
        self.declare_parameter('gait_floor_reference','')
        # Tracks driven through their contacts (track_plant.TrackPlant, JSON
        # robot_lab_utils.track_drive block); empty for every other robot.
        self.declare_parameter('track_config','')
        # The amphibious rover's wheels, springs, propellers and water
        # (amphibious_plant.AmphibiousPlant, JSON {surface_z, density}).
        self.declare_parameter('amphibious_config','')
        self._amphibious=None
        self._gait_plant=None
        self._tracks=None
        self.get_logger().info("Parameters declared")
        self.declare_parameter("gui", True)
        self.declare_parameter("physics_rate", 240.0)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("scan_rate", 5.0)
        self.declare_parameter("laser_link_name", "laser_link")
        self.declare_parameter("imu_link_name", "imu_link")
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
        self.declare_parameter('workcell_rgbd_pose', '')
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
        self._base_frame = "base_footprint"
        # Root link frame in PyBullet's base (inertial) frame; see _base_link_pose.
        self._base_inertial_offset = None
        self._joint_idx = {}
        self._joint_names = []
        self._articulation = None
        self._free_bodies = {}
        self._free_bodies_pub = None
        self._free_bodies_t = -1.0
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
        self._driven_assembly_joints = set()
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
            from robot_lab_utils.reset_notifications import ResetNotifications
            self._reset_notifications = ResetNotifications(self)
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

        self._last_mux_cmd_time = 0.0
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Twist, "/robot_lab_controller/cmd_vel_unstamped",
                                 self._on_mux_cmd, 10)
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
                extract_dynamic_bodies, extract_static_shapes, uri_resolver)
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
            # Loose primitive objects are free bodies of the physics client;
            # the mirror render client draws them from the physics state
            # (_mirror_free_bodies). The mirror takes visual geometry and its
            # SDF colours: what a camera sees.
            shapes, skipped = extract_static_shapes(
                world_path,
                uri_resolver(model_dirs, get_package_share_directory),
                prefer_visual=client != 0, exclude_dynamic=True)
        except Exception as exc:
            self.get_logger().warn(f"SDF world parse error: {exc}")
            return
        # Models drawn from their textured Collada visuals keep their
        # collision shapes for contacts and rays, without drawing them
        # (R7.2, 2026-10-16; the cameras saw one grey before).
        visuals = []
        try:
            visuals, _ = extract_static_shapes(
                world_path, uri_resolver(model_dirs, get_package_share_directory),
                prefer_visual=True)
        except Exception as exc:
            self.get_logger().warn(f"SDF visual parse error: {exc}")
        visuals = [shape for shape in visuals if shape.get("element") == "visual"
                   and shape["type"] == "mesh" and shape["mesh"].lower().endswith(".dae")]
        drawn = self._create_textured_visuals(visuals, client)
        # In a water world (a Buoyancy system) a model without collision is
        # the water surface or decoration, never a solid lid, as
        # gen_mjcf_worlds.py makes it non-colliding for MuJoCo. The amphibious
        # rover drove on the shore world's water plane until this
        # (2026-11-21). The mirror client still draws it.
        if client == 0:
            import xml.etree.ElementTree as ET
            source = ET.parse(world_path).getroot().find('world')
            if source is not None and any(plugin.get('name', '').endswith('::Buoyancy')
                                          for plugin in source.findall('plugin')):
                visual_only = {model.get('name') for model in source.findall('model')
                               if model.find('.//collision') is None}
                shapes = [shape for shape in shapes if shape.get('model') not in visual_only]
        created = 0
        for shape in shapes:
            if self._create_shape(shape, client=client, hidden=shape.get("model") in drawn):
                created += 1
            else:
                skipped.append("%s shape not created" % shape.get("type"))
        for body in extract_dynamic_bodies(world_path):
            if client == 0:
                self._create_dynamic_body(body)
            else:
                self._create_mirror_body(body, client)
        self.get_logger().info(
            "SDF world '%s': %d static shape(s) created, %d skipped, %d model(s) "
            "drawn from textured visuals, %d free body(ies)"
            % (os.path.basename(world_path), created, len(skipped), len(drawn),
               len(self._free_bodies) if client == 0 else 0))

    def _create_dynamic_body(self, body):
        """Build one physical object with all its colliders in the model frame."""
        try:
            collision = self._free_body_shape(body["shapes"], visual=False, client=0)
            visual = self._free_body_shape(body["shapes"], visual=True, client=0)
            if min(collision, visual) < 0:
                raise ValueError("could not create both collision and visual shapes")
            uid = p.createMultiBody(baseMass=float(body["mass"]), baseCollisionShapeIndex=collision,
                                    baseVisualShapeIndex=visual, basePosition=body["position"],
                                    baseOrientation=_xyzw(body["orientation"]))
            dynamics = {}
            if body.get("inertia"):
                dynamics["localInertiaDiagonal"] = body["inertia"]
            if body.get("mu") is not None:
                dynamics["lateralFriction"] = float(body["mu"])
            # Bullet friction anchors stop slow sliding creep: a block held
            # in TIAGo's grip yawed 0.5 rad as the base turned (2026-11-17).
            dynamics["frictionAnchor"] = True
            p.changeDynamics(uid, -1, **dynamics)
        except Exception as exc:
            self.get_logger().warn("Free body %s not created: %s" % (body["name"], exc))
            return
        self._free_bodies[body["name"]] = uid
        if self._free_bodies_pub is None:
            # Truth for loose objects, as the MuJoCo bridge publishes it:
            # JSON {name: [x, y, z, qw, qx, qy, qz]} at 10 Hz. Reports only.
            self._free_bodies_pub = self.create_publisher(String, "/simulation/free_bodies", 10)

    def _create_mirror_body(self, body, client):
        """A visual-only copy of a loose object in the mirror render client."""
        try:
            visual = self._free_body_shape(body["shapes"], visual=True, client=client)
            if visual < 0:
                raise ValueError("could not create visual shape")
            self._mirror_free_bodies[body["name"]] = p.createMultiBody(
                baseMass=0.0, baseVisualShapeIndex=visual, basePosition=body["position"],
                baseOrientation=_xyzw(body["orientation"]), physicsClientId=client)
        except Exception as exc:
            self.get_logger().warn("Free body %s not mirrored for the camera: %s" % (body["name"], exc))

    @staticmethod
    def _free_body_shape(shapes, *, visual, client):
        shape_types = {"box": p.GEOM_BOX, "sphere": p.GEOM_SPHERE, "cylinder": p.GEOM_CYLINDER}
        frame = "visualFrame" if visual else "collisionFrame"
        if len(shapes) == 1:
            shape, = shapes
            size = shape["size"]
            dims = {"halfExtents": [max(v, 1e-6) / 2.0 for v in size]} if shape["type"] == "box" else {
                "radius": size[0]}
            if shape["type"] == "cylinder":
                dims["length" if visual else "height"] = size[1]
            dims[frame + "Position"] = shape["position"]
            if shape["type"] != "sphere":
                dims[frame + "Orientation"] = _xyzw(shape["orientation"])
            if visual:
                return p.createVisualShape(shape_types[shape["type"]], rgbaColor=DYNAMIC_BODY_COLOUR,
                                           physicsClientId=client, **dims)
            return p.createCollisionShape(shape_types[shape["type"]], physicsClientId=client, **dims)
        params = dict(shapeTypes=[shape_types[s["type"]] for s in shapes],
                      halfExtents=[[v / 2.0 for v in s["size"]] if s["type"] == "box" else [0., 0., 0.]
                                   for s in shapes],
                      radii=[s["size"][0] if s["type"] != "box" else 0.0 for s in shapes],
                      lengths=[s["size"][1] if s["type"] == "cylinder" else 0.0 for s in shapes],
                      physicsClientId=client)
        params[frame + "Positions"] = [s["position"] for s in shapes]
        params[frame + "Orientations"] = [_xyzw(s["orientation"]) for s in shapes]
        if visual:
            return p.createVisualShapeArray(rgbaColors=[DYNAMIC_BODY_COLOUR] * len(shapes), **params)
        return p.createCollisionShapeArray(**params)

    def _pub_free_bodies(self):
        poses = {}
        for name, uid in self._free_bodies.items():
            position, orientation = p.getBasePositionAndOrientation(uid)
            poses[name] = [float(v) for v in position] + [float(orientation[3]), float(orientation[0]),
                                                          float(orientation[1]), float(orientation[2])]
        self._free_bodies_pub.publish(String(data=json.dumps(poses)))

    def _create_textured_visuals(self, visuals, client=0):
        """Visual-only bodies for textured Collada visuals; returns the models drawn."""
        try:
            from robot_lab_utils.textured_mesh import stage_textured_parts
        except ImportError:
            return set()
        drawn, textures = set(), {}
        for shape in visuals:
            try:
                parts = stage_textured_parts(shape["mesh"], shape.get("scale") or (1.0, 1.0, 1.0))
            except Exception as exc:
                self.get_logger().warn("Textured mesh %s not staged: %s" % (shape["mesh"], exc))
                continue
            for part in parts:
                visual = p.createVisualShape(
                    p.GEOM_MESH, fileName=part["obj"], meshScale=[1.0, 1.0, 1.0],
                    rgbaColor=part["rgba"], physicsClientId=client)
                body = p.createMultiBody(
                    baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=visual,
                    basePosition=[float(v) for v in shape.get("position", [0, 0, 0])],
                    baseOrientation=_xyzw(shape.get("orientation", [1, 0, 0, 0])),
                    physicsClientId=client)
                if part["texture"]:
                    if part["texture"] not in textures:
                        textures[part["texture"]] = p.loadTexture(part["texture"], physicsClientId=client)
                    p.changeVisualShape(body, -1, textureUniqueId=textures[part["texture"]],
                                        physicsClientId=client)
            if parts:
                drawn.add(shape["model"])
        return drawn

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
                WORLD_MAX_STL_FACES, binary_stl_face_count, stage_world_mesh)
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
        # Simplified as connected surfaces (the MuJoCo path's staging).
        # Capping kept every Nth facet, so a 3.9M-facet Celisca furniture
        # map showed walls with most of their triangles missing.
        try:
            staged = stage_world_mesh(path)
        except Exception as exc:
            self.get_logger().warn("world mesh %s: %s" % (os.path.basename(path), exc))
            staged = ""
        if not staged:
            self.get_logger().warn(
                "could not convert mesh %s" % os.path.basename(path))
            return ""
        self.get_logger().info(
            "World mesh '%s': staged %d face(s) for PyBullet"
            % (os.path.basename(path), binary_stl_face_count(staged)))
        return staged

    def _stage_heightfield(self, shape):
        """Convert an SDF heightmap to a cached OBJ PyBullet can collide with.

        PyBullet has no heightfield primitive, so the shared converter builds
        the triangle surface in metres - the same layout measured against
        Gazebo - and the OBJ is cached by content hash so repeated launches
        do not re-tessellate the terrain.
        """
        try:
            from robot_lab_utils.heightfield import (
                heightfield_grid, write_obj)
        except ImportError as exc:
            self.get_logger().warn(
                "robot_lab_utils.heightfield unavailable; skipping heightmap "
                "(%s)" % exc)
            return ""
        try:
            vertices, faces = heightfield_grid(shape)
        except Exception as exc:
            self.get_logger().warn(
                "heightmap %s: %s" % (os.path.basename(shape.get("heightmap", "")),
                                      exc))
            return ""
        cache = os.path.join(
            os.environ.get("ROBOT_LAB_RUNTIME_ROOT", tempfile.gettempdir()),
            "heightfields")
        os.makedirs(cache, exist_ok=True)
        digest = hashlib.sha1(
            ("%s|%s" % (os.path.abspath(shape["heightmap"]), shape["size"]))
            .encode("utf-8")).hexdigest()[:12]
        target = os.path.join(cache, "terrain-%s.obj" % digest)
        if not os.path.isfile(target):
            write_obj(vertices, faces, target)
        self.get_logger().info(
            "Heightfield '%s': %d triangle(s) staged for PyBullet"
            % (os.path.basename(shape["heightmap"]), len(faces)))
        return target

    def _create_shape(self, shape, client: int = 0, hidden=False):
        """Create one static PyBullet body from a shared shape record.

        ``hidden``: collide and reflect rays, but draw nothing (the model is
        drawn from its textured visuals)."""
        kind = shape.get("type")
        size = [float(v) for v in (shape.get("size") or [])]
        # SDF material colour where given (visual shapes of the mirror).
        colour = list(shape.get("rgba") or [0.55, 0.57, 0.6, 1.0])
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
            elif kind == "heightmap":
                path = self._stage_heightfield(shape)
                if not path:
                    return False
                # Concave trimesh: terrain is a heightfield surface, not a
                # convex hull, so it needs the trim flag.
                collision = p.createCollisionShape(
                    p.GEOM_MESH, fileName=path, meshScale=[1.0, 1.0, 1.0],
                    flags=p.GEOM_FORCE_CONCAVE_TRIMESH, physicsClientId=client)
                visual = p.createVisualShape(
                    p.GEOM_MESH, fileName=path, meshScale=[1.0, 1.0, 1.0],
                    rgbaColor=colour, physicsClientId=client)
            else:
                return False
        except Exception as exc:
            self.get_logger().warn(f"SDF shape load error: {exc}")
            return False
        uid = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=-1 if hidden else visual,
            basePosition=[float(v) for v in shape.get("position", [0, 0, 0])],
            baseOrientation=_xyzw(shape.get("orientation", [1, 0, 0, 0])),
            physicsClientId=client,
        )
        if shape.get("mu") is not None and float(shape["mu"]) < _STATIC_DEFAULT_FRICTION_ABOVE:
            # A declared low-friction surface (a passive station: mu 0.05).
            # Bullet multiplies the two coefficients where ODE takes the
            # smaller; below 1 the product stays close to it. Higher values
            # (grounds declare mu 100) keep Bullet's default 0.5: a product
            # with 100 would make every wheel grip a hundredfold. Before
            # 2026-11-17 no static friction was applied, and the MOLAR payload
            # stalled on its station at 0.5 x 0.8.
            p.changeDynamics(uid, -1, lateralFriction=float(shape["mu"]), physicsClientId=client)
        return True

    def _on_cmd(self, msg):
        # In interactive modes twist_mux arbitrates GUI/joystick commands
        # against Nav2's /cmd_vel.  Keep raw /cmd_vel usable when running the
        # spawner alone (for example the drive checks and display mode).
        if time.monotonic() - getattr(self, "_last_mux_cmd_time", 0.0) < 1.0:
            return
        with self._twist_lock:
            if self._articulation is not None and self._articulation.base_inhibited:
                self._twist = Twist()
                self._last_cmd_time = -math.inf
                return
            self._twist = msg
            self._last_cmd_time = time.monotonic()

    def _on_mux_cmd(self, msg):
        self._last_mux_cmd_time = time.monotonic()
        with self._twist_lock:
            if self._articulation is not None and self._articulation.base_inhibited:
                self._twist = Twist()
                self._last_cmd_time = -math.inf
                return
            self._twist = msg
            self._last_cmd_time = self._last_mux_cmd_time

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
        native_config = parse_drive_config(self.get_parameter('drive_config').value)
        source_springs = None
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
            self._base_frame = urdf_link_frames(urdf)[1] or "base_footprint"
            if native_config.get('pybullet_preserve_passive_springs', False):
                from robot_lab_utils.urdf_springs import joint_springs
                source_springs = joint_springs(urdf)
            if native_config.get('pybullet_explicit_massless_frames', False):
                from robot_lab_utils.urdf_inertia import bullet_frame_inertials
                urdf = bullet_frame_inertials(urdf)
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

        # Ground, except in a water world (a Buoyancy system), which has its
        # own seabed: the default plane at z 0 was a solid lid on the shore
        # world's water surface (the amphibious rover drove on it, 2026-11-21).
        self._water_world = _is_water_world(self.get_parameter("world_path").value)
        if not self._water_world:
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
        if 'pybullet_expected_mass_kg' in native_config:
            mass = sum(p.getDynamicsInfo(self._robot_id, index)[0]
                       for index in range(-1, p.getNumJoints(self._robot_id)))
            expected = float(native_config['pybullet_expected_mass_kg'])
            if not math.isfinite(mass) or not math.isclose(mass, expected, rel_tol=1e-3, abs_tol=1e-3):
                raise ValueError(f'Imported robot mass {mass} kg differs from source {expected} kg')
            self.get_logger().info(f'Imported source robot mass: {mass:.9f} kg (expected {expected:.9f})')

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
        from robot_lab_utils.urdf_base import fixed_to_world
        if lift > 0 and fixed_to_world(urdf):
            # A fixed-base arm stands where its description puts it (its
            # geometry-free 'world' root has no meaningful clearance).
            lift = 0.0
        if lift > 0:
            base_position = (base_position[0], base_position[1], base_position[2] + lift)
            p.resetBasePositionAndOrientation(self._robot_id, base_position, base_orientation)
            self.get_logger().info("Spawn ground clearance: raised root by %.4f m" % lift)
        self._spawn_base_pose = (base_position, base_orientation)
        capped = _cap_joint_damping(self._robot_id, self._dt)
        if capped:
            self.get_logger().info(
                "Joint damping capped for stability on %d joint(s): %s"
                % (len(capped), ", ".join(capped[:6])))
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
        if self.get_parameter("laser_link_name").value not in self._link_idx:
            # The scan is cast from above the root link; give it that frame.
            self._scan_frame_tf = publish_fallback_scan_frame(
                self, self._base_frame, self.get_parameter("laser_link_name").value)
        # A dedicated DIRECT client renders the RGB-D camera: the tiny
        # software renderer costs hundreds of ms per frame on this host,
        # which would otherwise stall the physics loop below its 50 Hz
        # publish contract.  The mirror world holds the same plane + robot
        # and is posed from the physics loop's latest state snapshot before
        # each render, so the image is geometrically faithful.
        self._render_client = -1
        self._camera_thread = None
        self._cam_pose = self._camera.get('workcell_pose') if self._camera else None
        self._mirror_free_bodies, self._free_poses = {}, {}
        if self._camera is not None:
            try:
                self._render_client = p.connect(p.DIRECT)
                p.setAdditionalSearchPath(
                    pybullet_data.getDataPath(),
                    physicsClientId=self._render_client)
                if not getattr(self, "_water_world", False):
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
        self._drive = self._make_drive()
        from robot_lab_utils.skid_steer import wheel_force_limit, wheel_friction_axes, bullet_contact_parameters
        wheel_config = parse_drive_config(self.get_parameter('drive_config').value)
        self._wheel_force_limit = wheel_force_limit(
            wheel_config)
        contact_parameters = bullet_contact_parameters(wheel_config)
        if contact_parameters:
            p.setPhysicsEngineParameter(**contact_parameters)
        friction_axes = wheel_friction_axes(wheel_config)
        if friction_axes is not None:
            for wheel in self._drive.wheel_joints:
                p.changeDynamics(self._robot_id,self._joint_idx[wheel],
                                 anisotropicFriction=friction_axes)
        # Every joint the drive commands (a car's four wheels and steering).
        self._drive_idx = {self._joint_idx[j] for j in
                           list(self._drive.wheel_joints) + list(self._drive.steer_joints)
                           if j in self._joint_idx}

        # Display-mode hold: joints stay at their spawn pose so RViz reflects
        # stable joint_states instead of a collapse/jump under gravity.  This
        # restores the pre-regression display behaviour for legged/humanoid
        # robots (e.g. unitree_h1_2, berkeley_humanoid_lite) while wheeled
        # robots still drive on /cmd_vel.  'auto' holds only when the robot
        # has no drive joints configured (the map-free display case);
        # 'true' always holds; 'false' keeps full physics.
        hold_mode = str(self.get_parameter("hold_position").value or "auto").lower()
        articulation_config = self.get_parameter('articulation_config').value
        if articulation_config:
            from robot_lab_utils.urdf_articulation import ArticulationROS
            self._articulation = ArticulationROS(self, articulation_config, urdf, str(model))
            added_inertia = self._articulation.contract.config.get('pybullet_added_link_inertia', {})
            if not isinstance(added_inertia, dict):
                raise ValueError('pybullet_added_link_inertia must map joint names to positive inertia')
            for name, inertia in added_inertia.items():
                if (name not in self._articulation.contract.joints or type(inertia) not in (int, float)
                        or not math.isfinite(inertia) or inertia <= 0.):
                    raise ValueError('Invalid PyBullet added link inertia for %s' % name)
                index = self._joint_idx[name]
                if p.getJointInfo(self._robot_id, index)[2] != p.JOINT_REVOLUTE:
                    raise ValueError('PyBullet added link inertia requires a revolute joint: %s' % name)
                # An isotropic addition preserves the imported principal axes.
                source = p.getDynamicsInfo(self._robot_id, index)[2]
                p.changeDynamics(self._robot_id, index, localInertiaDiagonal=tuple(value+inertia for value in source))
            gear_erp = self._articulation.contract.config.get('pybullet_gear_erp', .1)
            if type(gear_erp) not in (int, float) or not 0. < gear_erp <= 1.:
                raise ValueError('pybullet_gear_erp must be a finite number in (0, 1]')
            named_pads = self._articulation.contract.config.get('grip_contact_geoms')
            if named_pads:
                from robot_lab_pybullet.grip_contacts import GripContacts
                pads = GripContacts(p, self._robot_id, self._link_idx, urdf, named_pads)
                self._articulation.grip_contacts = lambda: pads.measure(self._free_bodies, self._sim_t)
                for link in pads.pads:
                    p.changeDynamics(self._robot_id, link, frictionAnchor=True)
            if set(self._articulation.contract.joints) & set(self._drive.wheel_joints):
                raise ValueError('Articulation cannot own drive wheels')
            # Joints whose range excludes 0 start at their home (contract).
            for name, value in self._articulation.contract.spawn_positions.items():
                if name in self._joint_idx:
                    p.resetJointState(self._robot_id, self._joint_idx[name], value)
            # Mechanically linked joints (a gripper's fingers on one motor) are
            # coupled physically too, as in PyBullet's own Panda example.
            for follower, coupling in self._articulation.contract.couplings.items():
                if not coupling.get('mechanical'):
                    continue
                # Bullet constrains qA + ratio*qB = target; qA is the follower.
                gear = p.createConstraint(
                    self._robot_id, self._joint_idx[follower], self._robot_id, self._joint_idx[coupling['joint']],
                    jointType=p.JOINT_GEAR, jointAxis=[1, 0, 0], parentFramePosition=[0, 0, 0],
                    childFramePosition=[0, 0, 0])
                p.changeConstraint(gear, gearRatio=-float(coupling['multiplier']), erp=gear_erp,
                                   relativePositionTarget=float(coupling['offset']),
                                   maxForce=self._articulation.contract.joints[follower]['effort'] * 2.5)
                self.get_logger().info('Mechanical coupling %s = %g * %s (gear constraint)'
                                       % (follower, coupling['multiplier'], coupling['joint']))
            grip = self._articulation.contract.grip
            if self._articulation.contract.config.get('pybullet_symmetric_grip') and len(grip) == 2:
                # Two independently servoed fingers on a floating base drift
                # sideways while they squeeze in Bullet: TIAGo's shoved a
                # block 2.4 cm (2026-11-17; the same fixed-base squeeze held).
                # Keep them level, as the gripper controller commands them.
                gear = p.createConstraint(
                    self._robot_id, self._joint_idx[grip[0]], self._robot_id, self._joint_idx[grip[1]],
                    jointType=p.JOINT_GEAR, jointAxis=[1, 0, 0], parentFramePosition=[0, 0, 0],
                    childFramePosition=[0, 0, 0])
                p.changeConstraint(gear, gearRatio=-1.0, erp=0.1,
                                   maxForce=self._articulation.contract.joints[grip[1]]['effort'] * 2.5)
                self.get_logger().info('Symmetric grip %s = %s (gear constraint)' % (grip[1], grip[0]))
            for name in grip:
                link = self._joint_idx[name]       # a joint's child link has the joint's index
                p.changeDynamics(self._robot_id, link, frictionAnchor=True)
            hold_mode = 'false'  # configured servos hold only lift/slides; casters remain passive
            # MOLAR's deck belt carries loose payloads with the measured roller
            # (Gazebo: TrackController; MuJoCo: ConveyorBelt; R11 transfer).
            links = {p.getJointInfo(self._robot_id, i)[12].decode(): i for i in range(p.getNumJoints(self._robot_id))}
            if 'telescope_stage2' in links and 'transfer_roller_joint' in self._joint_idx and self._free_bodies:
                from robot_lab_pybullet.pybullet_belt import PyBulletBelt
                self._belt = PyBulletBelt(self._robot_id, links['telescope_stage2'],
                                          self._joint_idx['transfer_roller_joint'], 0.012, self._free_bodies)
                self.get_logger().info('Deck belt on telescope_stage2 for %s' % sorted(self._free_bodies))
        has_drive = (self._lw >= 0 or self._rw >= 0
                     or any(j in self._joint_idx for j in self._drive.wheel_joints))
        self._hold_joints = (hold_mode == "true") or (hold_mode == "auto" and not has_drive)
        # Passive parts of a driven assembly (a mecanum's rollers) are driven
        # hardware: a hold motor would pin them and turn the roller wheel into
        # a solid tire, blocking the strafe the drive model commands (measured
        # numbers in robot_lab_utils.urdf_joints).
        self._driven_assembly_joints = driven_assembly_joints(
            urdf, self._drive.wheel_joints)
        if self._hold_joints:
            self._hold_pose = {}
            for jn in self._joint_names:
                index = self._joint_idx[jn]
                # A passive part of a driven wheel (a mecanum's rollers) is
                # driven hardware: a hold motor pins the rollers and turns the
                # roller wheel into a solid tire, which is what blocked the
                # mecanum's lateral motion (measured numbers in the MuJoCo
                # bridge's _is_driven_assembly_joint docstring).
                if jn in self._driven_assembly_joints:
                    continue
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

        if self.get_parameter('joint_pd_config').value:
            from robot_lab_pybullet.gait_plant import GaitPlant
            from std_msgs.msg import Float64MultiArray,String
            self._gait_plant=GaitPlant(p,self._robot_id,urdf,self.get_parameter('joint_pd_config').value,
                                       self.get_parameter('gait_probe_config').value,
                                       self.get_parameter('gait_contact_config').value)
            self._gait_plant.seed()
            self._gait_terrain_at=-math.inf
            reference=self.get_parameter('gait_floor_reference').value
            if reference:self._gait_plant.align(float(reference))
            self._spawn_base_pose=p.getBasePositionAndOrientation(self._robot_id)
            self._hold_joints=False
            self._gait_contact_pub=self.create_publisher(String,'/gait/contacts',10)
            self._gait_terrain_pub=self.create_publisher(String,'/gait/terrain',10)
            self.create_subscription(Float64MultiArray,self._gait_plant.config.get('topic','/gait/joint_targets'),
                lambda msg:self._gait_plant.receive(msg.data,self._sim_t),10)

        if self.get_parameter('track_config').value:
            from robot_lab_pybullet.track_plant import TrackPlant
            self._tracks=TrackPlant(p,self._robot_id,self.get_parameter('track_config').value)
            self._track_odom_pub=self.create_publisher(Odometry,'/robot_lab_controller/odom',10)
            self.create_subscription(Twist,'/tracked/cmd_vel',
                lambda msg:self._tracks.drive.command(msg.linear.x,msg.angular.z,self._sim_t),10)
            from std_msgs.msg import Float64
            for name,(servo,_) in self._tracks.flippers.items():
                self.create_subscription(Float64,'/tracked/flippers/%s/target'%name,
                    lambda msg,servo=servo:servo.command(msg.data),10)
            self.get_logger().info('PyBullet tracks on %d links' % len(self._tracks.links))

        if self.get_parameter('amphibious_config').value:
            from robot_lab_pybullet.amphibious_plant import AmphibiousPlant
            from std_msgs.msg import Float64,String
            self._amphibious=AmphibiousPlant(p,self._robot_id,urdf,self.get_parameter('amphibious_config').value)
            drive=self._amphibious.drive
            self.create_subscription(Twist,'/amphibious/wheels',lambda msg:drive.command_wheels(
                msg.linear.x,msg.angular.z,msg.linear.y,msg.linear.z,self._sim_t),10)
            for joint in drive.mounts:
                self.create_subscription(Float64,'/marine/thrusters/%s/force'%joint,
                    lambda msg,joint=joint:drive.command_thrust(joint,msg.data,self._sim_t),10)
            self._marine_odom_pub=self.create_publisher(Odometry,'/marine/odometry',10)
            self._marine_imu_pub=self.create_publisher(Imu,'/marine/imu',10)
            self._amphibious_pub=self.create_publisher(String,'/amphibious/plant',10)
            self._amphibious_state_at=-1.
            self.get_logger().info('PyBullet amphibious plant: %d buoyant shapes'%len(drive.shapes))

        self.get_logger().info(
            f"Joints: {self._joint_names}  lw={lw} rw={rw}")
        from robot_lab_utils.urdf_springs import joint_springs
        self._spring_joints = []
        for name, spring in (source_springs if source_springs is not None else joint_springs(urdf)).items():
            index = self._joint_idx.get(name, -1)
            if index < 0:
                raise ValueError('Imported passive spring joint missing: '+name)
            # Apply the complete authored spring law once, not also Bullet's
            # joint damping. These joints stay passive, not position-held.
            p.changeDynamics(self._robot_id, index, jointDamping=0.0)
            self._spring_joints.append((index, spring))

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
        while self._running and rclpy.ok():
            now = time.monotonic()
            elapsed = now - t0

            # cmd_vel — apply watchdog: stop if no recent command
            with self._twist_lock:
                t = self._twist
                stale = (time.monotonic() - self._last_cmd_time) > self._watchdog_timeout
            if stale:
                t = Twist()
                if self._drive.kind == "diff":
                    self._drive.reset()
            if self._articulation is not None:
                states = {name: p.getJointState(self._robot_id, self._joint_idx[name])
                          for name in self._articulation.contract.joints}
                base_linear, base_angular = p.getBaseVelocity(self._robot_id)
                articulation_targets = self._articulation.step(
                    {name: value[0] for name, value in states.items()},
                    {name: value[1] for name, value in states.items()}, self._dt,
                    base_moving=math.hypot(*base_linear[:2]) > self._articulation.base_limits[0]
                    or abs(base_angular[2]) > self._articulation.base_limits[1])
                for kind, values in articulation_targets.items():
                    for name, target in values.items():
                        if self._articulation.contract.couplings.get(name, {}).get('passive'):
                            continue  # physical gear transmits load to the sole powered joint
                        spec = self._articulation.contract.joints[name]
                        if kind == 'velocity':
                            # The light roller's explicit torque PD is unstable at
                            # Bullet's step size (alternating +/-100 rad/s). Use
                            # its native implicit velocity motor, still bounded
                            # by the source effort limit; never reset joint state.
                            p.setJointMotorControl2(self._robot_id, self._joint_idx[name],
                                p.VELOCITY_CONTROL, targetVelocity=target,
                                force=spec['effort'], velocityGain=1.)
                            continue
                        # Implicit prismatic motor avoids explicit-PD stop chatter.
                        # Finite source effort and velocity limits still apply.
                        # These gains are solver parameters, not hardware PID.
                        force = spec['effort']
                        if name in self._articulation.contract.grip:
                            # Two fingers squeezing an object both saturate this
                            # motor, and nothing centres the object: TIAGo's
                            # fingers shoved a block 2.4 cm (2026-11-17). Cap
                            # the force at the servo's kp x error, as a P law.
                            error = abs(target - states[name][0])
                            force = min(spec['effort'], max(.05 * spec['effort'], spec['kp'] * error))
                        p.setJointMotorControl2(self._robot_id, self._joint_idx[name],
                            p.POSITION_CONTROL, targetPosition=target,
                            force=force, maxVelocity=spec['max_velocity'],
                            positionGain=.1, velocityGain=1.)
                if self._articulation.base_inhibited:
                    with self._twist_lock:
                        self._twist = Twist()
                        self._last_cmd_time = -math.inf
                    t = Twist()
                    self._drive.reset()
            targets = self._drive.targets(
                t.linear.x, t.angular.z, dt=self._dt,
                **({"vy": t.linear.y}
                    if self._drive.kind in ("mecanum", "four_wheel_steer")
                    else {}))
            clamp = 50.0
            from robot_lab_utils.urdf_springs import spring_force
            if not getattr(self, '_hold_joints', False):
                for index, spring in self._spring_joints:
                    state = p.getJointState(self._robot_id, index)
                    p.setJointMotorControl2(self._robot_id, index, p.TORQUE_CONTROL,
                                           force=spring_force(spring, state[0], state[1]))
            if getattr(self, "_hold_joints", False):
                # Display hold: every non-drive joint servos to its spawn
                # angle through the physics engine's position motor, within
                # the joint's own effort limit.  The base is free, so the
                # robot stands on the floor and still moves physically
                # (settling, sagging, pushes), and /joint_states reflects
                # that; pinning the base and forcing 200 N*m froze it.
                for jn, (q0, force) in getattr(self, "_hold_pose", {}).items():
                    idx = self._joint_idx.get(jn, -1)
                    if idx < 0 or idx == self._lw or idx == self._rw \
                            or idx in self._drive_idx:
                        continue
                    try:
                        p.setJointMotorControl2(
                            self._robot_id, idx, p.POSITION_CONTROL,
                            targetPosition=q0, force=force)
                    except Exception:
                        pass
            if self._drive.kind == "diff":
                # Only a differential drive has the two legacy wheel joints;
                # the car/4WS/mecanum models name their own joints below.
                vl = targets.velocity.get(self._drive.left, 0.0)
                vr = targets.velocity.get(self._drive.right, 0.0)
                if self._lw >= 0:
                    p.setJointMotorControl2(
                        self._robot_id, self._lw, p.VELOCITY_CONTROL,
                        targetVelocity=max(-clamp, min(clamp, vl)), force=self._wheel_force_limit)
                if self._rw >= 0:
                    p.setJointMotorControl2(
                        self._robot_id, self._rw, p.VELOCITY_CONTROL,
                        targetVelocity=max(-clamp, min(clamp, vr)), force=self._wheel_force_limit)
            else:
                for joint, rate in targets.velocity.items():
                    idx = self._joint_idx.get(joint, -1)
                    if idx >= 0:
                        p.setJointMotorControl2(
                            self._robot_id, idx, p.VELOCITY_CONTROL,
                            targetVelocity=max(-clamp, min(clamp, rate)), force=self._wheel_force_limit)
                for joint, angle in targets.position.items():
                    idx = self._joint_idx.get(joint, -1)
                    if idx >= 0:
                        p.setJointMotorControl2(
                            self._robot_id, idx, p.POSITION_CONTROL,
                            targetPosition=angle, force=10.0, maxVelocity=3.0)

            if self._gait_plant is not None:self._gait_plant.step(self._sim_t)
            if getattr(self, '_belt', None) is not None:
                forces = self._belt.step()
                if forces and self._sim_step % int(round(2.0 / self._dt)) == 0:
                    self.get_logger().info('Belt: forces %s; contacts by robot link %s' % (
                        {k: round(v, 3) for k, v in forces.items()},
                        {k: {link: round(n, 2) for link, n in v.items()} for k, v in self._belt.contacts.items()}))
            if self._tracks is not None:self._tracks.step(self._sim_t,self._dt)
            if self._amphibious is not None:self._amphibious.step(self._sim_t)
            p.stepSimulation()
            if self._gait_plant is not None:self._gait_plant.sample()
            self._sim_step += 1
            self._sim_t = self._sim_step * self._dt

            pos, orn = self._base_link_pose()
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
            if self._camera is not None:
                if self._camera['link'] >= 0:
                    state = p.getLinkState(self._robot_id, self._camera['link'], computeForwardKinematics=True)
                    self._cam_pose = (list(state[4]), list(state[5]))
                self._free_poses = {name: p.getBasePositionAndOrientation(uid)
                                    for name, uid in self._free_bodies.items()}

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
                    if self._tracks is not None:self._pub_track_odom()
                    if self._amphibious is not None:self._pub_amphibious()
                    if self._free_bodies and self._sim_t - self._free_bodies_t + 1e-9 >= 0.1:
                        self._free_bodies_t = self._sim_t
                        self._pub_free_bodies()
                    if self._gait_plant is not None:
                        import json
                        from std_msgs.msg import String
                        from robot_lab_utils.gait_terrain import sample_probes
                        self._gait_contact_pub.publish(String(data=json.dumps(self._gait_plant.measure(self._sim_t),allow_nan=False)))
                        if time.monotonic()-self._gait_terrain_at>=.1:
                            self._gait_terrain_at=time.monotonic()
                            try:
                                terrain=sample_probes(self._gait_plant.probes,self._gait_plant.link_pose,
                                    self._gait_plant.sole_offset,self._gait_plant.ray,self._sim_t)
                            except ValueError as exc:terrain=dict(clock_time=self._sim_t,error=str(exc),feet={})
                            self._gait_terrain_pub.publish(String(data=json.dumps(terrain,allow_nan=False)))
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
        m.child_frame_id = self._base_frame
        m.pose.pose.position = Point(x=self._bpos[0], y=self._bpos[1], z=self._bpos[2])
        m.pose.pose.orientation = Quaternion(x=self._born[0], y=self._born[1], z=self._born[2], w=self._born[3])
        m.twist.twist.linear = Vector3(x=self._blin[0], y=self._blin[1], z=self._blin[2])
        m.twist.twist.angular = Vector3(x=self._bang[0], y=self._bang[1], z=self._bang[2])
        self._odom_pub.publish(m)

    def _pub_amphibious(self):
        """Gazebo's /marine/odometry (map -> base_link, body-frame twist), /marine/imu and /amphibious/plant."""
        from std_msgs.msg import String
        origin, R, com, linear, angular = self._amphibious._frame(-1)
        linear = linear + np.cross(angular, origin - com)
        body_linear, body_angular = R.T @ linear, R.T @ angular
        _, quaternion = p.multiplyTransforms(*p.getBasePositionAndOrientation(self._robot_id),
            *p.invertTransform(*p.getDynamicsInfo(self._robot_id, -1)[3:5]))
        stamp = self._stamp()
        q = Quaternion(x=float(quaternion[0]), y=float(quaternion[1]), z=float(quaternion[2]), w=float(quaternion[3]))
        m = Odometry()
        m.header.stamp, m.header.frame_id, m.child_frame_id = stamp, "map", "base_link"
        m.pose.pose.position = Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2]))
        m.pose.pose.orientation = q
        m.twist.twist.linear = Vector3(x=float(body_linear[0]), y=float(body_linear[1]), z=float(body_linear[2]))
        m.twist.twist.angular = Vector3(x=float(body_angular[0]), y=float(body_angular[1]), z=float(body_angular[2]))
        self._marine_odom_pub.publish(m)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, "base_link"
        imu.orientation = q
        imu.orientation_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        imu.angular_velocity = Vector3(x=float(body_angular[0]), y=float(body_angular[1]), z=float(body_angular[2]))
        self._marine_imu_pub.publish(imu)
        if self._sim_t - self._amphibious_state_at >= 0.1:
            self._amphibious_state_at = self._sim_t
            self._amphibious_pub.publish(String(data=self._amphibious.drive.plant_message(self._sim_t)))

    def _pub_track_odom(self):
        """Track dead reckoning, as Gazebo's TrackedVehicle odometry."""
        drive = self._tracks.drive
        x, y, yaw = drive.pose
        m = Odometry()
        m.header.stamp = self._stamp()
        m.header.frame_id = "odom"
        m.child_frame_id = self._base_frame
        m.pose.pose.position = Point(x=x, y=y, z=0.0)
        m.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
        m.twist.twist.linear = Vector3(x=drive.velocity[0], y=0.0, z=0.0)
        m.twist.twist.angular = Vector3(x=0.0, y=0.0, z=drive.velocity[1])
        self._track_odom_pub.publish(m)

    def _pub_imu(self):
        m = Imu()
        m.header.stamp = self._stamp()
        m.header.frame_id = self.get_parameter('imu_link_name').value
        m.orientation = Quaternion(x=self._born[0], y=self._born[1], z=self._born[2], w=self._born[3])
        m.orientation_covariance = [0.001,0.0,0.0, 0.0,0.001,0.0, 0.0,0.0,0.001]
        m.angular_velocity = Vector3(x=self._bang[0], y=self._bang[1], z=self._bang[2])
        m.angular_velocity_covariance = [0.01,0.0,0.0, 0.0,0.01,0.0, 0.0,0.0,0.01]
        m.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)
        m.linear_acceleration_covariance = [0.1,0.0,0.0, 0.0,0.1,0.0, 0.0,0.0,0.1]
        self._imu_pub.publish(m)

    def _make_drive(self):
        """Wheel/steering model for /cmd_vel (differential unless configured)."""
        args = (self.get_parameter("left_wheel_joint").value,
                self.get_parameter("right_wheel_joint").value,
                self.get_parameter("wheel_radius").value,
                self.get_parameter("wheel_separation").value)
        try:
            drive = drive_from_config(
                parse_drive_config(self.get_parameter("drive_config").value), *args)
        except ValueError as exc:
            self.get_logger().error("drive_config rejected (%s); using a "
                                    "differential drive" % exc)
            drive = drive_from_config({}, *args)
        if drive.kind != "diff":
            self.get_logger().info(
                "Drive: %s, wheels %s, steering %s"
                % (drive.kind, drive.wheel_joints, list(drive.steer_joints)))
        return drive

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
                if getattr(self, "_drive", None) is not None and hasattr(self._drive, "reset"):
                    self._drive.reset()
                # Zero actuated joint motion so the robot does not coast
                # away from the spawn pose on pre-reset wheel spin (R6.1
                # reset contract: velocities restored, not just the pose).
                for i in range(p.getNumJoints(self._robot_id)):
                    if p.getJointInfo(self._robot_id, i)[2] != p.JOINT_FIXED:
                        p.resetJointState(self._robot_id, i, 0.0, 0.0)
                if self._articulation is not None:
                    for name, value in self._articulation.contract.spawn_positions.items():
                        if name in self._joint_idx:
                            p.resetJointState(self._robot_id, self._joint_idx[name], value, 0.0)
                if self._gait_plant is not None:
                    self._gait_plant.seed();self._gait_plant.at=None
                if self._tracks is not None:
                    self._tracks.reset()
                if self._amphibious is not None:
                    self._amphibious.reset()
                # A still-fresh cmd_vel would re-accelerate the wheels on
                # the next physics tick and drag the robot out of its reset
                # pose, so the pending command is cleared as well.
                with self._twist_lock:
                    self._twist = Twist()
                    self._last_cmd_time = 0.0
                if self._articulation is not None:
                    self._articulation.reset()
                # Keep ROS/sensor time monotonic when resetting physical state.
                # Rewinding invalidates TF caches and estimator timer state.
                self._reset_notifications.notify()
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
        workcell = self.get_parameter('workcell_rgbd_pose').value
        if rate <= 0.0:
            return None
        if not workcell and link not in self._link_idx:
            self.get_logger().info(
                "Robot has no '%s' link; no RGB-D camera is published." % link)
            return None
        camera = {
            "link": -1 if workcell else self._link_idx[link],
            "rate": rate,
            "frame": self.get_parameter("camera_optical_frame").value,
            "width": int(self.get_parameter("camera_width").value),
            "height": int(self.get_parameter("camera_height").value),
            "horizontal_fov": float(
                self.get_parameter("camera_horizontal_fov").value),
            "near": float(self.get_parameter("camera_near").value),
            "far": float(self.get_parameter("camera_far").value),
        }
        if workcell:
            from tf2_ros import StaticTransformBroadcaster

            pose = np.asarray(json.loads(workcell), dtype=float)
            if pose.shape != (7,) or not np.isfinite(pose).all() or not np.isclose(np.linalg.norm(pose[3:]), 1.):
                raise ValueError('Workcell camera needs an XYZ position and a unit WXYZ optical quaternion')
            px, py, pz, w, x, y, z = map(float, pose)
            camera['frame'] = camera_model.WORKCELL_RGBD_FRAME
            camera['workcell_pose'] = ((px, py, pz), (x, y, z, w))
            optical = TransformStamped()
            optical.header.frame_id = 'world'
            optical.child_frame_id = camera['frame']
            optical.transform.translation = Vector3(x=px, y=py, z=pz)
            optical.transform.rotation = Quaternion(w=w, x=x, y=y, z=z)
            self._workcell_camera_tf = StaticTransformBroadcaster(self)
            self._workcell_camera_tf.sendTransform(optical)
            link = camera['frame']
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
                want_rgb = self._rgb_pub.get_subscription_count() > 0
                want_depth = self._depth_pub.get_subscription_count() > 0
                want_info = self._camera_info_pub.get_subscription_count() > 0
                if not (want_rgb or want_depth or want_info):
                    continue
                # Stamp at the snapshot moment: the image reflects this sim
                # time, and consecutive stamps then honour the camera_rate
                # interval exactly regardless of render duration.
                stamp = self._stamp()
                if not (want_rgb or want_depth):
                    self._camera_info_pub.publish(camera_info_msg(
                        stamp, camera["frame"], camera["width"], camera["height"],
                        camera["horizontal_fov"], camera_model.TINY_RENDERER))
                    continue
                p.resetBasePositionAndOrientation(
                    self._render_robot_id, self._bpos, self._born,
                    physicsClientId=self._render_client)
                for jn, jp, jv in zip(self._joint_names, self._jpos,
                                      self._jvel):
                    ji = self._render_joint_idx.get(jn)
                    if ji is not None:
                        p.resetJointState(self._render_robot_id, ji, jp, jv,
                                          physicsClientId=self._render_client)
                for name, (position, orientation) in self._free_poses.items():
                    uid = self._mirror_free_bodies.get(name)
                    if uid is not None:
                        p.resetBasePositionAndOrientation(uid, position, orientation,
                                                          physicsClientId=self._render_client)
                rgb, depth = _render_rgbd(
                    self._cam_pose[0], self._cam_pose[1], camera,
                    client=self._render_client)
                if want_rgb:
                    self._rgb_pub.publish(image_msg(stamp, camera["frame"], rgb, "rgb8"))
                if want_depth:
                    self._depth_pub.publish(image_msg(stamp, camera["frame"], depth, "32FC1"))
                # TinyRenderer's optical axis is at (w / 2, h / 2 - 1).
                self._camera_info_pub.publish(camera_info_msg(
                    stamp, camera["frame"], camera["width"], camera["height"],
                    camera["horizontal_fov"], camera_model.TINY_RENDERER))
            except Exception:
                # Shutting down: the physics client may already be gone.
                if not rclpy.ok() or not self._running:
                    break
                raise

    def _base_link_pose(self):
        """World pose of the root link frame, not of its centre of mass.

        PyBullet reports the base's inertial frame, which for a description
        with an offset <inertial> (Berkeley Humanoid Lite, the Unitree
        robots) is not where the link frame - and so the odometry frame - is.
        """
        pos, orn = p.getBasePositionAndOrientation(self._robot_id)
        if self._base_inertial_offset is None:
            info = p.getDynamicsInfo(self._robot_id, -1)
            self._base_inertial_offset = p.invertTransform(info[3], info[4])
        return p.multiplyTransforms(pos, orn, *self._base_inertial_offset)

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
    # Die with the launch that started this node, even if it never sends a
    # stop signal (see robot_lab_utils.process_lifetime).
    exit_with_parent()
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
