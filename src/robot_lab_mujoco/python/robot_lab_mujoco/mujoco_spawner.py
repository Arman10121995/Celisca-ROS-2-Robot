"""MuJoCo robot spawner node.

Loads a world + robot model, opens the MuJoCo passive viewer, runs the
physics step loop, and publishes the ROS 2 topics required by the
stack (joint_states, TF, odom, scan, imu, clock).
"""
import hashlib
import math
import os
import re
import shutil
import struct
import subprocess
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    # Graceful degradation: keep a module-like shim so that
    # ``mujoco.viewer is not None`` checks below remain valid without
    # crashing at import time (AttributeError: 'NoneType' has no 'viewer').
    import types
    mujoco = types.SimpleNamespace(viewer=None)

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


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _xacro_to_urdf(xacro_path):
    result = subprocess.run(
        ["xacro", xacro_path],
        capture_output=True, text=True, timeout=30,
    )
    if result.returncode != 0:
        raise RuntimeError("xacro failed: %s" % result.stderr[:500])
    return result.stdout


def _rewrite_package_uris(text, pkg_map):
    def _repl(m):
        pkg, rest = m.group(1), m.group(2)
        base = pkg_map.get(pkg, "")
        return ("file://" + os.path.join(base, rest)) if base else m.group(0)
    return re.sub(r"package://([^/]+)/(.+?)(?=[\"'\s<]|$)", _repl, text)


def _strip_gazebo_tags(urdf_text):
    for tag in ("ros2_control", "transmission", "gazebo"):
        urdf_text = re.sub(
            r"<%s[^>]*>.*?</%s>" % (tag, tag), "", urdf_text, flags=re.DOTALL
        )
    return urdf_text


def _quat_from_yaw(yaw):
    return Quaternion(x=0.0, y=0.0,
                      z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


# Fallback MJCF for differential-drive robots (box base + 2 wheel joints).
_FALLBACK_MJCF = """<mujoco model="fallback_robot">
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <geom type="plane" size="10 10 0.1"/>
    <light diffuse="0.8 0.8 0.8" pos="0 0 3" dir="0 0 -1"/>
    <body name="base" pos="0 0 0.08">
      <joint name="free" type="free"/>
      <inertial pos="0 0 0" mass="0.8" diaginertia="0.01 0.01 0.01"/>
      <geom type="box" size="0.1 0.08 0.03" rgba="0.2 0.2 0.8 1"/>
      <body name="wheel_left" pos="0 0.07 -0.03">
        <inertial pos="0 0 0" mass="0.05" diaginertia="1e-5 2e-5 1e-5"/>
        <geom type="cylinder" size="0.033 0.02" rgba="0 0 1 1"/>
        <joint name="wheel_left_joint" type="hinge" axis="0 1 0"/>
      </body>
      <body name="wheel_right" pos="0 -0.07 -0.03">
        <inertial pos="0 0 0" mass="0.05" diaginertia="1e-5 2e-5 1e-5"/>
        <geom type="cylinder" size="0.033 0.02" rgba="0 0 1 1"/>
        <joint name="wheel_right_joint" type="hinge" axis="0 1 0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <velocity joint="wheel_left_joint" ctrllimited="true" ctrlrange="-50 50"/>
    <velocity joint="wheel_right_joint" ctrllimited="true" ctrlrange="-50 50"/>
  </actuator>
</mujoco>
"""


# Mesh staging/conversion lives in robot_lab_utils so the PyBullet backend
# uses the identical pipeline (see robot_lab_utils/mesh_assets.py).  The
# historical private names are kept as aliases: they are what this module and
# its tests already reference.
from robot_lab_utils.mesh_assets import (  # noqa: E402
    _MUJOCO_MAX_STL_FACES,
    _binary_stl_face_count,
    _cap_faces,
    _is_ascii_stl,
    _mesh_staging_dir,
    _parse_ascii_stl,
    _parse_collada_mesh,
    _read_binary_stl,
    _resolve_mesh_source,
    _stage_stl,
    _write_binary_stl,
    _write_placeholder_stl,
)


def _stage_meshes(urdf_text, pkg_map, cache_dir, base_dir=""):
    """Stage every URDF mesh into *cache_dir* and rewrite the URDF.

    - package:// URIs are resolved to real files (MuJoCo cannot read them);
    - Collada (.dae) meshes are converted to binary STL (MuJoCo only reads
      STL/OBJ/MSH) using the built-in minimal Collada parser;
    - meshes that are missing or unconvertible get a small placeholder box
      so the robot still loads and stays visible.

    Returns (staged_urdf_text, notes, placeholder_count).
    """
    notes = []
    placeholders = 0
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError as exc:
        return urdf_text, ["URDF XML parse error: %s" % exc], 0

    for mesh_elem in root.iter("mesh"):
        uri = mesh_elem.get("filename") or ""
        source = _resolve_mesh_source(uri, pkg_map, base_dir=base_dir)
        stem = re.sub(r"[^A-Za-z0-9_.-]+", "_",
                      os.path.splitext(os.path.basename(uri))[0] or "mesh")
        ext = os.path.splitext(uri)[1].lower()
        staged = ""
        if source and os.path.isfile(source):
            if ext == ".stl":
                staged = os.path.join(cache_dir, stem + ".stl")
                if not os.path.exists(staged):
                    staged = _stage_stl(source, staged, notes, uri)
            elif ext in (".obj", ".msh"):
                staged = os.path.join(cache_dir, os.path.basename(source))
                if not os.path.exists(staged):
                    try:
                        shutil.copyfile(source, staged)
                    except OSError:
                        staged = ""
            elif ext == ".dae":
                staged = os.path.join(cache_dir, stem + ".stl")
                if not os.path.exists(staged):
                    try:
                        verts, tris = _parse_collada_mesh(source)
                        verts, tris = _cap_faces(verts, tris)
                        _write_binary_stl(staged, verts, tris)
                    except Exception as exc:
                        notes.append(
                            "mesh '%s' not convertible (%s); placeholder used"
                            % (os.path.basename(uri), exc))
                        staged = ""
            else:
                notes.append("unsupported mesh format '%s' in '%s'"
                             % (ext, os.path.basename(uri)))
        if staged and os.path.isfile(staged):
            mesh_elem.set("filename", os.path.basename(staged))
        else:
            placeholder = os.path.join(cache_dir, "placeholder_box.stl")
            if not os.path.exists(placeholder):
                try:
                    _write_placeholder_stl(placeholder)
                except OSError:
                    pass
            if os.path.exists(placeholder):
                mesh_elem.set("filename", "placeholder_box.stl")
                placeholders += 1
                notes.append("mesh '%s' unavailable -> placeholder box"
                             % os.path.basename(uri))

    return ET.tostring(root, encoding="unicode"), notes, placeholders


def _repair_urdf_inertias(urdf_text):
    """Clamp non-physical masses/inertias so MuJoCo can import the model.

    Unitree's B-series URDFs ship <inertia ixx=... izz="..."/> full-matrix
    blocks with tiny/zero/negative eigenvalues (sensor links) that MuJoCo
    rejects even with balanceinertia enabled, which only repairs diagonal
    inertias.  Elements whose inertia matrix is not positive-definite are
    replaced by an isotropic matrix of the same magnitude, and non-positive
    masses are raised to a small bound.
    """
    def _float(value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _inertia_ok(ixx, ixy, ixz, iyy, iyz, izz):
        matrix = np.array([
            [ixx, ixy, ixz],
            [ixy, iyy, iyz],
            [ixz, iyz, izz],
        ], dtype=np.float64)
        eigenvalues = np.linalg.eigvalsh(matrix)
        return bool(np.all(np.isfinite(eigenvalues))
                    and np.min(eigenvalues) > 1e-13)

    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError:
        return urdf_text

    repaired = 0
    for inertial in root.iter("inertial"):
        mass_elem = inertial.find("mass")
        if mass_elem is not None:
            mass_value = _float(mass_elem.get("value"))
            if mass_value is None or mass_value <= 0.0:
                mass_elem.set("value", "1e-4")
                repaired += 1
        inertia = inertial.find("inertia")
        if inertia is None:
            continue
        ixx = _float(inertia.get("ixx"))
        ixy = _float(inertia.get("ixy"))
        ixz = _float(inertia.get("ixz"))
        iyy = _float(inertia.get("iyy"))
        iyz = _float(inertia.get("iyz"))
        izz = _float(inertia.get("izz"))
        if None in (ixx, ixy, ixz, iyy, iyz, izz):
            continue
        if _inertia_ok(ixx, ixy, ixz, iyy, iyz, izz):
            continue
        scale = max((ixx + iyy + izz) / 3.0, 1e-4)
        inertia.set("ixx", "%.6g" % scale)
        inertia.set("ixy", "0")
        inertia.set("ixz", "0")
        inertia.set("iyy", "%.6g" % scale)
        inertia.set("iyz", "0")
        inertia.set("izz", "%.6g" % scale)
        repaired += 1

    if not repaired:
        return urdf_text
    return ET.tostring(root, encoding="unicode")


def _inject_mujoco_compiler(urdf_text, mesh_dir):
    """Add or patch the MuJoCo URDF-compiler block (meshdir + inertia repair).

    Unitree URDFs already ship a ``<mujoco><compiler .../></mujoco>``
    extension block — in that case the existing block is patched in place
    (adding meshdir/strippath and inertia bounds) instead of duplicated.
    """
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError:
        return urdf_text

    compiler_attrs = {
        "meshdir": mesh_dir.replace("\\", "/"),
        "strippath": "false",
        "discardvisual": "false",
        "balanceinertia": "true",
        "boundmass": "1e-4",
        "boundinertia": "1e-8",
    }

    existing = root.find("mujoco")
    if existing is not None:
        compiler = existing.find("compiler")
        if compiler is None:
            compiler = ET.SubElement(existing, "compiler")
        # meshdir/strippath/discardvisual must match the staged cache;
        # everything else (angle, balanceinertia...) is only filled in when
        # the URDF does not already carry a value.
        forced = ("meshdir", "strippath", "discardvisual")
        for key, value in compiler_attrs.items():
            if key in forced or not compiler.get(key):
                compiler.set(key, value)
        return ET.tostring(root, encoding="unicode")

    block = ET.Element("mujoco")
    compiler = ET.SubElement(block, "compiler")
    for key, value in compiler_attrs.items():
        compiler.set(key, value)
    # <mujoco> must follow <robot>'s opening tag as its first child.
    root.insert(0, block)
    return ET.tostring(root, encoding="unicode")


def _add_floating_base(mjcf_text, robot_name="robot"):
    """Give an imported robot a floating base so it can move.

    MuJoCo's URDF importer welds the root link to the world (URDF has no
    notion of a floating base), which leaves every wheeled/legged robot
    pinned in place and makes /cmd_vel do nothing.  Wrapping the robot's
    worldbody content in a single body that carries a ``<freejoint/>`` gives
    the base the 6 DOF the physics loop and the odometry publisher expect.

    A model that already has a free joint is returned unchanged.
    """
    try:
        root = ET.fromstring(mjcf_text)
    except ET.ParseError:
        return mjcf_text

    worldbody = root.find("worldbody")
    if worldbody is None or not len(worldbody):
        return mjcf_text

    for element in root.iter():
        if element.tag == "freejoint":
            return mjcf_text
        if element.tag == "joint" and element.get("type") == "free":
            return mjcf_text

    # Lights and cameras stay in the world; everything else rides the base.
    movable = [child for child in list(worldbody)
               if child.tag not in ("light", "camera")]
    if not movable:
        return mjcf_text

    base = ET.Element("body", {"name": "%s_base" % robot_name})
    ET.SubElement(base, "freejoint", {"name": "%s_freejoint" % robot_name})
    for child in movable:
        worldbody.remove(child)
        base.append(child)
    worldbody.append(base)
    return ET.tostring(root, encoding="unicode")


def _stage_world_meshes(mjcf_text, logger=None):
    """Convert a world MJCF's mesh assets into formats MuJoCo can load.

    Generated world MJCF (robot_lab_maps/mjcf/*.xml) points straight at the
    files the Gazebo world uses, which for the vendored Gazebo model library
    are Collada (.DAE).  MuJoCo reads only STL/OBJ/MSH, so each mesh is
    converted once into the same content-addressed cache the robot meshes
    use, and the MJCF is rewritten to reference the staged file.

    Meshes that cannot be converted become a small placeholder box, so one
    bad asset never costs the whole world.
    """

    def _log(level, message):
        if logger is not None:
            getattr(logger, level)(message)

    try:
        root = ET.fromstring(mjcf_text)
    except ET.ParseError:
        return mjcf_text

    assets = [mesh for mesh in root.iter("mesh") if mesh.get("file")]
    if not assets:
        return mjcf_text

    cache_dir = _mesh_staging_dir("world", mjcf_text)
    converted = placeholders = 0
    for mesh in assets:
        source = mesh.get("file") or ""
        extension = os.path.splitext(source)[1].lower()
        if extension in (".obj", ".msh"):
            continue
        stem = re.sub(r"[^A-Za-z0-9_.-]+", "_",
                      os.path.splitext(os.path.basename(source))[0] or "mesh")
        staged = os.path.join(cache_dir, stem + ".stl")
        notes = []
        if extension == ".stl" and os.path.isfile(source):
            result = _stage_stl(source, staged, notes, source)
            if result:
                mesh.set("file", result)
                continue
        if os.path.isfile(source):
            try:
                vertices, faces = _parse_collada_mesh(source)
                _write_binary_stl(staged, vertices, faces)
                mesh.set("file", staged)
                converted += 1
                continue
            except Exception as exc:
                notes.append("%s: %s" % (os.path.basename(source), exc))
        _write_placeholder_stl(staged)
        mesh.set("file", staged)
        placeholders += 1
        for note in notes[:1]:
            _log("warning", "MuJoCo world mesh: " + note)

    if converted or placeholders:
        _log("info", "MuJoCo world meshes staged: %d converted, %d placeholder(s)."
                     % (converted, placeholders))
    return ET.tostring(root, encoding="unicode")


def _build_mjcf_from_urdf(urdf_text, pkg_map, logger=None, robot_name="",
                          base_dir=""):
    """Convert a prepared URDF into MJCF via MuJoCo's URDF importer.

    Mesh staging (package:// resolution, DAE->STL conversion) happens first
    so the importer sees a self-contained model.  Failures are reported
    with the real MuJoCo error; the generic diff-drive template is a loud
    last resort, never a silent substitution (which previously made most
    robots appear as a box in the viewer).
    """

    def _log(level, message):
        if logger is not None:
            getattr(logger, level)(message)

    if mujoco is None or not hasattr(mujoco, "MjSpec"):
        _log("error", "mujoco (>=3.2 with MjSpec) not importable; "
                      "using fallback MJCF template.")
        return _FALLBACK_MJCF

    tmp = None
    try:
        cache_dir = _mesh_staging_dir(robot_name, urdf_text)
        urdf_text, notes, placeholders = _stage_meshes(
            urdf_text, pkg_map, cache_dir, base_dir=base_dir)
        for note in notes:
            _log("warning", "MuJoCo mesh staging: " + note)
        repaired = urdf_text
        urdf_text = _repair_urdf_inertias(urdf_text)
        if urdf_text != repaired:
            _log("warning", "MuJoCo inertia repair: clamped non-physical "
                            "mass/inertia entries.")
        urdf_text = _inject_mujoco_compiler(urdf_text, cache_dir)

        tmp = tempfile.NamedTemporaryFile(
            suffix=".urdf", delete=False, mode="w"
        )
        tmp.write(urdf_text)
        tmp.close()
        spec = mujoco.MjSpec.from_file(tmp.name)
        mjcf = _add_floating_base(spec.to_xml(), robot_name or "robot")
        if placeholders:
            _log("warning",
                 "MuJoCo URDF import OK with %d placeholder geom(s); "
                 "some meshes could not be converted." % placeholders)
        else:
            _log("info", "MuJoCo URDF import OK (all meshes staged).")
        return mjcf
    except Exception as exc:
        _log("error",
             "MuJoCo URDF import failed (%s); using fallback diff-drive "
             "template.%s" % (
                 exc,
                 " Staged URDF kept at %s." % tmp.name if tmp else ""))
        return _FALLBACK_MJCF
    finally:
        if tmp is not None:
            try:
                os.unlink(tmp.name)
            except Exception:
                pass


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class MuJoCoSpawner(Node):
    """Spawn a robot into a MuJoCo world, open the GUI viewer, and
    publish joint_states, TF, odom, scan, imu, and clock."""

    def __init__(self):
        super().__init__("mujoco_spawner")

        # --- parameters (mirror PyBullet spawner) ---
        self.declare_parameter("robot_name", "bumperbot")
        self.declare_parameter("robot_package", "robot_lab_robots")
        self.declare_parameter("robot_xacro", "")
        self.declare_parameter("model", "")
        self.declare_parameter("world_xml", "")
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_z", 0.0)
        self.declare_parameter("spawn_yaw", 0.0)
        # use_sim_time is auto-declared by rclpy when passed via launch
        # overrides — only declare it if not already present.
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("gui", True)
        self.declare_parameter("physics_rate", 240.0)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("scan_rate", 5.0)
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)
        self.declare_parameter("left_wheel_joint", "wheel_left_joint")
        self.declare_parameter("right_wheel_joint", "wheel_right_joint")
        self.declare_parameter("laser_link_name", "laser_link")
        self.declare_parameter("scan_samples", 360)
        self.declare_parameter("scan_range_min", 0.12)
        self.declare_parameter("scan_range_max", 12.0)

        # --- state ---
        self._model = None
        self._data = None
        self._viewer = None
        self._body_id = -1
        self._model_source = "none"
        self._free_joint_qpos_adr = -1
        self._joint_name2id = {}   # mujoco joint name -> qpos index
        self._joint_name2dofadr = {}  # mujoco joint name -> dof (qvel) index
        self._joint_names = []
        self._lw_name = ""
        self._rw_name = ""
        self._lw_qpos_adr = -1
        self._rw_qpos_adr = -1
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

        # --- publishers ---
        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        # Ground-truth odometry on /odom/ground_truth (perfect, from physics).
        # The fused estimate lives on /odom (published by the EKF).
        self._odom_pub = self.create_publisher(Odometry, "/odom/ground_truth", 10)
        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self._imu_pub = self.create_publisher(Imu, "/imu/out", 10)
        self._clock_pub = self.create_publisher(RosClock, "/clock", 10)
        # TF published by the EKF, not the spawner.

        # Readiness / health / reset contracts (R2.3).
        self._ready_pub = self.create_publisher(Bool, "/robot_lab/ready", 10)
        self._health_pub = self.create_publisher(
            DiagnosticArray, "/robot_lab/health", 10)
        self._reset_srv = self.create_service(
            Trigger, "/robot_lab/reset", self._on_reset)
        self._ready = False
        self._health_timer = self.create_timer(
            1.0, self._publish_health,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME))

        # --- subscriptions ---
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

        # --- timer to attempt spawn ---
        # NOTE: use an explicit wall-clock timer. With use_sim_time=true the
        # node default clock is frozen until a /clock publisher exists — and
        # this node IS the /clock publisher, so a default-clock spawn timer
        # would never fire (sim-time deadlock).
        self._timer = self.create_timer(
            0.5, self._try_spawn,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )
        self._thread = None

    def _on_cmd(self, msg):
        with self._twist_lock:
            self._twist = msg
            self._last_cmd_time = time.monotonic()

    def _try_spawn(self):
        if self._model is not None:
            return
        self._timer.cancel()
        if mujoco is None:
            self.get_logger().error("mujoco not importable")
            return
        try:
            self._spawn()
        except Exception as exc:
            self.get_logger().error("Spawn failed: %s" % exc)
            self._timer = self.create_timer(
                2.0, self._try_spawn,
                clock=Clock(clock_type=ClockType.SYSTEM_TIME),
            )

    def _spawn(self):
        from ament_index_python.packages import get_package_share_directory

        # --- resolve world XML (mjcf) ---
        world_xml = self.get_parameter("world_xml").value
        if not world_xml or not os.path.isfile(str(world_xml)):
            self.get_logger().warn(
                "world_xml not found (%s); building from URDF only."
                % world_xml
            )
            world_xml = None

        # --- resolve robot URDF -> MJCF ---
        model = self.get_parameter("model").value
        if not model:
            pkg = self.get_parameter("robot_package").value
            xacro = self.get_parameter("robot_xacro").value
            if pkg and xacro:
                model = os.path.join(
                    get_package_share_directory(pkg), xacro
                )

        use_fallback = False
        self._model_source = "fallback"
        if model and os.path.isfile(str(model)):
            urdf = _xacro_to_urdf(str(model))
            pkg_map = {}
            for pkg_name in re.findall(r"\\$\\(find\\s+([^)]+)\\)", urdf):
                try:
                    pkg_map[pkg_name] = get_package_share_directory(pkg_name)
                except Exception:
                    pass
            rp = self.get_parameter("robot_package").value
            # package:// mesh URIs may reference packages not named by
            # $(find ...) xacro args — resolve those shares too.
            for pkg_name in re.findall(r"package://([^/]+)/", urdf):
                if pkg_name not in pkg_map:
                    try:
                        pkg_map[pkg_name] = get_package_share_directory(pkg_name)
                    except Exception:
                        pass
            if rp and rp not in pkg_map:
                try:
                    pkg_map[rp] = get_package_share_directory(rp)
                except Exception:
                    pass
            urdf = _strip_gazebo_tags(urdf)
            robot_mjcf = _build_mjcf_from_urdf(
                urdf, pkg_map, logger=self.get_logger(),
                robot_name=self.get_parameter("robot_name").value,
                base_dir=os.path.dirname(os.path.abspath(str(model))))
            if robot_mjcf != _FALLBACK_MJCF:
                self._model_source = "urdf"
        else:
            self.get_logger().warn("URDF not found; using fallback MJCF.")
            robot_mjcf = _FALLBACK_MJCF
            use_fallback = True

        # --- combine world + robot into single XML ---
        if world_xml and not use_fallback:
            world_mjcf = self._merge_mjcf(
                world_xml, robot_mjcf, logger=self.get_logger())
        else:
            world_mjcf = robot_mjcf

        # --- build model ---
        self._model = mujoco.MjModel.from_xml_string(world_mjcf)
        self._data = mujoco.MjData(self._model)

        # --- find the root body that contains the free joint ---
        self._find_body_and_joints()

        # --- set spawn pose ---
        sx = self.get_parameter("spawn_x").value
        sy = self.get_parameter("spawn_y").value
        sz = self.get_parameter("spawn_z").value
        syaw = self.get_parameter("spawn_yaw").value

        if self._body_id >= 0 and self._free_joint_qpos_adr >= 0:
            adr = self._free_joint_qpos_adr
            self._data.qpos[adr] = sx
            self._data.qpos[adr + 1] = sy
            self._data.qpos[adr + 2] = sz
            self._data.qpos[adr + 3] = math.cos(syaw / 2.0)
            self._data.qpos[adr + 4] = 0.0
            self._data.qpos[adr + 5] = math.sin(syaw / 2.0)
            self._data.qpos[adr + 6] = 0.0

        mujoco.mj_forward(self._model, self._data)

        self.get_logger().info(
            "MuJoCo model loaded: %d bodies, %d joints"
            % (self._model.nbody, self._model.njnt)
        )

        # --- open viewer ---
        # Determine GUI mode: handle both boolean and string values from launch
        gui_param = self.get_parameter("gui").value
        # Convert string "true"/"false" to boolean if needed
        if isinstance(gui_param, str):
            gui_param = gui_param.lower() in ("true", "1", "yes")
        display = os.environ.get("DISPLAY")
        gui = bool(gui_param and display)
        self.get_logger().info(f"MuJoCo gui param={self.get_parameter('gui').value} (resolved={gui_param}), DISPLAY={display} -> mode={'GUI' if gui else 'DIRECT'}")
        if gui and mujoco.viewer is not None:
            try:
                self._viewer = mujoco.viewer.launch_passive(
                    self._model, self._data
                )
                self.get_logger().info("MuJoCo viewer opened (GUI).")
            except Exception as exc:
                self.get_logger().warn(
                    "Viewer GUI failed (%s); running headless." % exc
                )
                self._viewer = None
        else:
            self.get_logger().info("MuJoCo running headless (gui=%s, viewer=%s)." % (gui, mujoco.viewer is not None))
            self._viewer = None

        # --- start physics thread ---
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.get_logger().info("MuJoCo spawner running.")

        # Signal readiness (R2.3).
        self._ready = True
        self._ready_pub.publish(Bool(data=True))

    # MJCF top-level sections that belong to the robot and must survive the
    # merge into a world.  Dropping <asset> was why every mesh-based robot
    # failed to load with "mesh '<name>' not found in geom N": the geoms
    # referenced meshes whose definitions had been thrown away.
    _MERGED_SECTIONS = (
        "asset", "default", "contact", "equality", "tendon",
        "actuator", "sensor", "keyframe",
    )

    @staticmethod
    def _merge_mjcf(world_path, robot_mjcf, logger=None):
        """Combine a world MJCF with a robot MJCF into one loadable model.

        Both documents keep their own assets, defaults and actuators; the
        robot's worldbody content is appended to the world's worldbody.  Asset
        file paths of both sides are made absolute first, because
        ``MjModel.from_xml_string`` resolves relative paths against the
        process working directory rather than the source XML's directory.
        """
        with open(world_path, "r") as fh:
            world_text = fh.read()

        world_text = MuJoCoSpawner._absolutize_asset_paths(
            world_text, os.path.dirname(os.path.abspath(world_path))
        )
        world_text = _stage_world_meshes(world_text, logger=logger)

        try:
            world_root = ET.fromstring(world_text)
            robot_root = ET.fromstring(robot_mjcf)
        except ET.ParseError:
            # Unparseable input: fall back to the robot model alone rather
            # than emitting a half-merged document.
            return robot_mjcf

        # The robot compiler's meshdir is what makes its <asset> file paths
        # resolvable; bake it into the paths, then drop it so the merged
        # document does not inherit a directory the world does not share.
        robot_compiler = robot_root.find("compiler")
        meshdir = ""
        if robot_compiler is not None:
            meshdir = (robot_compiler.get("meshdir")
                       or robot_compiler.get("assetdir") or "")
        if meshdir:
            for asset in robot_root.iter():
                if asset.tag not in ("mesh", "hfield", "skin", "texture"):
                    continue
                path = asset.get("file")
                if path and not os.path.isabs(path):
                    asset.set("file", os.path.normpath(
                        os.path.join(meshdir, path)))

        # Carry over compiler settings that change how the robot is
        # interpreted (angles, inertia bounds) without the directory hints.
        if robot_compiler is not None:
            world_compiler = world_root.find("compiler")
            if world_compiler is None:
                world_compiler = ET.SubElement(world_root, "compiler")
            for key, value in robot_compiler.attrib.items():
                if key in ("meshdir", "assetdir", "texturedir"):
                    continue
                world_compiler.set(key, value)

        def section(root, tag):
            element = root.find(tag)
            if element is None:
                element = ET.SubElement(root, tag)
            return element

        for tag in MuJoCoSpawner._MERGED_SECTIONS:
            robot_section = robot_root.find(tag)
            if robot_section is None or not len(robot_section):
                continue
            target = section(world_root, tag)
            for child in list(robot_section):
                target.append(child)

        robot_worldbody = robot_root.find("worldbody")
        if robot_worldbody is not None:
            target = section(world_root, "worldbody")
            for child in list(robot_worldbody):
                target.append(child)

        return ET.tostring(world_root, encoding="unicode")

    @staticmethod
    def _absolutize_asset_paths(xml_text, base_dir):
        """Rewrite relative asset file paths to absolute ones.

        MuJoCo resolves relative asset paths against the process working
        directory when loading from a string (``MjModel.from_xml_string``),
        not against the XML file's own directory.  World MJCFs in the maps
        package reference meshes relative to their own location, so they
        must be made absolute before parsing.
        """
        try:
            root = ET.fromstring(xml_text)
        except ET.ParseError:
            return xml_text

        asset_tags = ("mesh", "hfield", "skin", "texture")
        changed = False
        for elem in root.iter():
            # Strip any namespace prefix for the tag comparison.
            tag = elem.tag.split("}")[-1]
            if tag in asset_tags:
                f = elem.get("file")
                if not f or f.startswith("package://") or os.path.isabs(f):
                    continue
                elem.set(
                    "file", os.path.normpath(os.path.join(base_dir, f))
                )
                changed = True
            elif tag == "include":
                f = elem.get("file")
                if f and not os.path.isabs(f):
                    elem.set(
                        "file", os.path.normpath(os.path.join(base_dir, f))
                    )
                    changed = True

        if not changed:
            return xml_text
        return ET.tostring(root, encoding="unicode")

    def _find_body_and_joints(self):
        m = self._model
        self._free_joint_qpos_adr = -1
        self._body_id = -1
        self._joint_name2id = {}
        self._joint_name2dofadr = {}
        self._joint_names = []

        for i in range(m.njnt):
            jtype = m.jnt_type[i]
            if jtype == mujoco.mjtJoint.mjJNT_FREE:
                self._free_joint_qpos_adr = m.jnt_qposadr[i]
                self._body_id = m.jnt_bodyid[i]
                break

        if self._body_id < 0:
            self._body_id = 0

        lw = self.get_parameter("left_wheel_joint").value
        rw = self.get_parameter("right_wheel_joint").value
        self._lw_qpos_adr = -1
        self._rw_qpos_adr = -1

        for i in range(m.njnt):
            jtype = m.jnt_type[i]
            if jtype == mujoco.mjtJoint.mjJNT_FREE:
                continue
            jname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
            if jname is None:
                jname = "joint_%d" % i
            jadr = m.jnt_qposadr[i]
            self._joint_name2id[jname] = jadr
            # qvel is indexed by DOF address, not qpos address — they differ
            # once a free joint (7 qpos / 6 dof) precedes the hinge joints.
            self._joint_name2dofadr[jname] = m.jnt_dofadr[i]
            self._joint_names.append(jname)
            if jname == lw:
                self._lw_qpos_adr = jadr
                self._lw_name = jname
            elif jname == rw:
                self._rw_qpos_adr = jadr
                self._rw_name = jname

        self.get_logger().info(
            "Joints: %s   lw=%s rw=%s"
            % (self._joint_names, self._lw_name, self._rw_name)
        )

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

            if self._lw_qpos_adr >= 0 and self._model.nu > 0:
                self._set_velocity_actuator(self._lw_name, vl)
            if self._rw_qpos_adr >= 0 and self._model.nu > 0:
                self._set_velocity_actuator(self._rw_name, vr)

            mujoco.mj_step(self._model, self._data)
            self._sim_step += 1
            self._sim_t = self._sim_step * self._dt

            bid = self._body_id
            self._bpos = list(self._data.xpos[bid])
            self._born = list(self._data.xquat[bid])

            cvel = list(self._data.cvel[bid])
            self._bang = cvel[:3]
            self._blin = cvel[3:6]

            self._jpos = []
            self._jvel = []
            for jn in self._joint_names:
                adr = self._joint_name2id.get(jn, -1)
                dadr = self._joint_name2dofadr.get(jn, -1)
                if adr >= 0:
                    self._jpos.append(float(self._data.qpos[adr]))
                    self._jvel.append(float(self._data.qvel[dadr]))
                else:
                    self._jpos.append(0.0)
                    self._jvel.append(0.0)

            if elapsed - last_pub >= pub_dt:
                last_pub = elapsed
                self._pub_joint_states()
                self._pub_odom()
                self._pub_imu()
                self._pub_clock()

            if elapsed - last_scan >= scan_dt:
                last_scan = elapsed
                self._pub_scan()

            if self._viewer is not None and self._viewer.is_running():
                self._viewer.sync()

            dt = time.monotonic() - now
            if dt < self._dt:
                time.sleep(self._dt - dt)

    def _set_velocity_actuator(self, joint_name, velocity):
        m = self._model
        for i in range(m.nu):
            aname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if aname and joint_name in aname:
                self._data.ctrl[i] = velocity
                return
            trnid = m.actuator_trnid[i]
            if trnid[0] >= 0:
                jname = mujoco.mj_id2name(
                    m, mujoco.mjtObj.mjOBJ_JOINT, trnid[0]
                )
                if jname == joint_name:
                    self._data.ctrl[i] = velocity
                    return

    # ------------------------------------------------------------------
    # ROS 2 publishers
    # ------------------------------------------------------------------
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
        self._js_pub.publish(m)

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
        m.orientation_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        m.angular_velocity = Vector3(x=self._bang[0], y=self._bang[1], z=self._bang[2])
        m.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        m.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)
        m.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        self._imu_pub.publish(m)

    def _on_reset(self, request, response):
        """Reset the simulation (R2.3)."""
        try:
            if self._body_id >= 0 and self._model is not None:
                # Reset robot position to spawn point.
                sx = self.get_parameter("spawn_x").value
                sy = self.get_parameter("spawn_y").value
                sz = self.get_parameter("spawn_z").value
                syaw = self.get_parameter("spawn_yaw").value
                orn = _rpy_to_quat(0, 0, syaw)
                # MuJoCo free joint qpos: [x, y, z, qw, qx, qy, qz]
                adr = self._free_joint_qpos_adr
                if adr >= 0:
                    self._data.qpos[adr:adr + 3] = [sx, sy, sz]
                    self._data.qpos[adr + 3:adr + 7] = [orn.w, orn.x, orn.y, orn.z]
                    # Reset velocity.
                    dadr = self._free_joint_qpos_adr
                    self._data.qvel[dadr:dadr + 6] = [0, 0, 0, 0, 0, 0]
                    mujoco.mj_forward(self._model, self._data)
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
        status.name = "mujoco_spawner"
        status.hardware_id = "mujoco"

        if self._ready and self._body_id >= 0:
            status.level = DiagnosticStatus.OK
            status.message = "running"
        elif self._body_id < 0:
            status.level = DiagnosticStatus.WARN
            status.message = "no robot loaded"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = "not ready"

        status.values.append(KeyValue(key="ready", value=str(self._ready)))
        status.values.append(
            KeyValue(key="body_id", value=str(self._body_id)))
        status.values.append(
            KeyValue(key="sim_t", value=f"{self._sim_t:.3f}"))
        status.values.append(
            KeyValue(key="model_source",
                     value=str(getattr(self, "_model_source", "unknown"))))
        msg.status.append(status)
        self._health_pub.publish(msg)

    def _pub_clock(self):
        clock_msg = RosClock()
        clock_msg.clock = self._stamp()
        self._clock_pub.publish(clock_msg)

    def _pub_scan(self):
        n = int(self.get_parameter("scan_samples").value)
        am = self.get_parameter("scan_range_min").value
        ax = self.get_parameter("scan_range_max").value
        ai = 2.0 * math.pi / n

        lp = np.array([
            self._bpos[0], self._bpos[1], self._bpos[2] + 0.12
        ], dtype=np.float64)

        qx, qy, qz, qw = self._born
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy ** 2 + qz ** 2)
        byaw = math.atan2(siny, cosy)

        geomid = np.zeros(1, dtype=np.int32)
        ranges = []
        for i in range(n):
            angle = byaw - math.pi + i * ai
            vec = np.array([
                math.cos(angle), math.sin(angle), 0.0
            ], dtype=np.float64)
            dist = mujoco.mj_ray(
                self._model, self._data, lp, vec, None, 1, -1, geomid
            )
            if 0.0 < dist < ax:
                ranges.append(max(am, float(dist)))
            else:
                ranges.append(float("inf"))

        msg = LaserScan()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.get_parameter("laser_link_name").value
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = ai
        msg.scan_time = 1.0 / max(
            self.get_parameter("scan_rate").value, 0.1
        )
        msg.range_min = am
        msg.range_max = ax
        msg.ranges = ranges
        self._scan_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        # NOTE: We intentionally do NOT call self._viewer.close() because
        # mujoco.viewer.launch_passive() creates a viewer on a separate
        # thread with its own OpenGL context.  Calling .close() on it
        # after the ROS shutdown sequence has begun causes a segfault on
        # some platforms (notably Jetson / ARM).  Instead we let the
        # viewer be reaped when the process exits.
        self._viewer = None
        super().destroy_node()


def main(args=None):
    import signal as _signal
    rclpy.init(args=args)
    node = MuJoCoSpawner()
    # Prevent Python-level KeyboardInterrupt traceback when the process is
    # killed with SIGINT — the shutdown path in destroy_node() handles cleanup.
    _signal.signal(_signal.SIGINT, _signal.SIG_DFL)
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
