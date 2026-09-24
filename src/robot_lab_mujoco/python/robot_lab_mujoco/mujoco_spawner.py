"""MuJoCo robot spawner node.

Loads a world + robot model, opens the MuJoCo passive viewer, runs the
physics step loop, and publishes the ROS 2 topics required by the
stack (joint_states, TF, odom, scan, imu, clock).
"""
import copy
import hashlib
import math
import os
import re
import signal
import shutil
import struct
import subprocess
import tempfile
import sys
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

from robot_lab_utils.drive_kinematics import drive_from_config, parse_drive_config
from robot_lab_utils.process_lifetime import exit_with_parent
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as RosClock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger

from robot_lab_utils import camera_model
from robot_lab_mujoco.joint_effort import JointEffortCommand, mjcf_joint_dynamics
from robot_lab_utils.camera_msgs import camera_info_msg, image_msg
from robot_lab_utils.ros_frames import publish_fallback_scan_frame
from robot_lab_utils.sim_frames import (
    urdf_link_frames,
    compose, mounted_pose, offset_from_root, wxyz_from_xyzw, xyzw_from_wxyz,
    yaw_of)

_CAMERA_NAME = "robot_lab_rgbd"

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
    _WORLD_MAX_STL_FACES,
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


def _urdf_root_inertial(urdf_text):
    """Return the URDF root link's ``<inertial>`` as MJCF attributes (or None).

    The MuJoCo URDF importer welds the root link to the world and DROPS its
    ``<inertial>``, so the floating-base wrapper built on top of the export
    would otherwise infer mass/inertia from its collision geoms. For the
    Berkeley Humanoid Lite that produced a 4.830 kg uniform-box base at the
    box centre instead of the URDF's 4.444 kg with its tensor and CoM 35 mm
    lower - a plant the walking policy was not trained on.
    """
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError:
        return None
    links = {link.get("name"): link for link in root.findall("link")}
    children = {joint.find("child").get("link")
                for joint in root.findall("joint")
                if joint.find("child") is not None}
    roots = [name for name in links if name not in children]
    if len(roots) != 1:
        return None
    inertial = links[roots[0]].find("inertial")
    if inertial is None:
        return None
    mass_elem = inertial.find("mass")
    inertia = inertial.find("inertia")
    if mass_elem is None or inertia is None:
        return None
    mass = float(mass_elem.get("value"))
    if not math.isfinite(mass) or mass <= 0:
        return None
    values = []
    for element in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"):
        value = float(inertia.get(element, 0.0))
        if not math.isfinite(value):
            return None
        values.append(value)
    origin = inertial.find("origin")
    xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
    rpy = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
    pos = [float(v) for v in xyz.split()]
    angles = [float(v) for v in rpy.split()]
    if len(pos) != 3 or len(angles) != 3:
        return None
    attrs = {
        "pos": " ".join("%.10g" % v for v in pos),
        "mass": "%.10g" % mass,
    }
    roll, pitch, yaw = angles
    if any(abs(v) > 1e-12 for v in angles):
        # MJCF <inertial quat> orients the fullinertia frame (ZYX rpy here,
        # the URDF fixed-frame convention), (w, x, y, z) order.
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        attrs["quat"] = "%.10g %.10g %.10g %.10g" % (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
    # MJCF fullinertia order is (ixx, iyy, izz, ixy, ixz, iyz).
    attrs["fullinertia"] = " ".join("%.10g" % v for v in values)
    return attrs


def _add_floating_base(mjcf_text, robot_name="robot", root_inertial=None):
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
    # Restore the URDF root link's inertial on the wrapper (see
    # _urdf_root_inertial): the importer dropped it, and a wrapper without an
    # explicit inertial gets its mass/inertia INFERRED from its collision
    # geoms - for the BHL a 4.830 kg uniform box at the box centre instead of
    # the URDF's 4.444 kg with its tensor and CoM 35 mm lower.
    if root_inertial:
        ET.SubElement(base, "inertial", root_inertial)
    else:
        # Fallback for importers that DO keep the root inertial on the first
        # movable child (the welded root-link body): hoist it so the wrapper
        # does not end up with two inertials or lose it entirely.
        first = movable[0]
        if first.tag == "body":
            root_inertial_elem = first.find("inertial")
            if root_inertial_elem is not None:
                base.append(copy.deepcopy(root_inertial_elem))
    for child in movable:
        worldbody.remove(child)
        base.append(child)
    worldbody.append(base)
    return ET.tostring(root, encoding="unicode")


def _emit_log(logger, level, message):
    """Log *message* at *level* through an rclpy (or stdlib-style) logger.

    rclpy caches the severity used at each logging call site and raises
    "Logger severity cannot be changed between calls" when the same site logs
    at a different level.  Routing every level through one
    ``getattr(logger, level)(message)`` line therefore made any warning
    followed by an info message abort the spawn: the Berkeley Humanoid Lite
    logs an inertia-repair warning and then "import OK", and failed on every
    attempt.  Each severity has its own call site here.
    """
    if logger is None:
        return
    if level == "error":
        logger.error(message)
    elif level in ("warning", "warn"):
        logger.warning(message)
    elif level == "debug":
        logger.debug(message)
    else:
        logger.info(message)


def _exclude_rest_pose_self_contacts(mjcf_text, logger=None):
    """Exclude robot body pairs that already interpenetrate at the rest pose.

    MuJoCo only auto-excludes parent<->child contacts. Some vendored
    descriptions ship collision meshes that overlap between bodies further
    apart in the tree: the Unitree H1-2 hand starts with its 1.9 g thumb
    links 3-5 mm inside the wrist link (a grandparent), so the very first
    contact force on a near-massless finger drives the solver to
    "Nan, Inf or huge value in QACC" on the first step.

    An overlap that exists at the model's own reference configuration is a
    description defect, not a collision the robot can resolve, so exactly
    those pairs get a <contact><exclude/>.  Legitimate self-collision (legs,
    arms meeting during motion) stays enabled, and masses, inertias and
    joint dynamics are left untouched.
    """


    if not hasattr(mujoco, "MjModel"):
        return mjcf_text
    try:
        root = ET.fromstring(mjcf_text)
        model = mujoco.MjModel.from_xml_string(mjcf_text)
    except Exception:
        return mjcf_text

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    pairs = set()
    for index in range(data.ncon):
        contact = data.contact[index]
        if contact.dist >= -1e-4:  # touching, not interpenetrating
            continue
        body1 = model.geom_bodyid[contact.geom1]
        body2 = model.geom_bodyid[contact.geom2]
        if body1 == 0 or body2 == 0 or body1 == body2:
            continue
        name1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body1)
        name2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body2)
        if name1 and name2:
            pairs.add(tuple(sorted((name1, name2))))
    if not pairs:
        return mjcf_text

    contact_section = root.find("contact")
    if contact_section is None:  # an empty element is falsy; compare to None
        contact_section = ET.SubElement(root, "contact")
    for name1, name2 in sorted(pairs):
        ET.SubElement(contact_section, "exclude",
                      {"body1": name1, "body2": name2})
    _emit_log(logger, "warning",
         "MuJoCo: excluded %d self-contact pair(s) that interpenetrate at the "
         "rest pose: %s" % (len(pairs), ", ".join("%s/%s" % pair
                                                  for pair in sorted(pairs))))
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


    try:
        root = ET.fromstring(mjcf_text)
    except ET.ParseError:
        return mjcf_text

    assets = [mesh for mesh in root.iter("mesh") if mesh.get("file")]
    if not assets:
        return mjcf_text

    from robot_lab_utils.mesh_assets import stage_world_mesh
    for mesh in assets:
        source = mesh.get("file") or ""
        mesh.set("file", stage_world_mesh(source, mujoco_format=True))
        # These are visual/raycast triangles; rigid flex below owns contact.
        # A detailed convex hull adds cost but cannot represent an open room.
        mesh.set("maxhullvert", "4")

    from robot_lab_mujoco.world_collision import add_static_mesh_collisions
    collision_meshes = add_static_mesh_collisions(root)
    if collision_meshes:
        _emit_log(logger, "info", "MuJoCo static triangle collision: %d mesh(es)." % collision_meshes)

    return ET.tostring(root, encoding="unicode")


def _native_joint_dynamics(model_path, names):
    """Read joint losses from the robot's own MJCF next to its description.

    The URDF's ``<dynamics damping="5.0">`` is a Gazebo stabiliser, not a
    property of the robot: its own MJCF - the model its balance law and
    walking policy were validated on - declares ``frictionloss="0.1"``,
    ``armature="0.005"`` and no damping at all.  A plant imported from the
    URDF therefore ran different joint dynamics than the controllers expect
    (the R5.3 walking-policy plant divergence), so the effort path mirrors
    the MJCF values instead.

    Looks for ``mjcf/*.xml`` beside the description directory (a scene file
    only ``<include>``s the robot file, so parsing it yields no joint element
    and it is skipped naturally) and returns ``(path, {joint: {loss: value}})``
    for the first file that names every configured effort joint, or
    ``(None, None)`` when no file does.
    """
    directory = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(str(model_path)))),
        "mjcf")
    if not os.path.isdir(directory):
        return None, None
    wanted = set(names)
    for entry in sorted(os.listdir(directory)):
        if not entry.endswith(".xml"):
            continue
        path = os.path.join(directory, entry)
        try:
            with open(path, "r") as handle:
                losses = mjcf_joint_dynamics(handle.read())
        except (ET.ParseError, OSError, ValueError):
            continue
        if wanted <= set(losses):
            return path, {name: losses[name] for name in names}
    return None, None


def _hide_collision_proxies(mjcf_text, logger=None):
    """Move collision geoms into MuJoCo's hidden collision group (3).

    MuJoCo's URDF import keeps the physics geometry (URDF ``<collision>``) in
    geom group 0 — a group the viewer draws — next to the visual meshes it
    puts in group 1.  A viewer therefore draws every link twice: the mesh and
    its collision proxy, which for these descriptions is an axis-rotated
    box/cylinder (reported as "the robot is copied twice and pasted
    orthogonally").  Group 3 is the conventional collision group: hidden by
    default, one checkbox away in the viewer's Rendering tab, and irrelevant
    to physics.  Bodies with no visual geometry keep their geoms visible, so
    a primitive-only description does not turn invisible.
    """
    try:
        root = ET.fromstring(mjcf_text)
    except ET.ParseError:
        return mjcf_text
    hidden = 0
    for body in root.iter("body"):
        geoms = body.findall("geom")
        visual = [geom for geom in geoms
                  if geom.get("contype") == "0"
                  and geom.get("conaffinity") == "0"]
        if not visual:
            continue
        for geom in geoms:
            if geom in visual:
                continue
            if geom.get("group") in (None, "", "0"):
                geom.set("group", "3")
                hidden += 1
    if not hidden:
        return mjcf_text
    _emit_log(logger, "info",
              "MuJoCo: %d collision proxy geom(s) moved to group 3 "
              "(viewer Rendering tab to show them)." % hidden)
    return ET.tostring(root, encoding="unicode")


def _contact_geoms(urdf_text):
    """Keep explicitly authored link friction through fixed-link fusion."""
    from robot_lab_utils.urdf_contact import gazebo_link_friction
    friction = gazebo_link_friction(urdf_text)
    root = ET.fromstring(urdf_text)
    geoms = {}
    for link in root.findall('link'):
        if link.get('name') not in friction:
            continue
        for index, collision in enumerate(link.findall('collision')):
            name = collision.get('name') or link.get('name') + '_contact_' + str(index)
            collision.set('name', name)
            geoms[name] = min(1.0, max(0.0, friction[link.get('name')]))
    return ET.tostring(root, encoding='unicode'), geoms


def _apply_contact_geoms(mjcf_text, friction):
    root = ET.fromstring(mjcf_text)
    for geom in root.iter('geom'):
        if geom.get('name') in friction:
            geom.set('friction', '%g 0.005 0.0001' % friction[geom.get('name')])
            # Otherwise MuJoCo combines the caster's low friction with the
            # floor using max(), turning a rolling support into a sticky skid.
            geom.set('priority', '1')
    return ET.tostring(root, encoding='unicode')


def _build_mjcf_from_urdf(urdf_text, pkg_map, logger=None, robot_name="",
                          base_dir=""):
    """Convert a prepared URDF into MJCF via MuJoCo's URDF importer.

    Mesh staging (package:// resolution, DAE->STL conversion) happens first
    so the importer sees a self-contained model.  Failures are reported
    with the real MuJoCo error; the generic diff-drive template is a loud
    last resort, never a silent substitution (which previously made most
    robots appear as a box in the viewer).
    """


    if mujoco is None or not hasattr(mujoco, "MjSpec"):
        _emit_log(logger, "error", "mujoco (>=3.2 with MjSpec) not importable; "
                      "using fallback MJCF template.")
        return _FALLBACK_MJCF

    tmp = None
    try:
        cache_dir = _mesh_staging_dir(robot_name, urdf_text)
        urdf_text, notes, placeholders = _stage_meshes(
            urdf_text, pkg_map, cache_dir, base_dir=base_dir)
        for note in notes:
            _emit_log(logger, "warning", "MuJoCo mesh staging: " + note)
        repaired = urdf_text
        urdf_text = _repair_urdf_inertias(urdf_text)
        if urdf_text != repaired:
            _emit_log(logger, "warning", "MuJoCo inertia repair: clamped non-physical "
                            "mass/inertia entries.")
        urdf_text = _inject_mujoco_compiler(urdf_text, cache_dir)
        urdf_text, contact_friction = _contact_geoms(urdf_text)
        urdf_text = _strip_gazebo_tags(urdf_text)

        tmp = tempfile.NamedTemporaryFile(
            suffix=".urdf", delete=False, mode="w"
        )
        tmp.write(urdf_text)
        tmp.close()
        spec = mujoco.MjSpec.from_file(tmp.name)
        mjcf = _add_floating_base(
            spec.to_xml(), robot_name or "robot",
            root_inertial=_urdf_root_inertial(urdf_text))
        mjcf = _apply_contact_geoms(mjcf, contact_friction)
        mjcf = _hide_collision_proxies(mjcf, logger=logger)
        mjcf = _exclude_rest_pose_self_contacts(mjcf, logger=logger)
        if placeholders:
            _emit_log(logger, "warning",
                 "MuJoCo URDF import OK with %d placeholder geom(s); "
                 "some meshes could not be converted." % placeholders)
        else:
            _emit_log(logger, "info", "MuJoCo URDF import OK (all meshes staged).")
        return mjcf
    except Exception as exc:
        _emit_log(logger, "error",
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

def _add_wheel_velocity_actuators(mjcf_text, joint_names, force_limit=5.0,
                                  armature=0.005):
    """Give the named wheel joints velocity actuators when they have none.

    MuJoCo's URDF importer creates no actuators, so every imported wheeled
    robot ignored /cmd_vel: the control loop drives a velocity actuator per
    wheel joint and found none.  The force limit mirrors the PyBullet
    bridge's 5 N*m velocity control.

    The driven joints also get rotor inertia (``armature``) when they have
    none.  A bare 53 g wheel has ~3e-5 kg*m^2 about its axle, and a velocity
    servo with gain kv is only stable while kv * timestep / inertia stays
    well below 2; at MuJoCo's 2 ms step it was ~130, the wheels spun up to
    300 rad/s from a zero command and the robot tumbled.  0.005 kg*m^2 is a
    small geared motor's reflected rotor inertia.  Joints that are absent or
    already actuated are left alone.
    """
    try:
        root = ET.fromstring(mjcf_text)
    except ET.ParseError:
        return mjcf_text
    hinges = {joint.get("name") for joint in root.iter("joint")
              if joint.get("name") and joint.get("type", "hinge") == "hinge"}
    for joint in root.iter("joint"):
        if joint.get("name") in joint_names and joint.get("name") in hinges \
                and not joint.get("armature"):
            joint.set("armature", "%g" % armature)
    actuator = root.find("actuator")
    actuated = set()
    if actuator is not None:
        actuated = {child.get("joint") for child in actuator
                    if child.get("joint")}
    missing = [name for name in joint_names
               if name in hinges and name not in actuated]
    if not missing:
        return mjcf_text
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    for name in missing:
        ET.SubElement(actuator, "velocity", {
            "name": name + "_velocity", "joint": name, "kv": "1",
            "ctrllimited": "true", "ctrlrange": "-50 50",
            "forcelimited": "true",
            "forcerange": "%g %g" % (-force_limit, force_limit),
        })
    return ET.tostring(root, encoding="unicode")


def _add_steer_position_actuators(mjcf_text, joint_names, kp=50.0,
                                  force_limit=10.0, armature=0.01):
    """Position servos for a car's steering joints (none are imported).

    Critically damped (dampratio 1), with rotor inertia on the joint so the
    servo stays stable at the physics step (see the wheel actuators).
    """
    try:
        root = ET.fromstring(mjcf_text)
    except ET.ParseError:
        return mjcf_text
    hinges = {joint.get("name"): joint for joint in root.iter("joint")
              if joint.get("name") and joint.get("type", "hinge") == "hinge"}
    missing = [name for name in joint_names if name in hinges]
    if not missing:
        return mjcf_text
    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    for name in missing:
        if not hinges[name].get("armature"):
            hinges[name].set("armature", "%g" % armature)
        ET.SubElement(actuator, "position", {
            "name": name + "_position", "joint": name, "kp": "%g" % kp,
            "dampratio": "1", "forcelimited": "true",
            "forcerange": "%g %g" % (-force_limit, force_limit),
        })
    return ET.tostring(root, encoding="unicode")


def _add_camera_to_base(mjcf_text, name, offset, fovy_degrees):
    """Attach a fixed camera to the floating base at a camera link's pose.

    *offset* is the camera link (x forward, z up) in the base frame, from
    ``sim_frames.offset_from_root``; MJCF cameras look along -z with +y up.
    The URDF importer fuses fixed links into the base, so the camera goes on
    the base body.  Unchanged when there is no floating base or the camera
    already exists.
    """
    try:
        root = ET.fromstring(mjcf_text)
    except ET.ParseError:
        return mjcf_text
    if any(camera.get("name") == name for camera in root.iter("camera")):
        return mjcf_text
    base = None
    for body in root.iter("body"):
        if body.find("freejoint") is not None or any(
                joint.get("type") == "free" for joint in body.findall("joint")):
            base = body
            break
    if base is None:
        return mjcf_text
    position, quaternion = compose(
        offset, ((0.0, 0.0, 0.0), camera_model.LINK_TO_OPENGL_CAMERA))
    ET.SubElement(base, "camera", {
        "name": name, "mode": "fixed",
        "pos": "%.6g %.6g %.6g" % tuple(position),
        "quat": "%.6g %.6g %.6g %.6g" % tuple(quaternion),
        "fovy": "%.6g" % fovy_degrees,
    })
    return ET.tostring(root, encoding="unicode")


def _body_frame_velocity(model, data, body_id):
    """(angular, linear) velocity of a body's frame origin, in that frame.

    This is what the odometry twist and the IMU rates mean.  ``cvel`` is
    world-aligned and taken about the subtree centre of mass, and
    ``mjOBJ_BODY`` reports in the body's inertial frame, which is rotated
    whenever the description's <inertial> is (Bumperbot's is, by
    rpy="0 0.25 0.3"); ``mjOBJ_XBODY`` is the body frame itself.
    """
    velocity = np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY, body_id,
                             velocity, 1)
    return ([float(v) for v in velocity[:3]],
            [float(v) for v in velocity[3:6]])


# Display hold stiffness (N*m/rad) and the stability bound on it.  An
# explicit spring on a light link is stable only while kp * dt^2 / I stays
# well below 1.  Held joints get rotor inertia (armature, as a geared motor
# has) so an ankle carrying a whole humanoid can be stiff enough without
# its light foot link limiting the spring; the damping is critical for each
# spring and integrated implicitly.
_HOLD_STIFFNESS = 3000.0
_HOLD_STABILITY = 0.3
_HOLD_ARMATURE = 0.05


def _add_joint_hold_springs(model, data, joint_names):
    """Spring-damp each named hinge/slide joint to its current position.

    Uses MuJoCo's passive joint springs (``jnt_stiffness``/``qpos_spring``)
    and dof damping, so the hold runs inside every physics step.  Returns
    the number of joints held.
    """
    timestep = float(model.opt.timestep)
    held = 0
    for name in joint_names:
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        # jnt_type is a NumPy int, which never equals the mjtJoint enum.
        if joint < 0 or int(model.jnt_type[joint]) not in (
                int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
            continue
        dof = model.jnt_dofadr[joint]
        qpos = model.jnt_qposadr[joint]
        added = max(0.0, _HOLD_ARMATURE - float(model.dof_armature[dof]))
        model.dof_armature[dof] += added
        inertia = max(float(model.dof_M0[dof]) + added, 1e-9)
        stiffness = min(_HOLD_STIFFNESS, _HOLD_STABILITY * inertia / timestep ** 2)
        model.jnt_stiffness[joint] = stiffness
        model.qpos_spring[qpos] = float(data.qpos[qpos])
        model.dof_damping[dof] = max(float(model.dof_damping[dof]),
                                     2.0 * math.sqrt(stiffness * inertia))
        held += 1
    return held


def _physics_substeps(period, timestep):
    """Model timesteps per physics tick, so physics time keeps pace."""
    return max(1, int(round(float(period) / max(float(timestep), 1e-9))))


def _ray_skipping_robot(model, data, origin, direction, robot_root, max_range,
                        geomid):
    """Distance along a ray to the first geom not belonging to the robot.

    The scan starts inside the sensor link's own geometry, and a fixed link
    may survive as a separate body rather than being fused into the base, so
    ``mj_ray``'s single ``bodyexclude`` is not enough: hits on any body under
    *robot_root* are stepped past.  Returns -1 when nothing is hit.
    """
    start = np.array(origin, dtype=np.float64)
    travelled = 0.0
    for _ in range(16):
        dist = mujoco.mj_ray(model, data, start, direction, None, 1, -1, geomid)
        if dist < 0.0 or geomid[0] < 0:
            return -1.0
        if model.body_rootid[model.geom_bodyid[geomid[0]]] != robot_root:
            return travelled + dist
        step = dist + 1e-4
        travelled += step
        if travelled >= max_range:
            return -1.0
        start = start + direction * step
    return -1.0


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
        self.declare_parameter("effort_controller_config", "")
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
        # The robot's whole drive block as JSON (drive_kinematics); empty
        # for the differential-drive parameters above.
        self.declare_parameter("drive_config", "")
        self.declare_parameter("laser_link_name", "laser_link")
        self.declare_parameter("scan_samples", 360)
        self.declare_parameter("scan_range_min", 0.12)
        self.declare_parameter("scan_range_max", 12.0)
        # Display-mode hold: 'auto' holds joints only for the map-free display
        # case, 'true' always holds joints at their spawn pose (passive
        # visualization for humanoids that would otherwise fall/jump under
        # gravity), 'false' runs full physics.  Bringup forwards
        # mode:=display automatically.
        self.declare_parameter("hold_position", "false")
        # RGB-D camera, rendered from the description's camera link with the
        # Gazebo sensor's intrinsics; 0 Hz disables it.  Rendering needs an
        # OpenGL context (GLFW with a display here).
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
        self._drive = self._make_drive()
        self._actuator_ids = {}
        self._effort_command = None
        self._effort_actuators = []
        self._effort_subscription = None
        # A reset service and the physics/rendering thread share MjData.
        self._physics_lock = threading.RLock()
        self._last_cmd_time = time.monotonic()
        self._watchdog_timeout = 0.5  # stop if no cmd_vel for 500ms
        self._sim_t = 0.0
        self._sim_step = 0
        self._bpos = [0.0, 0.0, 0.0]
        self._born = [0.0, 0.0, 0.0, 1.0]
        self._blin = [0.0, 0.0, 0.0]
        self._bang = [0.0, 0.0, 0.0]
        self._laser_offset = None  # laser link pose in the free body's frame
        self._base_frame = "base_footprint"  # odometry child: URDF root link
        self._camera = None
        self._jpos = []
        self._jvel = []
        self._running = True
        self._dt = 1.0 / max(self.get_parameter("physics_rate").value, 1.0)
        self._substeps = 1  # model timesteps per physics tick

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
        self._last_mux_cmd_time = 0.0
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Twist, "/robot_lab_controller/cmd_vel_unstamped",
                                 self._on_mux_cmd, 10)

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

    def _on_cmd(self, msg):
        if time.monotonic() - getattr(self, "_last_mux_cmd_time", 0.0) < 1.0:
            return
        with self._twist_lock:
            self._twist = msg
            self._last_cmd_time = time.monotonic()

    def _on_mux_cmd(self, msg):
        self._last_mux_cmd_time = time.monotonic()
        with self._twist_lock:
            self._twist = msg
            self._last_cmd_time = self._last_mux_cmd_time

    def _on_joint_effort(self, msg):
        with self._twist_lock:
            valid = self._effort_command.receive(msg.data, time.monotonic())
        if not valid:
            self.get_logger().error("Rejected invalid joint-effort command; cleared all efforts")

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

        self._robot_free = not model or str(model).strip().lower() == "none"
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
            # The free-joint body carries the URDF root link's frame, so the
            # scan origin is the laser link's pose in that frame.
            self._laser_offset = offset_from_root(
                urdf, self.get_parameter("laser_link_name").value)
            self._base_frame = urdf_link_frames(urdf)[1] or "base_footprint"
            if self._laser_offset is None:
                # The scan is cast from above the root link; give it that frame.
                self._scan_frame_tf = publish_fallback_scan_frame(
                    self, self._base_frame, self.get_parameter("laser_link_name").value)
            robot_mjcf = _build_mjcf_from_urdf(
                urdf, pkg_map, logger=self.get_logger(),
                robot_name=self.get_parameter("robot_name").value,
                base_dir=os.path.dirname(os.path.abspath(str(model))))
            if robot_mjcf != _FALLBACK_MJCF:
                self._model_source = "urdf"
                effort_config = self.get_parameter("effort_controller_config").value
                if effort_config:
                    try:
                        self._effort_command = JointEffortCommand(effort_config, urdf)
                    except ValueError as exc:
                        # The launch picks the Berkeley Humanoid Lite effort
                        # profile by model path, which also matches the
                        # legs-only biped: its missing arm joints made every
                        # spawn fail.  Run without the effort input instead.
                        _emit_log(self.get_logger(), "warning",
                                  "Effort controller profile %s does not fit "
                                  "this robot (%s); running without joint "
                                  "effort input." % (effort_config, exc))
                        self._effort_command = None
                if self._effort_command is not None:
                    # Mirror the robot's own MJCF joint losses instead of the
                    # URDF's Gazebo-substitute damping (see
                    # _native_joint_dynamics): the balance law and walking
                    # policy were validated on the MJCF plant.
                    path, native = _native_joint_dynamics(
                        str(model), self._effort_command.names)
                    if native is None:
                        _emit_log(self.get_logger(), "warning",
                                  "MuJoCo effort plant: no MJCF beside %s names "
                                  "every effort joint; keeping the URDF "
                                  "<dynamics> (damping=5.0 is a Gazebo "
                                  "stabiliser the robot does not have)." % model)
                    else:
                        _emit_log(self.get_logger(), "info",
                                  "MuJoCo effort plant: joint losses mirrored "
                                  "from the robot's own MJCF (%s)." % path)
                    robot_mjcf = self._effort_command.add_actuators(
                        robot_mjcf, native)
                robot_mjcf = _add_wheel_velocity_actuators(
                    robot_mjcf, self._drive.wheel_joints)
                robot_mjcf = _add_steer_position_actuators(
                    robot_mjcf, self._drive.steer_joints)
                camera_offset = offset_from_root(
                    urdf, self.get_parameter("camera_link_name").value)
                if camera_offset is not None \
                        and self.get_parameter("camera_rate").value > 0:
                    robot_mjcf = _add_camera_to_base(
                        robot_mjcf, _CAMERA_NAME, camera_offset,
                        math.degrees(camera_model.vertical_fov(
                            self.get_parameter("camera_horizontal_fov").value,
                            self.get_parameter("camera_width").value,
                            self.get_parameter("camera_height").value)))
        elif self._robot_free:
            robot_mjcf = "<mujoco><worldbody/></mujoco>"
            self._model_source = "world_only"
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
        self._substeps = _physics_substeps(self._dt, self._model.opt.timestep)
        self._camera = self._camera_setup()

        # --- find the root body that contains the free joint ---
        self._find_body_and_joints()
        if self._effort_command is not None:
            self._effort_actuators = [mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, name + "_effort")
                for name in self._effort_command.names]
            if min(self._effort_actuators) < 0:
                raise ValueError("configured effort actuator is missing")
            if self._effort_subscription is None:
                self._effort_subscription = self.create_subscription(
                    Float64MultiArray, self._effort_command.topic, self._on_joint_effort, 1)
            self.get_logger().info("Joint effort input: %s (%d joints, URDF limits)" % (
                self._effort_command.topic, len(self._effort_actuators)))

        # Spawn and reset use the same pose and initial physics state.
        self._reset_physics()

        # Display-mode hold: joints keep their spawn pose through physics,
        # not by freezing the state.  Each held joint gets a passive
        # spring-damper to its spawn angle, so a legged or humanoid robot
        # stands instead of collapsing, while the simulation still moves it:
        # it settles onto the floor, sags under load and responds to pushes
        # (e.g. dragging it in the viewer), and /joint_states reports that
        # motion to RViz.  The previous kinematic hold clamped qpos and
        # pinned the base every tick, so the published joints never changed.
        # 'auto' holds robots without drive wheels, 'true' holds every
        # robot, 'false' leaves all joints free.
        hold_mode = str(self.get_parameter("hold_position").value or "auto").lower()
        has_drive = (getattr(self, "_lw_qpos_adr", -1) >= 0
                     or getattr(self, "_rw_qpos_adr", -1) >= 0)
        self._hold_joints = (hold_mode == "true") or (hold_mode == "auto" and not has_drive)
        if self._hold_joints:
            held = _add_joint_hold_springs(
                self._model, self._data,
                [name for name in self._joint_names
                 if name not in (self._lw_name, self._rw_name)
                 and name not in self._drive.wheel_joints
                 and name not in self._drive.steer_joints])
            self.get_logger().info(
                "Display hold active (hold_position=%s): %d joint(s) held at "
                "their spawn pose by spring-dampers" % (hold_mode, held))

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
        camera_dt = 1.0 / max(self._camera["rate"], 0.1) if self._camera else 0.0
        last_pub = 0.0
        last_scan = 0.0
        last_camera = 0.0
        while self._running and rclpy.ok():
            now = time.monotonic()

            with self._physics_lock:
                self._step_physics()
                # Sensor rates describe simulated time. Scheduling the camera
                # by wall time made a slow render immediately trigger another
                # render, starving physics of all but one step per frame.
                sim_time = self._sim_t
                if self._sim_step == 1:
                    # Publish immediately after spawn or a clock-reset service.
                    last_pub = sim_time - pub_dt
                    last_scan = sim_time - scan_dt
                    last_camera = sim_time - camera_dt
                try:
                    if sim_time - last_pub + 1e-9 >= pub_dt:
                        last_pub = sim_time
                        if not getattr(self, "_robot_free", False):
                            self._pub_joint_states()
                            self._pub_odom()
                            self._pub_imu()
                        self._pub_clock()

                    if sim_time - last_scan + 1e-9 >= scan_dt:
                        last_scan = sim_time
                        if not getattr(self, "_robot_free", False):
                            self._pub_scan()

                    if self._camera is not None and sim_time - last_camera + 1e-9 >= camera_dt:
                        last_camera = sim_time
                        self._pub_camera()
                except Exception:
                    # Launch's SIGINT can shut ROS down before this thread.
                    if not rclpy.ok():
                        break
                    raise

                if self._viewer is not None and self._viewer.is_running():
                    self._viewer.sync()

            dt = time.monotonic() - now
            if dt < self._dt:
                time.sleep(self._dt - dt)

    def _step_physics(self):
        """Apply the current command and advance one tick under the physics lock."""
        with self._twist_lock:
            command = self._twist
            stale = (time.monotonic() - self._last_cmd_time) > self._watchdog_timeout
        if stale:
            command = Twist()
        if self._model.nu > 0 and not getattr(self, "_robot_free", False):
            targets = self._drive.targets(
                command.linear.x, command.angular.z, dt=self._dt)
            for joint, rate in targets.velocity.items():
                self._set_actuator(joint, "_velocity", max(-50.0, min(50.0, rate)))
            for joint, angle in targets.position.items():
                self._set_actuator(joint, "_position", angle)
        if self._effort_command is not None:
            with self._twist_lock:
                values = self._effort_command.command(time.monotonic(), self._watchdog_timeout)
            self._data.ctrl[self._effort_actuators] = values
        for _ in range(self._substeps):
            mujoco.mj_step(self._model, self._data)
        self._sim_step += 1
        self._read_physics_state()

    def _read_physics_state(self):
        """Refresh publisher state from the model after a step or reset."""
        self._sim_t = float(self._data.time)
        self._bpos = list(self._data.xpos[self._body_id])
        self._born = xyzw_from_wxyz(self._data.xquat[self._body_id])
        self._bang, self._blin = _body_frame_velocity(
            self._model, self._data, self._body_id)
        self._jpos = [float(self._data.qpos[self._joint_name2id[name]])
                      for name in self._joint_names]
        self._jvel = [float(self._data.qvel[self._joint_name2dofadr[name]])
                      for name in self._joint_names]

    def _reset_physics(self):
        """Restore all model state, spawn pose and a stopped command."""
        mujoco.mj_resetData(self._model, self._data)
        if hasattr(self._drive, "reset"):
            self._drive.reset()
        adr = self._free_joint_qpos_adr
        if adr >= 0:
            self._data.qpos[adr:adr + 3] = [
                self.get_parameter("spawn_" + axis).value for axis in ("x", "y", "z")]
            yaw = self.get_parameter("spawn_yaw").value
            # Free-joint quaternion is (w, x, y, z); yaw rotates about z.
            self._data.qpos[adr + 3:adr + 7] = [
                math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
        with self._twist_lock:
            self._twist = Twist()
            self._last_cmd_time = 0.0
            if self._effort_command is not None:
                self._effort_command.clear()
        self._sim_step = 0
        mujoco.mj_forward(self._model, self._data)
        # Place the lowest robot collision surface above the ground plane.
        # A trunk-rooted robot and a wheel-footprint-rooted robot cannot use
        # the same literal root z when the GUI selects a zero-height spawn.
        if adr >= 0:
            bottom = float("inf")
            for geom_id in range(self._model.ngeom):
                body_id = int(self._model.geom_bodyid[geom_id])
                while body_id > 0 and body_id != self._body_id:
                    body_id = int(self._model.body_parentid[body_id])
                if body_id != self._body_id or not (
                        self._model.geom_contype[geom_id] or self._model.geom_conaffinity[geom_id]):
                    continue
                bounds = self._model.geom_aabb[geom_id]
                z_row = self._data.geom_xmat[geom_id].reshape(3, 3)[2]
                z_min = self._data.geom_xpos[geom_id, 2] + z_row @ bounds[:3] - abs(z_row) @ bounds[3:]
                bottom = min(bottom, float(z_min))
            lift = max(0.0, .002 - bottom)
            self._data.qpos[adr + 2] += lift
            if lift > .001:
                self.get_logger().info("Spawn ground clearance: raised root by %.4f m" % lift)
            mujoco.mj_forward(self._model, self._data)
        self._read_physics_state()

    def _set_actuator(self, joint_name, suffix, value):
        """Set the control of the actuator the bridge added for a drive joint."""
        key = joint_name + suffix
        index = self._actuator_ids.get(key)
        if index is None:
            index = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, key)
            self._actuator_ids[key] = index
        if index >= 0:
            self._data.ctrl[index] = value
        elif suffix == "_velocity":
            self._set_velocity_actuator(joint_name, value)

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
        m.effort = [float(self._data.qfrc_actuator[self._joint_name2dofadr[name]])
                    for name in self._joint_names]
        self._js_pub.publish(m)

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
            with self._physics_lock:
                if self._body_id >= 0 and self._model is not None:
                    self._reset_physics()
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

        base_q = wxyz_from_xyzw(self._born)
        if self._laser_offset is not None:
            origin, laser_q = mounted_pose(
                self._bpos, base_q, self._laser_offset)
        else:
            # No laser link in the description: same fallback as PyBullet.
            origin = (self._bpos[0], self._bpos[1], self._bpos[2] + 0.12)
            laser_q = base_q
        lp = np.array(origin, dtype=np.float64)
        # Angles are in the laser frame; Bumperbot's laser is mounted facing
        # backwards, so the base heading is not the scan heading.
        byaw = yaw_of(laser_q)

        geomid = np.zeros(1, dtype=np.int32)
        ranges = []
        for i in range(n):
            angle = byaw - math.pi + i * ai
            vec = np.array([
                math.cos(angle), math.sin(angle), 0.0
            ], dtype=np.float64)
            dist = _ray_skipping_robot(
                self._model, self._data, lp, vec, self._body_id, ax, geomid)
            if 0.0 < dist < ax:
                ranges.append(max(am, float(dist)))
            else:
                ranges.append(float("inf"))

        msg = LaserScan()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self.get_parameter("laser_link_name").value
        msg.angle_min = -math.pi
        # n samples span n - 1 increments; angle_max = pi claimed n + 1.
        msg.angle_max = -math.pi + (n - 1) * ai
        msg.angle_increment = ai
        msg.scan_time = 1.0 / max(
            self.get_parameter("scan_rate").value, 0.1
        )
        msg.range_min = am
        msg.range_max = ax
        msg.ranges = ranges
        self._scan_pub.publish(msg)

    def _camera_setup(self):
        """Camera settings and publishers, or None when there is no camera."""
        link = self.get_parameter("camera_link_name").value
        camera_id = mujoco.mj_name2id(
            self._model, mujoco.mjtObj.mjOBJ_CAMERA, _CAMERA_NAME)
        if camera_id < 0:
            if self.get_parameter("camera_rate").value > 0:
                self.get_logger().info(
                    "Robot has no '%s' link; no RGB-D camera is published."
                    % link)
            return None
        camera = {
            "rate": float(self.get_parameter("camera_rate").value),
            "frame": self.get_parameter("camera_optical_frame").value,
            "width": int(self.get_parameter("camera_width").value),
            "height": int(self.get_parameter("camera_height").value),
            "horizontal_fov": float(
                self.get_parameter("camera_horizontal_fov").value),
            "near": float(self.get_parameter("camera_near").value),
            "far": float(self.get_parameter("camera_far").value),
            "renderer": None,
        }
        # Clip planes are global in MuJoCo, as fractions of the model extent.
        extent = max(float(self._model.stat.extent), 1e-6)
        self._model.vis.map.znear = camera["near"] / extent
        self._model.vis.map.zfar = camera["far"] / extent
        self._rgb_pub = self.create_publisher(
            Image, camera_model.OAKD["rgb_topic"], 5)
        self._depth_pub = self.create_publisher(
            Image, camera_model.OAKD["depth_topic"], 5)
        self._camera_info_pub = self.create_publisher(
            CameraInfo, camera_model.OAKD["info_topic"], 5)
        self.get_logger().info(
            "RGB-D camera on '%s': %dx%d at %.1f Hz"
            % (link, camera["width"], camera["height"], camera["rate"]))
        return camera

    def _pub_camera(self):
        camera = self._camera
        if camera["renderer"] is None:
            # Created on the physics thread, which owns the GL context.
            try:
                camera["renderer"] = mujoco.Renderer(
                    self._model, camera["height"], camera["width"])
            except Exception as exc:
                self.get_logger().warn(
                    "RGB-D camera disabled: MuJoCo could not create an "
                    "OpenGL context (%s). On this host GLFW needs a display."
                    % exc)
                self._camera = None
                return
        renderer = camera["renderer"]
        renderer.update_scene(self._data, camera=_CAMERA_NAME)
        rgb = renderer.render()
        renderer.enable_depth_rendering()
        renderer.update_scene(self._data, camera=_CAMERA_NAME)
        depth = renderer.render().astype(np.float32)
        renderer.disable_depth_rendering()
        depth[depth >= camera["far"] * (1.0 - 1e-6)] = np.inf
        stamp = self._stamp()
        self._rgb_pub.publish(image_msg(stamp, camera["frame"], rgb, "rgb8"))
        self._depth_pub.publish(
            image_msg(stamp, camera["frame"], depth, "32FC1"))
        self._camera_info_pub.publish(camera_info_msg(
            stamp, camera["frame"], camera["width"], camera["height"],
            camera["horizontal_fov"]))

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
    # Die with the launch that started this node, even if it never sends a
    # stop signal (see robot_lab_utils.process_lifetime).
    exit_with_parent()
    rclpy.init(args=args)
    node = MuJoCoSpawner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # A stop from launch or the GUI, not a failure.  SIGINT used to be
        # reset to its default action here, which killed the process before
        # cleanup and made every stop report "process has died".
        pass
    finally:
        # A group SIGINT reaches this process directly and again when launch
        # forwards it. Do not interrupt the physics-thread join and leave native
        # MuJoCo work running while Python tears down the process.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        had_viewer = getattr(node, "_viewer", None) is not None
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        if had_viewer:
            # The passive viewer's render thread still owns its OpenGL
            # context; Python's interpreter teardown then segfaults on the
            # Jetson (exit -11, "process has died" on every stop).  Cleanup
            # is complete here, so leave without that teardown.
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(0)
