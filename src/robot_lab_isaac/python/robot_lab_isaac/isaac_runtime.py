#!/usr/bin/env python3
"""Isaac Sim runtime child process (Robot Lab).

Runs under the dedicated Isaac Sim Python (3.12) virtual environment —
NOT under the ROS 2 interpreter (rclpy is 3.10-only on Humble, while
isaacsim >= 5.0 requires 3.12).  The parent ROS 2 node
(:mod:`robot_lab_isaac.isaac_spawner`) spawns this script and exchanges
line-delimited JSON over stdin/stdout:

* stdin  — commands:  {"cmd_vel": [linear_x, angular_z]}, {"reset": true}
                      /  EOF stops
* FIFO   — events:    {"event": "ready", "dofs": [...], "root_body": ...}
                      {"event": "state", "t": ..., "pos": ..., ...}
                      {"event": "scan", "t": ..., "ranges": [...]}
                      {"event": "rgbd", "t": ..., "rgb": b64, "depth": b64}
                      {"event": "error", "message": "..."}

The child owns SimulationApp, the stage (map meshes), the robot
articulation and the physics loop.
"""
import base64
import json
import math
import os
import struct
import sys
import threading

# Upper bound on SimulationApp.close() before the runtime exits hard.
_CLOSE_TIMEOUT_S = 5.0

# Damping of the wheel joints' velocity drive. A pure damping drive (zero
# stiffness) tracks a velocity target; the importer's default position
# stiffness instead holds the wheels at their initial angle.
_WHEEL_DRIVE_DAMPING = 1.0e4

# Rotor inertia added to driven wheel joints (kg*m^2).  Bumperbot's 53 g
# wheels have ~2e-5 kg*m^2 about the axle; under a velocity drive the PhysX
# articulation then swung the wheels between -24 and +31 rad/s for a 9 rad/s
# target.  0.005 is a small geared motor's reflected rotor inertia, the value
# the MuJoCo bridge uses for the same reason.
_WHEEL_ARMATURE = 0.005


def _wheel_velocities(linear, angular, radius, separation):
    """Differential-drive (left, right) wheel angular velocities in rad/s.

    The spawner forwards /cmd_vel as ``[linear_x, angular_z]``.  Applying
    those two numbers directly as the left and right wheel velocities made a
    0.3 m/s forward command turn only the left wheel at 0.3 rad/s, so the
    robot did not drive.
    """
    radius = max(float(radius), 1e-6)
    half_track = float(separation) / 2.0
    return ((float(linear) - float(angular) * half_track) / radius,
            (float(linear) + float(angular) * half_track) / radius)


def _isaac_quat_from_yaw(yaw):
    """Scalar-first (w, x, y, z) quaternion for a yaw, as Isaac Sim expects.

    Isaac Sim's core API is scalar-first.  The runtime used to pass ROS order
    (x, y, z, w), which turns a yaw of 0 into (w=0, x=0, y=0, z=1): every
    robot spawned rotated 180 degrees and a forward command drove it
    backwards.
    """
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def _ros_quat_from_isaac(quaternion):
    """Isaac's scalar-first (w, x, y, z) as ROS (x, y, z, w).

    The state events carry ROS order, which is what the spawner publishes on
    /odom/ground_truth and /imu/out; copying Isaac's order straight through
    scrambled every published orientation.
    """
    w, x, y, z = (float(v) for v in list(quaternion)[:4])
    return [x, y, z, w]


# Scan origin for a robot without a laser link, relative to its root body:
# the same 0.12 m above the base the PyBullet and MuJoCo bridges use.
_DEFAULT_SCAN_OFFSET = [[0.0, 0.0, 0.12], [1.0, 0.0, 0.0, 0.0]]


def _multiply_wxyz(a, b):
    aw, ax, ay, az = (float(v) for v in a)
    bw, bx, by, bz = (float(v) for v in b)
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def _mounted_origin_and_yaw(base_position, base_quaternion, offset):
    """World origin and heading of a sensor rigidly mounted on the base.

    *base_quaternion* and the offset's quaternion are scalar-first; *offset*
    is ``[[x, y, z], [w, x, y, z]]`` in the base frame.  This interpreter has
    no robot_lab_utils, so the two-line frame composition lives here.
    """
    offset_position, offset_quaternion = offset
    w, x, y, z = (float(v) for v in base_quaternion)
    vx, vy, vz = (float(v) for v in offset_position)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    rotated = (vx + w * tx + (y * tz - z * ty),
               vy + w * ty + (z * tx - x * tz),
               vz + w * tz + (x * ty - y * tx))
    origin = [float(b) + r for b, r in zip(base_position, rotated)]
    qw, qx, qy, qz = _multiply_wxyz(base_quaternion, offset_quaternion)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return origin, yaw


def _scan_ranges(cast, origin, yaw, samples, range_min, range_max,
                 ignored_bodies):
    """One planar 360-degree scan; angles run from -pi in the sensor frame.

    *cast(origin, direction, distance, report)* is PhysX ``raycast_all``.
    The rays start inside the sensor link's own collider, so hits on the
    robot's bodies are ignored.  No return is ``inf``, as in the PyBullet and
    MuJoCo bridges.
    """
    samples = max(int(samples), 1)
    increment = 2.0 * math.pi / samples
    ranges = []
    for index in range(samples):
        angle = yaw - math.pi + index * increment
        nearest = [float("inf")]

        def report(hit, nearest=nearest):
            if (hit.rigid_body not in ignored_bodies
                    and hit.distance < nearest[0]):
                nearest[0] = float(hit.distance)
            return True

        cast(tuple(origin), (math.cos(angle), math.sin(angle), 0.0),
             float(range_max), report)
        distance = nearest[0]
        ranges.append(max(float(range_min), distance)
                      if distance < float(range_max) else float("inf"))
    return ranges


def _articulation_root_body(robot):
    """Name of the articulation's root rigid body, or '' when unknown.

    ``get_world_pose`` reports this body, which the importer may choose
    differently from the URDF root link (Bumperbot: ``base_link``, not
    ``base_footprint``).
    """
    for owner in (getattr(robot, "_articulation_view", None), robot):
        try:
            return str(list(owner.body_names)[0])
        except Exception:
            continue
    return ""


def _spawn_body_pose(cfg, root_body):
    """Translate the requested URDF-root pose to PhysX's root rigid body."""
    position = [float(cfg.get("spawn_" + axis, 0.0)) for axis in "xyz"]
    quaternion = _isaac_quat_from_yaw(float(cfg.get("spawn_yaw", 0.0)))
    offset = cfg.get("root_offsets", {}).get(root_body)
    if offset is not None:
        position, _ = _mounted_origin_and_yaw(position, quaternion, offset)
        quaternion = _multiply_wxyz(quaternion, offset[1])
    return position, quaternion


def _prepare_scan(cfg, dt, robot, stage):
    """Settings for the planar scan, or None when scanning is off.

    The spawner sends the laser link's offset from each link that carries it;
    the one used is for the link Isaac reports as the articulation's root
    body, which the importer may pick differently from the URDF root.
    """
    scan_cfg = cfg.get("scan") or {}
    rate = float(scan_cfg.get("rate", 0.0) or 0.0)
    if rate <= 0.0:
        return None
    import carb
    from omni.physx import get_physx_scene_query_interface
    from pxr import UsdPhysics

    root_body = _articulation_root_body(robot)
    offsets = scan_cfg.get("offsets") or {}
    base = root_body if root_body in offsets else scan_cfg.get("urdf_root", "")
    offset = offsets.get(base) or _DEFAULT_SCAN_OFFSET
    ignored = {str(prim.GetPath()) for prim in stage.Traverse()
               if prim.HasAPI(UsdPhysics.RigidBodyAPI)
               and not str(prim.GetPath()).startswith("/World/map/")}
    query = get_physx_scene_query_interface()

    def cast(origin, direction, distance, report):
        query.raycast_all(carb.Float3(*origin), carb.Float3(*direction),
                          float(distance), report)

    every = max(1, int(round(1.0 / (dt * rate))))
    _emit({"event": "log",
           "msg": "Scan: %d rays every %d steps from offset %s of '%s' "
                  "(root body '%s'); %d robot bodies ignored"
                  % (int(scan_cfg.get("samples", 360)), every, offset[0],
                     base or "default", root_body or "?", len(ignored))})
    return {"cast": cast, "offset": offset, "every": every,
            "samples": int(scan_cfg.get("samples", 360)),
            "range_min": float(scan_cfg.get("range_min", 0.12)),
            "range_max": float(scan_cfg.get("range_max", 12.0)),
            "ignored": ignored}


# Rotation from a camera link (x forward, z up) to a USD camera, which looks
# along its -z axis with +y up; the same as camera_model.LINK_TO_OPENGL_CAMERA.
_LINK_TO_USD_CAMERA = (0.5, 0.5, -0.5, -0.5)


def _camera_apertures(horizontal_fov, width, height, focal_length):
    """(horizontal, vertical) USD apertures giving *horizontal_fov*.

    Only the aperture/focal-length ratio sets the field of view; square
    pixels make the vertical aperture follow the image aspect.
    """
    horizontal = 2.0 * float(focal_length) * math.tan(float(horizontal_fov) / 2.0)
    return horizontal, horizontal * float(height) / float(width)


def _encode_rgbd(rgba, depth, width, height, far):
    """JSON-safe RGB-D frame, or None while the annotators have no image yet.

    Colour is RGB uint8 and depth float32 metres along the optical axis with
    ``inf`` for no return, both base64 of their raw bytes.
    """
    import numpy as np

    rgba = np.asarray(rgba)
    depth = np.asarray(depth, dtype=np.float32)
    if rgba.size != width * height * 4 or depth.size != width * height:
        return None
    rgb = np.ascontiguousarray(
        rgba.reshape(height, width, 4)[:, :, :3], dtype=np.uint8)
    depth = depth.reshape(height, width).copy()
    depth[~np.isfinite(depth) | (depth <= 0.0) | (depth >= float(far))] = np.inf
    return {"width": int(width), "height": int(height),
            "rgb": base64.b64encode(rgb.tobytes()).decode("ascii"),
            "depth": base64.b64encode(depth.astype("<f4").tobytes()).decode("ascii")}


def _prepare_camera(cfg, dt, robot, stage):
    """USD camera plus replicator annotators, or None when there is no camera.

    The camera is parented to the camera link prim when the importer kept
    one, otherwise to the articulation's root body at the link's offset.
    """
    camera_cfg = cfg.get("camera") or {}
    rate = float(camera_cfg.get("rate", 0.0) or 0.0)
    if rate <= 0.0:
        return None
    from pxr import Gf, UsdGeom

    link = camera_cfg.get("link", "")
    root_body = _articulation_root_body(robot)
    parent, offset = None, [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]
    root_prim = None
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.startswith("/World/Physics") or path.startswith("/World/map"):
            continue
        if prim.GetName() == link and parent is None:
            parent = prim
        if prim.GetName() == root_body and root_prim is None:
            root_prim = prim
    if parent is None:
        offsets = camera_cfg.get("offsets") or {}
        if root_prim is None or root_body not in offsets:
            _emit({"event": "log",
                   "msg": "Camera link '%s' not found; no RGB-D camera" % link})
            return None
        parent, offset = root_prim, offsets[root_body]

    width, height = int(camera_cfg["width"]), int(camera_cfg["height"])
    focal_length = 18.0
    horizontal, vertical = _camera_apertures(
        camera_cfg["horizontal_fov"], width, height, focal_length)
    camera = UsdGeom.Camera.Define(
        stage, parent.GetPath().AppendChild("robot_lab_rgbd"))
    camera.CreateFocalLengthAttr(focal_length)
    camera.CreateHorizontalApertureAttr(horizontal)
    camera.CreateVerticalApertureAttr(vertical)
    camera.CreateClippingRangeAttr(Gf.Vec2f(float(camera_cfg["near"]),
                                            float(camera_cfg["far"])))
    xformable = UsdGeom.Xformable(camera.GetPrim())
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in offset[0]]))
    w, x, y, z = _multiply_wxyz(offset[1], _LINK_TO_USD_CAMERA)
    xformable.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))

    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("omni.replicator.core")
    import omni.replicator.core as rep
    render_product = rep.create.render_product(
        str(camera.GetPath()), (width, height))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
    rgb.attach([render_product])
    depth.attach([render_product])
    every = max(1, int(round(1.0 / (dt * rate))))
    _emit({"event": "log",
           "msg": "RGB-D camera %s: %dx%d every %d steps"
                  % (camera.GetPath(), width, height, every)})
    return {"rgb": rgb, "depth": depth, "every": every, "width": width,
            "height": height, "far": float(camera_cfg["far"])}


def _author_wheel_velocity_drives(stage, joint_names,
                                  damping=_WHEEL_DRIVE_DAMPING,
                                  armature=_WHEEL_ARMATURE):
    """Author zero-stiffness velocity drives on the named revolute joints.

    The joints also get rotor inertia (``physxJoint:armature``) when they
    carry none, which keeps a velocity drive on a light wheel stable.

    Isaac Sim 6's URDF importer places joints under ``<robot>/Physics``,
    beside the link hierarchy that holds the articulation root, so they are
    not descendants of the root prim; searching only under the root found
    none and the wheels kept their default drives.  The whole stage is
    searched instead.  Returns the prim paths that were driven.
    """
    from pxr import PhysxSchema, Usd, UsdPhysics

    names = {name for name in joint_names if name}
    driven = []
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if prim.GetName() not in names or prim.IsInstanceProxy():
            continue
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        (drive.GetStiffnessAttr() or drive.CreateStiffnessAttr()).Set(0.0)
        (drive.GetDampingAttr() or drive.CreateDampingAttr()).Set(float(damping))
        joint = PhysxSchema.PhysxJointAPI.Apply(prim)
        current = joint.GetArmatureAttr().Get() if joint.GetArmatureAttr() else None
        if armature and not current:
            (joint.GetArmatureAttr() or joint.CreateArmatureAttr()).Set(
                float(armature))
        driven.append(str(prim.GetPath()))
    return sorted(driven)


def _load_stl(path):
    """Return (vertices, triangles) for a binary or ASCII STL file."""
    with open(path, "rb") as fh:
        data = fh.read()
    if len(data) > 84:
        n_tri = struct.unpack_from("<I", data, 80)[0]
        if 84 + n_tri * 50 == len(data):
            verts = []
            off = 84
            for _ in range(n_tri):
                vals = struct.unpack_from("<12fH", data, off)
                verts.extend((vals[3:6], vals[6:9], vals[9:12]))
                off += 50
            tris = [(i, i + 1, i + 2) for i in range(0, len(verts), 3)]
            return [tuple(v) for v in verts], tris
    verts, tris, cur = [], [], []
    for line in data.decode("utf-8", "ignore").splitlines():
        line = line.strip()
        if line.startswith("vertex"):
            cur.append(tuple(float(v) for v in line.split()[1:4]))
            if len(cur) == 3:
                base = len(verts)
                verts.extend(cur)
                tris.append((base, base + 1, base + 2))
                cur = []
    return verts, tris


class _StdinReader(threading.Thread):
    """Reads JSON commands from stdin; sets stop on EOF."""

    daemon = True

    def __init__(self):
        super().__init__()
        self.cmd = [0.0, 0.0]
        self.stop = False
        self.reset_requested = False

    def run(self):
        try:
            for line in sys.stdin:
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except ValueError:
                    continue
                if "cmd_vel" in msg:
                    self.cmd = list(msg["cmd_vel"])[:2]
                elif msg.get("reset"):
                    # Sent by the spawner's /robot_lab/reset service, which
                    # used to report success while this reader ignored it.
                    self.reset_requested = True
                elif msg.get("cmd") == "shutdown":
                    self.stop = True
                    return
        except Exception:
            pass
        self.stop = True


_EVT_FIFO = os.environ.get("ISAAC_EVENT_FIFO", "")
_EVT_FILE = None
_EVT_OPENED = threading.Event()


def _open_evt():
    global _EVT_FILE
    if _EVT_FIFO:
        try:
            _EVT_FILE = open(_EVT_FIFO, "w", buffering=1)
            _EVT_OPENED.set()
            return
        except Exception:
            pass
    _EVT_FILE = None
    _EVT_OPENED.set()


def _emit(obj):
    line = json.dumps(obj)
    # Ensure the FIFO open has been attempted (non-blocking check)
    if not _EVT_OPENED.is_set() and _EVT_FIFO:
        threading.Thread(target=_open_evt, daemon=True).start()
        _EVT_OPENED.set()  # Mark as attempted to avoid re-spawning
    try:
        if _EVT_FILE is not None:
            _EVT_FILE.write(line + "\n")
            _EVT_FILE.flush()
            return
    except Exception:
        pass
    # Also write to stderr as fallback (Kit may not hijack stderr)
    try:
        sys.stderr.write(line + "\n")
        sys.stderr.flush()
    except Exception:
        pass


def _add_sdf_meshes(stage, world_path):
    """Create static mesh prims from the SDF world file."""
    import re
    import xml.etree.ElementTree as ET
    from ament_index_python.packages import get_package_share_directory
    from pxr import Gf, UsdGeom, UsdPhysics

    _emit({"event": "log", "msg": "Loading SDF world: %s" % world_path})
    tree = ET.parse(world_path)
    count = 0
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
        _emit({"event": "log", "msg": "Found mesh URI: %s" % uri})
        if not uri.startswith("package://"):
            continue
        pkg, _, rel = uri[len("package://"):].partition("/")
        try:
            abs_path = os.path.join(get_package_share_directory(pkg), rel)
        except Exception as e:
            _emit({"event": "log", "msg": "Package not found: %s (%s)" % (pkg, e)})
            continue
        if not os.path.isfile(abs_path):
            _emit({"event": "log", "msg": "Mesh file not found: %s" % abs_path})
            continue
        scale = [1.0, 1.0, 1.0]
        if scale_el is not None and scale_el.text:
            try:
                scale = [float(v) for v in scale_el.text.split()]
            except (ValueError, TypeError):
                pass
        _emit({"event": "log", "msg": "Loading STL: %s" % abs_path})
        verts, tris = _load_stl(abs_path)
        if not verts:
            _emit({"event": "log", "msg": "STL load returned no vertices: %s" % abs_path})
            continue
        name = re.sub(r"[^A-Za-z0-9_]", "_", rel)
        mesh = UsdGeom.Mesh.Define(stage, "/World/map_%s" % name)
        mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in verts])
        mesh.CreateFaceVertexCountsAttr([3] * len(tris))
        mesh.CreateFaceVertexIndicesAttr([i for t in tris for i in t])
        UsdGeom.XformCommonAPI(mesh.GetPrim()).SetScale(Gf.Vec3f(*scale))
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        count += 1
        _emit({"event": "log",
               "msg": "Loaded SDF mesh %s (%d tris, scale %s)"
               % (abs_path, len(tris), scale)})
    _emit({"event": "log", "msg": "SDF mesh loading complete: %d meshes" % count})


def run(cfg):
    from isaacsim import SimulationApp

    headless = not bool(cfg.get("gui", False)) or not os.environ.get("DISPLAY")
    app = SimulationApp({
        "headless": bool(headless),
        "width": 1280,
        "height": 720,
        "window_width": 1280,
        "window_height": 720,
    })
    reader = _StdinReader()
    reader.start()

    state = {"running": True}
    failed = False
    try:
        _run_stage(app, reader, cfg, state)
    except Exception as exc:
        failed = True
        _emit({"event": "error", "message": str(exc)[:500]})
    finally:
        # SimulationApp.close() can SIGABRT - or hang - during Kit teardown on
        # ARM; the state pipeline is done either way, so exit hard afterward.
        # The watchdog bounds a hung close: if the ROS spawner was killed
        # outright, stdin EOF is the only stop signal this process gets, and
        # nothing else would ever end it.
        watchdog = threading.Timer(
            _CLOSE_TIMEOUT_S, lambda: os._exit(1 if failed else 0))
        watchdog.daemon = True
        watchdog.start()
        try:
            app.close()
        except Exception:
            pass
        watchdog.cancel()
        _emit({"event": "exit"})
        try:
            sys.stdout.flush()
        except Exception:
            pass
        if _EVT_FILE is not None:
            try:
                _EVT_FILE.close()
            except Exception:
                pass
        os._exit(1 if failed else 0)


def _ensure_lighting(stage):
    """Add a dome and a sun light when the stage has no light of its own.

    The SDF worlds' <light> elements are not converted, and an unlit USD
    stage renders black: the RGB-D camera produced correct depth with an
    all-zero colour image.  Returns the prim paths added.
    """
    from pxr import Gf, UsdGeom, UsdLux

    if any(prim.HasAPI(UsdLux.LightAPI) for prim in stage.Traverse()):
        return []
    dome = UsdLux.DomeLight.Define(stage, "/World/robot_lab_lights/dome")
    dome.CreateIntensityAttr(600.0)
    sun = UsdLux.DistantLight.Define(stage, "/World/robot_lab_lights/sun")
    sun.CreateIntensityAttr(2500.0)
    sun.CreateAngleAttr(1.0)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 20.0, 0.0))
    return [str(dome.GetPath()), str(sun.GetPath())]


def _add_world_shapes(stage, shapes):
    """Create static collision prims from the spawner's world shapes.

    The ROS-side spawner parses the SDF world (robot_lab_utils.sdf_world)
    and hands over backend-neutral records, so primitives, the full pose
    chain and model:// includes are honoured without ROS in this
    interpreter.  Each prim carries translate -> orient -> scale ops.
    """
    import re
    from pxr import Gf, UsdGeom, UsdPhysics

    counts = {}
    for index, shape in enumerate(shapes):
        kind = shape.get("type")
        size = [float(v) for v in (shape.get("size") or [])]
        name = re.sub(r"[^A-Za-z0-9_]", "_",
                      "%s_%s_%d" % (shape.get("model", "map"), kind, index))
        path = "/World/map/%s" % name
        scale = Gf.Vec3f(1.0, 1.0, 1.0)
        if kind == "box":
            prim = UsdGeom.Cube.Define(stage, path)
            prim.CreateSizeAttr(1.0)  # unit cube scaled to full extents
            scale = Gf.Vec3f(*[max(v, 1e-6) for v in (size + [1.0] * 3)[:3]])
        elif kind == "plane":
            prim = UsdGeom.Cube.Define(stage, path)
            prim.CreateSizeAttr(1.0)  # thin box keeps the plane's pose/extent
            full = (size + [100.0, 100.0])[:2]
            scale = Gf.Vec3f(max(full[0], 1e-6), max(full[1], 1e-6), 0.01)
        elif kind == "sphere":
            prim = UsdGeom.Sphere.Define(stage, path)
            prim.CreateRadiusAttr((size + [0.5])[0])
        elif kind == "cylinder":
            prim = UsdGeom.Cylinder.Define(stage, path)
            radius, height = (size + [0.5, 1.0])[:2]
            prim.CreateRadiusAttr(radius)
            prim.CreateHeightAttr(height)
            prim.CreateAxisAttr("Z")
        elif kind == "mesh":
            verts, tris = _load_stl(shape.get("mesh", ""))
            if not verts:
                _emit({"event": "log",
                       "msg": "World mesh has no vertices: %s" % shape.get("mesh")})
                continue
            prim = UsdGeom.Mesh.Define(stage, path)
            prim.CreatePointsAttr([Gf.Vec3f(*v) for v in verts])
            prim.CreateFaceVertexCountsAttr([3] * len(tris))
            prim.CreateFaceVertexIndicesAttr([i for tri in tris for i in tri])
            scale = Gf.Vec3f(*[float(v) for v in
                               (shape.get("scale") or [1.0, 1.0, 1.0])[:3]])
        else:
            continue
        position = [float(v) for v in (shape.get("position") or [0, 0, 0])[:3]]
        w, x, y, z = [float(v) for v in
                      (shape.get("orientation") or [1, 0, 0, 0])[:4]]
        prim.AddTranslateOp().Set(Gf.Vec3d(*position))
        prim.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
        prim.AddScaleOp().Set(scale)
        UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
        if kind == "mesh":
            UsdPhysics.MeshCollisionAPI.Apply(prim.GetPrim()).CreateApproximationAttr("none")
        counts[kind] = counts.get(kind, 0) + 1
    _emit({"event": "log",
           "msg": "World shapes created: %s" % (
               ", ".join("%d %s" % (n, k) for k, n in sorted(counts.items()))
               or "none")})
    return counts


def _run_stage(app, reader, cfg, state):
    import isaacsim.core.utils.stage as stage_utils
    from isaacsim.core.api import World

    dt = 1.0 / max(float(cfg.get("physics_rate", 60.0)), 1.0)

    _emit({"event": "log", "msg": "Config received: world_path=%r world_stage=%r" % (cfg.get("world_path"), cfg.get("world_stage"))})

    if cfg.get("world_stage") and os.path.isfile(cfg["world_stage"]):
        _emit({"event": "log", "msg": "Opening USD stage: %s" % cfg["world_stage"]})
        stage_utils.open_stage(cfg["world_stage"])
    else:
        stage_utils.create_new_stage()
        if cfg.get("world_shapes") is not None:
            _add_world_shapes(stage_utils.get_current_stage(),
                              cfg["world_shapes"])
        elif cfg.get("world_path") and os.path.isfile(cfg["world_path"]):
            # Fallback when the spawner could not pre-parse the world.
            _emit({"event": "log", "msg": "Loading SDF world: %s" % cfg["world_path"]})
            _add_sdf_meshes(stage_utils.get_current_stage(), cfg["world_path"])
        else:
            _emit({"event": "log", "msg": "No world_path or file not found: %r" % cfg.get("world_path")})

    world = World(physics_dt=dt, stage_units_in_meters=1.0)
    try:
        world.scene.add_ground_plane()
    except Exception:
        pass
    try:
        lights = _ensure_lighting(stage_utils.get_current_stage())
        if lights:
            _emit({"event": "log", "msg": "Default lighting added: %s" % lights})
    except Exception as exc:
        _emit({"event": "log", "msg": "Lighting not added: %s" % exc})

    try:
        from isaacsim.asset.importer.urdf import URDFImporterConfig, URDFImporter
    except ImportError:
        URDFImporter = URDFImporterConfig = None

    robot_name = cfg.get("robot_name", "bumperbot")
    prim_path = "/World/%s" % robot_name

    robot_free = bool(cfg.get("robot_free")) or not cfg.get("urdf_file")
    root_prim = None
    if not robot_free:
        prim = None
        if URDFImporter is not None:
            # isaacsim 6.0 API: URDFImporter -> generated USD -> stage reference.
            try:
                imp_cfg = URDFImporterConfig(urdf_path=cfg["urdf_file"])
                imp_cfg.merge_fixed_joints = False
                imp_cfg.fix_base = False
                imp_cfg.collision_from_visuals = bool(
                    cfg.get("collision_from_visuals", False))
                imp_cfg.joint_drive_type = "force"
                usd_path = URDFImporter(imp_cfg).import_urdf()
                prim = stage_utils.add_reference_to_stage(usd_path, "/World")
            except Exception:
                prim = None
        if prim is None:
            # Legacy importer path (4.x/5.x style bindings).
            try:
                from isaacsim.asset.importer.urdf.impl import _urdf
            except ImportError:
                from omni.isaac.urdf import _urdf
            cfg_impl = _urdf.ImportConfig()
            cfg_impl.merge_fixed_joints = False
            cfg_impl.fix_base = False
            cfg_impl.import_inertia_tensor = True
            prim = _urdf.import_urdf(cfg_impl, cfg["urdf_file"],
                                     prim_path=prim_path)
        if prim is None:
            raise RuntimeError("URDF import failed (%s)" % cfg["urdf_file"])

    # Resolve the exact robot root: prefer /World/<robot_name>, else the
    # first child of /World carrying a PhysX ArticulationRoot API.
    from pxr import Usd, UsdPhysics
    stage_obj = stage_utils.get_current_stage()
    scan = None
    camera = None
    robot = None
    dof_names = []
    if not robot_free:
        root_prim = prim
        cand = stage_obj.GetPrimAtPath("/World/%s" % robot_name)
        if cand and cand.IsValid():
            root_prim = cand
        else:
            for cand_prim in Usd.PrimRange(stage_obj.GetPrimAtPath("/World")):
                if cand_prim.IsValid() and UsdPhysics.ArticulationRootAPI(cand_prim):
                    root_prim = cand_prim
                    break
        _emit({"event": "debug_prim", "path": str(prim.GetPath()),
               "root": str(root_prim.GetPath())})

    if not robot_free:
        # Author velocity drives on the wheel joints before the world reset, so
        # PhysX parses them with the articulation.
        driven = _author_wheel_velocity_drives(
            stage_obj, (cfg.get("left_wheel_joint", ""),
                        cfg.get("right_wheel_joint", "")))
        _emit({"event": "log",
               "msg": "Wheel velocity drives: %s" % (driven or "none found")})

        from isaacsim.core.api.robots import Robot
        syaw = float(cfg.get("spawn_yaw", 0.0))
        orn = _isaac_quat_from_yaw(syaw)
        robot = Robot(
            prim_path=str(root_prim.GetPath()), name=robot_name,
            position=(float(cfg.get("spawn_x", 0.0)),
                      float(cfg.get("spawn_y", 0.0)),
                      float(cfg.get("spawn_z", 0.0))),
            orientation=orn,
        )
        # The deprecated core API only initializes the PhysX articulation for
        # objects registered in the world scene.
        try:
            world.scene.add(robot)
        except Exception:
            pass
        world.reset()

        # Give the articulation a moment to initialize, then read the DOFs.
        try:
            robot.initialize()
        except Exception:
            pass
        try:
            dof_names = list(robot.dof_names)
        except Exception:
            dof_names = []
        # PhysX reports/moves the root rigid body, not necessarily the URDF
        # root frame selected in the GUI. Apply its fixed offset after the
        # importer has identified that body; preserve it for world.reset().
        spawn_position, spawn_orientation = _spawn_body_pose(
            cfg, _articulation_root_body(robot))
        robot.set_world_pose(position=spawn_position, orientation=spawn_orientation)
        robot.set_default_state(position=spawn_position, orientation=spawn_orientation)
        robot.set_linear_velocity([0.0, 0.0, 0.0])
        robot.set_angular_velocity([0.0, 0.0, 0.0])
    else:
        # Robot-free display: still step the world so the map renders; there
        # is simply no articulation to drive or report joints for.
        world.reset()
        _emit({"event": "ready", "dofs": [],
               "root_body": ""})
    lw = cfg.get("left_wheel_joint", "")
    rw = cfg.get("right_wheel_joint", "")
    lw_idx = dof_names.index(lw) if lw in dof_names else -1
    rw_idx = dof_names.index(rw) if rw in dof_names else -1
    if not robot_free:
        _emit({"event": "log", "msg": "Initial articulation pose %s; root body %s; URDF offset %s" % (
            robot.get_world_pose(), _articulation_root_body(robot),
            cfg.get("root_offsets", {}).get(_articulation_root_body(robot)))})
        _emit({"event": "ready", "dofs": dof_names,
               "root_body": _articulation_root_body(robot)})

    if not robot_free:
        try:
            scan = _prepare_scan(cfg, dt, robot, stage_obj)
        except Exception as exc:
            scan = None
            _emit({"event": "log", "msg": "Scan unavailable: %s" % exc})
        try:
            camera = _prepare_camera(cfg, dt, robot, stage_obj)
        except Exception as exc:
            camera = None
            _emit({"event": "log", "msg": "RGB-D camera unavailable: %s" % exc})

    wheel_radius = float(cfg.get("wheel_radius", 0.033))
    wheel_separation = float(cfg.get("wheel_separation", 0.17))
    action_error_reported = False
    sim_step = 0
    # Display hold: freeze joints at spawn so passive visualization of
    # legged/humanoid robots stays stable instead of collapsing under
    # gravity (the RViz "jump like crazy").  'true' always holds; 'auto'
    # holds only when there are no drive joints (the display case).
    hold_mode = str(cfg.get("hold_position", "auto") or "auto").lower()
    hold_joints = (hold_mode == "true") or (hold_mode == "auto" and lw_idx < 0 and rw_idx < 0)
    hold_pose = None
    if hold_joints and not robot_free and robot is not None:
        try:
            hold_pose = [float(v) for v in robot.get_joint_positions()]
            # Hold with stiff position targets each tick.
            from isaacsim.core.utils.types import ArticulationAction as _HoldAction
            robot.apply_action(_HoldAction(joint_positions=list(hold_pose)))
        except Exception:
            hold_pose = None
            hold_joints = False
    while state["running"] and not reader.stop and app.is_running():
        if reader.reset_requested:
            reader.reset_requested = False
            try:
                world.reset()
            except Exception as exc:
                _emit({"event": "log", "msg": "Reset failed: %s" % exc})
            else:
                # The step counter keeps running so /clock never goes back.
                _emit({"event": "log",
                       "msg": "Reset applied: world returned to its initial state"})
        linear, angular = reader.cmd
        vl, vr = _wheel_velocities(linear, angular, wheel_radius,
                                   wheel_separation)
        if hold_joints and not robot_free and robot is not None and hold_pose is not None:
            # Re-assert the frozen spawn pose instead of stepping physics
            # freely; drive joints still follow /cmd_vel below.
            try:
                from isaacsim.core.utils.types import ArticulationAction
                robot.apply_action(
                    ArticulationAction(joint_positions=list(hold_pose)))
            except Exception:
                pass
        try:
            from isaacsim.core.utils.types import ArticulationAction
            idx, vels = [], []
            if lw_idx >= 0:
                idx.append(lw_idx)
                vels.append(vl)
            if rw_idx >= 0:
                idx.append(rw_idx)
                vels.append(vr)
            if idx and robot is not None:
                robot.apply_action(
                    ArticulationAction(joint_velocities=vels,
                                       joint_indices=idx)
                )
        except Exception as exc:
            # Reported once: a silently swallowed failure here is exactly how
            # a robot that never moves looks healthy in every other signal.
            if not action_error_reported:
                action_error_reported = True
                _emit({"event": "log",
                       "msg": "Wheel command failed: %s" % exc})

        world.step(render=True)
        sim_step += 1
        t = sim_step * dt

        q = None
        if robot_free or robot is None:
            # No articulation: still advance the clock so the world renders.
            q = None
            pos = [0.0, 0.0, 0.0]
            orn = [0.0, 0.0, 0.0, 1.0]
            lin = [0.0, 0.0, 0.0]
            ang = [0.0, 0.0, 0.0]
            jpos = []
            jvel = []
        else:
            try:
                pose = robot.get_world_pose()
                pos = [float(v) for v in pose[0]]
                q = [float(v) for v in pose[1]]
                orn = _ros_quat_from_isaac(q)  # events carry ROS (x, y, z, w)
                lin = [float(v) for v in robot.get_linear_velocity()]
                ang = [float(v) for v in robot.get_angular_velocity()]
                jpos = [float(v) for v in robot.get_joint_positions()]
                jvel = [float(v) for v in robot.get_joint_velocities()]
            except Exception:
                q = None
                pos = [0.0, 0.0, 0.0]
                orn = [0.0, 0.0, 0.0, 1.0]
                lin = [0.0, 0.0, 0.0]
                ang = [0.0, 0.0, 0.0]
                jpos = [0.0] * len(dof_names)
                jvel = [0.0] * len(dof_names)

        _emit({"event": "state", "t": t, "pos": pos, "orn": orn,
               "lin": lin, "ang": ang, "jpos": jpos, "jvel": jvel})

        if scan is not None and q is not None and sim_step % scan["every"] == 0:
            origin, yaw = _mounted_origin_and_yaw(pos, q, scan["offset"])
            try:
                ranges = _scan_ranges(scan["cast"], origin, yaw,
                                      scan["samples"], scan["range_min"],
                                      scan["range_max"], scan["ignored"])
            except Exception as exc:
                _emit({"event": "log",
                       "msg": "Scan disabled after raycast failure: %s" % exc})
                scan = None
            else:
                _emit({"event": "scan", "t": t, "ranges": ranges})

        if camera is not None and sim_step % camera["every"] == 0:
            try:
                frame = _encode_rgbd(camera["rgb"].get_data(),
                                     camera["depth"].get_data(),
                                     camera["width"], camera["height"],
                                     camera["far"])
            except Exception as exc:
                _emit({"event": "log",
                       "msg": "RGB-D camera disabled after read failure: %s" % exc})
                camera = None
            else:
                if frame is not None:
                    frame.update(event="rgbd", t=t)
                    _emit(frame)

    state["running"] = False


if __name__ == "__main__":
    try:
        cfg = json.loads(sys.stdin.readline())
        run(cfg)
    except Exception as exc:  # report failures to the parent node
        _emit({"event": "error", "message": str(exc)[:500]})
        try:
            sys.stdout.flush()
        except Exception:
            pass
        if _EVT_FILE is not None:
            try:
                _EVT_FILE.close()
            except Exception:
                pass
        sys.exit(1)
