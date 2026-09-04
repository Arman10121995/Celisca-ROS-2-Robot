#!/usr/bin/env python3
"""Isaac Sim runtime child process (Robot Lab).

Runs under the dedicated Isaac Sim Python (3.12) virtual environment —
NOT under the ROS 2 interpreter (rclpy is 3.10-only on Humble, while
isaacsim >= 5.0 requires 3.12).  The parent ROS 2 node
(:mod:`robot_lab_isaac.isaac_spawner`) spawns this script and exchanges
line-delimited JSON over stdin/stdout:

* stdin  — commands:  {"cmd_vel": [linear_x, angular_z]}  /  EOF stops
* stdout — events:    {"event": "ready", "dofs": [...]}
                      {"event": "state", "t": ..., "pos": ..., ...}
                      {"event": "error", "message": "..."}

The child owns SimulationApp, the stage (map meshes), the robot
articulation and the physics loop.
"""
import json
import math
import os
import struct
import sys
import threading


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
        # SimulationApp.close() can SIGABRT during Kit teardown on ARM;
        # the state pipeline is done either way, so exit hard afterward.
        try:
            app.close()
        except Exception:
            pass
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
        if cfg.get("world_path") and os.path.isfile(cfg["world_path"]):
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
        from isaacsim.asset.importer.urdf import URDFImporterConfig, URDFImporter
    except ImportError:
        URDFImporter = URDFImporterConfig = None

    robot_name = cfg.get("robot_name", "bumperbot")
    prim_path = "/World/%s" % robot_name

    prim = None
    if URDFImporter is not None:
        # isaacsim 6.0 API: URDFImporter -> generated USD -> stage reference.
        try:
            imp_cfg = URDFImporterConfig(urdf_path=cfg["urdf_file"])
            imp_cfg.merge_fixed_joints = False
            imp_cfg.fix_base = False
            imp_cfg.collision_from_visuals = True
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

    from isaacsim.core.api.robots import Robot
    syaw = float(cfg.get("spawn_yaw", 0.0))
    orn = (0.0, 0.0, math.sin(syaw / 2.0), math.cos(syaw / 2.0))
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
    lw = cfg.get("left_wheel_joint", "")
    rw = cfg.get("right_wheel_joint", "")
    lw_idx = dof_names.index(lw) if lw in dof_names else -1
    rw_idx = dof_names.index(rw) if rw in dof_names else -1
    _emit({"event": "ready", "dofs": dof_names})

    sim_step = 0
    while state["running"] and not reader.stop and app.is_running():
        vl, vr = reader.cmd
        try:
            from isaacsim.core.utils.types import ArticulationAction
            idx, vels = [], []
            if lw_idx >= 0:
                idx.append(lw_idx)
                vels.append(vl)
            if rw_idx >= 0:
                idx.append(rw_idx)
                vels.append(vr)
            if idx:
                robot.apply_action(
                    ArticulationAction(joint_velocities=vels,
                                       joint_indices=idx)
                )
        except Exception:
            pass

        world.step(render=True)
        sim_step += 1
        t = sim_step * dt

        try:
            pose = robot.get_world_pose()
            pos = [float(v) for v in pose[0]]
            q = pose[1]
            orn = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
            lin = [float(v) for v in robot.get_linear_velocity()]
            ang = [float(v) for v in robot.get_angular_velocity()]
            jpos = [float(v) for v in robot.get_joint_positions()]
            jvel = [float(v) for v in robot.get_joint_velocities()]
        except Exception:
            pos = [0.0, 0.0, 0.0]
            orn = [0.0, 0.0, 0.0, 1.0]
            lin = [0.0, 0.0, 0.0]
            ang = [0.0, 0.0, 0.0]
            jpos = [0.0] * len(dof_names)
            jvel = [0.0] * len(dof_names)

        _emit({"event": "state", "t": t, "pos": pos, "orn": orn,
               "lin": lin, "ang": ang, "jpos": jpos, "jvel": jvel})

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
