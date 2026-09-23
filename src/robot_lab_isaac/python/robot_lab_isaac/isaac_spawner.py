"""Isaac Sim robot spawner node (ROS 2 side).

The node does not run Isaac Sim in-process: ``rclpy`` on ROS 2 Humble is
Python-3.10-only while ``isaacsim`` >= 5.0 requires Python 3.12.  Instead
it spawns :mod:`robot_lab_isaac.isaac_runtime` under the dedicated Isaac
Sim virtual environment (parameter ``isaac_python``) and exchanges
clock/state/commands over a JSON event FIFO plus stdin commands.  The
node publishes the ROS 2 topic contract (joint_states, odom, imu, scan,
RGB-D, clock) and forwards /cmd_vel to the runtime child.

If the Isaac Sim python environment is missing or the child fails, the
node logs a clear message and the rest of the launch graph continues in
offline mode.

.. note::
    Isaac Sim aarch64 builds are officially supported by NVIDIA only on
    DGX Spark systems.  On Jetson (AGX Orin) the wheel installs but Kit
    may abort during startup ("Cannot calculate frequency: TSC ran
    backwards").  In that case the node falls back to offline mode.
"""
import base64
import json
import math
import os
import re
import signal
import subprocess
import tempfile
import threading
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
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
from robot_lab_utils.sim_frames import (
    body_odometry, mounted_sensor_offsets, relative_frame, urdf_link_frames)

# TF is published by the EKF (odom→base_footprint), not by the simulator spawner.

# Portable default: the Isaac python path may be supplied per host via the
# ISAAC_PYTHON environment variable (or the ``isaac_python`` parameter by
# launch files)). Empty default => offline mode (R1.1/R1.2: no
# host-specific absolute paths in source.).
def _default_isaac_python():
    """Interpreter to run the Isaac runtime child with.

    ISAAC_PYTHON still wins; when it is unset a local Isaac Sim installation
    is detected, so a host that has Isaac installed can select it in the GUI
    without exporting anything first.
    """
    try:
        from robot_lab_utils.isaac_env import find_isaac_python
    except ImportError:  # pragma: no cover - robot_lab_utils always present
        return os.environ.get("ISAAC_PYTHON", "")
    return find_isaac_python()


_DEFAULT_ISAAC_PY = _default_isaac_python()


def _xacro_to_urdf(xacro_path):
    result = subprocess.run(
        ["xacro", xacro_path], capture_output=True, text=True, timeout=30,
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


def decode_rgbd_event(event):
    """``(t, rgb, depth)`` arrays from a runtime ``rgbd`` event.

    The runtime sends RGB uint8 and float32 depth as base64 of raw bytes.
    """
    width, height = int(event["width"]), int(event["height"])
    rgb = np.frombuffer(base64.b64decode(event["rgb"]), dtype=np.uint8)
    depth = np.frombuffer(base64.b64decode(event["depth"]), dtype="<f4")
    return (float(event.get("t", 0.0)), rgb.reshape(height, width, 3),
            depth.reshape(height, width).astype(np.float32))


def _group_alive(pgid):
    """True while any process remains in process group *pgid*."""
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _terminate_runtime(proc, graceful_timeout=2.0, term_timeout=2.0,
                       kill_timeout=5.0):
    """Stop the Isaac runtime and every process it started.

    The runtime is started through Isaac's ``python.sh``, which runs the real
    interpreter as a child instead of ``exec``-ing it.  Killing only
    ``proc`` therefore killed the shell wrapper and orphaned Kit (~11 GB and
    the GPU).  The runtime is started in its own session, so the whole
    process group can be signalled.

    Escalation is graceful (stdin EOF lets the runtime close Kit) ->
    SIGTERM -> SIGKILL, and the first two stages together stay below
    ros2 launch's own SIGINT->SIGTERM escalation, so the spawner finishes
    cleaning up before launch starts killing it.

    Returns how the group ended: "not-running", "graceful", "sigterm",
    "sigkill" or "unkillable".
    """
    if proc is None:
        return "not-running"
    pgid = proc.pid  # start_new_session=True makes the runtime a group leader
    if proc.poll() is not None and not _group_alive(pgid):
        return "not-running"

    def wait_for_group(timeout):
        deadline = time.monotonic() + timeout
        while True:
            proc.poll()  # reap the leader so it does not linger as a zombie
            if not _group_alive(pgid):
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)

    try:
        if proc.stdin is not None:
            proc.stdin.close()
    except Exception:
        pass
    if wait_for_group(graceful_timeout):
        return "graceful"
    for sig, timeout, outcome in ((signal.SIGTERM, term_timeout, "sigterm"),
                                  (signal.SIGKILL, kill_timeout, "sigkill")):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return outcome
        except PermissionError:
            pass
        if wait_for_group(timeout):
            return outcome
    return "unkillable"


class IsaacSpawner(Node):
    """Spawn robot + map into Isaac Sim via the runtime child process."""

    def __init__(self):
        super().__init__("isaac_spawner")
        self.declare_parameter("world_stage", "")
        self.declare_parameter("world_path", "")
        self.declare_parameter("robot_name", "bumperbot")
        self.declare_parameter("robot_package", "robot_lab_robots")
        self.declare_parameter("robot_xacro", "")
        self.declare_parameter("model", "")
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_z", 0.0)
        self.declare_parameter("spawn_yaw", 0.0)
        self.declare_parameter("gui", True)
        self.declare_parameter("hold_position", "false")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("isaac_python", _DEFAULT_ISAAC_PY)
        self.declare_parameter("left_wheel_joint", "wheel_left_joint")
        self.declare_parameter("right_wheel_joint", "wheel_right_joint")
        # Same differential-drive geometry parameters as the PyBullet and
        # MuJoCo spawners, so /cmd_vel means the same thing in every backend.
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)
        # URDF import: use the description's own <collision> geometry, as
        # Gazebo, PyBullet and MuJoCo do.  Colliders built from the visual
        # meshes gave Bumperbot faceted convex-hull wheels and casters that
        # bounced and dragged (0.10 rad/s yaw drift on a straight command);
        # with the URDF spheres it drives straight and turns at the command.
        self.declare_parameter("collision_from_visuals", False)
        # Planar scan, same parameters as the PyBullet and MuJoCo spawners.
        self.declare_parameter("laser_link_name", "laser_link")
        self.declare_parameter("scan_rate", 5.0)
        self.declare_parameter("scan_samples", 360)
        self.declare_parameter("scan_range_min", 0.12)
        self.declare_parameter("scan_range_max", 12.0)
        # RGB-D camera rendered from the description's camera link with the
        # Gazebo sensor's intrinsics, as in the PyBullet and MuJoCo bridges;
        # 0 Hz disables it.
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

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        # Ground-truth odometry on /odom/ground_truth (perfect, from physics).
        # The fused estimate lives on /odom (published by the EKF).
        self._odom_pub = self.create_publisher(Odometry, "/odom/ground_truth", 10)
        self._imu_pub = self.create_publisher(Imu, "/imu/out", 10)
        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
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

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

        wall = Clock(clock_type=ClockType.SYSTEM_TIME)
        self._timer = self.create_timer(0.5, self._try_spawn, clock=wall)
        rate = 1.0 / max(self.get_parameter("publish_rate").value, 1.0)
        self._pub_timer = self.create_timer(rate, self._publish, clock=wall)

        self._proc = None
        self._fifo_path = None
        self._lock = threading.Lock()
        self._dofs = []
        self._state = None
        self._spawned = False
        self._twist = Twist()
        self._urdf_text = ""
        self._camera = None  # camera settings once the runtime is launched
        self._camera_error_reported = False
        self._root_offset = None  # Isaac's root body in the URDF root frame

    # ------------------------------------------------------------------
    def _on_cmd(self, msg):
        self._twist = msg
        proc = self._proc
        if proc is not None and proc.stdin is not None:
            try:
                proc.stdin.write(json.dumps(
                    {"cmd_vel": [msg.linear.x, msg.angular.z]}) + "\n")
                proc.stdin.flush()
            except Exception:
                pass

    def _try_spawn(self):
        if self._spawned:
            return
        self._timer.cancel()
        # The parameter default already carries the detected interpreter;
        # an explicitly empty value means "do not start Isaac", so it is
        # honoured rather than being re-detected behind the caller's back.
        isaac_py = self.get_parameter("isaac_python").value
        if not isaac_py or not os.path.isfile(str(isaac_py)):
            self.get_logger().warn(
                "Isaac Sim python (%r) not found - running in offline "
                "mode.  Install isaacsim (see README) or set ISAAC_PYTHON / "
                "the isaac_python parameter." % isaac_py
            )
            self._spawned = True
            return
        try:
            self._launch_runtime(str(isaac_py))
            self._spawned = True
        except Exception as e:
            self.get_logger().error("Spawn failed: %s" % e)
            self._timer = self.create_timer(
                2.0, self._try_spawn,
                clock=Clock(clock_type=ClockType.SYSTEM_TIME),
            )

    def _launch_runtime(self, isaac_py):
        from ament_index_python.packages import get_package_share_directory

        model = self.get_parameter("model").value
        if not model:
            pkg = self.get_parameter("robot_package").value
            xacro = self.get_parameter("robot_xacro").value
            if pkg and xacro:
                model = os.path.join(get_package_share_directory(pkg), xacro)
        robot_free = (not model) or str(model).strip().lower() == "none"
        self._robot_free = robot_free
        urdf_file = ""
        urdf = ""
        if not robot_free:
            if not os.path.isfile(str(model)):
                raise RuntimeError("robot model not found: %r" % model)
            urdf_text = _xacro_to_urdf(model)
            pkg_map = {}
            for pkg_name in re.findall(r"\$\(find\s+([^)]+)\)", urdf_text):
                try:
                    pkg_map[pkg_name] = get_package_share_directory(pkg_name)
                except Exception:
                    pass
            rp = self.get_parameter("robot_package").value
            if rp and rp not in pkg_map:
                pkg_map[rp] = get_package_share_directory(rp)
            urdf = _strip_gazebo_tags(_rewrite_package_uris(urdf_text, pkg_map))
            self._urdf_text = urdf
            fd, urdf_file = tempfile.mkstemp(suffix=".urdf", dir=tempfile.gettempdir())
            with os.fdopen(fd, "w") as fh:
                fh.write(urdf)
            self.get_logger().info("URDF written to %s" % urdf_file)
        else:
            # Robot-free display: no description to expand, world only.
            self._urdf_text = ""
            self.get_logger().info("Robot-free run: loading world without a robot")

        root_offsets = {}
        if not robot_free:
            frames, root = urdf_link_frames(self._urdf_text)
            root_offsets = {name: relative_frame(frames, name, root) for name in frames}

        gui = self.get_parameter("gui").value
        if isinstance(gui, str):
            gui = gui.lower() in ("true", "1", "yes")
        cfg = {
            "world_stage": str(self.get_parameter("world_stage").value or ""),
            "world_path": str(self.get_parameter("world_path").value or ""),
            "urdf_file": urdf_file,
            "robot_name": self.get_parameter("robot_name").value,
            "robot_free": bool(robot_free),
            "root_offsets": root_offsets,
            "spawn_x": float(self.get_parameter("spawn_x").value),
            "spawn_y": float(self.get_parameter("spawn_y").value),
            "spawn_z": float(self.get_parameter("spawn_z").value),
            "spawn_yaw": float(self.get_parameter("spawn_yaw").value),
            "gui": bool(gui and os.environ.get("DISPLAY")),
            "physics_rate": 60.0,
            # Display hold forwarded to the runtime: freeze joints at spawn.
            "hold_position": str(self.get_parameter("hold_position").value or "auto"),
            "left_wheel_joint": self.get_parameter("left_wheel_joint").value,
            "right_wheel_joint": self.get_parameter("right_wheel_joint").value,
            "wheel_radius": float(self.get_parameter("wheel_radius").value),
            "wheel_separation": float(
                self.get_parameter("wheel_separation").value),
            "collision_from_visuals": bool(
                self.get_parameter("collision_from_visuals").value),
            "scan": self._scan_config(urdf),
            "camera": self._camera_config(urdf),
        }
        # The world is parsed here, on the ROS side, and handed over as plain
        # records: the runtime's interpreter has no ROS package index, and its
        # own SDF reader ignored primitives, poses and model:// includes.
        cfg["world_shapes"] = self._world_shapes(cfg["world_path"])

        runtime_py = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "isaac_runtime.py"
        )
        self.get_logger().info(
            "Launching Isaac runtime: %s %s (gui=%s)"
            % (isaac_py, runtime_py, cfg["gui"])
        )

        env = dict(os.environ)
        try:
            from robot_lab_utils.isaac_env import isaac_preload_libraries
            env["LD_PRELOAD"] = isaac_preload_libraries(
                isaac_py, env.get("LD_PRELOAD", ""))
        except ImportError:  # pragma: no cover - robot_lab_utils always present
            env.setdefault("LD_PRELOAD", "/lib/aarch64-linux-gnu/libgomp.so.1")
        self.get_logger().info("Isaac runtime LD_PRELOAD: %s" % env["LD_PRELOAD"])
        env.setdefault("ACCEPT_EULA", "YES")
        env.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

        # Dedicated event FIFO: Kit hijacks/closes the child's stdout, so
        # structured state flows through a named pipe instead.
        self._fifo_path = tempfile.mkstemp(
            prefix="isaac_evt_", dir=tempfile.gettempdir())[1]
        os.unlink(self._fifo_path)
        os.mkfifo(self._fifo_path)
        env["ISAAC_EVENT_FIFO"] = self._fifo_path

        self._proc = subprocess.Popen(
            [isaac_py, "-u", runtime_py],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
            cwd=os.path.dirname(runtime_py), env=env,
            # Own process group, so shutdown reaches Kit as well as the
            # python.sh wrapper that launched it (see _terminate_runtime).
            start_new_session=True,
        )
        self._proc.stdin.write(json.dumps(cfg) + "\n")
        self._proc.stdin.flush()

        # Drain Kit's raw stdout so its pipe buffer never fills and blocks.
        threading.Thread(target=self._drain_stdout, daemon=True).start()
        # Read structured events from the FIFO.
        threading.Thread(target=self._read_runtime, daemon=True).start()

    def _camera_config(self, urdf_text):
        """Camera settings for the runtime; creates the image publishers.

        The rate is 0 (no camera) when the description has no camera link.
        """
        link = self.get_parameter("camera_link_name").value
        rate = float(self.get_parameter("camera_rate").value)
        offsets = mounted_sensor_offsets(urdf_text, link) if rate > 0 else {}
        config = {
            "rate": rate if offsets else 0.0,
            "link": link,
            "width": int(self.get_parameter("camera_width").value),
            "height": int(self.get_parameter("camera_height").value),
            "horizontal_fov": float(
                self.get_parameter("camera_horizontal_fov").value),
            "near": float(self.get_parameter("camera_near").value),
            "far": float(self.get_parameter("camera_far").value),
            "offsets": {base: [list(position), list(quaternion)]
                        for base, (position, quaternion) in offsets.items()},
        }
        if rate > 0 and not offsets:
            self.get_logger().info(
                "Robot has no '%s' link; no RGB-D camera is published." % link)
        if config["rate"] > 0 and self._camera is None:
            self._rgb_pub = self.create_publisher(
                Image, camera_model.OAKD["rgb_topic"], 5)
            self._depth_pub = self.create_publisher(
                Image, camera_model.OAKD["depth_topic"], 5)
            self._camera_info_pub = self.create_publisher(
                CameraInfo, camera_model.OAKD["info_topic"], 5)
        self._camera = config if config["rate"] > 0 else None
        return config

    def _scan_config(self, urdf_text):
        """Scan settings for the runtime, including the laser link offsets.

        The runtime has no URDF parser, so the laser link's pose relative to
        each link that carries it is computed here.
        """
        link = self.get_parameter("laser_link_name").value
        offsets = mounted_sensor_offsets(urdf_text, link)
        try:
            _frames, urdf_root = urdf_link_frames(urdf_text)
        except Exception:
            urdf_root = None
        if not offsets:
            self.get_logger().warn(
                "Robot has no '%s' link; /scan originates 0.12 m above the "
                "base, as in the PyBullet and MuJoCo bridges." % link)
        return {
            "rate": float(self.get_parameter("scan_rate").value),
            "samples": int(self.get_parameter("scan_samples").value),
            "range_min": float(self.get_parameter("scan_range_min").value),
            "range_max": float(self.get_parameter("scan_range_max").value),
            "urdf_root": urdf_root or "",
            "offsets": {base: [list(position), list(quaternion)]
                        for base, (position, quaternion) in offsets.items()},
        }

    def _world_shapes(self, world_path):
        """Static world shapes for the runtime, or None to use its fallback.

        Meshes are staged to binary STL, the only mesh format the runtime
        reads; the vendored Gazebo model library ships Collada.
        """
        if not world_path or not os.path.isfile(world_path):
            return []
        try:
            from ament_index_python.packages import get_package_share_directory
            from robot_lab_utils.mesh_assets import (
                mesh_staging_dir, stage_mesh_file)
            from robot_lab_utils.sdf_world import (
                extract_static_shapes, uri_resolver)
        except ImportError as exc:
            self.get_logger().warn(
                "World geometry helpers unavailable (%s); the runtime falls "
                "back to its mesh-only loader." % exc)
            return None
        model_dirs = []
        try:
            model_dirs.append(os.path.join(
                get_package_share_directory("robot_lab_models"), "models"))
        except Exception:
            pass
        shapes, skipped = extract_static_shapes(
            world_path, uri_resolver(model_dirs, get_package_share_directory))
        cache_dir = mesh_staging_dir("isaac_world", world_path)
        loadable = []
        for shape in shapes:
            if shape["type"] == "mesh":
                staged = stage_mesh_file(shape["mesh"], cache_dir)
                if not staged or not staged.lower().endswith(".stl"):
                    skipped.append("mesh not convertible to STL: %s"
                                   % os.path.basename(shape["mesh"]))
                    continue
                shape = dict(shape, mesh=staged)
            loadable.append(shape)
        self.get_logger().info(
            "World '%s': %d static shape(s) for Isaac, %d skipped%s"
            % (os.path.basename(world_path), len(loadable), len(skipped),
               (" (%s)" % "; ".join(skipped[:3])) if skipped else ""))
        return loadable

    def _drain_stdout(self):
        try:
            for _line in self._proc.stdout:
                pass
        except Exception:
            pass

    def _read_runtime(self):
        if not self._fifo_path:
            return
        try:
            with open(self._fifo_path, "r") as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                    except ValueError:
                        continue
                    ev = msg.get("event")
                    if ev == "ready":
                        root_offset = self._root_body_offset(
                            msg.get("root_body", ""))
                        with self._lock:
                            self._dofs = msg.get("dofs", [])
                            self._root_offset = root_offset
                            self._spawned = True
                        # Signal readiness (R2.3).
                        self._ready = True
                        self._ready_pub.publish(Bool(data=True))
                        self.get_logger().info(
                            "Isaac runtime ready (dofs=%s)" % self._dofs)
                    elif ev == "state":
                        with self._lock:
                            self._state = msg
                    elif ev == "scan":
                        self._publish_scan(msg)
                    elif ev == "rgbd":
                        self._publish_rgbd(msg)
                    elif ev == "log":
                        self.get_logger().info(
                            "Isaac runtime: %s" % msg.get("msg"))
                    elif ev == "error":
                        self.get_logger().error(
                            "Isaac runtime error: %s" % msg.get("message"))
                        break
                    elif ev == "exit":
                        break
        except Exception as e:
            self.get_logger().error("FIFO read failed: %s" % e)
        proc = self._proc
        if proc is not None:
            try:
                proc.stdin.close()
            except Exception:
                pass
            rc = proc.wait()
            self.get_logger().warn("Isaac runtime exited (rc=%s)" % rc)
        with self._lock:
            self._proc = None
            self._spawned = True  # do not respawn a crashed Kit
        if self._fifo_path:
            try:
                os.unlink(self._fifo_path)
            except OSError:
                pass
            self._fifo_path = None

    # ------------------------------------------------------------------
    def _on_reset(self, request, response):
        """Reset the simulation (R2.3)."""
        try:
            if self._proc is not None and self._proc.stdin is not None:
                # Send reset command to Isaac runtime.
                self._proc.stdin.write('{"reset": true}\n')
                self._proc.stdin.flush()
                self.get_logger().info("Reset command sent to Isaac runtime")
                response.success = True
                response.message = "Reset command sent to Isaac runtime"
            else:
                response.success = False
                response.message = "Isaac runtime not running (offline mode)"
        except Exception as e:
            response.success = False
            response.message = f"Reset failed: {e}"
            self.get_logger().error(f"Reset failed: {e}")
        return response

    def _root_body_offset(self, root_body):
        """Pose of Isaac's articulation root body in the URDF root's frame.

        The other backends report the URDF root link; Isaac reports the root
        rigid body, which for Bumperbot is base_link, 0.033 m above
        base_footprint.  None when they are the same link or unknown.
        """
        try:
            frames, urdf_root = urdf_link_frames(self._urdf_text)
        except Exception:
            return None
        if not root_body or not urdf_root or root_body == urdf_root:
            return None
        offset = relative_frame(frames, root_body, urdf_root)
        if offset is not None:
            self.get_logger().info(
                "Isaac reports root body '%s'; odometry is re-expressed at "
                "URDF root '%s' (offset %s)"
                % (root_body, urdf_root,
                   [round(v, 4) for v in offset[0]]))
        return offset

    def _publish_rgbd(self, event):
        camera = self._camera
        if camera is None:
            return
        try:
            frame = decode_rgbd_event(event)
        except (KeyError, TypeError, ValueError) as exc:
            if not self._camera_error_reported:
                self._camera_error_reported = True
                self.get_logger().warn("Malformed RGB-D frame from the Isaac "
                                       "runtime: %s" % exc)
            return
        t, rgb, depth = frame
        stamp = Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))
        optical = self.get_parameter("camera_optical_frame").value
        self._rgb_pub.publish(image_msg(stamp, optical, rgb, "rgb8"))
        self._depth_pub.publish(image_msg(stamp, optical, depth, "32FC1"))
        self._camera_info_pub.publish(camera_info_msg(
            stamp, optical, camera["width"], camera["height"],
            camera["horizontal_fov"]))

    def _publish_scan(self, event):
        ranges = [float(v) for v in (event.get("ranges") or [])]
        if not ranges:
            return
        t = float(event.get("t", 0.0))
        msg = LaserScan()
        msg.header.stamp = Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))
        msg.header.frame_id = self.get_parameter("laser_link_name").value
        msg.angle_increment = 2.0 * math.pi / len(ranges)
        msg.angle_min = -math.pi
        msg.angle_max = -math.pi + (len(ranges) - 1) * msg.angle_increment
        msg.scan_time = 1.0 / max(
            float(self.get_parameter("scan_rate").value), 0.1)
        msg.range_min = float(self.get_parameter("scan_range_min").value)
        msg.range_max = float(self.get_parameter("scan_range_max").value)
        msg.ranges = ranges
        self._scan_pub.publish(msg)

    def _publish_health(self):
        """Publish health status (R2.3)."""
        msg = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = "isaac_spawner"
        status.hardware_id = "isaac"

        if self._ready and self._proc is not None:
            status.level = DiagnosticStatus.OK
            status.message = "running"
        elif self._proc is None:
            status.level = DiagnosticStatus.WARN
            status.message = "offline mode"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = "not ready"

        status.values.append(KeyValue(key="ready", value=str(self._ready)))
        status.values.append(
            KeyValue(key="spawned", value=str(self._spawned)))
        status.values.append(
            KeyValue(key="offline", value=str(self._proc is None)))
        msg.status.append(status)
        self._health_pub.publish(msg)

    # ------------------------------------------------------------------
    def _publish(self):
        with self._lock:
            state = self._state
            dofs = list(self._dofs)
            root_offset = self._root_offset
        if state is None:
            return
        t = float(state.get("t", 0.0))
        stamp = Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))
        pos = state.get("pos", [0.0, 0.0, 0.0])
        orn = state.get("orn", [0.0, 0.0, 0.0, 1.0])
        lin = state.get("lin", [0.0, 0.0, 0.0])
        ang = state.get("ang", [0.0, 0.0, 0.0])
        jpos = state.get("jpos", [])
        jvel = state.get("jvel", [])
        # Isaac reports its root body with world-frame velocities; odometry
        # is the URDF root's pose with the twist and IMU rates in the robot's
        # own frame, which is how robot_localization fuses vx / vy / vyaw.
        pos, orn, lin, ang = body_odometry(pos, orn, lin, ang, root_offset)

        clock_msg = RosClock()
        clock_msg.clock = stamp
        self._clock_pub.publish(clock_msg)
        if getattr(self, "_robot_free", False):
            return

        js = JointState()
        js.header.stamp = stamp
        js.name = dofs
        js.position = jpos
        js.velocity = jvel
        self._js_pub.publish(js)

        om = Odometry()
        om.header.stamp = stamp
        om.header.frame_id = "odom"
        om.child_frame_id = "base_footprint"
        om.pose.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
        om.pose.pose.orientation = Quaternion(
            x=orn[0], y=orn[1], z=orn[2], w=orn[3])
        om.twist.twist.linear = Vector3(x=lin[0], y=lin[1], z=lin[2])
        om.twist.twist.angular = Vector3(x=ang[0], y=ang[1], z=ang[2])
        self._odom_pub.publish(om)

        im = Imu()
        im.header.stamp = stamp
        im.header.frame_id = "imu_link"
        im.orientation = Quaternion(
            x=orn[0], y=orn[1], z=orn[2], w=orn[3])
        im.orientation_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0,
                                     0.0, 0.0, 0.001]
        im.angular_velocity = Vector3(x=ang[0], y=ang[1], z=ang[2])
        im.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0,
                                          0.0, 0.0, 0.01]
        im.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)
        im.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0,
                                             0.0, 0.0, 0.1]
        self._imu_pub.publish(im)

    def shutdown(self):
        # A second signal from launch's escalation must not abort cleanup
        # half-way and leave Kit running.
        try:
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
        except ValueError:  # not the main thread (tests)
            pass
        proc = self._proc
        if proc is not None:
            outcome = _terminate_runtime(proc)
            try:
                self.get_logger().info("Isaac runtime stopped (%s)" % outcome)
            except Exception:
                pass
        if self._fifo_path:
            try:
                os.unlink(self._fifo_path)
            except OSError:
                pass
            self._fifo_path = None


def _raise_system_exit(_signum, _frame):
    raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    # Without a handler SIGTERM ends the interpreter without running the
    # `finally` below, which is what orphaned the Isaac runtime.
    signal.signal(signal.SIGTERM, _raise_system_exit)
    node = IsaacSpawner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as exc:  # ExternalShutdownException on launch SIGINT
        if exc.__class__.__name__ != "ExternalShutdownException":
            raise
    finally:
        node.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
