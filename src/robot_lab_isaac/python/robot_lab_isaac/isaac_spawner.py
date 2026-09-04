"""Isaac Sim robot spawner node (ROS 2 side).

The node does not run Isaac Sim in-process: ``rclpy`` on ROS 2 Humble is
Python-3.10-only while ``isaacsim`` >= 5.0 requires Python 3.12.  Instead
it spawns :mod:`robot_lab_isaac.isaac_runtime` under the dedicated Isaac
Sim virtual environment (parameter ``isaac_python``) and exchanges
clock/state/commands over a JSON event FIFO plus stdin commands.  The
node publishes the ROS 2 topic contract (joint_states, TF, odom, imu,
clock) and forwards /cmd_vel to the runtime child.

If the Isaac Sim python environment is missing or the child fails, the
node logs a clear message and the rest of the launch graph continues in
offline mode.

.. note::
    Isaac Sim aarch64 builds are officially supported by NVIDIA only on
    DGX Spark systems.  On Jetson (AGX Orin) the wheel installs but Kit
    may abort during startup ("Cannot calculate frequency: TSC ran
    backwards").  In that case the node falls back to offline mode.
"""
import json
import os
import re
import subprocess
import tempfile
import threading

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

_DEFAULT_ISAAC_PY = "/workspace/isaac_env/bin/python"


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
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("isaac_python", _DEFAULT_ISAAC_PY)
        self.declare_parameter("left_wheel_joint", "wheel_left_joint")
        self.declare_parameter("right_wheel_joint", "wheel_right_joint")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._imu_pub = self.create_publisher(Imu, "/imu/out", 10)
        self._clock_pub = self.create_publisher(Time, "/clock", 10)
        self._tf_br = TransformBroadcaster(self)

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
        isaac_py = self.get_parameter("isaac_python").value
        if not isaac_py or not os.path.isfile(str(isaac_py)):
            self.get_logger().warn(
                "Isaac Sim python (%r) not found — running in offline "
                "mode.  Install isaacsim (see README) or set the "
                "isaac_python parameter." % isaac_py
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
        if not model or not os.path.isfile(str(model)):
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
        fd, urdf_file = tempfile.mkstemp(suffix=".urdf", dir="/workspace/.tmp")
        with os.fdopen(fd, "w") as fh:
            fh.write(urdf)
        self.get_logger().info("URDF written to %s" % urdf_file)

        gui = self.get_parameter("gui").value
        if isinstance(gui, str):
            gui = gui.lower() in ("true", "1", "yes")
        cfg = {
            "world_stage": str(self.get_parameter("world_stage").value or ""),
            "world_path": str(self.get_parameter("world_path").value or ""),
            "urdf_file": urdf_file,
            "robot_name": self.get_parameter("robot_name").value,
            "spawn_x": float(self.get_parameter("spawn_x").value),
            "spawn_y": float(self.get_parameter("spawn_y").value),
            "spawn_z": float(self.get_parameter("spawn_z").value),
            "spawn_yaw": float(self.get_parameter("spawn_yaw").value),
            "gui": bool(gui and os.environ.get("DISPLAY")),
            "physics_rate": 60.0,
            "left_wheel_joint": self.get_parameter("left_wheel_joint").value,
            "right_wheel_joint": self.get_parameter("right_wheel_joint").value,
        }

        runtime_py = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "isaac_runtime.py"
        )
        self.get_logger().info(
            "Launching Isaac runtime: %s %s (gui=%s)"
            % (isaac_py, runtime_py, cfg["gui"])
        )

        env = dict(os.environ)
        env.setdefault("LD_PRELOAD", "/lib/aarch64-linux-gnu/libgomp.so.1")
        env.setdefault("ACCEPT_EULA", "YES")
        env.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

        # Dedicated event FIFO: Kit hijacks/closes the child's stdout, so
        # structured state flows through a named pipe instead.
        self._fifo_path = tempfile.mkstemp(
            prefix="isaac_evt_", dir="/workspace/.tmp")[1]
        os.unlink(self._fifo_path)
        os.mkfifo(self._fifo_path)
        env["ISAAC_EVENT_FIFO"] = self._fifo_path

        self._proc = subprocess.Popen(
            [isaac_py, "-u", runtime_py],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1,
            cwd=os.path.dirname(runtime_py), env=env,
        )
        self._proc.stdin.write(json.dumps(cfg) + "\n")
        self._proc.stdin.flush()

        # Drain Kit's raw stdout so its pipe buffer never fills and blocks.
        threading.Thread(target=self._drain_stdout, daemon=True).start()
        # Read structured events from the FIFO.
        threading.Thread(target=self._read_runtime, daemon=True).start()

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
                        with self._lock:
                            self._dofs = msg.get("dofs", [])
                            self._spawned = True
                        self.get_logger().info(
                            "Isaac runtime ready (dofs=%s)" % self._dofs)
                    elif ev == "state":
                        with self._lock:
                            self._state = msg
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
    def _publish(self):
        with self._lock:
            state = self._state
            dofs = list(self._dofs)
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

        self._clock_pub.publish(stamp)

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

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation = Vector3(x=pos[0], y=pos[1], z=pos[2])
        tf.transform.rotation = Quaternion(
            x=orn[0], y=orn[1], z=orn[2], w=orn[3])
        self._tf_br.sendTransform(tf)

    def shutdown(self):
        proc = self._proc
        if proc is not None:
            try:
                proc.stdin.close()
                proc.wait(timeout=15)
            except Exception:
                proc.kill()
        if self._fifo_path:
            try:
                os.unlink(self._fifo_path)
            except OSError:
                pass
            self._fifo_path = None


def main(args=None):
    rclpy.init(args=args)
    node = IsaacSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
