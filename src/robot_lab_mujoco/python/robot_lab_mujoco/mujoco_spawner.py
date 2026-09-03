"""MuJoCo robot spawner node.

Loads a world + robot model, opens the MuJoCo passive viewer, runs the
physics step loop, and publishes the ROS 2 topics required by the
stack (joint_states, TF, odom, scan, imu, clock).
"""
import math
import os
import re
import subprocess
import tempfile
import threading
import time

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
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import TransformBroadcaster


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


def _build_mjcf_from_urdf(urdf_text, pkg_map):
    """Try MuJoCo MjSpec URDF import; fall back to the simple template."""
    if mujoco is not None and hasattr(mujoco.MjSpec, "from_file"):
        tmp = None
        try:
            tmp = tempfile.NamedTemporaryFile(
                suffix=".urdf", delete=False, mode="w"
            )
            tmp.write(urdf_text)
            tmp.close()
            spec = mujoco.MjSpec.from_file(tmp.name)
            return spec.to_xml()
        except Exception:
            pass
        finally:
            if tmp is not None:
                try:
                    os.unlink(tmp.name)
                except Exception:
                    pass
    return _FALLBACK_MJCF


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
        self._free_joint_qpos_adr = -1
        self._joint_name2id = {}   # mujoco joint name -> qpos index
        self._joint_names = []
        self._lw_name = ""
        self._rw_name = ""
        self._lw_qpos_adr = -1
        self._rw_qpos_adr = -1
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

        # --- publishers ---
        self._pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self._pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self._pub_scan = self.create_publisher(LaserScan, "/scan", 10)
        self._pub_imu = self.create_publisher(Imu, "/imu/out", 10)
        self._pub_clock = self.create_publisher(Time, "/clock", 10)
        self._tf_br = TransformBroadcaster(self)

        # --- subscriptions ---
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)

        # --- timer to attempt spawn ---
        self._timer = self.create_timer(0.5, self._try_spawn)
        self._thread = None

    def _on_cmd(self, msg):
        with self._twist_lock:
            self._twist = msg

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
            self._timer = self.create_timer(2.0, self._try_spawn)

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
        if model and os.path.isfile(str(model)):
            urdf = _xacro_to_urdf(str(model))
            pkg_map = {}
            for pkg_name in re.findall(r"\\$\\(find\\s+([^)]+)\\)", urdf):
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
            robot_mjcf = _build_mjcf_from_urdf(urdf, pkg_map)
        else:
            self.get_logger().warn("URDF not found; using fallback MJCF.")
            robot_mjcf = _FALLBACK_MJCF
            use_fallback = True

        # --- combine world + robot into single XML ---
        if world_xml and not use_fallback:
            world_mjcf = self._merge_mjcf(world_xml, robot_mjcf)
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

    @staticmethod
    def _merge_mjcf(world_path, robot_mjcf):
        """Insert robot MJCF bodies/actuators into the world XML."""
        with open(world_path, "r") as fh:
            world_text = fh.read()

        wb_match = re.search(
            r"(<worldbody>)(.*?)(</worldbody>)", robot_mjcf, re.DOTALL
        )
        act_match = re.search(
            r"(<actuator>)(.*?)(</actuator>)", robot_mjcf, re.DOTALL
        )
        robot_wb = wb_match.group(2) if wb_match else ""
        robot_act = act_match.group(2) if act_match else ""

        if "</worldbody>" in world_text:
            world_text = world_text.replace(
                "</worldbody>", robot_wb + "\n  </worldbody>"
            )
        else:
            world_text = "<mujoco><worldbody>" + world_text + \
                         robot_wb + "</worldbody>"
            if robot_act:
                world_text += "<actuator>" + robot_act + "</actuator>"
            world_text += "</mujoco>"

        if robot_act:
            if "</mujoco>" in world_text:
                world_text = world_text.replace(
                    "</mujoco>",
                    "<actuator>" + robot_act + "</actuator>\n</mujoco>",
                )

        return world_text

    def _find_body_and_joints(self):
        m = self._model
        self._free_joint_qpos_adr = -1
        self._body_id = -1
        self._joint_name2id = {}
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

            with self._twist_lock:
                t = self._twist
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
                if adr >= 0:
                    self._jpos.append(float(self._data.qpos[adr]))
                    self._jvel.append(float(self._data.qvel[adr]))
                else:
                    self._jpos.append(0.0)
                    self._jvel.append(0.0)

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
        self._pub_odom.publish(m)

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
        m.orientation_covariance = [0.001,0,0, 0,0.001,0, 0,0,0.001]
        m.angular_velocity = Vector3(x=self._bang[0], y=self._bang[1], z=self._bang[2])
        m.angular_velocity_covariance = [0.01,0,0, 0,0.01,0, 0,0,0.01]
        m.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)
        m.linear_acceleration_covariance = [0.1,0,0, 0,0.1,0, 0,0,0.1]
        self._pub_imu.publish(m)

    def _pub_clock(self):
        self._pub_clock.publish(self._stamp())

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
        self._pub_scan.publish(msg)

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
