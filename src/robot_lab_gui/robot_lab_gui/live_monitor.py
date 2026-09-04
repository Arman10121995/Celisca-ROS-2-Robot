"""Live Monitor tab — subscribes to ROS topics and displays real-time data."""
import math
import threading
import time
from collections import deque

import tkinter as tk
from tkinter import ttk

try:
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan, Imu
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

from .themes.modern import (
    BG_DARK, BG_CARD, FG_PRIMARY, FG_MUTED, ACCENT, ACCENT_GREEN, ACCENT_RED,
    ACCENT_YELLOW, STATUS_OK, STATUS_ERROR, STATUS_IDLE,
)


def _quat_to_euler(x, y, z, w):
    """Convert quaternion to (roll, pitch, yaw) in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw



class LiveMonitorTab(ttk.Frame):
    """Real-time telemetry monitor for the running simulation."""

    def __init__(self, notebook, app):
        super().__init__(notebook, padding=10)
        self.app = app
        notebook.add(self, text="Live Monitor")

        self._running = False
        self._ros_thread = None
        self._node = None
        self._subscriptions = []

        self._odom_buf = deque(maxlen=100)
        self._scan_buf = deque(maxlen=10)
        self._imu_buf = deque(maxlen=50)
        self._clock_buf = deque(maxlen=10)

        self._build_ui()
        self._update_display()
        # Register with the app so _on_close can stop our ROS thread first
        self.app.live_monitor_tab = self

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        bar = ttk.Frame(self)
        bar.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        bar.columnconfigure(4, weight=1)

        ttk.Label(bar, text="Live Monitor", style="Heading.TLabel").grid(
            row=0, column=0, sticky="w", padx=(0, 16))

        self.btn_connect = ttk.Button(bar, text="Connect", command=self._toggle_connect)
        self.btn_connect.grid(row=0, column=1, padx=(0, 4))

        self.status_dot = tk.Label(bar, text="●", fg=STATUS_IDLE, bg=BG_DARK,
                                    font=("Segoe UI", 12))
        self.status_dot.grid(row=0, column=2, padx=(0, 4))

        self.status_label = ttk.Label(bar, text="Disconnected", style="Status.Idle.TLabel")
        self.status_label.grid(row=0, column=3, sticky="w", padx=(0, 12))

        ttk.Label(bar, text="Hz:", style="Muted.TLabel").grid(row=0, column=5, sticky="e")
        self.hz_var = tk.StringVar(value="10")
        hz_combo = ttk.Combobox(bar, textvariable=self.hz_var,
                                values=["1", "2", "5", "10", "20", "30"],
                                state="readonly", width=4)
        hz_combo.grid(row=0, column=6, sticky="e")

        body = ttk.Frame(self)
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        left = ttk.Frame(body)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        left.columnconfigure(0, weight=1)

        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky="nsew", padx=(4, 0))
        right.columnconfigure(0, weight=1)

        odom_card = self._make_card(left, "Odometry", 0)
        self._build_odom(odom_card)

        imu_card = self._make_card(left, "IMU", 1)
        self._build_imu(imu_card)

        scan_card = self._make_card(right, "Laser Scan", 0)
        self._build_scan(scan_card)

        clock_card = self._make_card(right, "Simulation Clock", 1)
        self._build_clock(clock_card)

    def _make_card(self, parent, title, row):
        frame = tk.Frame(parent, bg=BG_CARD, highlightbackground="#45475a",
                         highlightthickness=1)
        frame.grid(row=row, column=0, sticky="nsew", pady=4)
        frame.columnconfigure(0, weight=1)
        hdr = tk.Label(frame, text=title, bg=BG_CARD, fg=FG_PRIMARY,
                        font=("Segoe UI", 11, "bold"), padx=10, pady=6)
        hdr.grid(row=0, column=0, sticky="w")
        inner = tk.Frame(frame, bg=BG_CARD, padx=10, pady=10)
        inner.grid(row=1, column=0, sticky="nsew")
        inner.columnconfigure(0, weight=1)
        return inner

    def _build_odom(self, parent):
        self.odom_labels = {}
        fields = ["x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "omega"]
        for i, field in enumerate(fields):
            row, col = i // 3, (i % 3) * 2
            tk.Label(parent, text=field + ":", bg=BG_CARD, fg=FG_MUTED,
                     font=("Segoe UI", 9)).grid(row=row, column=col, sticky="e", padx=(0, 2))
            var = tk.StringVar(value="—")
            tk.Label(parent, textvariable=var, bg=BG_CARD, fg=FG_PRIMARY,
                     font=("Consolas", 9)).grid(row=row, column=col + 1, sticky="w", padx=(0, 10))
            self.odom_labels[field] = var

    def _build_imu(self, parent):
        self.imu_labels = {}
        fields = ["ax", "ay", "az", "wx", "wy", "wz"]
        for i, field in enumerate(fields):
            row, col = i // 3, (i % 3) * 2
            tk.Label(parent, text=field + ":", bg=BG_CARD, fg=FG_MUTED,
                     font=("Segoe UI", 9)).grid(row=row, column=col, sticky="e", padx=(0, 2))
            var = tk.StringVar(value="—")
            tk.Label(parent, textvariable=var, bg=BG_CARD, fg=FG_PRIMARY,
                     font=("Consolas", 9)).grid(row=row, column=col + 1, sticky="w", padx=(0, 10))
            self.imu_labels[field] = var


    def _toggle_connect(self):
        if self._running:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if not ROS_AVAILABLE:
            self.app.log("[live_monitor] rclpy not available.\n")
            return
        self._running = True
        self.btn_connect.configure(text="Disconnect")
        self.status_dot.configure(fg=STATUS_OK)
        self.status_label.configure(text="Connected", style="Status.OK.TLabel")
        self._ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._ros_thread.start()

    def _disconnect(self, from_thread=False):
        """Disconnect; safe to call from the ROS background thread."""
        self._running = False
        def _ui():
            self.btn_connect.configure(text="Connect")
            self.status_dot.configure(fg=STATUS_IDLE)
            self.status_label.configure(text="Disconnected", style="Status.Idle.TLabel")
        if from_thread:
            # Never touch Tk widgets from a background thread (segfaults);
            # marshal the update onto the main Tk loop instead.
            try:
                self.after(0, _ui)
            except RuntimeError:
                pass  # widget already destroyed during shutdown
        else:
            _ui()
        for sub in self._subscriptions:
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass
        self._subscriptions.clear()
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None

    def _ros_spin(self):
        try:
            try:
                rclpy.init()
            except RuntimeError:
                pass  # already initialized by the main application
            self._node = rclpy.create_node("robot_lab_live_monitor")
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self._subscriptions.append(
                self._node.create_subscription(Odometry, "/odom", self._on_odom, qos))
            self._subscriptions.append(
                self._node.create_subscription(LaserScan, "/scan", self._on_scan, qos))
            self._subscriptions.append(
                self._node.create_subscription(Imu, "/imu/out", self._on_imu, qos))
            self._subscriptions.append(
                self._node.create_subscription(Clock, "/clock", self._on_clock, 10))
            while self._running:
                rclpy.spin_once(self._node, timeout_sec=0.1)
        except Exception as exc:
            self.app.output_queue.put(("live_monitor", "[ROS error: %s]\n" % exc))
        finally:
            self._disconnect(from_thread=True)

    def shutdown(self):
        """Stop the ROS thread and wait for it — called on app close."""
        self._running = False
        thread = self._ros_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = _quat_to_euler(q.x, q.y, q.z, q.w)
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        self._odom_buf.append({
            "x": p.x, "y": p.y, "z": p.z,
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "vx": v.x, "vy": v.y, "omega": w.z,
        })

    def _on_scan(self, msg):
        valid = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        self._scan_buf.append({
            "n": len(msg.ranges), "valid": len(valid),
            "min": min(valid) if valid else float("nan"),
            "max": max(valid) if valid else float("nan"),
            "angle_min": msg.angle_min, "angle_max": msg.angle_max,
        })

    def _on_imu(self, msg):
        a = msg.linear_acceleration
        g = msg.angular_velocity
        self._imu_buf.append({
            "ax": a.x, "ay": a.y, "az": a.z,
            "wx": g.x, "wy": g.y, "wz": g.z,
        })

    def _on_clock(self, msg):
        self._clock_buf.append(float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9)

    def _update_display(self):
        if self._running:
            self._refresh_labels()
        try:
            hz = int(self.hz_var.get())
        except Exception:
            hz = 10
        interval = max(50, 1000 // hz)
        self.after(interval, self._update_display)

    def _refresh_labels(self):
        if self._odom_buf:
            o = self._odom_buf[-1]
            for key, var in self.odom_labels.items():
                var.set("%.3f" % o.get(key, float("nan")))
        if self._imu_buf:
            im = self._imu_buf[-1]
            for key, var in self.imu_labels.items():
                var.set("%.2f" % im.get(key, float("nan")))
        if self._scan_buf:
            sc = self._scan_buf[-1]
            self.scan_text.configure(state="normal")
            self.scan_text.delete("1.0", "end")
            self.scan_text.insert("1.0",
                "Ranges: %d total, %d valid\nMin: %.2f m  Max: %.2f m\n"
                "Angle: %.2f to %.2f rad" % (
                    sc["n"], sc["valid"], sc["min"], sc["max"],
                    sc["angle_min"], sc["angle_max"]))
            self.scan_text.configure(state="disabled")
        if self._clock_buf:
            t = self._clock_buf[-1]
            self.clock_var.set("Sim time: %.2f s" % t)
            if len(self._clock_buf) >= 2:
                dt = self._clock_buf[-1] - self._clock_buf[0]
                n = len(self._clock_buf)
                self.fps_var.set("Elapsed: %.1f s  msgs: %d" % (dt, n))

    def _build_scan(self, parent):
        self.scan_text = tk.Text(parent, height=8, bg=BG_CARD, fg=FG_PRIMARY,
                                  font=("Consolas", 8), relief="flat",
                                  highlightthickness=0, wrap="word")
        self.scan_text.grid(row=0, column=0, sticky="nsew")
        self.scan_text.insert("1.0", "Waiting for /scan messages...")
        self.scan_text.configure(state="disabled")

    def _build_clock(self, parent):
        self.clock_var = tk.StringVar(value="Sim time: —")
        tk.Label(parent, textvariable=self.clock_var, bg=BG_CARD, fg=FG_PRIMARY,
                 font=("Consolas", 10, "bold")).grid(row=0, column=0, sticky="w")
        self.fps_var = tk.StringVar(value="FPS: —")
        tk.Label(parent, textvariable=self.fps_var, bg=BG_CARD, fg=FG_MUTED,
                 font=("Consolas", 9)).grid(row=1, column=0, sticky="w", pady=(4, 0))
