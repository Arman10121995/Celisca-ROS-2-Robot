"""Live R4 measured mission harness for simulator backends (R8.1).

Drives a real seeded mission - bumperbot in a deterministic box arena
(nav_empty) reaching a waypoint ahead of its spawn pose - through the R4.1
lifecycle phases on the MuJoCo and PyBullet backends, recording real rosbag2
artifacts, a trace table, R4.2 mission metrics computed with ``metrics.py``,
and an R4.1-style manifest under the run directory.

The point-to-point controller and the outcome decision are pure logic and
unit-tested in ``test/test_live_mission.py``; the live ROS work is opt-in via
the ``ROBOT_LAB_RUN_LIVE_MISSION=1`` environment variable so the regular
suite never touches the ROS graph.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from .metrics import (
    compute_rtf,
    contact_events,
    footprint_clearance,
    trajectory_distance,
)
from .truthful_outcomes import OutcomeKind, classify_outcome

# This harness stays on wall time (use_sim_time=false): it subscribes to
# the simulator's /clock to measure sim progress.  With use_sim_time=true
# an rclpy node's executors and timers freeze until the first /clock
# arrives, so a sim-time node can deadlock waiting for the very clock it
# needs to unblock it.  The simulator backends publish sim time; this
# harness only observes it.

# Bumperbot ground truth from the R5.1 golden harness (wheel separation
# 0.1402 m, base offset 0.033 m, mesh-defined footprint).
FOOTPRINT_RADIUS_M = 0.14
GOAL_TOLERANCE_M = 0.15
SCENARIO_TIMEOUT_SIM_S = 40.0

REPO_ROOT = Path(__file__).resolve().parents[4]

# Install dir holding setup.bash.  With a copied install the module lives
# under <ws>/install/<pkg>/..., so derive it from __file__; with
# --symlink-install __file__ resolves back to the source tree, so fall
# back to <root>/install.  Only trust a candidate that has setup.bash.
def _workspace_prefix() -> Path:
    here = Path(__file__).resolve()
    parts = here.parts
    if "site-packages" in parts:
        idx = parts.index("site-packages")
        # <ws>/install/<pkg>/lib/python3.x/site-packages/... ->
        # workspace install dir is <ws>/install
        cand = Path(*parts[: idx - 3])
        if (cand / "setup.bash").exists():
            return cand
    return REPO_ROOT / "install"


WORKSPACE_PREFIX = _workspace_prefix()


def _yaw_of(x: float, y: float, z: float, w: float) -> float:
    """Yaw angle of a ROS wxyz quaternion."""
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


@dataclass
class WaypointController:
    """Closed-loop waypoint follower (pure logic, unit-tested).

    Computes a bounded /cmd_vel twist toward ``goal`` from the current pose.
    The forward speed scales with the remaining distance so the mission
    terminates cleanly inside the goal tolerance and never exceeds the
    declared limits (vx <= 0.3 m/s, |wz| <= 1.0 rad/s).
    """

    goal: Tuple[float, float]
    kv: float = 0.6
    kh: float = 2.0
    max_vx: float = 0.3
    min_vx: float = 0.04
    max_wz: float = 1.0

    def step(self, x: float, y: float, yaw: float) -> Tuple[float, float]:
        dx, dy = self.goal[0] - x, self.goal[1] - y
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        heading_error = math.atan2(math.sin(bearing - yaw),
                                   math.cos(bearing - yaw))
        vx = max(self.min_vx, min(self.max_vx, self.kv * distance))
        wz = max(-self.max_wz, min(self.max_wz, self.kh * heading_error))
        return vx, wz


@dataclass
class MissionTrace:
    """Accumulated observations for one run (written to trace.csv)."""

    rows: List[Dict[str, Any]] = field(default_factory=list)

    def record(self, sim_t: float, x: float, y: float, yaw: float,
               vx: float, wz: float, min_range: Optional[float]) -> None:
        self.rows.append({
            "sim_t": sim_t, "x": x, "y": y, "yaw": yaw,
            "vx_cmd": vx, "wz_cmd": wz,
            "clearance_m": (min_range if min_range is not None else math.inf),
        })

    def poses(self) -> List[Tuple[float, float]]:
        return [(r["x"], r["y"]) for r in self.rows]

    def clearances(self) -> List[Optional[float]]:
        return [r["clearance_m"] for r in self.rows]

    def csv(self) -> str:
        lines = ["sim_t,x,y,yaw,vx_cmd,wz_cmd,clearance_m"]
        for r in self.rows:
            lines.append("%s,%s,%s,%s,%s,%s,%s" % (
                r["sim_t"], r["x"], r["y"], r["yaw"],
                r["vx_cmd"], r["wz_cmd"], r["clearance_m"]))
        return "\n".join(lines) + "\n"


def bag_topic_counts(metadata: Dict[str, Any]) -> Dict[str, int]:
    """Per-topic message counts from a rosbag2 ``metadata.yaml`` payload.

    Pure mapping so recorded-bag provenance can be asserted without a live
    recording (see ``load_bag_topic_counts``).  Topic entries without a
    numeric count are omitted rather than reported as zero, because "no
    count recorded" and "recorded nothing" are different facts and the R4
    rule is to keep them distinguishable.
    """
    info = (metadata or {}).get("rosbag2_bagfile_information") or {}
    counts: Dict[str, int] = {}
    for entry in info.get("topics_with_message_count") or []:
        name = (entry.get("topic_metadata") or {}).get("name")
        count = entry.get("message_count")
        if not name or not isinstance(count, (int, float)):
            continue
        counts[str(name)] = int(count)
    return counts


def load_bag_topic_counts(bag_path: Path) -> Dict[str, int]:
    """Read per-topic counts from ``<bag_path>/metadata.yaml``.

    Returns an empty mapping when the file is missing or unparsable; callers
    must treat that as "counts unavailable", not as an empty recording.
    """
    try:
        import yaml
        payload = yaml.safe_load((bag_path / "metadata.yaml").read_text())
    except Exception:
        return {}
    return bag_topic_counts(payload)


def decide_outcome(
    reached_goal: bool,
    collision_detected: bool,
    sim_elapsed: float,
    timeout_sim_s: float,
    process_alive: bool,
) -> OutcomeKind:
    """Truthful terminal outcome for a point-to-point run (unit-tested)."""
    return classify_outcome(
        collision_detected=collision_detected,
        timed_out=(not reached_goal and sim_elapsed >= timeout_sim_s),
        path_available=True,
        state_healthy=True,
        process_alive=process_alive,
        cancelled=False,
        launch_ok=True,
        reset_ok=True,
    )


SIMULATOR_LAUNCH = {
    "mujoco": {
        "package": "robot_lab_mujoco",
        "file": "mujoco_simulator.launch.py",
        "gui": "false",
    },
    "pybullet": {
        "package": "robot_lab_pybullet",
        "file": "pybullet_simulator.launch.py",
        "gui": "false",
    },
}


def _stop_proc(proc, timeout):
    if proc is None or proc.poll() is not None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=timeout)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def _stop_group(proc, timeout):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=timeout)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass


def _drive(
    simulator: str,
    world: str,
    seed: int,
    process,
    domain_id: int,
    run_dir: Path,
    log_path: Path,
    goal_m: float,
    timeout_sim_s: float,
    bag_proc,
    bag_path: Path,
) -> Dict[str, Any]:
    """Drive the live mission inside the launched simulator process.

    Waits for the /clock stream from the isolated domain, commands the
    robot toward ``goal_m`` metres ahead of its spawn pose, then computes
    R4.2 metrics, writes trace/manifest artifacts and returns the record.
    Assumes the caller stops the simulator and bag processes afterwards.
    """
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan

    # run_timeout_s here is the wall budget for sim-timeout plus startup
    # (legacy runs show ~60-90 s startup in a shared workspace).
    run_timeout_s = timeout_sim_s + 150.0
    # DDS discovery requires matching localhost scoping: the simulator
    # subprocess is launched with ROS_LOCALHOST_ONLY=1, so this process
    # must use the same setting *before* rclpy.init reads the env, or no
    # topics will ever match despite identical domain/QoS.  (This exact
    # mismatch starved the first live runs of all /clock data.)
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    context = rclpy.Context()
    # NOTE: the domain must be passed explicitly here — the parent shell's
    # ROS_DOMAIN_ID is the outer workspace default, not the isolated
    # per-seed domain the simulator was launched on.
    rclpy.init(args=["--ros-args", "-r", "__node:=live_mission",
                      "-p", "use_sim_time:=false"],
               domain_id=domain_id, context=context)
    node = Node("live_mission", context=context)
    print("r4 live mission: %s world=%s seed=%d domain=%d"
          % (simulator, world, seed, domain_id), flush=True)
    node.get_logger().info(
        "r4 live mission: %s world=%s seed=%d" % (simulator, world, seed))
    progress_path = run_dir / "progress.log"

    def _progress(msg: str) -> None:
        line = "[%s] %s" % (time.strftime("%H:%M:%S"), msg)
        print(line, flush=True)
        try:
            with progress_path.open("a") as fh:
                fh.write(line + "\n")
        except Exception:
            pass

    # Match the simulator publishers exactly: both spawners publish with the
    # default QoS (RELIABLE + VOLATILE), e.g. ``create_publisher(..., 10)``.
    # A BEST_EFFORT clock subscriber never matches a RELIABLE publisher on
    # this DDS backend, which starved earlier runs of all /clock data.
    qos_clock = QoSProfile(depth=10)
    qos_clock.history = HistoryPolicy.KEEP_LAST
    qos_clock.durability = DurabilityPolicy.VOLATILE
    qos_clock.reliability = ReliabilityPolicy.RELIABLE
    qos_sensor = QoSProfile(depth=10)
    qos_sensor.history = HistoryPolicy.KEEP_LAST
    qos_sensor.durability = DurabilityPolicy.VOLATILE
    # Spawner sensor publishers use the default RELIABLE QoS (depth 10),
    # so odom/scan subscriptions must request RELIABLE to match —
    # BEST_EFFORT on either side blocks delivery.
    qos_sensor.reliability = ReliabilityPolicy.RELIABLE

    latest: Dict[str, Any] = {"odom": None, "scan": None}
    clock_samples: List[Tuple[float, float]] = []
    scan_ranges: List[float] = []

    def on_clock(msg: Clock) -> None:
        sec = msg.clock.sec + msg.clock.nanosec * 1e-9
        clock_samples.append((sec, time.monotonic()))

    def on_odom(msg: Odometry) -> None:
        latest["odom"] = msg

    def on_scan(msg: LaserScan) -> None:
        latest["scan"] = msg
        scan_ranges.clear()
        for r in msg.ranges:
            f = float(r)
            if math.isfinite(f):
                scan_ranges.append(f)

    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)

    node.create_subscription(Clock, "/clock", on_clock, qos_clock)
    node.create_subscription(Odometry, "/odom/ground_truth", on_odom,
                             qos_sensor)
    node.create_subscription(LaserScan, "/scan", on_scan, qos_sensor)
    cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)

    def spin(secs: float) -> None:
        end = time.monotonic() + secs
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)

    def _stamp(msg: Odometry) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # --- startup: wait for clock and odometry on the isolated domain ---
    _progress("waiting for clock+odom (startup)")
    spin(2.0)
    deadline = time.monotonic() + run_timeout_s
    while (not clock_samples or latest["odom"] is None) \
            and time.monotonic() < deadline:
        spin(0.1)
    if process.poll() is not None:
        raise RuntimeError(
            "simulator exited during startup (rc=%s); see %s"
            % (process.returncode, log_path))
    if not clock_samples or latest["odom"] is None:
        raise RuntimeError(
            "timed out waiting for /clock and /odom/ground_truth "
            "on ROS_DOMAIN_ID=%d; see %s" % (domain_id, log_path))

    # Goal ``goal_m`` metres ahead of the spawn pose (open arena, R8.1).
    start = latest["odom"].pose.pose
    sx, sy = start.position.x, start.position.y
    syaw = _yaw_of(start.orientation.x, start.orientation.y,
                   start.orientation.z, start.orientation.w)
    goal = (sx + goal_m * math.cos(syaw), sy + goal_m * math.sin(syaw))
    controller = WaypointController(goal)
    _progress("startup ok: goal=(%.3f, %.3f) start=(%.3f, %.3f)"
              % (goal[0], goal[1], sx, sy))
    trace = MissionTrace()
    reached = False
    last_cmd = time.monotonic()
    start_sim = _stamp(latest["odom"])
    deadline_sim = start_sim + timeout_sim_s
    end_sim = start_sim
    wall_start = time.monotonic()

    while True:
        spin(0.02)
        now_sim = _stamp(latest["odom"])
        end_sim = now_sim
        if len(trace.rows) % 250 == 0 and len(trace.rows) > 0:
            pose = latest["odom"].pose.pose
            _progress("driving: sim=%.1fs samples=%d x=%.2f y=%.2f"
                      % (now_sim - start_sim, len(trace.rows),
                         pose.position.x, pose.position.y))
        if now_sim >= deadline_sim:
            break
        if time.monotonic() - last_cmd >= 0.05:
            pose = latest["odom"].pose.pose
            x, y = pose.position.x, pose.position.y
            yaw = _yaw_of(pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w)
            vx, wz = controller.step(x, y, yaw)
            tw = Twist()
            tw.linear.x = float(vx)
            tw.angular.z = float(wz)
            cmd_pub.publish(tw)
            last_cmd = time.monotonic()
            clear = min((c for c in scan_ranges), default=math.inf)
            trace.record(now_sim, x, y, yaw, vx, wz, clear)
            if math.hypot(goal[0] - x, goal[1] - y) < GOAL_TOLERANCE_M:
                reached = True
                break
        if time.monotonic() - wall_start > 600.0:
            break

    # --- stop the robot, then the topology ---
    stop_twist = Twist()
    cmd_pub.publish(stop_twist)
    spin(0.2)
    executor.shutdown()
    node.destroy_node()
    context.shutdown()

    bag_ok = False
    bag_topics: Dict[str, int] = {}
    if bag_proc is not None and bag_path is not None:
        _stop_proc(bag_proc, 5.0)
        bag_ok = bag_path.exists() and any(bag_path.iterdir())
        if bag_ok:
            bag_topics = load_bag_topic_counts(bag_path)
    # --- R4.2 metrics ---
    poses = trace.poses()
    clearances = trace.clearances()
    dist = trajectory_distance(poses)
    min_clear = footprint_clearance(clearances, FOOTPRINT_RADIUS_M)
    contacts = contact_events(clearances, threshold=0.05)
    sim_elapsed = end_sim - start_sim
    wall_elapsed = (clock_samples[-1][1] - clock_samples[0][1]
                    if clock_samples else None)
    rtf = compute_rtf(sim_elapsed, wall_elapsed)
    raw_min = min((c for c in clearances), default=math.inf)
    collision_detected = (contacts is not None and contacts > 0
                          and math.isfinite(raw_min) and raw_min < 0.05)
    outcome = decide_outcome(
        reached, collision_detected, sim_elapsed, timeout_sim_s,
        process_alive=True)

    metrics = {
        "success": reached,
        "outcome": outcome.value,
        "trajectory_distance_m": dist,
        "footprint_clearance_min_m": min_clear,
        "contact_events": contacts,
        "time_to_goal_sim_s": sim_elapsed if reached else None,
        "drive_sim_seconds": sim_elapsed,
        "realtime_factor": rtf,
        "goal_tolerance_m": GOAL_TOLERANCE_M,
        "footprint_radius_m": FOOTPRINT_RADIUS_M,
        "bag_capture": bag_ok,
        # Per-topic message counts parsed from the rosbag2 metadata.yaml.
        # Recorded here so a run's provenance (recorded N odom / N scan /
        # N clock / N cmd_vel messages) is recoverable from a committed
        # artifact instead of a hand-run ``ros2 bag info``.  An empty
        # mapping means "counts unavailable", not "recorded nothing".
        "bag_message_counts": bag_topics,
    }
    (run_dir / "trace.csv").write_text(trace.csv())
    (run_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n")
    record = {
        "schema_version": "1.0",
        "result_id": "r4_%d" % seed,
        "simulator": simulator,
        "robot_id": "bumperbot",
        "environment_id": world,
        "scenario_id": "point_to_point_navigation",
        "seed": seed,
        "goal_m": goal_m,
        "outcome": outcome.value,
        "metrics": metrics,
        "bag_path": str(bag_path) if bag_ok else None,
        "trace_path": str(run_dir / "trace.csv"),
        "launch_log": str(run_dir / "launch.log"),
        "ts_utc": datetime.now(timezone.utc).isoformat(),
    }
    (run_dir / "manifest.json").write_text(
        json.dumps(record, indent=2, default=str) + "\n")
    return record


def run_live_mission(simulator: str, seed: int, goal_m: float,
                     world: str, output_dir: Path, timeout_sim_s: float,
                     domain: int) -> Dict[str, Any]:
    """Start the backend on an isolated domain, drive the mission, stop."""
    from ament_index_python.packages import get_package_share_directory

    domain_id = domain + seed % 100
    if not 0 <= domain_id <= 232:
        raise ValueError("ROS_DOMAIN_ID=%d out of range 0..232" % domain_id)
    run_dir = output_dir / (
        "r4_%s_point_to_point_navigation_%s_%d" % (simulator, world, seed))
    run_dir.mkdir(parents=True, exist_ok=True)

    xacro = ("%s/bumperbot/urdf/bumperbot.urdf.xacro"
             % get_package_share_directory("robot_lab_robots"))
    launch = SIMULATOR_LAUNCH[simulator]
    if simulator == "mujoco":
        world_arg = "world_name:=%s" % world
    else:
        world_arg = ("world_path:=%s/maps/%s/worlds/%s.world"
                     % (get_package_share_directory("robot_lab_maps"),
                        world, world))
    bash = ("source %s/setup.bash" % WORKSPACE_PREFIX)
    venv_activate = REPO_ROOT / ".venv" / "bin" / "activate"
    if venv_activate.exists():
        bash += " && source %s" % venv_activate
    export = "export ROS_DOMAIN_ID=%d ROS_LOCALHOST_ONLY=1" % domain_id
    cmd = ("%s && %s && ros2 launch %s %s model:=%s gui:=%s %s"
           % (bash, export, launch["package"], launch["file"], xacro,
              launch["gui"], world_arg))
    log_path = run_dir / "launch.log"
    env = dict(os.environ)
    env.update({"ROS_DOMAIN_ID": str(domain_id), "ROS_LOCALHOST_ONLY": "1"})

    process = bag_proc = None
    bag_path = run_dir / "bags" / ("run_%d" % seed)
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen(
                ["bash", "-c", cmd], stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True, env=env)
        bag_cmd = ("%s && %s && ros2 bag record -o %s "
                   "/odom/ground_truth /scan /clock /cmd_vel"
                   % (bash, export, bag_path))
        bag_proc = subprocess.Popen(
            ["bash", "-c", bag_cmd], stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, start_new_session=True, env=env)
        return _drive(simulator, world, seed, process, domain_id, run_dir,
                      log_path, goal_m, timeout_sim_s, bag_proc, bag_path)
    finally:
        _stop_proc(bag_proc, 5.0)
        _stop_group(process, 8.0)


def _env_flag(name: str) -> bool:
    """True when an environment variable is set to a truthy string.

    Used for the documented opt-in gate below; kept as a pure function so the
    accepted spellings are unit-tested rather than implicit.
    """
    return os.environ.get(name, "").strip().lower() in (
        "1", "true", "yes", "on")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulator", choices=["mujoco", "pybullet"],
                        required=True)
    parser.add_argument("--seeds", nargs="+", type=int, required=True)
    parser.add_argument("--goal", type=float, default=1.5)
    parser.add_argument("--world", default="nav_empty")
    parser.add_argument("--output", default="benchmark_runs")
    parser.add_argument("--timeout", type=float,
                        default=SCENARIO_TIMEOUT_SIM_S)
    parser.add_argument("--domain", type=int, default=170)
    args = parser.parse_args(argv)

    # The documented opt-in gate: a bare `robot-lab-live-mission` invocation
    # must not silently start simulators on a shared workspace's DDS graph.
    # The live tests set ROBOT_LAB_RUN_LIVE_MISSION=1 explicitly.
    if not _env_flag("ROBOT_LAB_RUN_LIVE_MISSION"):
        print("live mission is opt-in: set ROBOT_LAB_RUN_LIVE_MISSION=1",
              file=sys.stderr)
        return 2

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    records = []
    for seed in args.seeds:
        try:
            record = run_live_mission(
                args.simulator, seed, args.goal, args.world, output,
                args.timeout, args.domain)
        except Exception as exc:
            record = {"simulator": args.simulator, "seed": seed,
                      "outcome": "process_death", "error": str(exc)}
            print("seed %d FAILED: %s" % (seed, exc))
        records.append(record)
        print(json.dumps(record, sort_keys=True, default=str))
    (output / ("summary_%s.json" % args.simulator)).write_text(
        json.dumps(records, indent=2, default=str) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

