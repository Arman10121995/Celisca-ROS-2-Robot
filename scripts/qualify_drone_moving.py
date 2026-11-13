#!/usr/bin/env python3
"""R5.11 flight: moving people cross the drone's route; then a sensor-loss landing.

Launches Flight mode on ``aerial_moving`` with the RGB-D X500, RTAB-Map and
the observed-obstacle route layer (``aerial_obstacles:=mapped_depth``), then:

1. takeoff check and takeoff; Plan to (10, 0, 1.2) and Execute: the route
   crosses two walking actors (x = 0 and x = 5 m). Recorded: planner states
   (holds for predicted moving obstacles, replans), tracked/moving obstacle
   counts, and the true distance between the drone (simulator truth) and
   each walker (Gazebo's own actor poses);
2. hover, then stop the depth camera bridge (sensor loss): the drone must hold
   and land after the configured timeout.

Pass: goal reached, every drone-walker distance above ``--contact-m``, at
least one moving-obstacle hold or replan, and a confirmed sensor-loss landing.
One simulation at a time; prints and writes a JSON report.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time

import numpy as np


def gz_environment():
    """GZ_* variables (partition, discovery ports) of the flight runtime's ``gz sim`` server."""
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            # Ruby rewrites the gz process title into one string ('gz sim ... robot_lab_flight.sdf').
            command = b' '.join((proc / 'cmdline').read_bytes().split(b'\0'))
            if not command.startswith(b'gz sim') or b'robot_lab_flight.sdf' not in command:
                continue
            found = {}
            for entry in (proc / 'environ').read_bytes().split(b'\0'):
                if entry.startswith(b'GZ_') and b'=' in entry:
                    key, value = entry.decode(errors='replace').split('=', 1)
                    found[key] = value
            if 'GZ_PARTITION' in found:
                return found
        except OSError:
            continue
    return {}


def processes_with(module, partition):
    """PIDs running ``module`` with this GZ_PARTITION (the runtime starts them in their own sessions)."""
    pids = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            if module.encode() not in (proc / 'cmdline').read_bytes():
                continue
            if ('GZ_PARTITION=%s' % partition).encode() in (proc / 'environ').read_bytes().split(b'\0'):
                pids.append(int(proc.name))
        except OSError:
            continue
    return pids


def walker_positions(sim_time):
    """Scripted walker positions at Gazebo simulation time (tools/gen_aerial_moving.py waypoints,
    linear between them; actors start at t = 0 and loop). Gazebo animates actors in rendering only:
    the server publishes no actor poses, so their script is the truth."""
    import importlib.util
    root = Path(__file__).resolve().parents[1] / 'src/robot_lab_maps/tools/gen_aerial_moving.py'
    spec = importlib.util.spec_from_file_location('gen_aerial_moving', root)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    found = {}
    for index, (x, y0, y1, leg) in enumerate(module.WALKERS):
        points = [(0., y0), (leg, y1), (leg + 1., y1), (2 * leg + 1., y0), (2 * leg + 2., y0)]
        t = sim_time % points[-1][0]
        for (ta, ya), (tb, yb) in zip(points[:-1], points[1:]):
            if ta <= t <= tb:
                found['walker_%d' % index] = (x, ya + (yb - ya) * (t - ta) / max(tb - ta, 1e-9), 0.)
                break
    return found


def gz_clock(stop, samples):
    """Background sampling of Gazebo simulation time, paired with this process's monotonic time."""
    environment = dict(os.environ)
    gz = gz_environment()
    environment.update(gz)
    samples['_partition'] = [gz.get('GZ_PARTITION')]
    pattern = re.compile(r'sim \{\s*sec: (\d+)(?:\s*nsec: (\d+))?')
    while not stop.is_set():
        # The gz wrapper starts gz-transport-topic as a child: run it in its own session and kill the
        # whole group on timeout, or the child is orphaned (it was, 2026-11-13).
        query = subprocess.Popen(['gz', 'topic', '-e', '-t', '/world/robot_lab_flight/clock', '-n', '1'],
                                 stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, env=environment,
                                 start_new_session=True)
        samples.setdefault('_queries', []).append(query.pid)
        try:
            text, _ = query.communicate(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(query.pid, signal.SIGKILL)
            query.wait()
            continue
        match = pattern.search(text)
        if match:
            sim = int(match.group(1)) + int(match.group(2) or 0) * 1e-9
            samples.setdefault('_clock', []).append((time.monotonic(), sim))
        # Each gz CLI call starts a Ruby process: sample every 3 s (sim time is linear) and interpolate.
        stop.wait(3.)


def sim_now(samples):
    """Current Gazebo time from recent (monotonic, sim) clock samples (linear fit), or None."""
    clock = samples.get('_clock') or []
    if len(clock) < 3:
        return None
    recent = clock[-8:]
    mono = [c[0] for c in recent]
    sim = [c[1] for c in recent]
    span = mono[-1] - mono[0]
    rate = (sim[-1] - sim[0]) / span if span > 0 else 1.
    return sim[-1] + rate * (time.monotonic() - mono[-1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--goal', type=float, nargs=3, default=(10., 0., 1.2))
    parser.add_argument('--contact-m', type=float, default=.6, help='minimum drone-walker centre distance')
    parser.add_argument('--timeout', type=float, default=900.)
    parser.add_argument('--robot', default='px4_x500')
    parser.add_argument('--simulator', default='gazebo')
    parser.add_argument('--map', default='aerial_moving')
    parser.add_argument('--plain', action='store_true',
                        help='no camera/observed obstacles: takeoff, route to the goal, land (e.g. px4_iris in MuJoCo)')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    report = dict(world=args.map, robot=args.robot, simulator=args.simulator, goal=list(args.goal), phases=[],
                  passed=False, plain=args.plain)
    command = ['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py', 'robot_model:=' + args.robot,
               'simulator:=' + args.simulator, 'mode:=flight', 'map_name:=' + args.map, 'gui:=false', 'start_rviz:=false']
    if not args.plain:
        command += ['px4_vehicle:=x500_depth', 'aerial_mapping:=rtabmap', 'aerial_obstacles:=mapped_depth']
    report['command'] = command
    log = open(args.output / 'launch.log', 'w')
    launch = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    stop, walkers = threading.Event(), {}
    rclpy.init()
    node = rclpy.create_node('drone_moving_check')
    status, planning, truth = {}, {}, []
    states = []
    track_history = []
    node.create_subscription(String, '/px4/status', lambda m: status.update(json.loads(m.data)), 10)

    def on_planning(message):
        data = json.loads(message.data)
        planning.clear()
        planning.update(data)
        obstacles = data.get('obstacles') or {}
        if obstacles.get('depth_clusters'):
            report['max_depth_clusters'] = max(report.get('max_depth_clusters', 0), obstacles['depth_clusters'])
        if not track_history or time.monotonic() - started - track_history[-1][0] >= 1.:
            track_history.append((round(time.monotonic() - started, 2), status.get('position_enu'),
                                  len(obstacles.get('tracks') or []), obstacles.get('moving_tracks'),
                                  [(tr['position'], tr['radius'], tr['moving']) for tr in (obstacles.get('tracks') or [])[:6]]))
        if not states or states[-1][1] != data.get('state') or data.get('sensor_loss', {}).get('state') != states[-1][2]:
            states.append((round(time.monotonic() - started, 2), data.get('state'),
                           data.get('sensor_loss', {}).get('state'), data.get('error', ''),
                           (data.get('obstacles') or {}).get('moving_tracks')))
    node.create_subscription(String, '/px4/planning/status', on_planning, 10)
    node.create_subscription(Odometry, '/px4/odometry_truth', lambda m: truth.append(
        (time.monotonic(), m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)), 10)
    goal_pub = node.create_publisher(PoseStamped, '/px4/plan_goal', 1)
    clients = {name: node.create_client(Trigger, '/px4/' + name) for name in ('takeoff_check', 'takeoff', 'plan_execute', 'hold')}
    started = time.monotonic()

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)

    def until(predicate, seconds, label):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return round(time.monotonic() - started, 2)
        raise TimeoutError(label)

    def call(name):
        clients[name].wait_for_service(timeout_sec=30)
        future = clients[name].call_async(Trigger.Request())
        until(future.done, 30, name)
        result = future.result()
        report['phases'].append(dict(call=name, success=result.success, message=result.message,
                                     t=round(time.monotonic() - started, 2)))
        return result.success
    sampler = None
    try:
        until(lambda: status.get('position_ready') and status.get('phase') == 'idle', args.timeout, 'FCU ready')
        if not args.plain:
            until(lambda: (planning.get('obstacles') or {}).get('ready'), 300, 'observed layer ready')
        sampler = None if args.plain else threading.Thread(target=gz_clock, args=(stop, walkers), daemon=True)
        if sampler:
            sampler.start()
        for takeoff_attempt in range(3):   # PX4 can deny arming on a transient sensor-health check under load
            for attempt in range(5):        # the landed estimate can still settle by > 5 cm during a check
                until(lambda: status.get('takeoff_ready'), 240, 'FCU takeoff-ready (preflight settled, landed, level)')
                if not call('takeoff_check'):
                    raise RuntimeError('takeoff check refused: %s' % report['phases'][-1]['message'])
                until(lambda: planning.get('takeoff', {}).get('state') in ('clear', 'failed', 'invalidated'), 120,
                      'takeoff check result')
                if planning['takeoff']['state'] != 'invalidated':
                    break
                spin(5.)
            if planning['takeoff']['state'] != 'clear':
                raise RuntimeError('takeoff check %s: %s' % (planning['takeoff']['state'], planning['takeoff'].get('error')))
            report['preflight_heading'] = dict(drift_deg_s=status.get('heading_drift_deg_s'),
                                               estimator_restarts=status.get('estimator_restarts'))
            if not call('takeoff'):
                raise RuntimeError('takeoff refused')
            try:
                until(lambda: status.get('phase') == 'flying', 120, 'airborne')
                break
            except TimeoutError:
                report.setdefault('takeoff_retries', 0)
                report['takeoff_retries'] += 1
                until(lambda: status.get('phase') == 'idle', 60, 'back to idle')
                spin(10.)
        else:
            raise RuntimeError('not airborne after three takeoff attempts (PX4 arming denied)')
        spin(3.)

        def fly_to(target, timeout=600):
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = target
            goal.pose.orientation.w = 1.
            for attempt in range(5):        # stale plans (hover drift) and execute-time rechecks plan again
                request = (planning.get('goal_request') or 0) + 1   # the planner numbers each goal request
                goal_pub.publish(goal)
                until(lambda: (planning.get('goal_request') or 0) >= request
                      and planning.get('state') in ('planned', 'failed', 'rejected'), 120, 'plan result')
                if planning['state'] != 'planned':
                    raise RuntimeError('plan %s: %s' % (planning['state'], planning.get('error')))
                if call('plan_execute'):
                    # Execute rechecks the route against the latest observations asynchronously.
                    until(lambda: planning.get('state') in ('executing', 'rejected', 'route_stopped'), 60,
                          'execute recheck')
                    if planning['state'] != 'rejected':
                        break
                    report.setdefault('execute_refusals', []).append(planning.get('error'))
                else:
                    report.setdefault('execute_refusals', []).append(report['phases'][-1]['message'])
                spin(3.)
            else:
                raise RuntimeError('execute refused: %s' % report['phases'][-1]['message'])
            until(lambda: math.dist(status.get('position_enu') or (1e9, 0, 0), target) < .4, timeout,
                  'reached %s' % (target,))
            # The controller ends a route within 5 cm of its last waypoint; the planner accepts the next
            # goal only once the route has ended, so end a hover-close route with Hold.
            try:
                until(lambda: planning.get('state') != 'executing', 15, 'route end')
            except TimeoutError:
                call('hold')
                until(lambda: planning.get('state') != 'executing', 15, 'route end after Hold')
        if not args.plain:
            # Descend first, so the whole crossing is at the walkers' height (not over their heads).
            here = status.get('position_enu')
            fly_to((here[0], here[1], args.goal[2]), timeout=120)
            spin(3.)
        if not args.plain:
            # Force a crossing: walker 0 crosses y = 0 at Gazebo time 3 and 10 s of its 14 s loop and the
            # drone needs ~26 s to reach x = 0, so publish the goal at sim time = 4 (mod 7). Test design only.
            until(lambda: sim_now(walkers) is not None, 60, 'Gazebo clock samples')
            until(lambda: 3.8 <= sim_now(walkers) % 7. <= 4.6, 30, 'crossing phase')
            report['goal_published_sim_s'] = round(sim_now(walkers), 2)
        fly_to(args.goal)
        report['goal_reached_s'] = round(time.monotonic() - started, 2)
        report['final_position'] = status.get('position_enu')
        spin(3.)
        if args.plain:
            clients.setdefault('land', node.create_client(Trigger, '/px4/land'))
            call('land')
            until(lambda: status.get('phase') == 'idle' and status.get('landed_state') == 1, 180, 'landed')
            report['landed_after_s'] = round(time.monotonic() - started, 2)
            raise StopIteration
        # Sensor loss: stop the depth bridge of this launch (by PID); the drone must hold, then land.
        bridges = processes_with('robot_lab_adapter.gz_camera_bridge', gz_environment().get('GZ_PARTITION', '?'))
        report['sensor_loss_bridge_pids'] = bridges
        for pid in bridges:
            os.kill(int(pid), signal.SIGTERM)
        loss_t = time.monotonic()
        until(lambda: planning.get('sensor_loss', {}).get('state') == 'holding', 30, 'sensor-loss hold')
        report['hold_after_s'] = round(time.monotonic() - loss_t, 2)
        until(lambda: status.get('phase') in ('landing', 'idle') and planning.get('sensor_loss', {}).get('state')
              in ('landing', 'ok'), 60, 'sensor-loss landing')
        report['landing_after_s'] = round(time.monotonic() - loss_t, 2)
        until(lambda: status.get('phase') == 'idle' and status.get('landed_state') == 1, 120, 'landed')
        report['landed_after_s'] = round(time.monotonic() - loss_t, 2)
    except StopIteration:
        pass
    except Exception as exc:
        report['error'] = '%s: %s' % (type(exc).__name__, exc)
    finally:
        stop.set()
        for pid in walkers.pop('_queries', []):                         # clock queries still in flight
            try:
                os.killpg(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        partition = (walkers.get('_partition') or [None])[0]
        if partition:
            for pid in processes_with('gz-transport-topic', partition):    # a clock query still in flight
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        report['states'] = states
        report['track_history'] = track_history
        report['max_moving_tracks'] = max((h[3] or 0 for h in track_history), default=0)
        report['last_planning'] = dict(planning)
        report['last_status'] = dict(status)
        report['tracks_seen'] = (planning.get('obstacles') or {}).get('tracks')
        # Drone-walker distances by time alignment of truth and actor samples.
        distances = {}
        partition = walkers.pop('_partition', None)
        report['walker_truth'] = 'scripted actor trajectories at Gazebo clock time (actors have no server poses)'
        report['gz_partition_found'] = bool(partition and partition[0])
        clock = walkers.pop('_clock', None) or []
        if len(clock) >= 2:
            mono = np.array([c[0] for c in clock])
            sims = np.array([c[1] for c in clock])
            best, closest = {}, {}
            for stamp, x, y, z in truth[::2]:
                if not mono[0] <= stamp <= mono[-1]:
                    continue
                sim = float(np.interp(stamp, mono, sims))
                for name, (wx, wy, wz) in walker_positions(sim).items():
                    # walker: a 1.8 m tall column standing on the ground
                    dz = max(0., z - (wz + 1.8))
                    distance = math.hypot(math.hypot(x - wx, y - wy), dz)
                    if distance < best.get(name, math.inf):
                        t = round(stamp - started, 2)
                        best[name] = distance
                        closest[name] = dict(t=t, drone=[round(x, 2), round(y, 2), round(z, 2)],
                                             walker=[round(wx, 2), round(wy, 2)],
                                             state=next((s[1] for s in reversed(states) if s[0] <= t), None))
            distances = {name: round(value, 3) for name, value in best.items()}
            report['closest_approach'] = closest
            report['walker_samples'] = dict(truth_samples=len(truth), clock_samples=len(clock))
        report['min_walker_distance_m'] = distances

        # Responses to observed moving obstacles: a replan, a walkway hold (looking both ways, timing
        # or overflying a walkway that moving tracks swept) or an evasive climb.
        holds = [s for s in states if s[1] in ('replanning', 'scanning', 'evading')]
        report['moving_holds_or_replans'] = len(holds)
        report['responses'] = {kind: sum(1 for s in holds if s[1] == kind) for kind in ('replanning', 'scanning', 'evading')}
        report['truth_samples'] = len(truth)
        if args.plain:
            report['passed'] = bool(not report.get('error') and report.get('goal_reached_s') and report.get('landed_after_s'))
        else:
          report['passed'] = bool(not report.get('error') and report.get('goal_reached_s') and holds
                                and all(d is not None and d > args.contact_m for d in distances.values()) and distances
                                and report.get('landed_after_s'))
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(launch.pid, signal.SIGINT)
            launch.wait(timeout=60)
        except Exception:
            try:
                os.killpg(launch.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        # The flight runtime starts PX4, Gazebo and RTAB-Map in their own sessions; stop any that
        # outlived the launch. They are identified by this run's folder in their command line.
        log.flush()
        runs = set(re.findall(r'(ros-flight-\d{8}-\d{6}-[0-9a-f]+)', (args.output / 'launch.log').read_text()))
        report['leftover_pids'] = []
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit() or int(proc.name) == os.getpid():
                continue
            try:
                cmdline = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            except OSError:
                continue
            if runs and any(run in cmdline for run in runs):
                report['leftover_pids'].append(int(proc.name))
                try:
                    os.kill(int(proc.name), signal.SIGTERM)
                except ProcessLookupError:
                    pass
        time.sleep(5.)
        for pid in report['leftover_pids']:
            try:
                os.kill(pid, signal.SIGKILL)       # gz sim can ignore SIGTERM while shutting down
            except ProcessLookupError:
                pass
        (args.output / 'report.json').write_text(json.dumps(report, indent=1) + '\n')
    print(json.dumps({k: report.get(k) for k in ('passed', 'error', 'goal_reached_s', 'min_walker_distance_m',
                                                 'moving_holds_or_replans', 'landed_after_s')}, indent=1))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
