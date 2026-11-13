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


def gz_actor_poses(world, names, stop, samples):
    """Background sampling of actor positions from Gazebo's pose topic (gz CLI)."""
    pattern = re.compile(r'name: "(%s)".*?position \{\s*x: ([-\d.e]+)\s*y: ([-\d.e]+)(?:\s*z: ([-\d.e]+))?' %
                         '|'.join(map(re.escape, names)), re.S)
    while not stop.is_set():
        try:
            text = subprocess.run(['gz', 'topic', '-e', '-t', '/world/%s/pose/info' % world, '-n', '1'],
                                  capture_output=True, text=True, timeout=10).stdout
        except subprocess.TimeoutExpired:
            continue
        now = time.monotonic()
        for name, x, y, z in pattern.findall(text):
            samples.setdefault(name, []).append((now, float(x), float(y), float(z or 0.)))
        stop.wait(.3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--goal', type=float, nargs=3, default=(10., 0., 1.2))
    parser.add_argument('--contact-m', type=float, default=.6, help='minimum drone-walker centre distance')
    parser.add_argument('--timeout', type=float, default=900.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    report = dict(world='aerial_moving', goal=list(args.goal), phases=[], passed=False)
    command = ['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py', 'robot_model:=px4_x500',
               'simulator:=gazebo', 'mode:=flight', 'map_name:=aerial_moving', 'px4_vehicle:=x500_depth',
               'aerial_mapping:=rtabmap', 'aerial_obstacles:=mapped_depth', 'gui:=false', 'start_rviz:=false']
    report['command'] = command
    log = open(args.output / 'launch.log', 'w')
    launch = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    stop, walkers = threading.Event(), {}
    rclpy.init()
    node = rclpy.create_node('drone_moving_check')
    status, planning, truth = {}, {}, []
    states = []
    node.create_subscription(String, '/px4/status', lambda m: status.update(json.loads(m.data)), 10)

    def on_planning(message):
        data = json.loads(message.data)
        planning.clear()
        planning.update(data)
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
        until(lambda: (planning.get('obstacles') or {}).get('ready'), 300, 'observed layer ready')
        sampler = threading.Thread(target=gz_actor_poses, args=('robot_lab_flight', ['walker_0', 'walker_1'], stop, walkers),
                                   daemon=True)
        sampler.start()
        call('takeoff_check')
        until(lambda: planning.get('takeoff', {}).get('state') == 'clear', 120, 'takeoff clearance')
        if not call('takeoff'):
            raise RuntimeError('takeoff refused')
        until(lambda: status.get('phase') == 'flying', 180, 'airborne')
        spin(3.)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = args.goal
        goal.pose.orientation.w = 1.
        goal_pub.publish(goal)
        until(lambda: planning.get('state') == 'planned', 120, 'planned')
        call('plan_execute')
        until(lambda: math.dist(status.get('position_enu') or (1e9, 0, 0), args.goal) < .4, 600, 'goal reached')
        report['goal_reached_s'] = round(time.monotonic() - started, 2)
        report['final_position'] = status.get('position_enu')
        spin(3.)
        # Sensor loss: stop the depth bridge of this launch (by PID); the drone must hold, then land.
        bridges = subprocess.run(['pgrep', '-s', str(launch.pid), '-f', 'robot_lab_adapter.gz_camera_bridge'],
                                 capture_output=True, text=True).stdout.split()
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
    except Exception as exc:
        report['error'] = '%s: %s' % (type(exc).__name__, exc)
    finally:
        stop.set()
        report['states'] = states
        report['tracks_seen'] = (planning.get('obstacles') or {}).get('tracks')
        # Drone-walker distances by time alignment of truth and actor samples.
        distances = {}
        for name, samples in walkers.items():
            best = math.inf
            for t, x, y, z in samples:
                near = min(truth, key=lambda s: abs(s[0] - t), default=None)
                if near and abs(near[0] - t) < .3:
                    # walker modelled as a 0.3 m radius, 1.8 m tall column standing on z
                    dz = max(0., near[3] - (z + 1.8)) if near[3] > z else 0.
                    best = min(best, math.hypot(math.hypot(near[1] - x, near[2] - y), dz))
            distances[name] = round(best, 3) if math.isfinite(best) else None
        report['min_walker_distance_m'] = distances
        report['walker_samples'] = {name: len(s) for name, s in walkers.items()}
        holds = [s for s in states if s[3] and 'moving obstacle' in s[3]] + \
                [s for s in states if s[1] in ('replanning',)]
        report['moving_holds_or_replans'] = len(holds)
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
        (args.output / 'report.json').write_text(json.dumps(report, indent=1) + '\n')
    print(json.dumps({k: report.get(k) for k in ('passed', 'error', 'goal_reached_s', 'min_walker_distance_m',
                                                 'moving_holds_or_replans', 'landed_after_s')}, indent=1))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
