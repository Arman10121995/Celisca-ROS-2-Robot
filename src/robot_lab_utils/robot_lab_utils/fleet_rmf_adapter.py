#!/usr/bin/python3 -s
"""Humble Open-RMF RobotCommandHandle -> actual shared-world Nav2 missions.

Uses the official Apache-2.0 Open-RMF Python API (fleet_adapter_template,
humble). Battery telemetry, charging, doors and lifts are deliberately absent.
The nominal RMF cost objects below are disabled, never measured energy data.
"""
import argparse
from datetime import timedelta
import json
import math
from pathlib import Path
from queue import SimpleQueue
import threading
import time
import uuid

import numpy as np
if int(np.__version__.split('.')[0]) >= 2:
    raise RuntimeError('Humble Open-RMF Eigen bindings require system NumPy 1.x; run /usr/bin/python3 -s, as the installed executable does')
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import rmf_adapter as adpt
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan
import rmf_adapter.vehicletraits as traits

from robot_lab_utils.fleet_missions import rmf_navigation_graph
from robot_lab_utils.fleet_session import SpawnGrid


class Robot(adpt.RobotCommandHandle):
    def __init__(self, owner, name):
        super().__init__()
        self.owner, self.name = owner, name
        self.update = None
        self.lock = threading.RLock()
        self.request = None
        self.sequence = 0

    def follow_new_path(self, waypoints, next_arrival_estimator, path_finished_callback):
        with self.lock:
            if not self.owner.enabled:
                return  # A late negotiation cannot reacquire disabled ownership.
            self.sequence += 1
            if not 1 <= len(waypoints) <= 64:
                if self.update:
                    self.update.log_error('Robot Lab requires a path of 1–64 waypoints')
                return
            positions = [list(map(float, wp.position)) for wp in waypoints]
            request_id = 'rmf-' + uuid.uuid4().hex
            self.request = dict(id=request_id, positions=positions, arrived=set(),
                estimator=next_arrival_estimator, finished=path_finished_callback, sequence=self.sequence,
                acknowledged=False, failed=False, sent=False, accepted_at=time.monotonic(),
                departures=[max(0.,waypoints[max(0,index-1)].time.timestamp()) for index in range(len(positions))])
            self.owner.pending.put((self.name, self.sequence, dict(op='route', targets=[self.name],
                request_id=request_id, goals=[dict(x=p[0], y=p[1], yaw=p[2], timeout=180.,
                    not_before=max(0.,waypoints[max(0,index-1)].time.timestamp())) for index,p in enumerate(positions)])))

    def stop(self):
        with self.lock:
            self.sequence += 1
            self.request = None
            self.owner.pending.put((self.name, self.sequence, dict(op='stop', targets=[self.name])))

    def dock(self, dock_name, docking_finished_callback):
        self.stop()
        if self.update:
            self.update.log_error('No physical docking adapter installed: ' + str(dock_name))
        # Never report docking success without a physical adapter.

    def observe(self, row, outcomes):
        position = row.get('estimate')
        if not self.update or not row.get('ready') or not position:
            return
        p = [position[k] for k in ('x', 'y', 'yaw')]
        # Humble's update_position binding takes Start sets. Use tight merge
        # bounds and retain actual XY on lanes; a one-metre waypoint snap can
        # place two distinct bodies on the same holding node.
        starts = plan.compute_plan_starts(self.owner.graph, self.owner.session['map'], p,
                                         self.owner.adapter.now(), .15, .35)
        if starts:
            self.update.update_position(starts)
        else:
            self.update.update_lost_position(self.owner.session['map'], np.array(p), .15, .35)
        with self.lock:
            request = self.request
            if request is None or request['failed']:
                return
            source = 'route:' + request['id']
            matching = [o for o in outcomes if o['source'] == source]
            waiting = [o for o in row.get('queued_tasks', []) if o['source'] == source]
            request['acknowledged'] |= bool(matching or waiting)
            if waiting and row.get('queue_paused'):
                request['failed'] = True
                self.update.log_error('Fleet recovery paused the route; requesting a new RMF plan from actual position')
                self.update.replan()
                return
            for outcome in matching:
                if outcome['state'] != 'succeeded':
                    request['failed'] = True
                    self.update.log_error('Robot Lab task ended: ' + outcome['reason'])
                    self.update.replan()
                    return
                request['arrived'].add(outcome['task'])
            index = len(request['arrived'])
            if index >= len(request['positions']):
                callback = request['finished']
                self.request = None
                callback()  # Only after every real Nav2 + measured-settle result.
            elif request['acknowledged']:
                target = request['positions'][index]
                wait = max(0.,request['departures'][index]-self.owner.adapter.now().timestamp())
                request['estimator'](index, timedelta(seconds=wait+math.hypot(p[0] - target[0], p[1] - target[1]) / .15))
            elif request['sent'] and time.monotonic() - request['accepted_at'] > 10.:
                request['failed'] = True
                self.update.log_error('No actual fleet acknowledgement for the RMF route')
                self.update.replan()


class RmfBridge(Node):
    def __init__(self, session, adapter, nav_graph, fleet, vertices):
        super().__init__('robot_lab_rmf_bridge')
        self.session, self.adapter, self.graph, self.fleet = session, adapter, nav_graph, fleet
        self.vertices = vertices
        self.operator = 'rmf-' + uuid.uuid4().hex
        self.enabled = False
        self.traffic_requested = False
        self.runtime = None
        self.runtime_at = -math.inf
        self.pending = SimpleQueue()
        self.robots = {}
        self.command = self.create_publisher(String, '/fleet/command', 10)
        self.heartbeat = self.create_publisher(String, '/fleet/operator_heartbeat', 10)
        self.status = self.create_publisher(String, '/fleet/rmf/state', 10)
        self.create_subscription(String, '/fleet/state', self.observe, 10)
        self.create_subscription(String, '/fleet/rmf/control', self.control, 10)
        self.create_timer(.2, self.tick)

    def send(self, command):
        self.command.publish(String(data=json.dumps(dict(session_id=Path(self.session['directory']).name,
            operator_id=self.operator, **command), allow_nan=False)))

    def control(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('session_id') != Path(self.session['directory']).name:
                return
            if data['op'] == 'enable':
                self.enabled = True
                self.traffic_requested = False
            elif data['op'] == 'disable':
                self.send(dict(op='all_stop', targets=[]))
                self.enabled = False
                for robot in self.robots.values():
                    robot.stop()
            elif data['op'] == 'patrol':
                if not self.enabled or data['robot'] not in self.robots:
                    raise ValueError('Enable RMF and select an initialized robot')
                target = data['waypoint']
                if target not in {v['name'] for v in self.vertices}:
                    raise ValueError('Choose a waypoint from the actual generated graph')
                robot = self.robots[data['robot']]
                if not robot.update:
                    raise ValueError('Robot RMF update handle is not ready')
                robot.update.submit_direct_request(dict(category='patrol', description=dict(places=[target], rounds=1)),
                    'gui-' + uuid.uuid4().hex, lambda response: self.get_logger().info('RMF task response: ' + json.dumps(response)))
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning('RMF request rejected: ' + str(exc))

    def observe(self, msg):
        try:
            state = json.loads(msg.data)
            if state['session_id'] != Path(self.session['directory']).name:
                return
            self.runtime, self.runtime_at = state, time.monotonic()
            for row in state['members']:
                if row['id'] not in self.robots and row.get('ready') and row.get('estimate'):
                    p = [row['estimate'][k] for k in ('x', 'y', 'yaw')]
                    starts = plan.compute_plan_starts(self.graph, self.session['map'], p, self.adapter.now(), .15, .35)
                    if not starts:
                        continue
                    robot = Robot(self, row['id'])
                    self.robots[row['id']] = robot
                    self.fleet.add_robot(robot, row['id'], self.profile, starts,
                                         lambda update, r=robot: setattr(r, 'update', update))
                robot = self.robots.get(row['id'])
                if robot:
                    robot.observe(row, state.get('task_history', []))
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning('RMF state rejected: ' + str(exc))

    def tick(self):
        if self.enabled and time.monotonic() - self.runtime_at > 1.:
            self.send(dict(op='all_stop', targets=[]))
            self.enabled = False
            for robot in self.robots.values():
                robot.stop()
        if self.enabled:
            self.heartbeat.publish(String(data=json.dumps(dict(session_id=Path(self.session['directory']).name,
                                                               operator_id=self.operator))))
        if self.enabled and self.runtime and self.runtime['operator_id'] == self.operator:
            if self.runtime.get('traffic',{}).get('serial',True):
                if not self.traffic_requested and all(not m.get('nav2_pending') and not m.get('cancel_pending')
                        and m['mode'] in ('idle','waiting') for m in self.runtime['members']):
                    self.send(dict(op='traffic',targets=[],serial=False))
                    self.traffic_requested = True
            while not self.pending.empty() and not self.runtime.get('traffic',{}).get('serial',True):
                name, sequence, command = self.pending.get()
                robot = self.robots[name]
                with robot.lock:
                    if sequence == robot.sequence:
                        self.send(command)
                        if robot.request:
                            robot.request['sent'] = True
                            robot.request['accepted_at'] = time.monotonic()
        routes = {}
        for name, robot in self.robots.items():
            with robot.lock:
                request = robot.request
                routes[name] = (dict(id=request['id'], acknowledged=request['acknowledged'],
                    failed=request['failed'], arrived=len(request['arrived']),
                    total=len(request['positions']), positions=request['positions']) if request else None)
        self.status.publish(String(data=json.dumps(dict(enabled=self.enabled,
            session_id=Path(self.session['directory']).name, operator_id=self.operator,
            robots=list(self.robots), waypoints=self.vertices, routes=routes, battery=None,
            scope='Actual Open-RMF adapter; energy accounting disabled; no doors/lifts/docking or mission qualification'), allow_nan=False)))


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent()
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True)
    options = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    session = json.loads(Path(options.session).read_text())
    data = rmf_navigation_graph(SpawnGrid(session['map_yaml']), session['members'])
    (Path(session['directory']) / 'rmf-graph.json').write_text(json.dumps(data, indent=2) + '\n')
    adpt.init_rclcpp()
    adapter = adpt.Adapter.make('robot_lab_rmf_adapter')
    if not adapter:
        raise RuntimeError('Open-RMF schedule must be running before the fleet adapter')
    adapter.node.use_sim_time()
    graph_value = graph.Graph()
    for v in data['vertices']:
        wp = graph_value.add_waypoint(session['map'], np.array([v['x'], v['y']]))
        wp.holding_point = True
        # Humble requires a charger graph anchor even with energy accounting
        # disabled. This is a LOGICAL API holding anchor, not a charger or a
        # claim of battery/docking physics. Charge/dock tasks are unsupported.
        if v['name'] == 'p0':
            wp.charger = True
        graph_value.add_key(v['name'], wp.index)
    for first, second in data['lanes']:
        graph_value.add_bidir_lane(first, second)
    radius = max(m['radius'] for m in session['members'])
    profile = traits.Profile(geometry.make_final_convex_circle(radius), geometry.make_final_convex_circle(radius + .15))
    vehicle = traits.VehicleTraits(traits.Limits(.25, .3), traits.Limits(.6, .8), profile)
    adapter.start()
    fleet = adapter.add_fleet('robot_lab', vehicle, graph_value)
    # Required RMF API cost objects are nominal and explicitly unused for
    # energy accounting. Do not publish the library's default battery values.
    fleet.fleet_state_publish_period(None)
    fleet.fleet_state_topic_publish_period(None)
    power = battery.PowerSystem.make(1.)
    nominal_battery = battery.BatterySystem.make(24., 1., 1.)
    motion = battery.SimpleMotionPowerSink(nominal_battery, battery.MechanicalSystem.make(1., 1., .1))
    device = battery.SimpleDevicePowerSink(nominal_battery, power)
    if not fleet.set_task_planner_params(nominal_battery, motion, device, device, 0., 1., False, 'nothing'):
        raise RuntimeError('Open-RMF task planner initialization failed')
    def consider_patrol(description):
        # Humble's binding takes one JSON argument and returns Confirmation;
        # newer callback examples with a second argument abort this C++ ABI.
        confirmation = adpt.fleet_update_handle.Confirmation()
        confirmation.accept()
        return confirmation
    fleet.consider_patrol_requests(consider_patrol)
    bridge = RmfBridge(session, adapter, graph_value, fleet, data['vertices'])
    bridge.profile = profile
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            bridge.send(dict(op='all_stop', targets=[]))
        adapter.stop()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
