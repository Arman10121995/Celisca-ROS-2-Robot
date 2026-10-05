#!/usr/bin/env python3
"""Measure GUI-command movement, loss, reset and live mapping in an existing launch."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, Image, PointCloud2
from std_srvs.srv import Trigger
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from sensor_msgs_py import point_cloud2


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def distance(a, b):
    return math.hypot(a['x']-b['x'], a['y']-b['y'])


class Workflow(Node):
    def __init__(self):
        super().__init__('sim_workflow_check')
        self.samples = []
        self.clock_samples = []
        self.counts = {}
        self.map_updates = []
        self.cloud_points = []
        self.cloud_summary = None
        self.map_alignment = None
        self.last_map = None
        self.scan_summary = None
        self.slam_poses = []
        self.command_pub = self.create_publisher(Twist, '/key_vel', 10)
        self.reset_client = self.create_client(Trigger, '/robot_lab/reset')
        self.create_subscription(Odometry, '/odom/ground_truth', self.on_truth, qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', lambda m: self.clock_samples.append(
            m.clock.sec + m.clock.nanosec*1e-9), 10)
        for topic, typ in (('/scan', LaserScan), ('/oakd/rgb/image_raw', Image),
                           ('/oakd/depth/image_raw', Image), ('/cloud_map', PointCloud2),
                           ('/rtabmap/cloud_map', PointCloud2)):
            self.create_subscription(typ, topic, lambda m, key=topic: self.on_sensor(key, m), qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/pose',
            lambda m:self.slam_poses.append({'x':m.pose.pose.position.x,'y':m.pose.pose.position.y}),10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.samples.append({'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
            'arrival': time.monotonic(), 'x': p.x, 'y': p.y, 'z': p.z, 'yaw': yaw(q)})

    def on_sensor(self, topic, msg):
        self.counts[topic] = self.counts.get(topic, 0)+1
        if topic == '/scan':
            values = [float(v) for v in msg.ranges if math.isfinite(v)]
            self.scan_summary = {'frame':msg.header.frame_id, 'samples':len(msg.ranges),
                'finite':len(values), 'min_m':min(values) if values else None,
                'max_m':max(values) if values else None, 'range_min':msg.range_min,
                'range_max':msg.range_max}
        if topic.endswith('/cloud_map'):
            self.cloud_points.append(msg.width*msg.height)
            # Inspect real reconstructed points, not just a nonempty header.
            step = max(1, msg.width*msg.height // 5000)
            points = [tuple(p) for i,p in enumerate(point_cloud2.read_points(
                msg, field_names=('x','y','z'), skip_nans=True))
                if i % step == 0 and all(math.isfinite(float(v)) for v in p)]
            if points:
                self.cloud_summary = {'frame': msg.header.frame_id, 'sampled_points': len(points),
                    'min_xyz': [min(float(p[a]) for p in points) for a in range(3)],
                    'max_xyz': [max(float(p[a]) for p in points) for a in range(3)]}

    def on_map(self, msg):
        self.last_map = msg
        self.map_updates.append({'width': msg.info.width, 'height': msg.info.height,
            'known': sum(v >= 0 for v in msg.data), 'occupied': sum(v >= 65 for v in msg.data),
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9})

    def spin_until(self, predicate, timeout):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def estimate(self):
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
            return {'x': tr.translation.x, 'y': tr.translation.y, 'yaw': yaw(tr.rotation)}
        except Exception:
            return None

    def align_mapping_frame(self):
        # SLAM initializes its own map frame; it need not share world origin.
        # Measure that single rigid transform before motion and hold it fixed.
        estimate = self.estimate()
        truth = self.samples[-1]
        angle = truth['yaw']-estimate['yaw']
        c,s = math.cos(angle), math.sin(angle)
        self.map_alignment = {'yaw':angle, 'x':truth['x']-c*estimate['x']+s*estimate['y'],
                              'y':truth['y']-s*estimate['x']-c*estimate['y']}

    def localization_error(self):
        estimate = self.estimate()
        if estimate is None:
            return None
        if self.map_alignment is not None:
            tr = self.map_alignment
            c,s = math.cos(tr['yaw']), math.sin(tr['yaw'])
            estimate = {'x':tr['x']+c*estimate['x']-s*estimate['y'],
                        'y':tr['y']+s*estimate['x']+c*estimate['y']}
        return distance(estimate,self.samples[-1])

    def phase(self, duration, vx=0.0, vy=0.0, wz=0.0, publish=True):
        first = len(self.samples)-1
        begin = self.samples[-1]['stamp']
        end = time.monotonic()+max(45, duration*30)
        next_send = 0.0
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = vx, vy, wz
        while self.samples[-1]['stamp'] < begin+duration and time.monotonic() < end:
            if publish and time.monotonic() >= next_send:
                self.command_pub.publish(message)
                next_send = time.monotonic()+0.1
            rclpy.spin_once(self, timeout_sec=0.02)
        points = self.samples[first:]
        a, b = points[0], points[-1]
        dt = b['stamp']-a['stamp']
        tail = [p for p in points if p['stamp'] >= b['stamp']-0.5]
        speed = None
        if len(tail) > 1 and tail[-1]['stamp'] > tail[0]['stamp']:
            speed = distance(tail[0], tail[-1])/(tail[-1]['stamp']-tail[0]['stamp'])
        yaw_speed = (abs(math.atan2(math.sin(tail[-1]['yaw']-tail[0]['yaw']),
                      math.cos(tail[-1]['yaw']-tail[0]['yaw']))) /
                     (tail[-1]['stamp']-tail[0]['stamp'])) if len(tail)>1 else None
        angle = sum(math.atan2(math.sin(n['yaw']-p['yaw']), math.cos(n['yaw']-p['yaw']))
                    for p, n in zip(points, points[1:]))
        return {'completed': dt >= duration-0.05, 'sim_s': round(dt, 3),
                'distance_m': round(distance(a, b), 4), 'yaw_change_rad': round(angle, 4),
                'tail_speed_mps': speed, 'tail_yaw_speed_rps': yaw_speed,
                'end': {k:b[k] for k in ('x','y','z','yaw')}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('loc','slam','3d_slam'), default='loc')
    parser.add_argument('--lateral', action='store_true')
    parser.add_argument('--timeout', type=float, default=240)
    parser.add_argument('--skip-reset', action='store_true')
    parser.add_argument('--drive-node',default='mujoco_spawner')
    parser.add_argument('--expected-steering-mode',choices=('ackermann','crab','in_phase','pivot'))
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Workflow()
    report = {'mode': args.mode, 'command_topic': '/key_vel', 'phases': {}, 'checks': {}}
    checks = report['checks']
    try:
        ready = node.spin_until(lambda: bool(node.samples) and node.counts.get('/scan',0)>1
                                and node.estimate() is not None, args.timeout)
        checks['ready'] = ready
        if ready and args.expected_steering_mode:
            from robot_lab_utils.qualification import inspect_drive_mode
            report['drive_configuration']=inspect_drive_mode(node,args.drive_node,args.expected_steering_mode)
            checks['steering_configuration']=report['drive_configuration']['passed']
            ready=checks['steering_configuration']
            if not ready:report['error']='Running steering mode differs from the requested trial'
        if not ready:
            report.setdefault('error','No fresh truth, scan or map-to-base TF within timeout')
        else:
            checks['settled'] = node.phase(2)['completed']
            if args.mode != 'loc':
                node.align_mapping_frame()
                report['initial_map_to_world'] = dict(node.map_alignment)
            initial = node.samples[-1].copy()
            report['initial_truth'] = {k:initial[k] for k in ('x','y','z','yaw')}
            before = dict(node.counts)
            for label, command in [('forward',(0.2,0.0,0.0)),('reverse',(-0.2,0.0,0.0)),
                                   ('turn',(0.0,0.0,0.4)),('turn_reverse',(0.0,0.0,-0.4))]:
                measured = node.phase(3, *command)
                report['phases'][label] = measured
                checks[label] = measured['completed'] and (
                    measured['yaw_change_rad'] > 0.7 if label == 'turn' else
                    measured['yaw_change_rad'] < -0.7 if label == 'turn_reverse' else
                    measured['distance_m'] > 0.25)
                node.phase(1)
            if args.lateral:
                measured = node.phase(3, vy=0.2)
                report['phases']['strafe'] = measured
                checks['strafe'] = measured['completed'] and measured['distance_m'] > 0.25
                node.phase(1)
                measured = node.phase(3, vy=-0.2)
                report['phases']['strafe_reverse'] = measured
                checks['strafe_reverse'] = measured['completed'] and measured['distance_m'] > 0.25
                node.phase(1)
            node.phase(2, vx=0.2)
            node.destroy_publisher(node.command_pub)
            silence = node.phase(3, publish=False)
            report['phases']['publisher_loss'] = silence
            checks['publisher_loss'] = (silence['completed'] and silence['distance_m'] < 0.15 and
                                       silence['tail_speed_mps'] is not None and silence['tail_speed_mps'] < 0.02
                                       and silence['tail_yaw_speed_rps'] is not None
                                       and silence['tail_yaw_speed_rps'] < 0.02)
            report['localization_error_m'] = node.localization_error()
            checks['localization'] = (report['localization_error_m'] is not None and
                                      report['localization_error_m'] < 0.25)
            report['sensor_messages_during_drive'] = {k:v-before.get(k,0) for k,v in node.counts.items()}
            if args.mode == 'slam':
                report['map_updates'] = node.map_updates
                report['accepted_slam_poses'] = list(node.slam_poses)
                checks['live_2d_map'] = (len(node.map_updates) > 1 and node.map_updates[-1]['known'] > 100 and
                                         node.map_updates[-1]['occupied'] > 10 and len(node.slam_poses)>2)
            if args.mode == '3d_slam':
                report['cloud_message_points'] = node.cloud_points
                checks['live_3d_cloud'] = (len(node.cloud_points)>1 and max(node.cloud_points)>100)
                report['cloud_geometry'] = node.cloud_summary
                checks['3d_geometry'] = (node.cloud_summary is not None and
                    node.cloud_summary['max_xyz'][2]-node.cloud_summary['min_xyz'][2] > 0.3 and
                    max(node.cloud_summary['max_xyz'][a]-node.cloud_summary['min_xyz'][a]
                        for a in (0,1)) > 1.0)
            if not args.skip_reset:
                before_reset_maps, before_reset_clouds = len(node.map_updates), len(node.cloud_points)
                node.command_pub = node.create_publisher(Twist, '/key_vel', 10)
                node.command_pub.publish(Twist())
                available = node.reset_client.wait_for_service(timeout_sec=5)
                future = node.reset_client.call_async(Trigger.Request()) if available else None
                responded = future is not None and node.spin_until(future.done, 15)
                checks['reset_service'] = responded and future.result().success
                report['reset_response'] = future.result().message if responded else 'reset service did not respond'
                returned = node.spin_until(lambda: distance(initial,node.samples[-1]) < 0.05, 20)
                checks['reset_spawn'] = returned
                idle = node.phase(2, publish=False)
                report['phases']['post_reset_idle'] = idle
                checks['reset_stopped'] = idle['completed'] and idle['distance_m'] < 0.05
                if args.mode != 'loc' and node.estimate() is not None:
                    node.align_mapping_frame()
                    report['post_reset_map_to_world'] = dict(node.map_alignment)
                report['post_reset_localization_error_m'] = node.localization_error()
                checks['reset_localization'] = (report['post_reset_localization_error_m'] is not None
                                               and report['post_reset_localization_error_m']<0.25)
                resumed = node.phase(2, vx=0.2)
                report['phases']['post_reset_drive'] = resumed
                checks['reset_drive_resumes'] = resumed['completed'] and resumed['distance_m']>0.15
                node.phase(1)
                report['post_reset_drive_localization_error_m'] = node.localization_error()
                checks['reset_estimator_tracks_motion'] = (report['post_reset_drive_localization_error_m']
                    is not None and report['post_reset_drive_localization_error_m'] < 0.25)
                if args.mode == 'slam':
                    checks['reset_mapping_resumes'] = len(node.map_updates)>before_reset_maps
                elif args.mode == '3d_slam':
                    checks['reset_mapping_resumes'] = len(node.cloud_points)>before_reset_clouds
                report['map_updates_after_reset'] = len(node.map_updates)-before_reset_maps
                report['cloud_updates_after_reset'] = len(node.cloud_points)-before_reset_clouds
            report['clock_backward_jumps'] = sum(b<a-1e-8 for a,b in zip(node.clock_samples,node.clock_samples[1:]))
            checks['monotonic_clock'] = report['clock_backward_jumps'] == 0
            report['truth_samples'] = len(node.samples)
            report['sensor_counts'] = node.counts
            report['scan_geometry'] = node.scan_summary
    except Exception as exc:
        report['error'] = str(exc)
        checks['completed'] = False
    finally:
        report['passed'] = bool(checks) and all(checks.values())
        out = Path(args.out)
        out.write_text(json.dumps(report, indent=2)+'\n')
        out.with_suffix('.trace.json').write_text(json.dumps({'samples':node.samples})+'\n')
        if args.mode == 'slam' and node.last_map is not None:
            import numpy as np
            grid = node.last_map
            np.savez_compressed(out.with_suffix('.map.npz'), data=np.asarray(grid.data,dtype=np.int8),
                width=grid.info.width, height=grid.info.height, resolution=grid.info.resolution,
                origin=[grid.info.origin.position.x,grid.info.origin.position.y,
                        yaw(grid.info.origin.orientation)])
        print(json.dumps(report))
        if node.command_pub is not None:
            try:
                node.command_pub.publish(Twist())
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
