"""Record actual isolated-run ROS telemetry; unavailable metrics stay null."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    options, ros_args = parser.parse_known_args(args)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState, LaserScan
    from geometry_msgs.msg import PoseWithCovarianceStamped

    options.output.mkdir(parents=True, exist_ok=True)
    raw = (options.output/'telemetry.jsonl').open('w', buffering=1)
    rclpy.init(args=ros_args)
    node = Node('robot_lab_experiment_recorder')
    counts = Counter()
    clocks = []
    positions, invalid_truth, last_messages = {}, set(), {}
    last_clock_wall = None

    def record(topic, payload):
        counts[topic] += 1
        last_messages[topic] = time.monotonic()
        raw.write(json.dumps(dict(topic=topic, monotonic=last_messages[topic], **payload), allow_nan=False)+'\n')

    def clock(message):
        nonlocal last_clock_wall
        last_clock_wall = time.monotonic()
        value = message.clock.sec+message.clock.nanosec*1e-9
        if not clocks:
            clocks.append((last_clock_wall, value))
        if len(clocks) == 1:
            clocks.append((last_clock_wall, value))
        else:
            clocks[-1] = (last_clock_wall, value)
        record('/clock', dict(sim_time=value))

    def truth(message, topic='/odom/ground_truth'):
        p = message.pose.pose.position
        values = [p.x, p.y, p.z]
        if not all(map(math.isfinite, values)):
            invalid_truth.add(topic)
            record(topic, dict(valid=False))
            return
        trace = positions.setdefault(topic, [])
        if trace:
            trace.append([*values, math.dist(trace[-1][:3], values)+trace[-1][3]])
            trace[:] = trace[-2:]
        else:
            trace.append([*values, 0.])
        q = message.pose.pose.orientation
        record(topic, dict(position=values, frame=message.header.frame_id,
            sim_time=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
            velocity=[v if math.isfinite(v) else None for v in (message.twist.twist.linear.x,message.twist.twist.linear.y,message.twist.twist.angular.z)],
            position_covariance=[message.pose.covariance[i] if math.isfinite(message.pose.covariance[i]) else None for i in (0,7,14)],
            orientation_xyzw=[v if math.isfinite(v) else None for v in (q.x, q.y, q.z, q.w)]))

    def joints(message):
        record('/joint_states', dict(names=message.name,
            velocities=[value if math.isfinite(value) else None for value in message.velocity],
            efforts=[value if math.isfinite(value) else None for value in message.effort],
            positions=[value if math.isfinite(value) else None for value in message.position]))

    def scan(message):
        values = [float(value) for value in message.ranges
                  if math.isfinite(value) and message.range_min <= value <= message.range_max]
        record('/scan', dict(finite_returns=len(values), nearest_range=min(values) if values else None))

    from std_msgs.msg import String
    def state(message,topic):
        try:
            if len(message.data)>128*1024:return
            record(topic,dict(state=json.loads(message.data)))
        except (ValueError,TypeError):record(topic,dict(valid=False))
    for topic in ('/aerial_swarm/state','/gait/odometry_state','/marine/mission/state'):
        node.create_subscription(String,topic,lambda message,topic=topic:state(message,topic),10)
    def localized(message):
        p,q=message.pose.pose.position,message.pose.pose.orientation
        values=[p.x,p.y,q.x,q.y,q.z,q.w,*[message.pose.covariance[i] for i in (0,7,35)]]
        record('/gait/amcl_pose',dict(frame=message.header.frame_id,sim_time=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
            pose_and_covariance=[v if math.isfinite(v) else None for v in values]))
    node.create_subscription(PoseWithCovarianceStamped,'/gait/amcl_pose',localized,10)
    node.create_subscription(Clock, '/clock', clock, 10)
    node.create_subscription(Odometry, '/odom/ground_truth', truth, qos_profile_sensor_data)
    for topic in ('/px4/odometry_truth', '/px4/odometry', '/gait/odometry', '/marine/odometry', '/marine/estimated_odometry'):
        node.create_subscription(Odometry, topic, lambda message, topic=topic: truth(message, topic), qos_profile_sensor_data)
    node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)

    def summary():
        wall = clocks[-1][0]-clocks[0][0] if len(clocks) > 1 else 0
        sim = clocks[-1][1]-clocks[0][1] if len(clocks) > 1 else 0
        payload = dict(topic_counts=dict(counts), last_clock_monotonic=last_clock_wall,
            last_topic_monotonic=dict(last_messages),
            real_time_factor=sim/wall if wall > 0 and sim >= 0 else None,
            truth_distance_3d_m=(positions['/odom/ground_truth'][-1][3] if counts['/odom/ground_truth'] >= 2
                                 and '/odom/ground_truth' not in invalid_truth else None),
            topic_trace_distance_3d_m={topic: trace[-1][3] if counts[topic] >= 2 and topic not in invalid_truth else None
                for topic, trace in positions.items()},
            contact_events=None, footprint_clearance_m=None, mission_result=None,
            scope='Actual topic capture only; scan range is not footprint clearance; no inferred mission success')
        temporary = options.output/'telemetry-summary.new'
        temporary.write_text(json.dumps(payload, indent=2)+'\n')
        temporary.replace(options.output/'telemetry-summary.json')

    node.create_timer(1., summary)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        summary()
        raw.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
