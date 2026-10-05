"""Restore estimator/map state after an acknowledged physical robot reset."""
import math
import copy

from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, Trigger
from rosgraph_msgs.msg import Clock as ClockMessage


class ResetNotifications:
    def __init__(self, node):
        self.node = node
        self.remaining = 0
        self.publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.mapper_clients = []
        self.mapper_clients.append((node.create_client(Trigger, '/robot_lab/mapping_reset'), Trigger))
        self.mapper_clients.append((node.create_client(Empty, '/rtabmap/reset'), Empty))
        self.estimator = None
        self.waiting_for_odom = False
        self.reset_stamp = 0
        self.reset_started = 0
        self.sim_stamp = None
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        # Estimator reset is optional in robot-only Display.
        try:
            from robot_localization.srv import SetPose
            self.estimator_type = SetPose
            self.estimator = node.create_client(SetPose, '/set_pose')
        except ImportError:
            pass
        raw_topic = (node._odom_pub.topic_name if hasattr(node,'_odom_pub')
                     else '/robot_lab_controller/odom')
        node.create_subscription(Odometry, raw_topic, self.on_odom, 10)
        node.create_subscription(ClockMessage,'/clock',self.on_clock,10)
        self.timer = node.create_timer(0.25, self.publish_pose, clock=self.steady_clock)

    def notify(self):
        stamp = self.stamp()
        self.reset_stamp = (int(self.node._sim_t*1e9) if hasattr(self.node,'_sim_t')
                            else stamp.sec*1_000_000_000+stamp.nanosec)
        self.reset_started = self.steady_clock.now().nanoseconds
        self.waiting_for_odom = self.estimator is not None and self.estimator.service_is_ready()
        if not self.waiting_for_odom:
            self.reset_mappers()

    def on_clock(self,msg):
        self.sim_stamp = msg.clock

    def stamp(self):
        # Native workers schedule control on wall time but stamp data with
        # their physics clock. Estimator notifications must use that clock.
        return copy.deepcopy(self.sim_stamp) if self.sim_stamp is not None else self.node.get_clock().now().to_msg()

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if not self.waiting_for_odom or stamp < self.reset_stamp:
            return
        self.waiting_for_odom = False
        request = self.estimator_type.Request()
        request.pose.header = copy.deepcopy(msg.header)
        request.pose.pose = copy.deepcopy(msg.pose)
        request.pose.pose.covariance[0] = request.pose.pose.covariance[7] = 0.0025
        request.pose.pose.covariance[35] = 0.0076
        future = self.estimator.call_async(request)
        future.add_done_callback(self.estimator_reset_done)

    def estimator_reset_done(self, future):
        try:
            future.result()
            self.reset_mappers()
        except Exception as exc:
            self.node.get_logger().error(f'Estimator reset failed: {exc}')

    def reset_mappers(self):
        for client, typ in self.mapper_clients:
            if client.service_is_ready():
                future = client.call_async(typ.Request())
                future.add_done_callback(self.mapper_reset_done)
        # Physical state/TF arrive asynchronously. Reissue after fresh state is
        # published rather than sending only against the discarded old pose.
        self.remaining = 4

    def mapper_reset_done(self, future):
        try:
            result = future.result()
            if hasattr(result, 'success') and not result.success:
                self.node.get_logger().error(f'Mapping reset failed: {result.message}')
        except Exception as exc:
            self.node.get_logger().error(f'Mapping reset failed: {exc}')

    def publish_pose(self):
        if self.waiting_for_odom and (self.steady_clock.now().nanoseconds-self.reset_started)>5_000_000_000:
            self.waiting_for_odom = False
            self.node.get_logger().error('No fresh wheel odometry after reset; estimator/map reset unavailable')
        if not self.remaining:
            return
        self.remaining -= 1
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.stamp()
        pose.pose.pose.position.x = float(self.node.get_parameter('spawn_x').value)
        pose.pose.pose.position.y = float(self.node.get_parameter('spawn_y').value)
        angle = float(self.node.get_parameter('spawn_yaw').value)
        pose.pose.pose.orientation.z = math.sin(angle/2)
        pose.pose.pose.orientation.w = math.cos(angle/2)
        pose.pose.covariance[0] = pose.pose.covariance[7] = 0.0025
        pose.pose.covariance[35] = 0.0076
        self.publisher.publish(pose)
