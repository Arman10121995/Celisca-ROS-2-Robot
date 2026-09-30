#!/usr/bin/env python3
"""RViz goals for Nav2, restamped with the stack's own clock.

``rviz_default_plugins/SetGoal`` (the "2D Goal Pose" toolbar tool) publishes a
``PoseStamped`` on ``/robot_lab/goal_pose``.  This node forwards
each one to the ``navigate_to_pose`` action of the running Nav2 stack with the
header restamped to *this node's* clock, because the goal a client sends
cannot be trusted to be on the run's clock: nav2's RViz panel stamps goals
with the system clock (see ``robot_lab_utils.nav_goals``), which makes every
goal in a ``use_sim_time`` run abort with an "extrapolation into the future"
error before a path is planned.

A new goal supersedes the previous one (the active goal is cancelled first).
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from robot_lab_utils.nav_goals import pose_is_finite, restamped

#: action_msgs/GoalStatus code -> name, for legible result logs.
_STATUS_NAMES = {
    1: "accepted", 2: "executing", 3: "canceling",
    4: "succeeded", 5: "canceled", 6: "aborted",
}


class Nav2GoalRelay(Node):
    """Receive a goal pose and send it restamped to the NavigateToPose action."""

    def __init__(self):
        super().__init__("nav2_goal_relay")
        self.declare_parameter("goal_topic", "/robot_lab/goal_pose")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("fallback_frame", "map")
        self.declare_parameter("cancel_previous_goal", True)
        self.declare_parameter("server_timeout_s", 2.0)
        goal_topic = str(self.get_parameter("goal_topic").value)
        self.action_name = str(self.get_parameter("action_name").value).strip("/")
        self.fallback_frame = str(self.get_parameter("fallback_frame").value)
        self.cancel_previous = bool(
            self.get_parameter("cancel_previous_goal").value)
        self.server_timeout_s = float(
            self.get_parameter("server_timeout_s").value)
        self.client = ActionClient(self, NavigateToPose, self.action_name)
        self.subscription = self.create_subscription(
            PoseStamped, goal_topic, self.on_goal, 10)
        self.active_handle = None
        self.get_logger().info(
            "relaying %s to '%s' with the stack clock: client stamps "
            "(e.g. the nav2 RViz panel's system-time stamp) are discarded"
            % (goal_topic, self.action_name))

    def on_goal(self, msg):
        """A new goal arrived: restamp it and supersede any active goal."""
        if not pose_is_finite(msg.pose):
            self.get_logger().warn(
                "ignored a goal pose with non-finite coordinates")
            return
        if not self.client.server_is_ready():
            self.client.wait_for_server(timeout_sec=self.server_timeout_s)
        if not self.client.server_is_ready():
            self.get_logger().warn(
                "action '%s' is not available; goal dropped" % self.action_name)
            return
        goal = NavigateToPose.Goal()
        goal.pose = msg
        restamped(goal.pose.header, self.get_clock().now().to_msg(),
                  fallback_frame=self.fallback_frame)
        stamp = goal.pose.header.stamp
        if self.get_parameter("use_sim_time").value and stamp.sec == 0 and stamp.nanosec == 0:
            self.get_logger().warn("simulation clock has not started; goal dropped")
            return
        if self.active_handle is not None and self.cancel_previous:
            self.get_logger().info("cancelling the previous goal")
            self.active_handle.cancel_goal_async()
            self.active_handle = None
        self.get_logger().info(
            "sending goal x=%.3f y=%.3f frame=%s at %.3f s"
            % (goal.pose.pose.position.x, goal.pose.pose.position.y,
               goal.pose.header.frame_id, stamp.sec + stamp.nanosec * 1e-9))
        self.client.send_goal_async(goal).add_done_callback(self.on_response)

    def on_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("goal rejected by the navigator")
            return
        self.active_handle = handle
        self.get_logger().info("goal accepted")
        handle.get_result_async().add_done_callback(
            lambda future: self.on_result(future, handle))

    def on_result(self, future, handle):
        status = future.result().status
        self.get_logger().info(
            "goal finished: %s" % _STATUS_NAMES.get(status, status))
        if self.active_handle is handle:
            self.active_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        # Humble's executor can race a subscription take with SIGINT's
        # context shutdown and raise this conversion error instead of its
        # documented ExternalShutdownException.
        if rclpy.ok() or "Unable to convert call argument" not in str(exc):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
