"""Clock and sanity rules for Nav2 goals that client tools send in a /clock run.

Nav2's RViz plugin stamps every goal it sends with the *system* clock: in
Humble ``Nav2Panel::onNewGoal`` runs ``pose.header.stamp =
rclcpp::Clock().now()`` (``rclcpp::Clock`` defaults to ``RCL_SYSTEM_TIME``), so
a simulated run -- whose stack and TF are on ``/clock`` -- receives a goal
stamped with epoch wall time.  ``planner_server`` then refuses it with

    Extrapolation Error looking up target frame: Lookup would require
    extrapolation into the future.  Requested time 1790755942.8 but the latest
    data is at time 15.36

and the goal ends in its recovery behaviors without ever planning.  No RViz
parameter changes the panel's clock, so the stack restamps instead: the relay
that receives the goal sets the header to its own (sim) clock.  These helpers
are deliberately ROS-free so the rule can be pinned without a running graph.
"""

import math

#: Frame a goal without one is interpreted in.
FALLBACK_FRAME = "map"


def restamped(header, now, fallback_frame=FALLBACK_FRAME):
    """Point ``header`` at the relay's clock and a usable frame, in place."""
    frame = str(getattr(header, "frame_id", "") or "").strip()
    header.frame_id = frame or fallback_frame
    header.stamp = now
    return header


def pose_is_finite(pose, limit=1.0e6):
    """True when every pose number is finite and plausibly inside a map.

    A tool that is not tracking the run's TF (or tracks a different one) can
    send NaN or a point far outside the map; nav2 would abort later with a less
    legible error, so the relay refuses those at the door.
    """
    values = (pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z,
              pose.orientation.w)
    return all(math.isfinite(value) and abs(value) <= limit for value in values)
