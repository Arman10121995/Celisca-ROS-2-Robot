"""Retired zero-thrust AttitudeTarget demonstration.

Zero thrust does not hover, and the former no-MAVROS fallback emitted no
commands. The measured replacement is px4_x500 in mode:=flight, using
px4_ros_controller and the upstream X500 dynamics rather than this stub.
"""


class MavrosOffboardController:
    def __init__(self):
        raise RuntimeError(
            'The zero-thrust MAVROS demonstration is retired. Launch '
            'robot_model:=px4_x500 simulator:=gazebo mode:=flight '
            'map_name:=nav_empty; see docs/tutorials/px4_x500.md.')


def main(args=None):
    MavrosOffboardController()
