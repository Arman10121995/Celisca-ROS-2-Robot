#!/usr/bin/env python3
"""Shared marine ownership, real wheels ashore and propellers when immersed."""
import json
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from robot_lab_utils.marine_model import AmphibiousModel
from robot_lab_utils.surface_controller import SurfaceController


class AmphibiousController(SurfaceController):
    def __init__(self):
        super().__init__()
        self.wheel_pub=self.create_publisher(Twist,'/amphibious/wheels',10)
        self.plant=None;self.plant_at=-math.inf
        self.create_subscription(String,'/amphibious/plant',self.observe_plant,10)

    def load_model(self,description,density):return AmphibiousModel(description,density)

    def observe_plant(self,msg):
        try:
            value=json.loads(msg.data)
            if not 0<=value['wet_fraction']<=1:return
            self.plant=value;self.plant_at=time.monotonic()
        except (ValueError,KeyError,TypeError):return

    def envelope_ok(self):
        return (max(abs(v) for v in self.orientation[:2])<=.65
                and time.monotonic()-self.plant_at<.6)

    def extra_state(self):return dict(propulsion_kind='wheels_propellers',native_plant=self.plant)

    def stop(self,reason):
        super().stop(reason)
        if hasattr(self,'wheel_pub'):self.wheel_pub.publish(Twist())

    def _tick(self):
        # A disabled controller sends neutral setpoints; the native wall-clock
        # lease removes wheel effort and propeller power if this process stalls.
        super()._tick()
        msg=Twist()
        if self.enabled and time.monotonic()-self.cmd_at<.35:
            msg.linear.x=self.command[0];msg.angular.z=self.command[3]
        self.wheel_pub.publish(msg)


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent();rclpy.init(args=args);node=AmphibiousController()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        try:
            if rclpy.ok():node.stop('Amphibious controller shutdown')
        except RCLError:pass
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
