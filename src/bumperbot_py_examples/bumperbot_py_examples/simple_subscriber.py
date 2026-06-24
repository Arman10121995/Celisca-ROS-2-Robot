import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import pygame
import math
import numpy as np

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__("simple_subscriber")
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_
        
        self.start_x = None
        self.start_y = None

    def odom_callback(self, msg):
        if self.initial_x == None:
            self.initial_x = curr_x
            self.initial_y = curr_y
            
        raise 


def main():
    rclpy.init()
    node = DistanceTracker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
