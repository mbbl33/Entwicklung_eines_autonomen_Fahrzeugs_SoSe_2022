#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

class Odometry(Node):
    def __init__(self):
        super().__init__("odometry")

        self.period = 0.005

        self.speed_sub = self.create_subscription(Float64, "/speed", self.speed_callback, 1)
        self.odometry_pub = self.create_publisher(Float64, "/odometry", 1)

        self.speed = 0.0
        self.timer = self.create_timer(self.period, self.timer_callback)
        self.distance = 0.0

    def timer_callback(self):
        self.distance += self.speed * self.period
        msg = Float64()
        msg.data = self.distance
        self.odometry_pub.publish(msg)

    def speed_callback(self, msg):
        self.speed = msg.data


def main(args=None):
    rclpy.init(args=args)
    odometry = Odometry()

    rclpy.spin(odometry)

    odometry.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
