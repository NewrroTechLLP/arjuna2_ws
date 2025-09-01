#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
import math

class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_listener')

        # Global-like class variable
        self.quaternion = None

        # Subscriber for IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10  # QoS profile depth
        )

        # Timer to periodically print the yaw
        self.timer = self.create_timer(0.1, self.print_yaw) # 10 Hz

        self.get_logger().info('IMU Listener node initialized and subscribed to /imu/data')

    def imu_callback(self, msg):
        """
        Callback function to store the quaternion data.
        """
        self.quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

    def print_yaw(self):
        """
        Periodically calculates and prints the yaw angle from the latest quaternion data.
        """
        if self.quaternion:
            # Convert quaternion to Euler angles
            (roll, pitch, yaw) = euler_from_quaternion(self.quaternion)
            
            # Convert yaw to degrees and print
            yaw_degrees = round(math.degrees(yaw), 2)
            self.get_logger().info(f"Yaw in degrees: {yaw_degrees}")
        else:
            self.get_logger().info("Waiting for IMU data...")

def main(args=None):
    rclpy.init(args=args)
    imu_listener_node = ImuListener()
    rclpy.spin(imu_listener_node)
    imu_listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()