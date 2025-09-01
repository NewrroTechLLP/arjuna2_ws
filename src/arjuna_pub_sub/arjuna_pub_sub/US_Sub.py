#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class UltrasonicSubscriber(Node):
    def __init__(self):
        super().__init__('US_Sub')
        
        self.subscription_1 = self.create_subscription(
            Float64,
            'ultrasonic_distance_1',
            self.distance_1_callback,
            10
        )
        self.subscription_2 = self.create_subscription(
            Float64,
            'ultrasonic_distance_2',
            self.distance_2_callback,
            10
        )
        
        self.get_logger().info("Ready to receive data on both ultrasonic topics.")

    def distance_1_callback(self, msg):
        self.get_logger().info(f'Subscribed to Sensor 1: {msg.data:.2f} cm')

    def distance_2_callback(self, msg):
        self.get_logger().info(f'Subscribed to Sensor 2: {msg.data:.2f} cm')

def main(args=None):
    rclpy.init(args=args)
    us_sub = UltrasonicSubscriber()
    try:
        rclpy.spin(us_sub)
    except KeyboardInterrupt:
        pass
    finally:
        us_sub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()