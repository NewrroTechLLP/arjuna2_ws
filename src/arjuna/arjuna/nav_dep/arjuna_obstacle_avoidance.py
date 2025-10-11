#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        
        # Create subscription to laser scan topic
        self.laser_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            10
        )
        
        self.get_logger().info('LIDAR Subscriber initialized')

    def laser_callback(self, msg):
        """Process laser scan data and extract regions"""
        regions = {
            'front_L': min(min(msg.ranges[0:130]), 10.0),
            'fleft': min(min(msg.ranges[131:230]), 10.0),
            'left': min(min(msg.ranges[231:280]), 10.0),
            'right': min(min(msg.ranges[571:620]), 10.0),
            'fright': min(min(msg.ranges[621:720]), 10.0),
            'front_R': min(min(msg.ranges[721:850]), 10.0)
        }
        
        # Log the regions data
        self.get_logger().info(f'Regions: {regions}')


def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    
    try:
        rclpy.spin(lidar_subscriber)
    except KeyboardInterrupt:
        pass
    
    lidar_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()