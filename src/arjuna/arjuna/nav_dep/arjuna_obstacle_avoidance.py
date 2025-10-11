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
        
        # Helper function to safely get min from a range
        def safe_min(range_slice, default=10.0):
            return min(min(range_slice), default) if len(range_slice) > 0 else default
        
        regions = {
            'front_L': safe_min(msg.ranges[0:130]),
            'fleft': safe_min(msg.ranges[131:230]),
            'left': safe_min(msg.ranges[231:280]),
            'right': safe_min(msg.ranges[571:620]),
            'fright': safe_min(msg.ranges[621:720]),
            'front_R': safe_min(msg.ranges[721:850])
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