#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

# Navigation parameters
LINEAR_VELOCITY = 0.5            # Forward speed (m/s)
ANGULAR_VELOCITY = 0.8           # Angular velocity (rad/s)
OBSTACLE_DIST_THRESHOLD = 0.3    # Distance threshold for obstacle detection (30cm)

# State Machine
STATE_NORMAL = 0
STATE_AVOIDING_OBSTACLE = 1

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('improved_obstacle_avoidance')
        
        # ROS 2 Publisher for command velocity
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        # ROS 2 Subscriber for laser scan data
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        # Node state and data
        self.current_state = STATE_NORMAL
        self.regions = {}

        self.get_logger().info("Obstacle avoidance node initialized")
        self.get_logger().info(f"Threshold distance: {OBSTACLE_DIST_THRESHOLD} meters")

    def laser_callback(self, msg):
        """Processes LIDAR data into regions and decides on action"""
        # Divide laser scan into regions
        self.regions = {
            'front_L': min(min(msg.ranges[0:130]), 10),
            'fleft': min(min(msg.ranges[131:230]), 10),
            'left': min(min(msg.ranges[231:280]), 10),
            'right': min(min(msg.ranges[571:620]), 10),
            'fright': min(min(msg.ranges[621:720]), 10),
            'front_R': min(min(msg.ranges[721:850]), 10)
        }
        
        self.take_action()

    def is_path_clear(self):
        """Checks if the path is clear of obstacles"""
        # Path is clear if front and front-side regions are clear
        return (self.regions['front_L'] > OBSTACLE_DIST_THRESHOLD and 
                self.regions['front_R'] > OBSTACLE_DIST_THRESHOLD and
                self.regions['fleft'] > OBSTACLE_DIST_THRESHOLD and
                self.regions['fright'] > OBSTACLE_DIST_THRESHOLD)

    def take_action(self):
        """Handles the state machine logic for obstacle avoidance"""
        twist_msg = Twist()
        
        if self.current_state == STATE_NORMAL:
            if not self.is_path_clear():
                self.get_logger().info('Obstacle detected, entering AVOIDING_OBSTACLE state')
                self.current_state = STATE_AVOIDING_OBSTACLE
            else:
                self.get_logger().info('Normal state - moving forward')
                twist_msg.linear.x = LINEAR_VELOCITY
                twist_msg.angular.z = 0.0
                self.cmd_pub.publish(twist_msg)

        elif self.current_state == STATE_AVOIDING_OBSTACLE:
            # Check for clearance before returning to NORMAL state
            if self.is_path_clear():
                self.get_logger().info('Path clear, returning to NORMAL state')
                self.current_state = STATE_NORMAL
                return # Skip publishing a new command and let the next cycle handle it

            # Execute avoidance logic
            front_obstacle = self.regions['front_L'] < OBSTACLE_DIST_THRESHOLD or self.regions['front_R'] < OBSTACLE_DIST_THRESHOLD
            left_obstacle = self.regions['fleft'] < OBSTACLE_DIST_THRESHOLD or self.regions['left'] < OBSTACLE_DIST_THRESHOLD
            right_obstacle = self.regions['fright'] < OBSTACLE_DIST_THRESHOLD or self.regions['right'] < OBSTACLE_DIST_THRESHOLD

            if front_obstacle:
                # If there's a front obstacle, find the clearer side to turn
                if self.regions['left'] > self.regions['right']:
                    self.get_logger().info('Front obstacle - turning left to avoid')
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = ANGULAR_VELOCITY
                else:
                    self.get_logger().info('Front obstacle - turning right to avoid')
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = -ANGULAR_VELOCITY
            elif left_obstacle:
                self.get_logger().info('Left obstacle - slight turn right')
                twist_msg.linear.x = 0.1
                twist_msg.angular.z = -ANGULAR_VELOCITY * 0.5
            elif right_obstacle:
                self.get_logger().info('Right obstacle - slight turn left')
                twist_msg.linear.x = 0.1
                twist_msg.angular.z = ANGULAR_VELOCITY * 0.5
            
            self.cmd_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()