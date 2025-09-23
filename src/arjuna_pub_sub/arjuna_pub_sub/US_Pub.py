#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

class US_Pub(Node):
    def __init__(self):
        # Initialize ROS 2 node
        super().__init__('US_Pub')
        
        # Publisher for sensor data
        self.sensor_pub = self.create_publisher(Float32MultiArray, '/ultrasonic_distances', 10)
        
        # Alternative publisher using Point message (x=sensor1, y=sensor2)
        self.point_pub = self.create_publisher(Point, '/ultrasonic_point', 10)
        
        # Declare and get parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # Initialize serial connection
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {str(e)}")
            return
            
        # Create a timer to periodically check for data (optional approach)
        # Alternatively, we can use the run() method with rclpy.spin()
        
    def run(self):
        """Continuous loop to read and publish data"""
        while rclpy.ok():
            try:
                # Continuously read from Arduino serial port
                line = self.arduino.readline().decode('utf-8').strip()
                
                if line:
                    # Parse JSON data
                    data = json.loads(line)
                    sensor1_distance = data.get('sensor1', 0)
                    sensor2_distance = data.get('sensor2', 0)
                    
                    # Create Float32MultiArray message
                    array_msg = Float32MultiArray()
                    array_msg.data = [float(sensor1_distance), float(sensor2_distance)]
                    
                    # Create Point message
                    point_msg = Point()
                    point_msg.x = float(sensor1_distance)
                    point_msg.y = float(sensor2_distance)
                    point_msg.z = 0.0
                    
                    # Publish messages immediately
                    self.sensor_pub.publish(array_msg)
                    self.point_pub.publish(point_msg)
                    
                    self.get_logger().info(f"Published: Sensor1={sensor1_distance}cm, Sensor2={sensor2_distance}cm")
                    
            except json.JSONDecodeError:
                self.get_logger().warn(f"Invalid JSON received: {line}")
            except Exception as e:
                self.get_logger().error(f"Error reading from Arduino: {str(e)}")
                break
                
            # Allow ROS 2 to process callbacks
            rclpy.spin_once(self, timeout_sec=0.001)
        
        # Close serial connection
        if hasattr(self, 'arduino'):
            self.arduino.close()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        publisher = US_Pub()
        publisher.run()
    except KeyboardInterrupt:
        pass
    finally:
        if 'publisher' in locals():
            publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
