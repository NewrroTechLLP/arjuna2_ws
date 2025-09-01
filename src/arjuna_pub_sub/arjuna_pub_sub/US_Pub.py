#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import serial.tools.list_ports
import sys

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('US_Pub')
        
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = 115200

        self.get_logger().info(f"Connecting to serial port: {self.serial_port}")

        self.publisher_1 = self.create_publisher(Float64, 'ultrasonic_distance_1', 10)
        self.publisher_2 = self.create_publisher(Float64, 'ultrasonic_distance_2', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.ser = self.open_serial_port()

    def open_serial_port(self):
        try:
            ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Successfully connected to {self.serial_port}.")
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port}: {e}")
            sys.exit(1)

    def timer_callback(self):
        if self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        distances = line.split(',')
                        if len(distances) == 2:
                            try:
                                # Convert to float and publish
                                dist1 = float(distances[0])
                                dist2 = float(distances[1])

                                msg1 = Float64()
                                msg1.data = dist1
                                self.publisher_1.publish(msg1)
                                self.get_logger().info(f'Published 1: {msg1.data:.2f} cm')

                                msg2 = Float64()
                                msg2.data = dist2
                                self.publisher_2.publish(msg2)
                                self.get_logger().info(f'Published 2: {msg2.data:.2f} cm')
                                
                            except ValueError:
                                self.get_logger().warn("Received non-numeric data, skipping.")

            except serial.SerialException as e:
                self.get_logger().error(f"Serial communication error: {e}")
                self.ser.close()
                self.ser = None
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)
    us_pub = UltrasonicPublisher()
    try:
        rclpy.spin(us_pub)
    except KeyboardInterrupt:
        pass
    finally:
        us_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()