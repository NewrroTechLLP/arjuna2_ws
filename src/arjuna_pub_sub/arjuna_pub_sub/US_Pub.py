#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import serial
import json


class US_Pub(Node):
    def __init__(self):
        super().__init__('us_pub')

        # Declare parameters with defaults
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.arduino = None
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"✅ Connected to Arduino on {self.serial_port} at {self.baud_rate} baud")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to Arduino: {str(e)}")
            return

        # Publishers
        self.sensor_pub = self.create_publisher(Float32MultiArray, '/ultrasonic_distances', 10)
        self.point_pub = self.create_publisher(Point, '/ultrasonic_point', 10)

        # Timer (runs at 10Hz)
        self.timer = self.create_timer(0.1, self.read_and_publish)

    def read_and_publish(self):
        """Timer callback to read from serial and publish messages."""
        if not self.arduino:
            self.get_logger().error("Arduino serial connection not available.")
            return

        try:
            line = self.arduino.readline().decode('utf-8').strip()
            if not line:
                return  # Ignore empty lines

            # Expecting a JSON string like: {"sensor1": 123, "sensor2": 456}
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                self.get_logger().warn(f"⚠️ Invalid JSON received: {line}")
                return

            sensor1 = float(data.get('sensor1', 0.0))
            sensor2 = float(data.get('sensor2', 0.0))

            # Publish Float32MultiArray
            array_msg = Float32MultiArray()
            array_msg.data = [sensor1, sensor2]
            self.sensor_pub.publish(array_msg)

            # Publish Point
            point_msg = Point()
            point_msg.x = sensor1
            point_msg.y = sensor2
            point_msg.z = 0.0
            self.point_pub.publish(point_msg)

            self.get_logger().info(f"📡 Published: Sensor1 = {sensor1} cm, Sensor2 = {sensor2} cm")

        except Exception as e:
            self.get_logger().error(f"❌ Error reading from Arduino: {str(e)}")

    def destroy_node(self):
        if self.arduino:
            self.arduino.close()
            self.get_logger().info("🔌 Serial connection closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = US_Pub()
        if node.arduino:  # Only spin if serial connection was successful
            rclpy.spin(node)
        else:
            node.get_logger().error("Node not running due to serial connection failure.")
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
