#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_arm_interfaces.msg import TargetPose
import serial


class Esp32BridgeNode(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # Parameters for serial connection
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=1.0)
            self.get_logger().info(f'Opened serial port {port} at {baud} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            self.ser = None

        # Subscribe to TargetPose (in meters)
        self.sub = self.create_subscription(
            TargetPose,
            '/target_pose',
            self.target_pose_callback,
            10
        )

    def target_pose_callback(self, msg: TargetPose):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial port not open, cannot send command')
            return

        # Convert meters to centimeters for the ESP32 IK code
        x_cm = msg.x 
        y_cm = msg.y
        z_cm = msg.z 
        roll_deg = msg.roll  # already degrees

        cmd = f"A,{x_cm:.1f},{y_cm:.1f},{z_cm:.1f},{roll_deg:.1f}\n"
        try:
            self.ser.write(cmd.encode('utf-8'))
            self.get_logger().info(f'Sent to ESP32: {cmd.strip()}')
        except Exception as e:
            self.get_logger().error(f'Error writing to serial: {e}')

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Closed serial port')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
