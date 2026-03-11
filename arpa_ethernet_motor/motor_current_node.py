#!/usr/bin/env python3

import serial
import serial.serialutil

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MotorCurrentNode(Node):
    def __init__(self):
        super().__init__('motor_current_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic', 'motor_current')

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        topic = self.get_parameter('topic').value

        self.publisher_ = self.create_publisher(Float32, topic, 10)

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Opened {port} @ {baud}')
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f'Could not open serial port {port}: {e}')
            raise

        self.timer = self.create_timer(0.001, self.read_serial)

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            current = float(line)

            msg = Float32()
            msg.data = current
            self.publisher_.publish(msg)

        except ValueError:
            return
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')

    def destroy_node(self):
        try:
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorCurrentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()