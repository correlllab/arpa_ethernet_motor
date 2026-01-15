#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_ros_messages.srv import EthernetMotor
import socket

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # Motor controller configuration
        self.host = "128.138.224.6"
        self.port = 8888
        self.timeout = 1.0
        
        # Create service with custom message type
        self.srv = self.create_service(
            EthernetMotor, 
            'motor_control', 
            self.motor_control_callback
        )
        
        self.get_logger().info('Motor control node started')
        self.get_logger().info(f'Service available: /motor_control')
        self.get_logger().info(f'Target: {self.host}:{self.port}')
    
    def send_motor_command(self, enable, speed):
        """
        Send motor control command via UDP
        
        Args:
            enable: 0 (off) or 1 (on)
            speed: 0-255 (motor speed)
        
        Returns:
            success: True if command sent successfully
            message: Status message
        """
        try:
            # Validate speed range
            if not 0 <= speed <= 255:
                return False, f"Invalid speed {speed}. Must be 0-255"
            
            # Create UDP socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(self.timeout)
            
            # Create 2-byte command
            command = bytes([enable, speed])
            
            # Send command
            sock.sendto(command, (self.host, self.port))
            sock.close()
            
            action = "ON" if enable else "OFF"
            return True, f"Motor {action} at speed {speed}"
            
        except socket.timeout:
            return False, "Socket timeout - no response from motor controller"
        except socket.error as e:
            return False, f"Socket error: {str(e)}"
        except Exception as e:
            return False, f"Error: {str(e)}"
    
    def motor_control_callback(self, request, response):
        """
        Handle motor control service request
        
        Args:
            request.enable: True to turn ON, False to turn OFF
            request.speed: Motor speed (0-255)
        
        Returns:
            response.success: True if successful
            response.message: Status message
        """
        enable = 1 if request.enable and request.speed > 0 else 0
        speed = request.speed
        action = "ON" if request.enable else "OFF"
        
        self.get_logger().info(f'Request: Motor {action} at speed {speed}')
        
        success, message = self.send_motor_command(enable, speed)
        
        response.success = success
        response.message = message
        
        if success:
            self.get_logger().info(f'Success: {message}')
        else:
            self.get_logger().error(f'Failed: {message}')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()