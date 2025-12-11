#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import socket

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # Motor controller configuration
        self.host = "128.138.224.6"
        self.port = 8888
        self.timeout = 1.0
        
        # Create services
        self.srv_on = self.create_service(
            Trigger, 
            'motor_on', 
            self.motor_on_callback
        )
        self.srv_off = self.create_service(
            Trigger, 
            'motor_off', 
            self.motor_off_callback
        )
        
        self.get_logger().info('Motor control node started')
        self.get_logger().info(f'Services available: /motor_on, /motor_off')
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
            # Create UDP socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(self.timeout)
            
            # Create 2-byte command
            command = bytes([enable, speed])
            
            # Send command
            sock.sendto(command, (self.host, self.port))
            sock.close()
            
            return True, f"Command sent: enable={enable}, speed={speed}"
            
        except socket.timeout:
            return False, "Socket timeout - no response from motor controller"
        except socket.error as e:
            return False, f"Socket error: {str(e)}"
        except Exception as e:
            return False, f"Error: {str(e)}"
    
    def motor_on_callback(self, request, response):
        """Handle motor ON service request"""
        self.get_logger().info('Turning motor ON at full speed')
        
        success, message = self.send_motor_command(enable=1, speed=255)
        
        response.success = success
        response.message = message
        
        if success:
            self.get_logger().info('Motor turned ON successfully')
        else:
            self.get_logger().error(f'Failed to turn motor ON: {message}')
        
        return response
    
    def motor_off_callback(self, request, response):
        """Handle motor OFF service request"""
        self.get_logger().info('Turning motor OFF')
        
        success, message = self.send_motor_command(enable=0, speed=255)
        
        response.success = success
        response.message = message
        
        if success:
            self.get_logger().info('Motor turned OFF successfully')
        else:
            self.get_logger().error(f'Failed to turn motor OFF: {message}')
        
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