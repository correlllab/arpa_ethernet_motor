#!/usr/bin/env python3
import sys
import os
import time

# Add the script's directory to Python path for imports
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from custom_ros_messages.srv import ParkerGetPoseTrigger
from custom_ros_messages.action import ParkerGotoPose

from PARKER_core import SocketTelnetClient


# -------------------------
# ROS2 Node Wrapper
# -------------------------
class ParkerMotorNode(Node):
    def __init__(self):
        super().__init__('parker_motor_node')

        # Initialize the Parker motor client
        self.client = SocketTelnetClient()
        self.client.init_motor()
        self.client.start_monitoring()

        # Allow some time for monitoring to initialize
        time.sleep(1.0)

        # Use reentrant callback group to allow service and action to run concurrently
        self.callback_group = ReentrantCallbackGroup()

        # Create service for getting position
        self.get_pose_service = self.create_service(
            ParkerGetPoseTrigger,
            'parker_get_position',
            self.get_position_callback,
            callback_group=self.callback_group
        )

        # Create action server for goto pose
        self.goto_pose_action_server = ActionServer(
            self,
            ParkerGotoPose,
            'parker_goto_pose',
            execute_callback=self.goto_pose_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Parker Motor Node initialized')
        self.get_logger().info('Service available: parker_get_position')
        self.get_logger().info('Action available: parker_goto_pose')

    def get_position_callback(self, request, response):
        """Service callback to get current position."""
        try:
            current_position = self.client._last_position
            if current_position is None:
                current_position = self.client.get_position(self.client.main_sock)
            response.ismoving = self.client.is_moving
            response.mm_distance = current_position
            self.get_logger().info(f'Position requested: {current_position:.4f} mm, moving: {self.client.is_moving}')
        except Exception as e:
            self.get_logger().error(f'Error getting position: {e}')
            response.ismoving = False
            response.mm_distance = float('nan')

        return response

    def goal_callback(self, goal_request):
        """Accept or reject goal requests."""
        self.get_logger().info(f'Received goal request: target={goal_request.target_position_mm} mm')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancellation requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def goto_pose_callback(self, goal_handle):
        """Action callback to move to target position."""
        self.get_logger().info(f'Executing goal: moving to {goal_handle.request.target_position_mm} mm')

        feedback_msg = ParkerGotoPose.Feedback()
        check_rate = 5  # Hz - check at 5Hz as requested
        check_interval = 1.0 / check_rate
        settling_time = 1.0  # seconds - must be stationary for this long
        position_tolerance = 0.5  # mm - acceptable error from target (increased for motor settling behavior)

        try:
            # Send goto command
            target_position = goal_handle.request.target_position_mm
            response = self.client.goto_pose(target_position)

            # Wait for movement to start (give it a moment)
            time.sleep(0.1)

            # Track how long we've been stationary and close to goal
            stationary_start_time = None

            # Publish feedback while moving or settling
            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = ParkerGotoPose.Result()
                    result.success = False
                    result.final_position_mm = self.client._last_position
                    result.message = 'Goal canceled'
                    self.get_logger().info('Goal canceled')
                    return result

                # Update and publish feedback
                current_pos = self.client._last_position
                if current_pos is None:
                    # Position not yet available, wait and continue
                    time.sleep(check_interval)
                    continue

                position_error = abs(current_pos - target_position)
                feedback_msg.current_position_mm = current_pos
                feedback_msg.is_moving = self.client.is_moving
                goal_handle.publish_feedback(feedback_msg)

                # Check if we're close to goal and not moving
                if not self.client.is_moving and position_error < position_tolerance:
                    if stationary_start_time is None:
                        stationary_start_time = time.time()
                        self.get_logger().info(f'Reached target, settling... (error: {position_error:.4f} mm)')
                    else:
                        elapsed = time.time() - stationary_start_time
                        if elapsed >= settling_time:
                            # Successfully settled at goal
                            self.get_logger().info(f'Settled for {elapsed:.2f}s, completing action')
                            break
                else:
                    # Still moving or not at goal - reset settling timer
                    if stationary_start_time is not None:
                        self.get_logger().info(f'Movement detected or tolerance exceeded, resetting settle timer (moving={self.client.is_moving}, error={position_error:.4f} mm)')
                    stationary_start_time = None

                time.sleep(check_interval)

            # Movement complete and settled
            goal_handle.succeed()
            result = ParkerGotoPose.Result()
            result.success = True
            result.final_position_mm = self.client._last_position
            result.message = 'Movement completed successfully'

            self.get_logger().info(f'Goal succeeded: final position = {result.final_position_mm:.4f} mm, error = {abs(result.final_position_mm - target_position):.4f} mm')
            return result

        except Exception as e:
            goal_handle.abort()
            result = ParkerGotoPose.Result()
            result.success = False
            result.final_position_mm = self.client._last_position
            result.message = f'Error during movement: {str(e)}'
            self.get_logger().error(f'Goal aborted: {e}')
            return result

    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info('Shutting down Parker Motor Node')
        self.client.close()
        super().destroy_node()


# -------------------------
# Main script
# -------------------------
def main():
    rclpy.init()

    parker_node = ParkerMotorNode()

    # Use MultiThreadedExecutor to handle concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(parker_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        parker_node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        parker_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
