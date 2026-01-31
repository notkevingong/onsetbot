#!/usr/bin/env python3
"""
ROS2 Node to command a BLDC motor via ODrive using CAN.
Cycles through velocity sequence: 1, 0, -0.5, 0, repeat.
"""

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState
import time


class VelocityCommander(Node):
    def __init__(self):
        super().__init__('velocity_commander')
        
        # Create publisher for velocity commands
        self.publisher = self.create_publisher(
            ControlMessage,
            '/odrive_axis0/control_message',
            10
        )

        # Create client for axis state control
        self.axis_state_client = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        
        # Define the velocity sequence
        self.velocity_sequence = [0.3, 0.0, -1.0, 0.0]
        self.current_index = 0
        self.axis_id = 0  # Modify if using different axis
        
        # Time interval between velocity changes (seconds)
        self.interval = 2.0
        
        # Initialize closed loop control
        self.initialize_closed_loop()
        
        # Create timer for periodic publishing (velocity profile)
        self.timer = self.create_timer(self.interval, self.publish_velocity)
        self.get_logger().info('Velocity Commander node started')
    
    def publish_velocity(self):
        """Publish the next velocity in the sequence"""
        velocity = self.velocity_sequence[self.current_index]
        
        # Create velocity command message
        msg = ControlMessage()
        msg.control_mode = 2  # Velocity control mode
        msg.input_mode = 1  # Passthrough set point
        msg.input_pos = 0.0
        msg.input_torque = 0.0
        msg.input_vel = velocity
        
        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published velocity: {velocity} m/s')
        
        # Move to next velocity in sequence
        self.current_index = (self.current_index + 1) % len(self.velocity_sequence)
    
    def initialize_closed_loop(self):
        """Initialize the motor in closed loop control mode (axis state 8)"""
        # Wait for service to be available
        if not self.axis_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Axis state service not available!')
            return
        
        # Create and send the request
        request = AxisState.Request()
        request.axis_requested_state = 8  # CLOSED_LOOP_CONTROL state
        
        future = self.axis_state_client.call_async(request)
        future.add_done_callback(self.axis_state_callback)
    
    def axis_state_callback(self, future):
        """Callback for axis state service response"""
        try:
            response = future.result()
            self.get_logger().info(
                f'Motor initialized - Axis State: {response.axis_state}, '
                f'Active Errors: {response.active_errors}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to set axis state: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VelocityCommander()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down velocity commander...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

