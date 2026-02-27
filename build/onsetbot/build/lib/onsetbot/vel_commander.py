#!/usr/bin/env python3
"""
ROS2 Node to command a BLDC motor via ODrive using CAN.
"""

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState
from onset_interfaces.msg import LaunchCommand
from sensor_msgs.msg import JointState
import math

class VelocityCommander(Node):
    def __init__(self):
        super().__init__('velocity_commander')
        
        # Create publisher for launcher
        self.publisher = self.create_publisher(
            ControlMessage,
            'actuator_command',
            10
        )

        # Create subscriber for launch_info
        self.launch_subscriber = self.create_subscription(
            LaunchCommand,
            'launch_info',
            self.convert_callback,
            10
        )

        # Create subscriber for odrive_states
        self.odrive_subscriber = self.create_subscription(
            JointState,
            '/odrive/joint_states',
            self.update_odrive_position,
            10
        )

        # Create client for axis state control
        self.axis_state_client = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        
        # Define variables and conversion constants
        self.velocity = 0.0
        self.angle_launch = 0.0
        self._velocity_conversion_constant = (1 / 0.0375) * (60 / (2*math.pi)) * (1 / 60) * 2
        self._angle_conversion_constant = (0.5 / 180)

        # Time interval between velocity changes (seconds)
        self.interval = 2.0
        
        # Initialize closed loop control
        self.initialize_closed_loop()
        
        # Create timer for periodic publishing (velocity profile)
        self.timer = self.create_timer(self.interval, self.publish_velocity)
        self.get_logger().info('Velocity Commander node started')
    
    def publish_velocity(self):
        """Publish the velocity in the sequence"""
        
        # Create velocity command message
        msg = ControlMessage()
        msg.control_mode = 2  # Velocity control mode
        msg.input_mode = 2  # Ramped Vel Mode
        msg.input_pos = 0.0
        msg.input_torque = 0.0
        msg.input_vel = self.velocity
        
        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published velocity: {self.velocity} m/s')
    
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

    def convert_callback (self, msg: LaunchCommand):
        msg = LaunchCommand
        self.velocity = msg.velocity * self._velocity_conversion_constant
        self.angle_launch = msg.angle_launch * self._angle_conversion_constant

    def update_odrive_position(self, msg: JointState):
        msg = JointState



        



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

