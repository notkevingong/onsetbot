#!/usr/bin/env python3
"""
ROS2 Node to command a BLDC motor via ODrive using CAN.
Cycles through velocity sequence: 1, 0, -0.5, 0, repeat.
"""

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState
from onsetbot_interfaces.msg import LaunchCommand
import time
import math


class ActuatorCommander(Node):
    def __init__(self):
        super().__init__('actuators_commander')
        
        ### PUBLISHERS
        # Create publisher for ODrives (3x)
        # 0/1 - launcher motors
        #   2 - turret motor
        self.pub = {
            0: self.create_publisher(ControlMessage, '/odrive_axis0/control_message', 10),
            1: self.create_publisher(ControlMessage, '/odrive_axis1/control_message', 10),
            2: self.create_publisher(ControlMessage, '/odrive_axis2/control_message', 10)
        }

        ### SUBSCRIBERS
        # Create subscription for Launch info
        self.create_subscription(LaunchCommand, 'launch_info', self.launch_topic_callback, 10)
        
        # Create subscription for STM32 info


        # Create client for axis state control
        self.axis_state_client = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        
        # Launch variable
        self.velocity = 0.0 # (turns/sec)
        self.velocity_ramp_rate = 0.0 # (turns/sec^2)
        self.angle_turret = 0.0 
        self.angle_launch = 0.0

        # Variables for calculations
        self._vel_conversion_constant = (1 / 0.0375) * (60 / (2 * math.pi)) * 2 * (1 / 60) # Converts tangential velocity to RPM
        self._turret_conversion_constant = (0.5 / 180) # 0.5 turns / 180 degrees

        # Time interval between velocity changes (seconds)
        self.interval = 2.0
        
        # Initialize closed loop control
        self.initialize_closed_loop()
        
        # Create timer for periodic publishing (velocity profile)
        self.timer = self.create_timer(self.interval, self.publish_velocity)
        self.get_logger().info('Actuator Commander node started')
    
    def publish_velocity(self, axis_id,):
        """Publish the next velocity in the sequence"""
        msg = ControlMessage()

        if axis_id == 2:
            msg.control_mode = 3 # Positional control mode
            msg.input_mode = 3 # Filtered position
            msg.input_pos =  # Target position

            msg.vel_ramp_rate = 0.0
            msg.input_vel = 0.0
            msg.input_torque = 0.0

        else:
            msg.control_mode = 2  # Velocity control mode
            msg.input_mode = 2  # Ramped Vel Mode
            msg.vel_ramp_rate =  # (turns/sec^2)
            msg.input_pos = 0.0
            msg.input_torque = 0.0
            msg.input_vel = self.velocity # (turns/sec)
        
        # Create velocity command message

        
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

    def launch_topic_callback(self, msg: LaunchCommand):
        """Callback for storing launch info"""
        msg = LaunchCommand
        self.velocity = msg.velocity * self._vel_conversion_constant # converts m/s to turns/sec
        self.angle_launch = msg.angle_launch 
        self.angle_turret = msg.angle_turret * self._turret_conversion_constant # converts from deg to turns



def main(args=None):
    rclpy.init(args=args)
    node = ActuatorCommander()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down velocity commander...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

