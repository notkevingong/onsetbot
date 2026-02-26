#!/usr/bin/env python3
"""
ROS2 Node for zero sequence on ODrive motor via CAN.
Finds mechanical limits by detecting current spikes in each direction,
then moves to center position.
"""

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControlMessage, ControllerStatus
from odrive_can.srv import AxisState
import time
from enum import Enum


class ZeroSequenceState(Enum):
    IDLE = 0
    MOVING_POSITIVE = 1
    MOVING_NEGATIVE = 2
    MOVING_TO_CENTER = 3
    COMPLETED = 4


class ZeroSequencer(Node):
    def __init__(self):
        super().__init__('zero_sequencer')
        
        # Publisher for velocity commands
        self.publisher = self.create_publisher(
            ControlMessage,
            '/odrive_axis0/control_message',
            10
        )
        
        # Subscriber for controller status feedback
        self.subscription = self.create_subscription(
            ControllerStatus,
            '/odrive_axis0/controller_status',
            self.controller_status_callback,
            10
        )
        
        # Client for axis state control
        self.axis_state_client = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        
        # Zero sequence parameters
        self.axis_id = 0
        self.current_spike_threshold = 1.0 # Amperes - adjust based on your motor
        self.search_velocity = 0.3  # m/s for finding limits
        self.center_velocity = 0.2  # m/s slower speed when moving to center
        
        # State tracking
        self.state = ZeroSequenceState.IDLE
        self.current_position = 0.0
        self.pos_limit = None  # Position at positive limit
        self.neg_limit = None  # Position at negative limit
        self.center_position = None
        self.last_iq = 0.0
        
        # Initialize closed loop control
        self.initialize_closed_loop()
        
        # Create timer for state machine
        self.timer = self.create_timer(0.1, self.state_machine_step)
        self.get_logger().info('Zero Sequencer node started')
    
    def controller_status_callback(self, msg):
        """Callback for controller status feedback"""
        self.current_position = msg.pos_estimate
        self.last_iq = abs(msg.iq_measured)  # Use magnitude of current
    
    def publish_velocity(self, velocity):
        """Publish velocity command"""
        msg = ControlMessage()
        msg.control_mode = 2  # Velocity control mode
        msg.input_mode = 1  # Passthrough set point
        msg.input_pos = 0.0
        msg.input_torque = 0.0
        msg.input_vel = velocity
        self.publisher.publish(msg)
    
    def stop_motor(self):
        """Stop motor movement"""
        self.publish_velocity(0.0)
    
    def state_machine_step(self):
        """State machine for zero sequence procedure"""
        
        if self.state == ZeroSequenceState.IDLE:
            self.get_logger().info('Starting positive direction search...')
            self.state = ZeroSequenceState.MOVING_POSITIVE
            
        elif self.state == ZeroSequenceState.MOVING_POSITIVE:
            # Move positive and detect current spike
            self.publish_velocity(self.search_velocity)
            
            if self.last_iq > self.current_spike_threshold:
                self.pos_limit = self.current_position
                self.get_logger().info(f'Positive limit found at position: {self.pos_limit}')
                self.stop_motor()
                time.sleep(0.5)  # Pause before changing direction
                self.state = ZeroSequenceState.MOVING_NEGATIVE
        
        elif self.state == ZeroSequenceState.MOVING_NEGATIVE:
            self.get_logger().info('Starting negative direction search...')
            # Move negative and detect current spike
            self.publish_velocity(-self.search_velocity)
            time.sleep(0.5)
            
            if self.last_iq > self.current_spike_threshold:
                self.neg_limit = self.current_position
                self.get_logger().info(f'Negative limit found at position: {self.neg_limit}')
                self.stop_motor()
                
                # Calculate center position
                self.center_position = (self.pos_limit + self.neg_limit) / 2.0
                self.get_logger().info(f'Center position: {self.center_position}')
                
                time.sleep(0.5)  # Pause before moving to center
                self.state = ZeroSequenceState.MOVING_TO_CENTER
        
        elif self.state == ZeroSequenceState.MOVING_TO_CENTER:
            # Move to center position using closed loop
            position_error = self.center_position - self.current_position
            
            # Simple proportional control
            velocity_command = position_error * 0.5  # Gain factor
            velocity_command = max(-self.center_velocity, min(self.center_velocity, velocity_command))
            
            self.publish_velocity(velocity_command)
            
            # Check if we've reached center (within tolerance)
            if abs(position_error) < 0.01:  # 0.01 position units tolerance
                self.stop_motor()
                self.get_logger().info('Reached center position! Zero sequence complete.')
                self.state = ZeroSequenceState.COMPLETED
        
        elif self.state == ZeroSequenceState.COMPLETED:
            self.stop_motor()
            # Optional: Log final results periodically
            pass
    
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
    node = ZeroSequencer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down zero sequencer...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
