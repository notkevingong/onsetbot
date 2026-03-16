#!/usr/bin/env python3
"""
ROS2 Node to command a BLDC motor via ODrive using CAN and to command STM32 peripherals. Subscribes to /launch_info for launch commands and publishes velocity commands to ODrive and position commands to STM32. Also subscribes to ODrive joint states for feedback.
"""

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState
from onset_interfaces.msg import LaunchCommand, HomeCommand, STM32Message, STM32State, OnsetStatus
from sensor_msgs.msg import JointState
import math
import time

class ActuatorCommand(Node):
    def __init__(self):
        super().__init__('actuator_commands')

        # Active axes: both motors commanded simultaneously; axis1 used for position feedback
        self._odrive_axes = [('axis0', 0), ('axis1', 1)]
        self._odrive_feedback_axis_name = 'axis1'

        # Create per-axis publishers for velocity/torque commands
        self.odrive_axis_publishers = {
            axis_id: self.create_publisher(
                ControlMessage,
                f'/odrive_axis{axis_id}/control_message',
                10
            )
            for _, axis_id in self._odrive_axes
        }

        # Create publisher for position commands to /stm32_control
        self.stm32_control_publisher = self.create_publisher(
            STM32Message,
            'stm32_control',
            10
        )

        # Create subscriber for receiving launch data from /launch_info
        self.launch_subscriber = self.create_subscription(
            LaunchCommand,
            'launch_info',
            self.convert_callback,
            10
        )

        # Create subscriber for receiving home commands from /home_info
        self.home_subscriber = self.create_subscription(
            HomeCommand,
            'home_info',
            self.home_callback,
            10
        )

        # Create subscriber for reading state of ODrives from /joint_states
        self.odrive_subscriber = self.create_subscription(
            JointState,
            '/odrive/joint_states',
            self.update_odrive_position,
            10
        )

        # Create subscriber for reading STM32 switch/power state from /stm32_states
        self.stm32_state_subscriber = self.create_subscription(
            STM32State,
            'stm32_states',
            self.update_stm32_state,
            10
        )

        # Create publisher for homing/ready status to /onset_status
        self.onset_status_publisher = self.create_publisher(
            OnsetStatus,
            'onset_status',
            10
        )

        # Create per-axis state clients
        self.axis_state_clients = {
            axis_id: self.create_client(
                AxisState,
                f'/odrive_{axis_name}/request_axis_state'
            )
            for axis_name, axis_id in self._odrive_axes
        }
        self.get_logger().info(
            f'ODrive command targets: {[(n, i) for n, i in self._odrive_axes]}, '
            f'feedback axis: {self._odrive_feedback_axis_name}'
        )
        
        # Define variables and conversion constants
        self.velocity = 0.0 # in turns/sec
        self.angle_launch = 0.0
        self._velocity_conversion_constant = (1 / 0.0375) * (60 / (2*math.pi)) * (1 / 60) * (46.3 / 25.9)
        self._op_range_conversion_constant = (25.0 / 46.0) * (2 * math.pi * 0.0375)
        self._angle_conversion_constant = (0.5 / 180)
        self._odrive_direction_sign_by_axis = {
            'axis0': 1.0,
            'axis1': -1.0,
        }
        self.get_logger().info(
            f'ODrive direction signs: {self._odrive_direction_sign_by_axis}'
        )

        #Define state variables
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.current_position_by_axis = {'axis0': 0.0, 'axis1': 0.0}  # All axis positions
        self.current_velocity_by_axis = {'axis0': 0.0, 'axis1': 0.0}  # All axis velocities
        self.switch2_state = False
        self.switch3_state = False
        self.elbow_moving_status = 0
        self.elbow_powered_status = 0
        self._stm32_state_seen = False
        
        # Define homing status variables
        self.is_homed = False
        self.home_active = False
        self.homing_state = 'idle'
        self.home_min_position = None
        self.home_max_position = None
        self.offset_min_position = None
        self.offset_max_position = None
        self.offset_min_fraction = 0.2
        self.offset_max_fraction = 0.8
        self._stm32_home_complete = False
        self._bldc_home_complete = False

        # Home constants
        self._home_velocity = 1.0
        self._home_clear_velocity = 1.0
        self._home_clear_duration_sec = 0.4
        self._home_position_tolerance = 0.05 # in turns
        self._home_clear_start_time = 0.0

        # Launch tuning constants
        self._launch_velocity_limit_turns_per_sec = 70.0
        self._launch_seek_to_start_velocity_turns_per_sec = 3.0
        self._launch_decel_start_fraction = 0.8

        # Launch state
        self.launch_active = False
        self.launch_state = 'idle'
        self.launch_target_velocity = 0.0
        self.launch_start_position = 0.0
        self.launch_end_position = 0.0
        self.launch_decel_position = 0.0

        # Timers
        self.homing_timer = self.create_timer(0.05, self._homing_step)

        # Publish initial status
        self._publish_onset_status(homed=False, busy=False)

        # Initialize all ODrive axes for command acceptance
        self.initialize_closed_loop()

    def publish_velocity(self):
        """Publish the velocity in the sequence"""
        
        # Create velocity command message
        msg = ControlMessage()
        msg.control_mode = 2  # Velocity control mode
        msg.input_mode = 2  # Ramped Velocity Mode
        msg.input_pos = 0.0
        msg.input_torque = 0.0
        msg.input_vel = self.velocity
        
        # Publish the message to all axes
        while self.current_position < self.angle_launch:
            for axis_name, axis_id in self._odrive_axes:
                direction = self._odrive_direction_sign_by_axis.get(axis_name, 1.0)
                axis_msg = ControlMessage()
                axis_msg.control_mode = msg.control_mode
                axis_msg.input_mode = msg.input_mode
                axis_msg.input_pos = 0.0
                axis_msg.input_torque = direction * msg.input_torque
                axis_msg.input_vel = direction * msg.input_vel
                self.odrive_axis_publishers[axis_id].publish(axis_msg)
            self.get_logger().info(f'Published velocity: {self.velocity} m/s')
    
    def initialize_closed_loop(self):
        """Initialize all configured motors in closed loop control mode (axis state 8)"""
        for axis_name, axis_id in self._odrive_axes:
            client = self.axis_state_clients[axis_id]
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'Axis state service not available for {axis_name}!')
                continue
            request = AxisState.Request()
            request.axis_requested_state = 8  # CLOSED_LOOP_CONTROL state
            future = client.call_async(request)
            future.add_done_callback(
                lambda f, name=axis_name: self.axis_state_callback(f, name)
            )

    def axis_state_callback(self, future, axis_name: str = ''):
        """Callback for axis state service response"""
        try:
            response = future.result()
            self.get_logger().info(
                f'Motor initialized ({axis_name}) - Axis State: {response.axis_state}, '
                f'Active Errors: {response.active_errors}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to set axis state ({axis_name}): {e}')

    def home_callback(self, msg: HomeCommand):
        if not bool(getattr(msg, "home_onset_request", False)):
            return

        self.get_logger().info('Received home_onset_request=1 from /home_info')
        self.home_motors()

    def convert_callback(self, msg: LaunchCommand):
        self.velocity = msg.velocity * self._velocity_conversion_constant
        self.angle_launch = msg.angle_launch * self._angle_conversion_constant
        self.get_logger().info(
            f'Launch command received (not executed in actuator_command2 yet): '
            f'vel={self.velocity:.3f} turns/s, angle={self.angle_launch:.3f} turns'
        )

    def home_motors(self):
        if self.home_active:
            self.get_logger().info('Homing already in progress')
            return

        self.is_homed = False
        self.home_active = True
        self.homing_state = 'clear_sw2'
        self.home_min_position = None
        self.home_max_position = None
        self.offset_min_position = None
        self.offset_max_position = None
        self._bldc_home_complete = False
        self._stm32_home_complete = False
        self._home_clear_start_time = time.monotonic()

        self._publish_onset_status(homed=False, busy=True)

        self._publish_stm32_command(power_on_status=1, home_elbow_request=1, angle_launch=0.0)
        self._publish_stm32_command(power_on_status=1, home_elbow_request=0, angle_launch=0.0)

        self.get_logger().info(
            f'Started homing: clear_sw2 for {self._home_clear_duration_sec:.2f}s, '
            f'then seek_sw2 -> seek_sw3. Waiting for STM32 elbow_moving_status==3 in parallel.'
        )

    def _homing_step(self):
        if not self.home_active:
            return

        if not self._stm32_home_complete and self.elbow_moving_status == 3:
            self._stm32_home_complete = True
            self.get_logger().info('STM32 elbow homing complete (elbow_moving_status==3)')

        if self.homing_state == 'clear_sw2':
            self._publish_odrive_velocity(abs(self._home_clear_velocity))
            if (time.monotonic() - self._home_clear_start_time) >= self._home_clear_duration_sec:
                self._publish_odrive_velocity(0.0)
                self.homing_state = 'seek_sw2'
                self.get_logger().info(
                    f'Transition: seek_sw2 at { -abs(self._home_velocity):.2f} turns/s'
                )
            return

        if self.homing_state == 'seek_sw2':
            self._publish_odrive_velocity(-abs(self._home_velocity))
            if self.switch2_state:
                self._publish_odrive_velocity(0.0)
                self.home_min_position = self.current_position
                self.homing_state = 'seek_sw3'
                self.get_logger().info(
                    f'SW2 hit. Saved min position: {self.home_min_position:.4f}. '
                    f'Transition: seek_sw3 at {abs(self._home_velocity):.2f} turns/s'
                )
            return

        if self.homing_state == 'seek_sw3':
            self._publish_odrive_velocity(abs(self._home_velocity))
            if self.switch3_state:
                self._publish_odrive_velocity(0.0)
                self.home_max_position = self.current_position
                self._compute_homing_offsets()
                self._bldc_home_complete = True

                if self._stm32_home_complete:
                    self._complete_homing()
                else:
                    self.homing_state = 'wait_stm32_home'
                    self.get_logger().info(
                        'BLDC homing complete; waiting for STM32 elbow_moving_status==3'
                    )
            return

        if self.homing_state == 'wait_stm32_home':
            self._publish_odrive_velocity(0.0)
            if self._stm32_home_complete:
                self._complete_homing()
            return

    def _compute_homing_offsets(self):
        if self.home_min_position is None or self.home_max_position is None:
            self.get_logger().error('Cannot compute offsets: home min/max are not available')
            return

        span = self.home_max_position - self.home_min_position
        span_abs = abs(span)
        if span_abs <= self._home_position_tolerance:
            self.get_logger().error(
                f'Homing span too small ({span_abs:.4f} turns); keeping raw min/max as offsets'
            )
            self.offset_min_position = self.home_min_position
            self.offset_max_position = self.home_max_position
            return

        min_frac = min(max(self.offset_min_fraction, 0.0), 1.0)
        max_frac = min(max(self.offset_max_fraction, 0.0), 1.0)
        if max_frac <= min_frac:
            max_frac = min(1.0, min_frac + 0.1)

        direction = 1.0 if span >= 0.0 else -1.0
        self.offset_min_position = self.home_min_position + direction * span_abs * min_frac
        self.offset_max_position = self.home_min_position + direction * span_abs * max_frac

        self.get_logger().info(
            f'SW3 hit. Saved max position: {self.home_max_position:.4f}. '
            f'Offset limits: min={self.offset_min_position:.4f} (frac={min_frac:.2f}), '
            f'max={self.offset_max_position:.4f} (frac={max_frac:.2f})'
        )

    def _complete_homing(self):
        self._publish_odrive_velocity(0.0)
        self.home_active = False
        self.homing_state = 'idle'
        self.is_homed = True
        self._publish_onset_status(homed=True, busy=False)
        self.get_logger().info(
            f'Homing complete. home_min={self.home_min_position:.4f}, '
            f'home_max={self.home_max_position:.4f}, '
            f'offset_min={self.offset_min_position:.4f}, '
            f'offset_max={self.offset_max_position:.4f}, '
            f'stm32_home_complete={int(self._stm32_home_complete)}'
        )

    def _publish_odrive_velocity(self, velocity: float):
        for axis_name, axis_id in self._odrive_axes:
            direction = self._odrive_direction_sign_by_axis.get(axis_name, 1.0)
            msg = ControlMessage()
            msg.control_mode = 2
            msg.input_mode = 2
            msg.input_pos = 0.0
            msg.input_torque = 0.0
            msg.input_vel = float(direction * velocity)
            self.odrive_axis_publishers[axis_id].publish(msg)

    def _publish_stm32_command(self, power_on_status: int, home_elbow_request: int, angle_launch: float):
        msg = STM32Message()
        msg.angle_launch = float(angle_launch)
        msg.power_on_status = int(power_on_status)
        msg.home_elbow_request = int(home_elbow_request)
        msg.stm32_state_request = 0
        self.stm32_control_publisher.publish(msg)

    def _publish_onset_status(self, homed: bool, busy: bool):
        msg = OnsetStatus()

        if hasattr(msg, 'onset_is_homed'):
            msg.onset_is_homed = int(1 if homed else 0)
        if hasattr(msg, 'bool_homed'):
            msg.bool_homed = bool(homed)

        if hasattr(msg, 'onset_is_busy'):
            msg.onset_is_busy = int(1 if busy else 0)
        if hasattr(msg, 'bool_busy'):
            msg.bool_busy = bool(busy)

        self.onset_status_publisher.publish(msg)

    def update_odrive_position(self, msg: JointState):
        for i, axis_name in enumerate(msg.name):
            if axis_name not in self._odrive_direction_sign_by_axis:
                continue
            direction = self._odrive_direction_sign_by_axis[axis_name]
            if i < len(msg.position):
                self.current_position_by_axis[axis_name] = float(direction * msg.position[i])
            if i < len(msg.velocity):
                self.current_velocity_by_axis[axis_name] = float(direction * msg.velocity[i])

        target_name = self._odrive_feedback_axis_name
        if target_name not in msg.name:
            return
        idx = msg.name.index(target_name)
        sign = self._odrive_direction_sign_by_axis.get(target_name, 1.0)
        if idx < len(msg.position):
            self.current_position = float(sign * msg.position[idx])
        if idx < len(msg.velocity):
            self.current_velocity = float(sign * msg.velocity[idx])

    def update_stm32_state(self, msg: STM32State):
        self._stm32_state_seen = True
        self.switch2_state = bool(msg.sw2 == 1)
        self.switch3_state = bool(msg.sw3 == 1)
        self.elbow_moving_status = int(msg.elbow_moving_status)
        self.elbow_powered_status = int(msg.elbow_power_status)


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down actuator command2...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

