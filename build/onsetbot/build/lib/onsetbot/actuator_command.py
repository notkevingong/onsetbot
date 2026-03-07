#!/usr/bin/env python3
"""
ROS2 Node to command a BLDC motor via ODrive using CAN and to command STM32 peripherals. Subscribes to /launch_info for launch commands and publishes velocity commands to ODrive and position commands to STM32. Also subscribes to ODrive joint states for feedback.
"""

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState
from onset_interfaces.msg import LaunchCommand, STM32Message, STM32State, OnsetStatus
from sensor_msgs.msg import JointState
import math
import time

class ActuatorCommand(Node):
    def __init__(self):
        super().__init__('actuator_command')
        
        # Create publisher for velocity commands to ODrive to /odrive_control
        self.odrive_control_publisher = self.create_publisher(
            ControlMessage,
            'odrive_control',
            10
        )

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

        # Create client for axis state control
        self.axis_state_client = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        
        # Define variables and conversion constants
        self.velocity = 0.0
        self.angle_launch = 0.0
        self._velocity_conversion_constant = (1 / 0.0375) * (60 / (2*math.pi)) * (1 / 60) * 2
        self._angle_conversion_constant = (0.5 / 180)

        # Define variables for storing position and velocity feedback from ODrive
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.switch2_state = False
        self.switch3_state = False
        self.elbow_moving_status = 0
        self.elbow_power_status = 0
        self._stm32_state_seen = False
        self._last_stm32_state_time = 0.0
        self._last_power_wait_log_time = 0.0

        # Homing state
        self.is_homed = False
        self.homing_active = False
        self.homing_state = 'idle'
        self.home_min_position = None
        self.home_max_position = None
        self.home_target_position = None
        self._last_homing_debug_log_time = 0.0

        # Homing tuning constants
        self._home_seek_velocity = 1.0
        self._home_return_velocity = 2.0
        self._home_safety_offset_turns = 0.05
        self._home_position_tolerance_turns = 0.3

        # Launch tuning constants
        self._launch_velocity_limit_turns_per_sec = 25.0
        self._launch_return_to_min_velocity_turns_per_sec = 3.0
        self._launch_position_tolerance_turns = 0.01
        self._launch_min_position_tolerance_turns = 0.5
        self._launch_max_position_tolerance_turns = 1.5

        # Launch state
        self.launch_active = False
        self.launch_state = 'idle'
        self.launch_target_velocity = 0.0
        self.launch_mid_position = 0.0
        self.launch_end_position = 0.0
        self.launch_direction = 1.0
        self._launch_last_cmd_vel = 0.0

        # Timer to run homing state machine without blocking callbacks
        self.homing_timer = self.create_timer(0.05, self._homing_step)
        self.launch_timer = self.create_timer(0.02, self._launch_step)

        # Publish initial status
        self._publish_onset_status(homed=False, busy=False)
        
        # Initialize closed loop control
        self.initialize_closed_loop()
        
    
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
        while self.current_position < self.angle_launch:
            self.odrive_control_publisher.publish(msg)
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

    def convert_callback(self, msg: LaunchCommand):
        if bool(getattr(msg, "home_onset_request", False)):
            self.get_logger().info('Received home_onset_request=1 from launch_info')
            self.home_motors()
            return

        if not self.is_homed:
            self.get_logger().info('Ignoring launch command: system not homed yet')
            return

        self.velocity = msg.velocity * self._velocity_conversion_constant
        self.angle_launch = msg.angle_launch * self._angle_conversion_constant
        self.launch_ball()

    def _start_launch_sequence(self, target_velocity_turns_per_sec: float):
        if self.homing_active:
            self.get_logger().warn('Ignoring launch command: homing in progress')
            return

        if self.home_min_position is None or self.home_max_position is None:
            self.get_logger().warn('Ignoring launch command: home bounds are not available')
            return

        span = self.home_max_position - self.home_min_position
        if abs(span) <= self._launch_position_tolerance_turns:
            self.get_logger().warn('Ignoring launch command: home range is too small')
            return

        if self.launch_active:
            self.get_logger().warn('Ignoring launch command: launch sequence already active')
            return

        self.launch_direction = 1.0 if span >= 0.0 else -1.0
        self.launch_mid_position = (self.home_min_position + self.home_max_position) * 0.5
        self.launch_end_position = self.home_max_position
        limited_mag = min(abs(target_velocity_turns_per_sec), self._launch_velocity_limit_turns_per_sec)
        self.launch_target_velocity = self.launch_direction * limited_mag
        self.launch_state = 'seek_min_start'
        self.launch_active = True
        self._launch_last_cmd_vel = 0.0
        self._publish_onset_status(homed=True, busy=True)
        self.get_logger().info(
            f'Launch start: target_vel={self.launch_target_velocity:.2f} turns/s, '
            f'mid={self.launch_mid_position:.4f}, end={self.launch_end_position:.4f}, '
            f'return_to_min_vel={self._launch_return_to_min_velocity_turns_per_sec:.2f} turns/s'
        )

    def _position_progress(self, position: float, start: float, end: float) -> float:
        if end >= start:
            return position - start
        return start - position

    def _is_at_min_position(self) -> bool:
        if self.home_min_position is None:
            return False
        if self.launch_direction >= 0.0:
            return self.current_position <= (self.home_min_position + self._launch_min_position_tolerance_turns)
        return self.current_position >= (self.home_min_position - self._launch_min_position_tolerance_turns)

    def _is_at_max_position(self) -> bool:
        if self.home_max_position is None:
            return False
        if self.launch_direction >= 0.0:
            return self.current_position >= (self.home_max_position - self._launch_max_position_tolerance_turns)
        return self.current_position <= (self.home_max_position + self._launch_max_position_tolerance_turns)

    def _stop_launch_sequence(self, reason: str):
        self._publish_odrive_velocity(0.0)
        self.launch_active = False
        self.launch_state = 'idle'
        self._launch_last_cmd_vel = 0.0
        self._publish_onset_status(homed=True, busy=False)
        self.get_logger().info(f'Launch complete: {reason}')

    def _launch_step(self):
        if not self.launch_active:
            return

        if self.homing_active:
            self._stop_launch_sequence('aborted due to homing start')
            return

        start_to_end = self._position_progress(self.launch_end_position, self.home_min_position, self.launch_end_position)
        start_to_mid = self._position_progress(self.launch_mid_position, self.home_min_position, self.launch_end_position)
        pos_from_start = self._position_progress(self.current_position, self.home_min_position, self.launch_end_position)

        if self.launch_state == 'seek_min_start':
            min_error = self.home_min_position - self.current_position
            if self._is_at_min_position():
                self._publish_odrive_velocity(0.0)
                self.launch_state = 'ramp_up_to_mid'
                self.get_logger().info('Launch transition: ramp_up_to_mid (started at min, vel=0)')
                return

            cmd_vel = self._launch_return_to_min_velocity_turns_per_sec if min_error > 0.0 else -self._launch_return_to_min_velocity_turns_per_sec
            self._publish_odrive_velocity(cmd_vel)
            return

        if self._is_at_max_position() or pos_from_start >= (start_to_end - self._launch_max_position_tolerance_turns):
            if self.launch_state != 'return_to_min':
                self._publish_odrive_velocity(0.0)
                self.launch_state = 'return_to_min'
                self.get_logger().info('Launch transition: return_to_min')
                return

        if self.launch_state == 'return_to_min':
            min_error = self.home_min_position - self.current_position
            if self._is_at_min_position():
                self._stop_launch_sequence('returned to min position')
                return

            cmd_vel = self._launch_return_to_min_velocity_turns_per_sec if min_error > 0.0 else -self._launch_return_to_min_velocity_turns_per_sec
            self._publish_odrive_velocity(cmd_vel)
            return

        if self.launch_state == 'ramp_up_to_mid' and pos_from_start >= (start_to_mid - self._launch_position_tolerance_turns):
            self.launch_state = 'ramp_down_to_max'
            self.get_logger().info('Launch transition: ramp_down_to_max')

        target_mag = abs(self.launch_target_velocity)
        if self.launch_state == 'ramp_up_to_mid':
            accel_distance = max(start_to_mid, self._launch_position_tolerance_turns)
            ramp_ratio = min(max(pos_from_start / accel_distance, 0.0), 1.0)
            cmd_vel = self.launch_direction * (target_mag * ramp_ratio)
        else:
            decel_distance = max(start_to_end - start_to_mid, self._launch_position_tolerance_turns)
            travelled_after_mid = max(pos_from_start - start_to_mid, 0.0)
            ramp_ratio = min(max(1.0 - (travelled_after_mid / decel_distance), 0.0), 1.0)
            cmd_vel = self.launch_direction * (target_mag * ramp_ratio)

        self._launch_last_cmd_vel = cmd_vel
        self._publish_odrive_velocity(cmd_vel)

    def home_motors(self):
        """Start homing sequence: SW2 -> SW3 -> return near SW2 with safety offset."""
        if self.homing_active:
            self.get_logger().info('Homing already in progress')
            return

        self.is_homed = False
        self.homing_active = True
        self.homing_state = 'seek_sw2'
        self.home_min_position = None
        self.home_max_position = None
        self.home_target_position = None
        self._last_homing_debug_log_time = 0.0

        self._publish_onset_status(homed=False, busy=True)

        # Ensure elbow power command is ON while homing
        self._publish_stm32_command(power_on_status=1, home_elbow_request=0, angle_launch=0.0)
        self.get_logger().info(f'Started homing: state=seek_sw2, command velocity={-abs(self._home_seek_velocity):.2f} turns/s until SW2')

    def _log_homing_debug(self, message: str, interval_s: float = 0.5):
        now = time.monotonic()
        if (now - self._last_homing_debug_log_time) >= interval_s:
            self.get_logger().info(message)
            self._last_homing_debug_log_time = now

    def _homing_step(self):
        if not self.homing_active:
            return

        # Keep power requested ON during homing
        if self._stm32_state_seen and self.elbow_power_status != 1:
            self._publish_stm32_command(power_on_status=1, home_elbow_request=0, angle_launch=0.0)
            self._publish_odrive_velocity(0.0)
            now = time.monotonic()
            if (now - self._last_power_wait_log_time) >= 1.0:
                self.get_logger().warn(
                    f'Homing wait: elbow power is OFF (elbow_power_status={self.elbow_power_status}); '
                    f'holding motor at 0.0 turns/s'
                )
                self._last_power_wait_log_time = now
            return

        if not self._stm32_state_seen:
            self._log_homing_debug(
                'Homing proceeding without STM32 state feedback yet; elbow power interlock not enforced',
                interval_s=2.0
            )

        if self.homing_state == 'seek_sw2':
            self._publish_odrive_velocity(-abs(self._home_seek_velocity))
            self._log_homing_debug(
                f'Homing state=seek_sw2 | pos={self.current_position:.4f} | '
                f'sw2={int(self.switch2_state)} sw3={int(self.switch3_state)} | '
                f'cmd_vel={-abs(self._home_seek_velocity):.2f} turns/s'
            )
            if self.switch2_state:
                self._publish_odrive_velocity(0.0)
                self.home_min_position = self.current_position
                self.homing_state = 'seek_sw3'
                self.get_logger().info(f'SW2 hit. Min position stored: {self.home_min_position:.4f}')
                self.get_logger().info(f'Transition: state=seek_sw3, command velocity={self._home_seek_velocity:.2f} turns/s until SW3')
            return

        if self.homing_state == 'seek_sw3':
            self._publish_odrive_velocity(abs(self._home_seek_velocity))
            self._log_homing_debug(
                f'Homing state=seek_sw3 | pos={self.current_position:.4f} | '
                f'sw2={int(self.switch2_state)} sw3={int(self.switch3_state)} | '
                f'cmd_vel={abs(self._home_seek_velocity):.2f} turns/s'
            )
            if self.switch3_state:
                self._publish_odrive_velocity(0.0)
                self.home_max_position = self.current_position
                span = abs(self.home_max_position - self.home_min_position)
                direction_from_sw2 = 1.0 if self.home_max_position >= self.home_min_position else -1.0
                max_safe_offset = max(span - self._home_position_tolerance_turns, 0.0)
                if max_safe_offset <= 0.0:
                    applied_offset = 0.0
                else:
                    min_offset = min(self._home_position_tolerance_turns, max_safe_offset)
                    applied_offset = min(self._home_safety_offset_turns, max_safe_offset)
                    applied_offset = max(applied_offset, min_offset)
                self.home_target_position = self.home_min_position + (direction_from_sw2 * applied_offset)
                self.homing_state = 'return_sw2_offset'
                self.get_logger().info(
                    f'SW3 hit. Max position stored: {self.home_max_position:.4f}. '
                    f'Transition: state=return_sw2_offset (direct), target={self.home_target_position:.4f}, '
                    f'cmd_vel={-abs(self._home_return_velocity):.2f} turns/s'
                )
                return
            return

        if self.homing_state == 'return_sw2_offset':
            if self.home_target_position is None:
                self._publish_odrive_velocity(0.0)
                self.homing_active = False
                self.homing_state = 'idle'
                self.get_logger().error('Homing return target is undefined; aborting homing')
                self._publish_onset_status(homed=False, busy=False)
                return

            position_error = self.home_target_position - self.current_position
            if abs(position_error) <= self._home_position_tolerance_turns:
                self._publish_odrive_velocity(0.0)
                self.homing_active = False
                self.homing_state = 'idle'
                self.is_homed = True
                self._publish_onset_status(homed=True, busy=False)
                self.get_logger().info(
                    f'Homing complete. SW2={self.home_min_position:.4f}, '
                    f'SW3={self.home_max_position:.4f}, '
                    f'final={self.current_position:.4f}, '
                    f'target={self.home_target_position:.4f}'
                )
                return

            cmd_vel = self._home_return_velocity if position_error > 0.0 else -self._home_return_velocity
            self._publish_odrive_velocity(cmd_vel)
            self._log_homing_debug(
                f'Homing state=return_sw2_offset | pos={self.current_position:.4f} | '
                f'target={self.home_target_position:.4f} | err={position_error:.4f} | '
                f'cmd_vel={cmd_vel:.2f} turns/s'
            )
            return

    def _publish_odrive_velocity(self, velocity: float):
        msg = ControlMessage()
        msg.control_mode = 2  # Velocity control
        msg.input_mode = 2  # Ramped velocity
        msg.input_pos = 0.0
        msg.input_torque = 0.0
        msg.input_vel = velocity
        self.odrive_control_publisher.publish(msg)

    def _publish_stm32_command(self, power_on_status: int, home_elbow_request: int, angle_launch: float):
        msg = STM32Message()
        msg.angle_launch = angle_launch
        msg.power_on_status = power_on_status
        msg.home_elbow_request = home_elbow_request
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

        homed_log = None
        busy_log = None
        if hasattr(msg, 'onset_is_homed'):
            homed_log = msg.onset_is_homed
        elif hasattr(msg, 'bool_homed'):
            homed_log = int(msg.bool_homed)

        if hasattr(msg, 'onset_is_busy'):
            busy_log = msg.onset_is_busy
        elif hasattr(msg, 'bool_busy'):
            busy_log = int(msg.bool_busy)

        self.get_logger().info(f'Published OnsetStatus on onset_status: homed={homed_log}, busy={busy_log}')

    def launch_ball(self):
        """Run launch sequence from zero point to max with midpoint cruise and controlled decel."""
        self._start_launch_sequence(self.velocity)

    def update_odrive_position(self, msg: JointState):
        target_name = "axis0"
        if target_name not in msg.name:
            return
        idx = msg.name.index(target_name)
        self.current_position = msg.position[idx]
        self.current_velocity = msg.velocity[idx]

    def update_stm32_state(self, msg: STM32State):
        self._stm32_state_seen = True
        self._last_stm32_state_time = time.monotonic()
        self.switch2_state = msg.sw2 == 1
        self.switch3_state = msg.sw3 == 1
        self.elbow_moving_status = msg.elbow_moving_status
        self.elbow_power_status = msg.elbow_power_status


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorCommand()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down actuator command...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

