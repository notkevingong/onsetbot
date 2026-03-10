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
        self.home_operating_min_position = None
        self.home_operating_max_position = None
        self._home_operating_min_buffer_ratio = 0.10
        self._home_operating_max_buffer_ratio = 0.30
        self.home_target_position = None
        self._last_homing_debug_log_time = 0.0

        # Homing tuning constants
        self._home_seek_velocity = 1.0
        self._home_return_velocity = 2.0
        self._home_safety_offset_turns = 0.05
        self._home_position_tolerance_turns = 0.3

        # Launch tuning constants
        self._launch_velocity_limit_turns_per_sec = 100.0
        self._launch_return_to_min_velocity_turns_per_sec = 3.0
        self._launch_min_ramp_rate_turns_per_sec2 = 0.1
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
        self.launch_accel_rate_turns_per_sec2 = self._launch_min_ramp_rate_turns_per_sec2
        self.launch_decel_rate_turns_per_sec2 = self._launch_min_ramp_rate_turns_per_sec2
        self._launch_last_cmd_vel = 0.0
        self._launch_last_step_time = time.monotonic()
        self._post_launch_hold_duration_s = 5.0
        self._post_launch_hold_start_time = 0.0
        self._post_launch_home_pulse_sent = False
        self._launch_pending_after_elbow_position = False
        self._pending_launch_velocity_turns_per_sec = 0.0
        self._pending_launch_angle = 0.0
        self._pending_launch_requested_time = 0.0
        self._pending_launch_timeout_s = 10.0

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

        if self.homing_active:
            self.get_logger().info('Ignoring launch command: homing in progress')
            return

        if self.launch_active or self._launch_pending_after_elbow_position:
            self.get_logger().info('Ignoring launch command: launch already active or pending elbow positioning')
            return

        self.velocity = msg.velocity * self._velocity_conversion_constant
        self.angle_launch = msg.angle_launch * self._angle_conversion_constant

        self._pending_launch_velocity_turns_per_sec = self.velocity
        self._pending_launch_angle = msg.angle_launch
        self._launch_pending_after_elbow_position = True
        self._pending_launch_requested_time = time.monotonic()

        # Send launch angle to STM32 first; ODrive launch starts only after elbow_moving_status == 5
        self._publish_stm32_command(power_on_status=1, home_elbow_request=0, angle_launch=self._pending_launch_angle)
        self._publish_onset_status(homed=True, busy=True)
        self.get_logger().info(
            f'Launch queued: sent elbow angle={self._pending_launch_angle:.3f}; '
            f'waiting for elbow_moving_status==5 before ODrive launch'
        )

    def _start_launch_sequence(self, target_velocity_turns_per_sec: float):
        if self.homing_active:
            self.get_logger().warn('Ignoring launch command: homing in progress')
            return

        op_min, op_max = self._get_operating_min_max()
        if op_min is None or op_max is None:
            self.get_logger().warn('Ignoring launch command: operating bounds are not available')
            return

        span = op_max - op_min
        if abs(span) <= self._launch_position_tolerance_turns:
            self.get_logger().warn('Ignoring launch command: operating range is too small')
            return

        if self.launch_active:
            self.get_logger().warn('Ignoring launch command: launch sequence already active')
            return

        self._launch_pending_after_elbow_position = False
        self._pending_launch_requested_time = 0.0

        self.launch_direction = 1.0 if span >= 0.0 else -1.0
        self.launch_mid_position = (op_min + op_max) * 0.5
        self.launch_end_position = op_max
        limited_mag = min(abs(target_velocity_turns_per_sec), self._launch_velocity_limit_turns_per_sec)
        self.launch_target_velocity = self.launch_direction * limited_mag

        start_to_end = self._position_progress(self.launch_end_position, op_min, self.launch_end_position)
        start_to_mid = self._position_progress(self.launch_mid_position, op_min, self.launch_end_position)
        mid_to_end = max(start_to_end - start_to_mid, self._launch_position_tolerance_turns)

        accel_distance = max(start_to_mid, self._launch_position_tolerance_turns)
        target_mag = abs(self.launch_target_velocity)
        if target_mag <= 0.0:
            accel_rate = self._launch_min_ramp_rate_turns_per_sec2
            decel_rate = self._launch_min_ramp_rate_turns_per_sec2
        else:
            accel_rate = (target_mag * target_mag) / (2.0 * accel_distance)
            decel_rate = (target_mag * target_mag) / (2.0 * mid_to_end)

        self.launch_accel_rate_turns_per_sec2 = max(accel_rate, self._launch_min_ramp_rate_turns_per_sec2)
        self.launch_decel_rate_turns_per_sec2 = max(decel_rate, self._launch_min_ramp_rate_turns_per_sec2)

        self.launch_state = 'seek_min_start'
        self.launch_active = True
        self._launch_last_cmd_vel = 0.0
        self._launch_last_step_time = time.monotonic()
        self._post_launch_hold_start_time = 0.0
        self._post_launch_home_pulse_sent = False
        self._publish_onset_status(homed=True, busy=True)
        self.get_logger().info(
            f'Launch start: target_vel={self.launch_target_velocity:.2f} turns/s, '
            f'op_min={op_min:.4f}, '
            f'mid={self.launch_mid_position:.4f}, end={self.launch_end_position:.4f}, '
            f'accel_rate={self.launch_accel_rate_turns_per_sec2:.2f} turns/s^2, '
            f'decel_rate={self.launch_decel_rate_turns_per_sec2:.2f} turns/s^2, '
            f'return_to_min_vel={self._launch_return_to_min_velocity_turns_per_sec:.2f} turns/s'
        )

    def _get_operating_min_max(self):
        if self.home_operating_min_position is not None and self.home_operating_max_position is not None:
            return self.home_operating_min_position, self.home_operating_max_position
        return self.home_min_position, self.home_max_position

    def _position_progress(self, position: float, start: float, end: float) -> float:
        if end >= start:
            return position - start
        return start - position

    def _is_at_min_position(self) -> bool:
        op_min, _ = self._get_operating_min_max()
        if op_min is None:
            return False
        if self.launch_direction >= 0.0:
            return self.current_position <= (op_min + self._launch_min_position_tolerance_turns)
        return self.current_position >= (op_min - self._launch_min_position_tolerance_turns)

    def _is_at_max_position(self) -> bool:
        _, op_max = self._get_operating_min_max()
        if op_max is None:
            return False
        if self.launch_direction >= 0.0:
            return self.current_position >= (op_max - self._launch_max_position_tolerance_turns)
        return self.current_position <= (op_max + self._launch_max_position_tolerance_turns)

    def _get_post_launch_hold_max_position(self):
        if self.home_min_position is None or self.home_max_position is None:
            _, op_max = self._get_operating_min_max()
            return op_max
        return self.home_max_position

    def _stop_launch_sequence(self, reason: str):
        stop_ramp_rate = max(self.launch_decel_rate_turns_per_sec2, self._launch_min_ramp_rate_turns_per_sec2)
        self._publish_odrive_velocity(0.0, stop_ramp_rate)
        self.launch_active = False
        self.launch_state = 'idle'
        self._launch_last_cmd_vel = 0.0
        self._launch_last_step_time = time.monotonic()
        self._publish_onset_status(homed=True, busy=False)
        self.get_logger().info(f'Launch complete: {reason}')

    def _slew_limited_velocity(self, target_velocity: float, ramp_rate: float) -> float:
        now = time.monotonic()
        dt = now - self._launch_last_step_time
        if dt <= 0.0:
            dt = 0.02
        self._launch_last_step_time = now

        safe_ramp_rate = max(ramp_rate, self._launch_min_ramp_rate_turns_per_sec2)
        max_delta = safe_ramp_rate * dt
        delta = target_velocity - self._launch_last_cmd_vel

        if abs(delta) <= max_delta:
            cmd_vel = target_velocity
        else:
            cmd_vel = self._launch_last_cmd_vel + math.copysign(max_delta, delta)

        self._launch_last_cmd_vel = cmd_vel
        return cmd_vel

    def _launch_step(self):
        if not self.launch_active:
            return

        if self.homing_active:
            self._stop_launch_sequence('aborted due to homing start')
            return

        op_min, _ = self._get_operating_min_max()
        if op_min is None:
            self._stop_launch_sequence('aborted: operating min bound unavailable')
            return

        start_to_end = self._position_progress(self.launch_end_position, op_min, self.launch_end_position)
        start_to_mid = self._position_progress(self.launch_mid_position, op_min, self.launch_end_position)
        pos_from_start = self._position_progress(self.current_position, op_min, self.launch_end_position)

        if self.launch_state == 'seek_min_start':
            min_error = op_min - self.current_position
            if self._is_at_min_position():
                self._publish_odrive_velocity(0.0, self.launch_decel_rate_turns_per_sec2)
                self.launch_state = 'ramp_up_to_mid'
                self.get_logger().info('Launch transition: ramp_up_to_mid (started at min, vel=0)')
                self._launch_last_step_time = time.monotonic()
                return

            cmd_vel = self._launch_return_to_min_velocity_turns_per_sec if min_error > 0.0 else -self._launch_return_to_min_velocity_turns_per_sec
            cmd_vel = self._slew_limited_velocity(cmd_vel, self.launch_accel_rate_turns_per_sec2)
            self._publish_odrive_velocity(cmd_vel, self.launch_accel_rate_turns_per_sec2)
            return

        if self.launch_state in ('seek_min_start', 'ramp_up_to_mid', 'ramp_down_to_max') and (
            self._is_at_max_position() or pos_from_start >= (start_to_end - self._launch_max_position_tolerance_turns)
        ):
            if self.launch_state != 'return_to_min':
                self._publish_odrive_velocity(0.0)
                self.launch_state = 'return_to_min'
                self.get_logger().info('Launch transition: return_to_min')
                return

        if self.launch_state == 'return_to_min':
            min_error = op_min - self.current_position
            if self._is_at_min_position():
                self._publish_odrive_velocity(0.0, self.launch_decel_rate_turns_per_sec2)
                self.launch_state = 'post_launch_home_elbow'
                self.get_logger().info('Launch transition: post_launch_home_elbow')
                return

            cmd_vel = self._launch_return_to_min_velocity_turns_per_sec if min_error > 0.0 else -self._launch_return_to_min_velocity_turns_per_sec
            cmd_vel = self._slew_limited_velocity(cmd_vel, self.launch_accel_rate_turns_per_sec2)
            self._publish_odrive_velocity(cmd_vel, self.launch_accel_rate_turns_per_sec2)
            return

        if self.launch_state == 'post_launch_home_elbow':
            if not self._post_launch_home_pulse_sent:
                self._publish_stm32_command(power_on_status=1, home_elbow_request=1, angle_launch=0.0)
                self._publish_stm32_command(power_on_status=1, home_elbow_request=0, angle_launch=0.0)
                self._post_launch_home_pulse_sent = True
                self.get_logger().info('Post-launch: sent elbow home request pulse')
            self.launch_state = 'post_launch_seek_max'
            self.get_logger().info('Launch transition: post_launch_seek_max')
            return

        if self.launch_state == 'post_launch_seek_max':
            post_launch_max_target = self._get_post_launch_hold_max_position()
            if post_launch_max_target is None:
                self._stop_launch_sequence('aborted: post-launch max target unavailable')
                return

            if self.launch_direction >= 0.0:
                at_post_launch_max = self.current_position >= (post_launch_max_target - self._launch_max_position_tolerance_turns)
            else:
                at_post_launch_max = self.current_position <= (post_launch_max_target + self._launch_max_position_tolerance_turns)

            if at_post_launch_max:
                self._publish_odrive_velocity(0.0, self.launch_decel_rate_turns_per_sec2)
                self._post_launch_hold_start_time = time.monotonic()
                self.launch_state = 'post_launch_hold_max'
                self.get_logger().info(
                    f'Launch transition: post_launch_hold_max (holding at max for 5s, target={post_launch_max_target:.4f})'
                )
                return

            cmd_vel = abs(self._launch_return_to_min_velocity_turns_per_sec)
            if self.launch_direction < 0.0:
                cmd_vel = -cmd_vel
            cmd_vel = self._slew_limited_velocity(cmd_vel, self.launch_accel_rate_turns_per_sec2)
            self._publish_odrive_velocity(cmd_vel, self.launch_accel_rate_turns_per_sec2)
            return

        if self.launch_state == 'post_launch_hold_max':
            self._publish_odrive_velocity(0.0, self.launch_decel_rate_turns_per_sec2)
            if (time.monotonic() - self._post_launch_hold_start_time) >= self._post_launch_hold_duration_s:
                self.launch_state = 'post_launch_return_min'
                self.get_logger().info('Launch transition: post_launch_return_min')
            return

        if self.launch_state == 'post_launch_return_min':
            min_error = op_min - self.current_position
            if self._is_at_min_position():
                self._stop_launch_sequence('post-launch sequence complete: elbow home + max hold + return min')
                return

            cmd_vel = self._launch_return_to_min_velocity_turns_per_sec if min_error > 0.0 else -self._launch_return_to_min_velocity_turns_per_sec
            cmd_vel = self._slew_limited_velocity(cmd_vel, self.launch_accel_rate_turns_per_sec2)
            self._publish_odrive_velocity(cmd_vel, self.launch_accel_rate_turns_per_sec2)
            return

        if self.launch_state == 'ramp_up_to_mid' and pos_from_start >= (start_to_mid - self._launch_position_tolerance_turns):
            self.launch_state = 'ramp_down_to_max'
            self.get_logger().info('Launch transition: ramp_down_to_max')

        if self.launch_state == 'ramp_up_to_mid':
            target_cmd_vel = self.launch_target_velocity
            ramp_rate = self.launch_accel_rate_turns_per_sec2
        else:
            target_cmd_vel = 0.0
            ramp_rate = self.launch_decel_rate_turns_per_sec2

        cmd_vel = self._slew_limited_velocity(target_cmd_vel, ramp_rate)
        self._publish_odrive_velocity(cmd_vel, ramp_rate)

    def home_motors(self):
        """Start homing sequence: SW2 -> SW3 -> return near SW2 with safety offset."""
        if self.homing_active:
            self.get_logger().info('Homing already in progress')
            return

        if self.launch_active:
            self._stop_launch_sequence('aborted due to homing request')

        self._launch_pending_after_elbow_position = False
        self._pending_launch_velocity_turns_per_sec = 0.0
        self._pending_launch_requested_time = 0.0

        self.is_homed = False
        self.homing_active = True
        self.homing_state = 'seek_sw2'
        self.home_min_position = None
        self.home_max_position = None
        self.home_operating_min_position = None
        self.home_operating_max_position = None
        self.home_target_position = None
        self._last_homing_debug_log_time = 0.0

        self._publish_onset_status(homed=False, busy=True)

        # Pulse elbow-home request at homing start: 1 then 0
        self._publish_stm32_command(power_on_status=1, home_elbow_request=1, angle_launch=0.0)
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

                # Define buffered operating range: 10% inward from min, 20% inward from max
                min_buffer_turns = span * self._home_operating_min_buffer_ratio
                max_buffer_turns = span * self._home_operating_max_buffer_ratio
                op_min = self.home_min_position + (direction_from_sw2 * min_buffer_turns)
                op_max = self.home_max_position - (direction_from_sw2 * max_buffer_turns)
                if abs(op_max - op_min) <= self._launch_position_tolerance_turns:
                    self.home_operating_min_position = self.home_min_position
                    self.home_operating_max_position = self.home_max_position
                    self.get_logger().warn(
                        'Operating range buffer collapsed span; using full recorded homing range'
                    )
                else:
                    self.home_operating_min_position = op_min
                    self.home_operating_max_position = op_max

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
                    f'Operating range: min={self.home_operating_min_position:.4f}, '
                    f'max={self.home_operating_max_position:.4f}. '
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
                post_launch_hold_max = self._get_post_launch_hold_max_position()
                self.get_logger().info(
                    f'Homing complete. SW2={self.home_min_position:.4f}, '
                    f'SW3={self.home_max_position:.4f}, '
                    f'op_min={self.home_operating_min_position:.4f}, '
                    f'op_max={self.home_operating_max_position:.4f}, '
                    f'post_launch_hold_max={post_launch_hold_max:.4f}, '
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

    def _publish_odrive_velocity(self, velocity: float, vel_ramp_rate: float = 0.0):
        msg = ControlMessage()
        msg.control_mode = 2  # Velocity control
        msg.input_mode = 2  # Ramped velocity
        msg.input_pos = 0.0
        msg.input_torque = 0.0
        msg.input_vel = velocity
        if hasattr(msg, 'vel_ramp_rate'):
            msg.vel_ramp_rate = max(0.0, float(vel_ramp_rate))
        self.odrive_control_publisher.publish(msg)

    def _publish_stm32_command(self, power_on_status: int, home_elbow_request: int, angle_launch: float):
        msg = STM32Message()
        msg.angle_launch = angle_launch
        msg.power_on_status = power_on_status
        if hasattr(msg, 'home_elbow'):
            msg.home_elbow = home_elbow_request
        if hasattr(msg, 'home_elbow_request'):
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

        if not self._launch_pending_after_elbow_position:
            return

        if (time.monotonic() - self._pending_launch_requested_time) > self._pending_launch_timeout_s:
            self._launch_pending_after_elbow_position = False
            self._pending_launch_requested_time = 0.0
            self._publish_onset_status(homed=self.is_homed, busy=False)
            self.get_logger().warn('Clearing pending launch: timeout waiting for elbow_moving_status==5 (10s)')
            return

        if not self.is_homed:
            self._launch_pending_after_elbow_position = False
            self._pending_launch_requested_time = 0.0
            self._publish_onset_status(homed=False, busy=False)
            self.get_logger().warn('Clearing pending launch: system no longer homed')
            return

        if self.elbow_moving_status == 5:
            self.get_logger().info('Elbow positioned (elbow_moving_status==5); starting ODrive launch')
            pending_velocity = self._pending_launch_velocity_turns_per_sec
            self._launch_pending_after_elbow_position = False
            self._pending_launch_requested_time = 0.0
            self._start_launch_sequence(pending_velocity)
            return

        if self.elbow_moving_status == 0:
            self._launch_pending_after_elbow_position = False
            self._pending_launch_requested_time = 0.0
            self._publish_onset_status(homed=True, busy=False)
            self.get_logger().warn('Clearing pending launch: elbow reports unhomed status (elbow_moving_status==0)')
            return

        if self.elbow_moving_status == 2:
            self._launch_pending_after_elbow_position = False
            self._pending_launch_requested_time = 0.0
            self._publish_onset_status(homed=True, busy=False)
            self.get_logger().warn('Clearing pending launch: elbow homing error (elbow_moving_status==2)')
            return

        if self.elbow_moving_status == 6:
            self._launch_pending_after_elbow_position = False
            self._pending_launch_requested_time = 0.0
            self._publish_onset_status(homed=True, busy=False)
            self.get_logger().warn('Clearing pending launch: elbow move error (elbow_moving_status==6)')
            return


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

