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
        super().__init__('actuator_command')

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
        self.velocity = 0.0
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

        # Define variables for storing position and velocity feedback from ODrive
        self.current_position = 0.0
        self.current_velocity = 0.0  # Primary feedback (axis1 for launch control)
        self.current_velocity_by_axis = {'axis0': 0.0, 'axis1': 0.0}  # All axis velocities
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
        self._home_operating_min_buffer_ratio = 0.2
        self._home_operating_max_buffer_ratio = 0.10
        self.home_target_position = None
        self._last_homing_debug_log_time = 0.0

        # Homing tuning constants
        self._home_seek_velocity = 2.0
        self._home_return_velocity = 1.0
        self._home_safety_offset_turns = 0.05
        self._home_position_tolerance_turns = 0.15
        self._home_clear_sw2_velocity = 2.0   # forward speed while clearing SW2 (turns/s)
        self._home_clear_sw2_extra_turns = 0.5  # extra turns to travel after SW2 releases before seeking

        # Launch tuning constants (easy-to-adjust profile knobs)
        self._launch_velocity_limit_turns_per_sec = 70.0
        self._launch_seek_to_start_velocity_turns_per_sec = 3.0
        self._launch_start_offset_from_min_fraction = 0.15
        self._launch_end_offset_from_max_fraction = 0.10
        self._launch_decel_start_fraction = 0.8
        self._launch_position_tolerance_turns = 0.01
        self._launch_return_position_tolerance_turns = 0.05
        self._launch_return_slow_zone_turns = 1.5      # distance from target at which return speed drops to crawl
        self._launch_return_crawl_velocity_turns_per_sec = 0.5  # slow approach speed near start position
        self._launch_decel_complete_velocity_threshold_turns_per_sec = 0.20
        self._launch_decel_to_return_min_delay_s = 0.30
        self._launch_decel_to_return_timeout_s = 2.0
        self._launch_speed_tolerance_turns_per_sec = 0.2
        self._require_elbow_ready_before_launch = False
        self._launch_drive_mode = 'velocity'  # 'velocity' or 'torque'
        self._launch_velocity_input_mode = 2
        self._launch_torque_input_mode = 1
        self._launch_torque_per_turns_per_sec = 0.020
        self._launch_torque_limit_nm = 2.5
        self._launch_brake_torque_ratio = 1.0
        self._launch_brake_torque_limit_nm = 2.5

        # Launch state
        self.launch_active = False
        self.launch_state = 'idle'
        self.launch_target_velocity = 0.0
        self.launch_target_torque = 0.0
        self.launch_brake_torque = 0.0
        self.launch_start_position = 0.0
        self.launch_decel_start_position = 0.0
        self.launch_end_position = 0.0
        self.launch_direction = 1.0
        self._launch_exit_velocity_logged = False
        self._launch_stop_command_sent = False
        self._launch_stop_command_time = 0.0
        self._launch_pending_after_elbow_position = False
        self._pending_launch_velocity_turns_per_sec = 0.0
        self._pending_launch_angle = 0.0
        self._pending_launch_requested_time = 0.0
        self._pending_launch_timeout_s = 10.0
        self._launch_last_vel_log_time = 0.0

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
        msg.control_mode = 1  # Torque control mode
        msg.input_mode = 1  # Torque Mode
        msg.input_pos = 0.0
        msg.input_torque = 0.5 * ((self.velocity * self._velocity_conversion_constant) ** 2 / (2 * 0.6)) * 0.0375 # m*(v^2/2s)*r with m=0.5kg, v=velocity m/s, s = 0.6 OR op_range * (25/46) * 2*pi*0.0375, r = 0.0375
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
        if not self.is_homed:
            self.get_logger().info('Ignoring launch command: system not homed yet')
            return

        if self.homing_active:
            self.get_logger().info('Ignoring launch command: homing in progress')
            return

        if self.launch_active or (self._require_elbow_ready_before_launch and self._launch_pending_after_elbow_position):
            self.get_logger().info('Ignoring launch command: launch already active or pending elbow positioning')
            return

        self.velocity = msg.velocity * self._velocity_conversion_constant
        self.angle_launch = msg.angle_launch * self._angle_conversion_constant

        if not self._require_elbow_ready_before_launch:
            self._launch_pending_after_elbow_position = False
            self._pending_launch_requested_time = 0.0
            self._publish_onset_status(homed=True, busy=True)
            self.get_logger().info('Launch starting immediately: elbow readiness check disabled')
            self._start_launch_sequence(self.velocity)
            return

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

        if self.home_min_position is None or self.home_max_position is None:
            self.get_logger().warn('Ignoring launch command: homing bounds are not available')
            return

        raw_span = self.home_max_position - self.home_min_position
        full_span = abs(raw_span)
        if full_span <= self._launch_position_tolerance_turns:
            self.get_logger().warn('Ignoring launch command: homed travel range is too small')
            return

        if self.launch_active:
            self.get_logger().warn('Ignoring launch command: launch sequence already active')
            return

        self._launch_pending_after_elbow_position = False
        self._pending_launch_requested_time = 0.0

        self.launch_direction = 1.0 if raw_span >= 0.0 else -1.0

        min_offset_fraction = min(max(self._launch_start_offset_from_min_fraction, 0.0), 0.49)
        max_offset_fraction = min(max(self._launch_end_offset_from_max_fraction, 0.0), 0.49)
        decel_start_fraction = min(max(self._launch_decel_start_fraction, 0.0), 1.0)

        self.launch_start_position = self.home_min_position + (self.launch_direction * full_span * min_offset_fraction)
        self.launch_end_position = self.home_max_position - (self.launch_direction * full_span * max_offset_fraction)

        travel_span = self._position_progress(
            self.launch_end_position,
            self.launch_start_position,
            self.launch_end_position,
        )
        if travel_span <= self._launch_position_tolerance_turns:
            self.get_logger().warn('Ignoring launch command: configured travel range is too small')
            return

        self.launch_decel_start_position = self.launch_start_position + (
            self.launch_direction * travel_span * decel_start_fraction
        )

        limited_mag = min(abs(target_velocity_turns_per_sec), self._launch_velocity_limit_turns_per_sec)
        self.launch_target_velocity = self.launch_direction * limited_mag
        limited_torque_mag = min(
            abs(limited_mag) * self._launch_torque_per_turns_per_sec,
            self._launch_torque_limit_nm,
        )
        self.launch_target_torque = self.launch_direction * limited_torque_mag
        brake_torque_mag = min(
            abs(self.launch_target_torque) * self._launch_brake_torque_ratio,
            self._launch_brake_torque_limit_nm,
        )
        self.launch_brake_torque = -self.launch_direction * brake_torque_mag

        self.launch_state = 'seek_start'
        self.launch_active = True
        self._launch_exit_velocity_logged = False
        self._launch_stop_command_sent = False
        self._launch_stop_command_time = 0.0
        self._publish_onset_status(homed=True, busy=True)
        self.get_logger().info(
            f'Launch start: mode={self._launch_drive_mode}, '
            f'target_vel={self.launch_target_velocity:.2f} turns/s, '
            f'target_torque={self.launch_target_torque:.3f} Nm, '
            f'brake_torque={self.launch_brake_torque:.3f} Nm, '
            f'start={self.launch_start_position:.4f}, '
            f'decel_start={self.launch_decel_start_position:.4f}, end={self.launch_end_position:.4f}, '
            f'offset positions(min={self._launch_start_offset_from_min_fraction:.2f}, max={self._launch_end_offset_from_max_fraction:.2f}), '
            f'decel_start_fraction={decel_start_fraction:.2f}'
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
        start_pos = self.launch_start_position
        if start_pos is None:
            return False
        position_tolerance = max(self._launch_return_position_tolerance_turns, self._launch_position_tolerance_turns)
        if self.launch_direction >= 0.0:
            return self.current_position <= (start_pos + position_tolerance)
        return self.current_position >= (start_pos - position_tolerance)

    def _stop_launch_sequence(self, reason: str):
        if not self._launch_stop_command_sent:
            self._publish_launch_stop_command()
        self.launch_active = False
        self.launch_state = 'idle'
        self._launch_exit_velocity_logged = False
        self._launch_stop_command_sent = False
        self._launch_stop_command_time = 0.0
        self._publish_onset_status(homed=True, busy=False)
        self.get_logger().info(f'Launch complete: {reason}')

    def _log_exit_velocity_at_decel_point(self, pos_from_start: float, start_to_decel: float):
        if self._launch_exit_velocity_logged:
            return

        conv = self._velocity_conversion_constant
        if abs(conv) <= 1e-9:
            self.get_logger().warn('Cannot convert measured turns/s to launch velocity units: conversion constant is zero')
            self._launch_exit_velocity_logged = True
            return

        measured_turns_per_sec = self.current_velocity
        target_turns_per_sec = self.launch_target_velocity
        measured_velocity = measured_turns_per_sec / conv
        target_velocity = target_turns_per_sec / conv

        self.get_logger().info(
            'Decel point reached: '
            f'measured={measured_velocity:.3f} (from {measured_turns_per_sec:.3f} turns/s), '
            f'target={target_velocity:.3f} (from {target_turns_per_sec:.3f} turns/s), '
            f'error={measured_velocity - target_velocity:+.3f}, '
            f'progress={pos_from_start:.4f}/{start_to_decel:.4f} turns'
        )
        self._launch_exit_velocity_logged = True

    def _launch_step(self):
        if not self.launch_active:
            return

        # Log velocity every 0.05 s throughout the launch
        now_vel = time.monotonic()
        if (now_vel - self._launch_last_vel_log_time) >= 0.05:
            conv = self._velocity_conversion_constant
            measured_turns = self.current_velocity
            measured_ms = (measured_turns / conv) if abs(conv) > 1e-9 else 0.0
            axis0_vel = self.current_velocity_by_axis.get('axis0', 0.0)
            axis1_vel = self.current_velocity_by_axis.get('axis1', 0.0)
            self.get_logger().info(
                f'Launch velocity: {measured_turns:.3f} turns/s ({measured_ms:.3f} m/s) | '
                f'motors: axis0={axis0_vel:.3f}, axis1={axis1_vel:.3f} turns/s | '
                f'state={self.launch_state} | pos={self.current_position:.4f}'
            )
            self._launch_last_vel_log_time = now_vel

        if self.homing_active:
            self._stop_launch_sequence('aborted due to homing start')
            return

        if self.launch_start_position is None or self.launch_end_position is None:
            self._stop_launch_sequence('aborted: launch bounds unavailable')
            return

        start_to_end = self._position_progress(self.launch_end_position, self.launch_start_position, self.launch_end_position)
        start_to_decel = self._position_progress(self.launch_decel_start_position, self.launch_start_position, self.launch_end_position)
        pos_from_start = self._position_progress(self.current_position, self.launch_start_position, self.launch_end_position)

        if self.launch_state == 'seek_start':
            if self._is_at_min_position():
                self._publish_launch_drive_command()
                self.launch_state = 'hold_exit_velocity'
                self.get_logger().info(f'Launch transition: hold_exit_velocity (maintaining {self._launch_drive_mode} launch command)')
                return

            self._stop_launch_sequence('aborted: not at launch start position for two-command profile')
            return

        if self.launch_state == 'hold_exit_velocity':
            self._publish_launch_drive_command()
            if pos_from_start >= (start_to_decel - self._launch_position_tolerance_turns):
                self._log_exit_velocity_at_decel_point(pos_from_start, start_to_decel)
                self._publish_launch_decel_command()
                self._launch_stop_command_sent = True
                self._launch_stop_command_time = time.monotonic()
                self.launch_state = 'decel_to_end'
                self.get_logger().info(f'Launch transition: decel_to_end (published {self._launch_drive_mode} decel command once)')
                return
            return

        if self.launch_state == 'decel_to_end':
            if self._launch_drive_mode == 'torque':
                self._publish_launch_decel_command()
            stop_elapsed = time.monotonic() - self._launch_stop_command_time if self._launch_stop_command_time > 0.0 else 0.0
            stopped_by_velocity = (
                stop_elapsed >= self._launch_decel_to_return_min_delay_s and
                abs(self.current_velocity) <= self._launch_decel_complete_velocity_threshold_turns_per_sec
            )
            reached_end_position = pos_from_start >= (start_to_end - self._launch_position_tolerance_turns)
            decel_timeout = stop_elapsed >= self._launch_decel_to_return_timeout_s
            near_end_for_velocity_stop = pos_from_start >= (start_to_end - max(self._launch_return_position_tolerance_turns, self._launch_position_tolerance_turns))
            stopped_by_velocity_near_end = stopped_by_velocity and near_end_for_velocity_stop

            if stopped_by_velocity_near_end or reached_end_position or decel_timeout:
                transition_reason = 'velocity_near_end' if stopped_by_velocity_near_end else ('end_position' if reached_end_position else 'timeout')
                # Use launch_direction to calculate proper return velocity direction
                return_cmd_vel = -self.launch_direction * self._launch_seek_to_start_velocity_turns_per_sec
                self._publish_odrive_velocity(return_cmd_vel)
                self._launch_stop_command_sent = False
                self._launch_stop_command_time = 0.0
                self.launch_state = 'return_to_start'
                self.get_logger().info(
                    f'Launch transition: return_to_start reason={transition_reason} (cmd_vel={return_cmd_vel:.2f} turns/s, '
                    f'stop_elapsed={stop_elapsed:.3f}s, vel={self.current_velocity:.3f} turns/s, '
                    f'progress={pos_from_start:.4f}/{start_to_end:.4f})'
                )
            return

        if self.launch_state == 'return_to_start':
            # SW2 hardware guard: stop immediately if switch is physically triggered
            if self.switch2_state:
                self._publish_odrive_velocity(0.0)
                self._stop_launch_sequence('return stopped: SW2 triggered during return')
                return

            # Calculate signed distance to target
            start_error = (self.launch_start_position - self.current_position) * self.launch_direction
            distance_to_target = abs(start_error)
            
            # Check if close enough to target to stop
            position_tolerance = max(self._launch_return_position_tolerance_turns, self._launch_position_tolerance_turns)
            if distance_to_target <= position_tolerance:
                # Within tolerance: stop and complete launch
                self._publish_odrive_velocity(0.0)
                self._stop_launch_sequence(f'return complete: reached offset position (distance={distance_to_target:.4f})')
                return
            
            # Two-speed return: fast until slow-zone, then crawl back to target
            if distance_to_target <= self._launch_return_slow_zone_turns:
                # In slow zone: crawl toward target
                return_cmd_vel = -self.launch_direction * self._launch_return_crawl_velocity_turns_per_sec
            else:
                # Far from target: fast return
                return_cmd_vel = -self.launch_direction * self._launch_seek_to_start_velocity_turns_per_sec
            
            self._publish_odrive_velocity(return_cmd_vel)
            return

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
        self.home_min_position = None
        self.home_max_position = None
        self.home_operating_min_position = None
        self.home_operating_max_position = None
        self.home_target_position = None
        self._home_clear_sw2_release_position = None
        self._last_homing_debug_log_time = 0.0

        self._publish_onset_status(homed=False, busy=True)

        # Pulse elbow-home request at homing start: 1 then 0
        self._publish_stm32_command(power_on_status=1, home_elbow_request=1, angle_launch=0.0)
        self._publish_stm32_command(power_on_status=1, home_elbow_request=0, angle_launch=0.0)

        if self.switch2_state:
            self.homing_state = 'clear_sw2'
            self.get_logger().info(
                f'Started homing: SW2 already pressed at pos={self.current_position:.4f}. '
                f'state=clear_sw2, moving forward at {self._home_clear_sw2_velocity:.2f} turns/s '
                f'until SW2 releases + {self._home_clear_sw2_extra_turns:.2f} extra turns'
            )
        else:
            self.homing_state = 'seek_sw2'
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

        if self.homing_state == 'clear_sw2':
            self._publish_odrive_velocity(abs(self._home_clear_sw2_velocity))
            self._log_homing_debug(
                f'Homing state=clear_sw2 | pos={self.current_position:.4f} | '
                f'sw2={int(self.switch2_state)} sw3={int(self.switch3_state)} | '
                f'cmd_vel={abs(self._home_clear_sw2_velocity):.2f} turns/s'
            )
            if not self.switch2_state and self._home_clear_sw2_release_position is None:
                # Switch just released — record position and keep moving for extra clearance
                self._home_clear_sw2_release_position = self.current_position
                self.get_logger().info(
                    f'SW2 released at pos={self.current_position:.4f}. '
                    f'Continuing forward {self._home_clear_sw2_extra_turns:.2f} turns before seeking SW2'
                )
            if self._home_clear_sw2_release_position is not None:
                extra_traveled = abs(self.current_position - self._home_clear_sw2_release_position)
                if extra_traveled >= self._home_clear_sw2_extra_turns:
                    self._publish_odrive_velocity(0.0)
                    self.homing_state = 'seek_sw2'
                    self.get_logger().info(
                        f'SW2 cleared (extra_traveled={extra_traveled:.3f} turns). '
                        f'Transition: state=seek_sw2, cmd_vel={-abs(self._home_seek_velocity):.2f} turns/s'
                    )
            return

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

                self.home_operating_min_position = self.home_min_position
                self.home_operating_max_position = self.home_max_position

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
                self.get_logger().info(
                    f'Homing complete. SW2={self.home_min_position:.4f}, '
                    f'SW3={self.home_max_position:.4f}, '
                    f'op_min={self.home_operating_min_position:.4f}, '
                    f'op_max={self.home_operating_max_position:.4f}, '
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
        self._publish_odrive_command(
            control_mode=2,
            input_mode=self._launch_velocity_input_mode,
            velocity=velocity,
            torque=0.0,
        )

    def _publish_odrive_torque(self, torque_nm: float):
        self._publish_odrive_command(
            control_mode=1,
            input_mode=self._launch_torque_input_mode,
            velocity=0.0,
            torque=torque_nm,
        )

    def _publish_odrive_command(self, control_mode: int, input_mode: int, velocity: float, torque: float):
        for axis_name, axis_id in self._odrive_axes:
            direction = self._odrive_direction_sign_by_axis.get(axis_name, 1.0)
            msg = ControlMessage()
            msg.control_mode = int(control_mode)
            msg.input_mode = int(input_mode)
            msg.input_pos = 0.0
            msg.input_torque = float(direction * torque)
            msg.input_vel = float(direction * velocity)
            self.odrive_axis_publishers[axis_id].publish(msg)

    def _publish_launch_drive_command(self):
        if self._launch_drive_mode == 'torque':
            self._publish_odrive_torque(self.launch_target_torque)
            return
        self._publish_odrive_velocity(self.launch_target_velocity)

    def _publish_launch_decel_command(self):
        if self._launch_drive_mode == 'torque':
            self._publish_odrive_torque(self.launch_brake_torque)
            return
        self._publish_odrive_velocity(0.0)

    def _publish_launch_stop_command(self):
        if self._launch_drive_mode == 'torque':
            self._publish_odrive_torque(0.0)
            return
        self._publish_odrive_velocity(0.0)

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
        # Capture velocities from all axes in feedback
        for i, axis_name in enumerate(msg.name):
            if axis_name in self._odrive_direction_sign_by_axis:
                direction = self._odrive_direction_sign_by_axis[axis_name]
                if i < len(msg.velocity):
                    self.current_velocity_by_axis[axis_name] = float(direction * msg.velocity[i])
        
        # Update primary feedback from designated axis
        target_name = self._odrive_feedback_axis_name
        if target_name not in msg.name:
            return
        idx = msg.name.index(target_name)
        feedback_sign = self._odrive_direction_sign_by_axis.get(target_name, -1.0)
        self.current_position = float(feedback_sign * msg.position[idx])
        self.current_velocity = float(feedback_sign * msg.velocity[idx])

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

