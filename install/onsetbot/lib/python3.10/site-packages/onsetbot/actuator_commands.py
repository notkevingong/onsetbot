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
        self._odrive_axis_names = [axis_name for axis_name, _ in self._odrive_axes]
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
        self._odrive_velocity_scale_by_axis = {
            'axis0': 1.0,
            'axis1': 1.0,
        }
        self._axis1_velocity_scale_up = 0.95
        self._axis1_velocity_scale_down = 0.95
        self.get_logger().info(
            f'ODrive direction signs: {self._odrive_direction_sign_by_axis}'
        )

        #Define state variables
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.current_position_by_axis = {axis_name: 0.0 for axis_name in self._odrive_axis_names}
        self.current_velocity_by_axis = {axis_name: 0.0 for axis_name in self._odrive_axis_names}
        self.switch2_state = False
        self.switch3_state = False
        self.elbow_moving_status = 0
        self.elbow_powered_status = 0
        self.led_status = 0
        self._stm32_state_seen = False
        
        # Define homing status variables
        self.is_homed = False
        self.home_active = False
        self.homing_state = 'idle'
        self.home_min_position = {axis_name: None for axis_name in self._odrive_axis_names}
        self.home_max_position = {axis_name: None for axis_name in self._odrive_axis_names}
        self.offset_min_position = {axis_name: None for axis_name in self._odrive_axis_names}
        self.offset_max_position = {axis_name: None for axis_name in self._odrive_axis_names}
        self.offset_min_fraction = 0.1
        self.offset_max_fraction = 0.9
        self._require_stm32_home_confirmation = True
        self._stm32_home_complete = (not self._require_stm32_home_confirmation)
        self._bldc_home_complete = False

        # Home constants
        self._home_velocity = 1.0
        self._home_clear_velocity = 1.0
        self._home_clear_duration_sec = 0.4
        self._home_position_tolerance = 0.05 # in turns
        self._home_clear_start_time = 0.0

        # Launch tuning constants
        self._launch_velocity_limit_turns_per_sec = 40.0
        self._launch_seek_to_start_velocity_turns_per_sec = 3.0
        self._launch_decel_start_fraction = 0.6
        self._launch_brake_velocity_threshold_turns_per_sec = 0.2
        self._launch_brake_timeout_sec = 2.0
        self._launch_post_sw3_hold_sec = 5.0
        self._require_stm32_launch_confirmation = True
        self._launch_led_restore_delay_sec = 0.5

        # STM32 LED command values (mapped in stm32_bridge)
        self._stm32_led_state_off = 0
        self._stm32_led_state_launch = 1
        self._stm32_led_state_idle_color = 2
        self._stm32_loader_request_pulse_value = 1

        # Launch state
        self.launch_active = False
        self.launch_state = 'idle'
        self.launch_target_velocity = 0.0
        self.launch_start_position = 0.0
        self.launch_end_position = 0.0
        self.launch_decel_position = 0.0
        self.launch_requested_theta_rad = 0.0
        self.launch_direction = 1.0
        self._launch_brake_start_time = 0.0
        self._launch_post_sw3_hold_start_time = 0.0
        self._launch_elbow_seen_moving_status = False
        self._launch_led_launch_mode_sent = False
        self._launch_led_seen_idle_status = False
        self._launch_led_restore_pending = False
        self._launch_led_restore_start_time = 0.0
        self._last_stm32_angle_command = 0.0
        self._launch_command_velocity = 0.0
        self._launch_velocity_log_period_sec = 0.1
        self._last_launch_velocity_log_time = 0.0

        # Timers
        self.homing_timer = self.create_timer(0.05, self._homing_step)
        self.launch_timer = self.create_timer(0.02, self._launch_step)

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
        self.home_motors(force_restart=True)

    def convert_callback(self, msg: LaunchCommand):
        if self.home_active:
            self.get_logger().info('Ignoring launch command: homing in progress')
            return

        if self.launch_active:
            self.get_logger().info('Ignoring launch command: launch already active')
            return

        if not self.is_homed:
            self.get_logger().info('Ignoring launch command: onsetbot is not homed')
            return

        feedback_axis = self._odrive_feedback_axis_name
        feedback_offset_min = self.offset_min_position.get(feedback_axis)
        feedback_offset_max = self.offset_max_position.get(feedback_axis)
        if feedback_offset_min is None or feedback_offset_max is None:
            self.get_logger().warn('Ignoring launch command: offset limits are not available')
            return

        span = feedback_offset_max - feedback_offset_min
        if abs(span) <= self._home_position_tolerance:
            self.get_logger().warn('Ignoring launch command: usable offset span is too small')
            return

        self.velocity = float(msg.velocity) * self._velocity_conversion_constant
        self.launch_target_velocity = min(abs(self.velocity), self._launch_velocity_limit_turns_per_sec)
        if self.launch_target_velocity <= 1e-6:
            self.get_logger().warn('Ignoring launch command: requested launch velocity is too small')
            return

        self.launch_requested_theta_rad = float(msg.angle_launch)
        self.launch_direction = 1.0 if span >= 0.0 else -1.0
        self.launch_start_position = float(feedback_offset_min)
        self.launch_end_position = float(feedback_offset_max)
        self.launch_decel_position = self.launch_start_position + (span * self._launch_decel_start_fraction)
        self.launch_state = 'wait_elbow_position' if self._require_stm32_launch_confirmation else 'launch_to_decel'
        self._launch_elbow_seen_moving_status = False
        self._launch_led_launch_mode_sent = False
        self._launch_led_seen_idle_status = False
        self._launch_led_restore_pending = False
        self.launch_active = True

        self._publish_onset_status(homed=True, busy=True)
        self._publish_stm32_command(
            power_on_status=1,
            home_elbow_request=0,
            angle_launch=self.launch_requested_theta_rad,
            onset_state=self._stm32_led_state_idle_color,
        )

        self.get_logger().info(
            f'Launch requested: theta_rad={self.launch_requested_theta_rad:.4f}, '
            f'cmd_vel={self.launch_target_velocity:.3f} turns/s (limited), '
            f'offset_range=({self.launch_start_position:.4f} -> {self.launch_end_position:.4f}), '
            f'decel_at={self.launch_decel_position:.4f}. Waiting for elbow_moving_status sequence 4 -> 5, then led_status sequence 2 -> 3'
        )

    def _abort_launch_sequence(self, reason: str):
        self._publish_odrive_velocity(0.0)
        self._log_launch_velocity(force=True)
        self.launch_active = False
        self.launch_state = 'idle'
        self._launch_elbow_seen_moving_status = False
        self._launch_led_seen_idle_status = False
        self._schedule_launch_led_restore()
        self._publish_onset_status(homed=self.is_homed, busy=False)
        self.get_logger().warn(f'Launch aborted: {reason}')

    def _complete_launch_sequence(self):
        self._publish_odrive_velocity(0.0)
        self._log_launch_velocity(force=True)
        self.launch_active = False
        self.launch_state = 'idle'
        self._launch_elbow_seen_moving_status = False
        self._launch_led_seen_idle_status = False
        self._schedule_launch_led_restore()
        self._publish_onset_status(homed=self.is_homed, busy=False)
        self.get_logger().info(
            'Launch sequence complete: recalibrated SW3/SW2, recomputed offsets, and returned to offset_min_position'
        )

    def _schedule_launch_led_restore(self):
        if not self._launch_led_launch_mode_sent:
            return
        self._launch_led_restore_pending = True
        self._launch_led_restore_start_time = time.monotonic()

    def _restore_idle_led_if_due(self):
        if not self._launch_led_restore_pending:
            return
        if (time.monotonic() - self._launch_led_restore_start_time) < self._launch_led_restore_delay_sec:
            return

        self._publish_stm32_command(
            power_on_status=1,
            home_elbow_request=0,
            angle_launch=self._last_stm32_angle_command,
            onset_state=self._stm32_led_state_idle_color,
        )
        self._launch_led_launch_mode_sent = False
        self._launch_led_restore_pending = False
        self.get_logger().info(
            f'Launch LED hold complete. Restored idle yellow LED after {self._launch_led_restore_delay_sec:.1f}s delay'
        )

    def _turns_per_sec_to_m_per_sec(self, turns_per_sec: float) -> float:
        if abs(self._velocity_conversion_constant) <= 1e-9:
            return 0.0
        return float(turns_per_sec) / self._velocity_conversion_constant

    def _log_launch_velocity(self, force: bool = False):
        if not self.launch_active and not force:
            return

        now = time.monotonic()
        if (not force) and ((now - self._last_launch_velocity_log_time) < self._launch_velocity_log_period_sec):
            return

        self._last_launch_velocity_log_time = now
        target_velocity_mps = self._turns_per_sec_to_m_per_sec(self._launch_command_velocity)
        real_velocity_mps = self._turns_per_sec_to_m_per_sec(self.current_velocity)
        self.get_logger().info(
            f'Launch velocity [{self.launch_state}]: '
            f'target={target_velocity_mps:.3f} m/s, '
            f'real={real_velocity_mps:.3f} m/s'
        )

    def _launch_step(self):
        self._restore_idle_led_if_due()

        if not self.launch_active:
            return

        if self.home_active:
            self._abort_launch_sequence('homing became active during launch')
            return

        if self.launch_state == 'wait_elbow_position':
            if not self._require_stm32_launch_confirmation:
                self.launch_state = 'launch_to_decel'
                return

            if not self._launch_elbow_seen_moving_status:
                if self.elbow_moving_status == 4:
                    self._launch_elbow_seen_moving_status = True
                    self.get_logger().info(
                        'Elbow move started (elbow_moving_status==4). Waiting for elbow_moving_status==5'
                    )
                return

            if self.elbow_moving_status == 5:
                if not self._launch_led_launch_mode_sent:
                    self._publish_stm32_command(
                        power_on_status=1,
                        home_elbow_request=0,
                        angle_launch=self.launch_requested_theta_rad,
                        onset_state=self._stm32_led_state_launch,
                    )
                    self._launch_led_launch_mode_sent = True
                    self.get_logger().info('Elbow reached launch angle after movement. Sent launch LED command and waiting for led_status==3')

                self._launch_led_seen_idle_status = False
                self.launch_state = 'wait_launch_led_ready'
                self.get_logger().info(
                    'Elbow move confirmed (4 -> 5). Waiting for STM32 led_status sequence 2 -> 3 before starting ODrive launch motion'
                )
            return

        if self.launch_state == 'wait_launch_led_ready':
            self._publish_odrive_velocity(0.0)
            if self.elbow_moving_status != 5:
                self.launch_state = 'wait_elbow_position'
                self._launch_elbow_seen_moving_status = False
                self._launch_led_seen_idle_status = False
                if self._launch_led_launch_mode_sent:
                    self._publish_stm32_command(
                        power_on_status=1,
                        home_elbow_request=0,
                        angle_launch=self._last_stm32_angle_command,
                        onset_state=self._stm32_led_state_idle_color,
                    )
                    self._launch_led_launch_mode_sent = False
                self.get_logger().warn(
                    f'Elbow status changed to {self.elbow_moving_status} while waiting for LED ready; waiting again for elbow_moving_status==5'
                )
                return

            if (not self._launch_led_seen_idle_status) and self.led_status == 2:
                self._launch_led_seen_idle_status = True
                self.get_logger().info('Observed STM32 led_status==2. Now waiting for led_status==3 to launch')
                return

            if not self._launch_led_seen_idle_status:
                return

            if self.led_status == 3:
                self.launch_state = 'launch_to_decel'
                self.get_logger().info('Observed STM32 led_status sequence 2 -> 3. Starting ODrive launch motion')
            return

        if self.launch_state == 'launch_to_decel':
            cmd_vel = self.launch_direction * self.launch_target_velocity
            self._publish_odrive_velocity(cmd_vel)

            progress = (self.current_position - self.launch_start_position) * self.launch_direction
            decel_progress = (self.launch_decel_position - self.launch_start_position) * self.launch_direction
            if progress >= decel_progress:
                self._publish_odrive_velocity(0.0)
                self._launch_brake_start_time = time.monotonic()
                self.launch_state = 'brake_to_stop'
                self.get_logger().info(
                    f'Deceleration point reached at {self.current_position:.4f}; commanding velocity=0 for brake stop'
                )
            return

        if self.launch_state == 'brake_to_stop':
            self._publish_odrive_velocity(0.0)
            elapsed = time.monotonic() - self._launch_brake_start_time
            stopped = abs(self.current_velocity) <= self._launch_brake_velocity_threshold_turns_per_sec
            if stopped or elapsed >= self._launch_brake_timeout_sec:
                self.launch_state = 'post_seek_sw3'
                self.get_logger().info(
                    f'Brake phase complete (stopped={int(stopped)}, elapsed={elapsed:.2f}s). Seeking SW3 for post-launch re-reference'
                )
            return

        if self.launch_state == 'post_seek_sw3':
            self._publish_odrive_velocity(abs(self._home_velocity))
            if self.switch3_state:
                self._publish_odrive_velocity(0.0)
                for axis_name in self._odrive_axis_names:
                    self.home_max_position[axis_name] = self.current_position_by_axis.get(axis_name, 0.0)
                self._publish_stm32_command(
                    power_on_status=1,
                    home_elbow_request=0,
                    angle_launch=0.0,
                    onset_state=self._stm32_led_state_launch,
                )
                # Send a momentary loader pulse when SW3 is reached.
                self._publish_stm32_command(
                    power_on_status=1,
                    home_elbow_request=0,
                    angle_launch=0.0,
                    onset_state=self._stm32_led_state_launch,
                    loader_request=self._stm32_loader_request_pulse_value,
                )
                self._publish_stm32_command(
                    power_on_status=1,
                    home_elbow_request=0,
                    angle_launch=0.0,
                    onset_state=self._stm32_led_state_launch,
                    loader_request=0,
                )
                self._launch_post_sw3_hold_start_time = time.monotonic()
                self.launch_state = 'post_hold_sw3'
                self.get_logger().info(
                    'SW3 reached. Updated home_max_position, commanded STM32 elbow to 0 rad, sent momentary loader request, and starting 5s hold'
                )
            return

        if self.launch_state == 'post_hold_sw3':
            self._publish_odrive_velocity(0.0)
            if (time.monotonic() - self._launch_post_sw3_hold_start_time) >= self._launch_post_sw3_hold_sec:
                self.launch_state = 'post_seek_sw2'
                self.get_logger().info('SW3 hold complete. Seeking SW2 to refresh home_min_position')
            return

        if self.launch_state == 'post_seek_sw2':
            self._publish_odrive_velocity(-abs(self._home_velocity))
            if self.switch2_state:
                self._publish_odrive_velocity(0.0)
                for axis_name in self._odrive_axis_names:
                    self.home_min_position[axis_name] = self.current_position_by_axis.get(axis_name, 0.0)
                self._compute_homing_offsets()
                self.launch_state = 'post_return_offset_min'
                self.get_logger().info(
                    'SW2 reached. Updated home_min_position and recomputed offsets. Returning to offset_min_position'
                )
            return

        if self.launch_state == 'post_return_offset_min':
            feedback_axis = self._odrive_feedback_axis_name
            target = self.offset_min_position.get(feedback_axis)
            if target is None:
                self._abort_launch_sequence(
                    f'post-launch return target offset_min_position is undefined for {feedback_axis}'
                )
                return

            position_error = target - self.current_position
            if abs(position_error) <= self._home_position_tolerance:
                self._complete_launch_sequence()
                return

            cmd_vel = abs(self._home_velocity) if position_error > 0.0 else -abs(self._home_velocity)
            self._publish_odrive_velocity(cmd_vel)
            return

    def home_motors(self, force_restart: bool = False):
        if self.home_active and not force_restart:
            self.get_logger().info('Homing already in progress')
            return

        if self.home_active and force_restart:
            self._publish_odrive_velocity(0.0)
            self.get_logger().info('Restarting homing sequence from new /home_info request')

        self.is_homed = False
        self.home_active = True
        self.homing_state = 'clear_sw2'
        self.home_min_position = {axis_name: None for axis_name in self._odrive_axis_names}
        self.home_max_position = {axis_name: None for axis_name in self._odrive_axis_names}
        self.offset_min_position = {axis_name: None for axis_name in self._odrive_axis_names}
        self.offset_max_position = {axis_name: None for axis_name in self._odrive_axis_names}
        self._bldc_home_complete = False
        self._stm32_home_complete = (not self._require_stm32_home_confirmation)
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

        if self._require_stm32_home_confirmation and (not self._stm32_home_complete) and self.elbow_moving_status == 3:
            self._stm32_home_complete = True
            self.get_logger().info('STM32 elbow homing complete (elbow_moving_status==3)')

        if self.homing_state == 'clear_sw2':
            self._publish_odrive_velocity(abs(self._home_clear_velocity))
            if (time.monotonic() - self._home_clear_start_time) >= self._home_clear_duration_sec:
                self._publish_odrive_velocity(0.0)
                self.homing_state = 'seek_sw2'
                self.get_logger().info(
                    f'Transition: seek_sw2 at {-abs(self._home_velocity):.2f} turns/s'
                )
            return

        if self.homing_state == 'seek_sw2':
            self._publish_odrive_velocity(-abs(self._home_velocity))
            if self.switch2_state:
                self._publish_odrive_velocity(0.0)
                for axis_name in self._odrive_axis_names:
                    self.home_min_position[axis_name] = self.current_position_by_axis.get(axis_name, 0.0)
                self.homing_state = 'seek_sw3'
                fb_min = self.home_min_position.get(self._odrive_feedback_axis_name)
                self.get_logger().info(
                    f'SW2 hit. Saved min position ({self._odrive_feedback_axis_name}): {fb_min:.4f}. '
                    f'Transition: seek_sw3 at {abs(self._home_velocity):.2f} turns/s'
                )
            return

        if self.homing_state == 'seek_sw3':
            self._publish_odrive_velocity(abs(self._home_velocity))
            if self.switch3_state:
                self._publish_odrive_velocity(0.0)
                for axis_name in self._odrive_axis_names:
                    self.home_max_position[axis_name] = self.current_position_by_axis.get(axis_name, 0.0)
                self._compute_homing_offsets()
                self.homing_state = 'return_offset_min'
                fb_offset_min = self.offset_min_position.get(self._odrive_feedback_axis_name)
                self.get_logger().info(
                    f'SW3 hit. Reversing direction to return to offset_min_position '
                    f'({self._odrive_feedback_axis_name})={fb_offset_min:.4f}'
                )
            return

        if self.homing_state == 'return_offset_min':
            feedback_offset_min = self.offset_min_position.get(self._odrive_feedback_axis_name)
            if feedback_offset_min is None:
                self._publish_odrive_velocity(0.0)
                self.home_active = False
                self.homing_state = 'idle'
                self._publish_onset_status(homed=False, busy=False)
                self.get_logger().error(
                    f'Homing aborted: offset_min_position is undefined for {self._odrive_feedback_axis_name}'
                )
                return

            if self.switch2_state:
                self._publish_odrive_velocity(0.0)
                self.home_active = False
                self.homing_state = 'idle'
                self.is_homed = False
                self._publish_onset_status(homed=False, busy=False)
                self.get_logger().error(
                    'Homing safety stop: SW2 triggered during return_offset_min before reaching target. '
                    f'target={feedback_offset_min:.4f}, '
                    f'feedback(axis={self._odrive_feedback_axis_name})={self.current_position:.4f}, '
                    f'axis0={self.current_position_by_axis.get("axis0", 0.0):.4f}, '
                    f'axis1={self.current_position_by_axis.get("axis1", 0.0):.4f}'
                )
                return

            position_error = feedback_offset_min - self.current_position
            if abs(position_error) <= self._home_position_tolerance:
                self._publish_odrive_velocity(0.0)
                self._bldc_home_complete = True
                if self._stm32_home_complete:
                    self._complete_homing()
                else:
                    self.homing_state = 'wait_stm32_home'
                    self.get_logger().info(
                        'BLDC homing complete at offset_min_position; waiting for STM32 elbow_moving_status==3'
                    )
                return

            cmd_vel = abs(self._home_velocity) if position_error > 0.0 else -abs(self._home_velocity)
            self._publish_odrive_velocity(cmd_vel)
            return

        if self.homing_state == 'wait_stm32_home':
            self._publish_odrive_velocity(0.0)
            if self._stm32_home_complete:
                self._complete_homing()
            return

    def _compute_homing_offsets(self):
        if any(self.home_min_position.get(axis_name) is None for axis_name in self._odrive_axis_names):
            self.get_logger().error('Cannot compute offsets: home min position is not available for all axes')
            return
        if any(self.home_max_position.get(axis_name) is None for axis_name in self._odrive_axis_names):
            self.get_logger().error('Cannot compute offsets: home max position is not available for all axes')
            return

        min_frac = min(max(self.offset_min_fraction, 0.0), 1.0)
        max_frac = min(max(self.offset_max_fraction, 0.0), 1.0)
        if max_frac <= min_frac:
            max_frac = min(1.0, min_frac + 0.1)

        for axis_name in self._odrive_axis_names:
            axis_min = float(self.home_min_position[axis_name])
            axis_max = float(self.home_max_position[axis_name])
            span = axis_max - axis_min
            span_abs = abs(span)

            if span_abs <= self._home_position_tolerance:
                self.get_logger().error(
                    f'Homing span too small for {axis_name} ({span_abs:.4f} turns); '
                    'keeping raw min/max as offsets'
                )
                self.offset_min_position[axis_name] = axis_min
                self.offset_max_position[axis_name] = axis_max
                continue

            direction = 1.0 if span >= 0.0 else -1.0
            self.offset_min_position[axis_name] = axis_min + direction * span_abs * min_frac
            self.offset_max_position[axis_name] = axis_min + direction * span_abs * max_frac

        self.get_logger().info(
            f'Offset limits computed: '
            f'axis0[min={self.offset_min_position.get("axis0", 0.0):.4f}, max={self.offset_max_position.get("axis0", 0.0):.4f}], '
            f'axis1[min={self.offset_min_position.get("axis1", 0.0):.4f}, max={self.offset_max_position.get("axis1", 0.0):.4f}] '
            f'(fractions min={min_frac:.2f}, max={max_frac:.2f})'
        )

    def _complete_homing(self):
        self._publish_odrive_velocity(0.0)
        self.home_active = False
        self.homing_state = 'idle'
        self.is_homed = True
        self._publish_onset_status(homed=True, busy=False)
        self.get_logger().info(
            f'Homing complete. '
            f'home_min(axis0={self.home_min_position.get("axis0", 0.0):.4f}, axis1={self.home_min_position.get("axis1", 0.0):.4f}), '
            f'home_max(axis0={self.home_max_position.get("axis0", 0.0):.4f}, axis1={self.home_max_position.get("axis1", 0.0):.4f}), '
            f'offset_min(axis0={self.offset_min_position.get("axis0", 0.0):.4f}, axis1={self.offset_min_position.get("axis1", 0.0):.4f}), '
            f'offset_max(axis0={self.offset_max_position.get("axis0", 0.0):.4f}, axis1={self.offset_max_position.get("axis1", 0.0):.4f}), '
            f'stm32_home_complete={int(self._stm32_home_complete)}'
        )

    def _publish_odrive_velocity(self, velocity: float):
        feedback_axis = self._odrive_feedback_axis_name
        feedback_scale = self._odrive_velocity_scale_by_axis.get(feedback_axis, 1.0)
        if feedback_axis == 'axis1':
            feedback_scale = self._axis1_velocity_scale_up if velocity >= 0.0 else self._axis1_velocity_scale_down
        self._launch_command_velocity = float(feedback_scale * velocity)
        for axis_name, axis_id in self._odrive_axes:
            direction = self._odrive_direction_sign_by_axis.get(axis_name, 1.0)
            scale = self._odrive_velocity_scale_by_axis.get(axis_name, 1.0)
            if axis_name == 'axis1':
                scale = self._axis1_velocity_scale_up if velocity >= 0.0 else self._axis1_velocity_scale_down
            msg = ControlMessage()
            msg.control_mode = 2
            msg.input_mode = 2
            msg.input_pos = 0.0
            msg.input_torque = 0.0
            msg.input_vel = float(direction * scale * velocity)
            self.odrive_axis_publishers[axis_id].publish(msg)

    def _publish_stm32_command(
        self,
        power_on_status: int,
        home_elbow_request: int,
        angle_launch: float,
        onset_state: int = None,
        loader_request: int = 0,
    ):
        msg = STM32Message()
        msg.angle_launch = float(angle_launch)
        msg.power_on_status = int(power_on_status)
        msg.home_elbow_request = int(home_elbow_request)
        msg.stm32_state_request = int(loader_request)
        if hasattr(msg, 'loader_reqest'):
            msg.loader_reqest = int(loader_request)
        if hasattr(msg, 'loader_request'):
            msg.loader_request = int(loader_request)
        if onset_state is None:
            onset_state = self._stm32_led_state_idle_color if int(power_on_status) == 1 else self._stm32_led_state_off
        msg.onset_state = int(onset_state)
        self._last_stm32_angle_command = float(angle_launch)
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

        if self.launch_active:
            self._log_launch_velocity()

    def update_stm32_state(self, msg: STM32State):
        self._stm32_state_seen = True
        self.switch2_state = bool(msg.sw2 == 1)
        self.switch3_state = bool(msg.sw3 == 1)
        self.elbow_moving_status = int(msg.elbow_moving_status)
        self.elbow_powered_status = int(msg.elbow_power_status)
        self.led_status = int(msg.led_status)


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

