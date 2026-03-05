#!/usr/bin/env python3
"""
ROS2 Node to communicate/command STM32 peripherals.
"""

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from onset_interfaces.msg import STM32Message, STM32State

import serial
import threading
import re
import math
import time


@dataclass
class ParsedStatus:
    sw1: int
    sw2: int
    sw3: int
    error_code: int


class STM32Bridge(Node):
	def __init__(self) -> None:
		super().__init__("stm32_bridge")

        # Create publisher for STM32 state messages to /stm32_states
		self.stm32_state_pub = self.create_publisher(
			STM32State,
			"stm32_states", 
			10
			)
		
		self.stm32_control_sub = self.create_subscription(
			STM32Message, 
			"stm32_control", 
			self._on_command, 
			10
		)

		# Initialize variables
		self.message_to_stm32 = ""  # The message to send to STM32, set by the subscriber callback
		self.ser = None
		self.home_elbow_status = False
		self._homing_in_progress = False
		self._home_wait_seconds = 3.0
		self._last_power_on_status: Optional[bool] = None

		# Start serial reading thread
		thread = threading.Thread(target=self._poll_stm32, daemon=True)
		thread.start()

		self._test_toggle = False
		self._poll_timer = self.create_timer(0.1, self._poll_stm32)


	def _poll_stm32(self) -> None:
		raw_status = self._read_serial()
		parsed = self._parse_status_string(raw_status) if raw_status else None

		if parsed is None:
			return

		msg = STM32State()
		msg.sw1 = parsed.sw1
		msg.sw2 = parsed.sw2
		msg.sw3 = parsed.sw3
		msg.error_code = parsed.error_code
		self.stm32_state_pub.publish(msg)

	def _read_serial (self) -> Optional[str]:
		# TODO: Implement USB read once STM32 protocol is defined.
		raw = ""
		try:
			if self.ser is None:
				self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
			if self.ser.is_open:
				raw = self.ser.readline().decode('utf-8').strip()
		except Exception as e:
			self.get_logger().error(f"Serial read error: {e}")

		return raw
	
	def close_serial(self) -> None:
		if self.ser and self.ser.is_open:
			self.ser.close()

	def _parse_status_string(self, raw: str) -> Optional[ParsedStatus]:
		# TODO: Implement parsing once STM32 protocol is defined.
		raw = raw.strip()
		match = re.search(r"\{(.*?)\}", raw)
		if match:
			try:
				parts = match.group(1).split(',')
				sw1 = int(parts[0])
				sw2 = int(parts[1])
				sw3 = int(parts[2])
				error_code = int(parts[3])
				return ParsedStatus(sw1=int(sw1), sw2=int(sw2), sw3=int(sw3), error_code=error_code)
			except (IndexError, ValueError) as e:
				self.get_logger().error(f"Error parsing status string: {e}")
				return None

		return None

	def _on_command(self, msg: STM32Message) -> None:
		power_on = msg.power_on_status == 1
		home_requested = msg.home_elbow_request == 1

		# Only send power status if it has changed since last command
		if self._last_power_on_status is None or power_on != self._last_power_on_status:
			self._send_to_stm32(f"<P,{1 if power_on else 0}>")
			self._last_power_on_status = power_on

		# If power is off, ignore all other commands and reset homing status
		if not power_on:
			self.home_elbow_status = False
			self._homing_in_progress = False
			self.get_logger().info("Power is OFF; ignoring motor command")
			return

		# If home is requested and not already homed or in progress, send home command
		if home_requested and not self.home_elbow_status and not self._homing_in_progress:
			if self._send_to_stm32("<H>"):
				self._homing_in_progress = True
				threading.Thread(target=self._complete_home_after_delay, daemon=True).start()
			return

		# If home is not complete, ignore angle commands
		if self.home_elbow_status:
			angle_deg = msg.angle_launch
			angle_rad = math.radians(angle_deg)
			self.message_to_stm32 = str(angle_rad)
			self.get_logger().info(f"Sending elbow angle: {angle_deg} deg ({angle_rad} rad)")
			self._send_to_stm32(f"<M,{angle_rad}>")
		else:
			self.get_logger().info("Home not complete; ignoring elbow angle command")

	
	def _send_to_stm32(self, payload: str) -> bool:
		self.message_to_stm32 = payload
		try:
			if self.ser is None:
				self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
			if self.ser.is_open:
				self.ser.write(payload.encode("utf-8"))
				return True
		except Exception as e:
			self.get_logger().error(f"Serial write error: {e}")
		return False

	def _complete_home_after_delay(self) -> None:
		time.sleep(self._home_wait_seconds)
		self.home_elbow_status = True
		self._homing_in_progress = False
		self.get_logger().info("Elbow homing complete")


def main() -> None:
	rclpy.init()
	node = STM32Bridge()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.close_serial()
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()