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
    sw2: int
    sw3: int
    elbow_moving_status: int
    elbow_power_status: int


class STM32Bridge(Node):
    def __init__(self) -> None:
        super().__init__("stm32_bridge")

        self.stm32_state_pub = self.create_publisher(
            STM32State,
            "stm32_states",
            10,
        )

        self.stm32_control_sub = self.create_subscription(
            STM32Message,
            "stm32_control",
            self._on_command,
            10,
        )

        self.message_to_stm32 = ""
        self.ser = None
        self.home_elbow_status = False
        self._homing_in_progress = False
        self._home_wait_seconds = 3.0
        self._last_power_on_status: Optional[bool] = None
        self._last_serial_error_log_time = 0.0
        self.usb_port = '/dev/ttyACM0'

        self._poll_timer = self.create_timer(0.1, self._poll_stm32)

    def _poll_stm32(self) -> None:
        raw_status = self._read_serial_latest_line()
        parsed = self._parse_status_string(raw_status) if raw_status else None

        if parsed is None:
            return

        msg = STM32State()
        msg.sw2 = parsed.sw2
        msg.sw3 = parsed.sw3
        msg.elbow_moving_status = parsed.elbow_moving_status
        msg.elbow_power_status = parsed.elbow_power_status
        self.stm32_state_pub.publish(msg)

    def _read_serial_latest_line(self) -> Optional[str]:
        latest_line: Optional[str] = None
        try:
            if self.ser is None:
                self.ser = serial.Serial(self.usb_port, 115200, timeout=0.0, write_timeout=0.02)
                self.ser.reset_input_buffer()
            if self.ser.is_open:
                # Drain available lines and keep only the freshest status update.
                reads = 0
                while self.ser.in_waiting > 0 and reads < 50:
                    raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if raw:
                        latest_line = raw
                    reads += 1
        except Exception as e:
            self._log_serial_error_throttled(f"Serial read error: {e}")
            self.close_serial()
            self.ser = None

        return latest_line

    def _log_serial_error_throttled(self, msg: str, period_s: float = 1.0) -> None:
        now = time.monotonic()
        if (now - self._last_serial_error_log_time) >= period_s:
            self.get_logger().error(msg)
            self._last_serial_error_log_time = now

    def close_serial(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _parse_status_string(self, raw: str) -> Optional[ParsedStatus]:
        raw = raw.strip()
        match = re.search(r"<(.*?)>", raw)
        if match:
            try:
                parts = [part.strip() for part in match.group(1).split(',')]
                if len(parts) != 4:
                    return None

                switch2 = int(parts[0])
                switch3 = int(parts[1])
                moving_status = int(parts[2])
                power_status = int(parts[3])

                return ParsedStatus(
                    sw2=switch2,
                    sw3=switch3,
                    elbow_moving_status=moving_status,
                    elbow_power_status=power_status,
                )
            except (IndexError, ValueError) as e:
                self.get_logger().error(f"Error parsing status string: {e}")
                return None

        return None

    def _on_command(self, msg: STM32Message) -> None:
        power_on = msg.power_on_status == 1
        home_requested = msg.home_elbow_request == 1

        if self._last_power_on_status is None or power_on != self._last_power_on_status:
            self._send_to_stm32(f"<P,{1 if power_on else 0}>")
            self._last_power_on_status = power_on

        if not power_on:
            self.home_elbow_status = False
            self._homing_in_progress = False
            self.get_logger().info("Power is OFF; ignoring motor command")
            return

        if home_requested and not self.home_elbow_status and not self._homing_in_progress:
            if self._send_to_stm32("<H>"):
                self._homing_in_progress = True
                self.get_logger().info("Homing command sent; waiting for completion...")
                threading.Thread(target=self._complete_home_after_delay, daemon=True).start()
            return

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
                self.ser = serial.Serial(self.usb_port, 115200, timeout=0.0, write_timeout=0.02)
            if self.ser.is_open:
                self.ser.write(payload.encode("utf-8"))
                return True
        except Exception as e:
            self._log_serial_error_throttled(f"Serial write error: {e}")
            self.close_serial()
            self.ser = None
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