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
import re
import time


@dataclass
class ParsedStatus:
    sw2: int
    sw3: int
    elbow_moving_status: int
    elbow_power_status: int
    led_status: int
    loader_status: int


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
        self._last_power_on_status: Optional[bool] = None
        self._last_led_payload: Optional[str] = None
        self._startup_idle_led_applied = False
        self._last_loader_request_value = 0
        self._last_angle_command_rad: Optional[float] = None
        self._angle_command_epsilon_rad = 1e-4
        self._idle_led_rgb = (255, 100, 0)
        self._last_serial_error_log_time = 0.0
        self.usb_port = '/dev/ttyACM0'

        self._poll_timer = self.create_timer(0.1, self._poll_stm32)

    def _poll_stm32(self) -> None:
        self._ensure_startup_idle_led()

        raw_status = self._read_serial_latest_line()
        parsed = self._parse_status_string(raw_status) if raw_status else None

        if parsed is None:
            return

        msg = STM32State()
        msg.sw2 = parsed.sw2
        msg.sw3 = parsed.sw3
        msg.elbow_moving_status = parsed.elbow_moving_status
        msg.elbow_power_status = parsed.elbow_power_status
        msg.led_status = parsed.led_status
        msg.loader_status = parsed.loader_status
        self.stm32_state_pub.publish(msg)

        status = parsed.elbow_moving_status
        if status == 0:  # STATUS_NEEDS_HOME
            self.home_elbow_status = False
            self._homing_in_progress = False
        elif status == 1:  # STATUS_HOMING
            self.home_elbow_status = False
            self._homing_in_progress = True
        elif status == 2:  # STATUS_HOME_ERROR
            self.home_elbow_status = False
            self._homing_in_progress = False
            self.get_logger().warn("STM32 elbow home error (status=2)")
        elif status == 3:  # STATUS_HOME_SUCCESS
            was_homing = self._homing_in_progress
            self.home_elbow_status = True
            self._homing_in_progress = False
            if was_homing:
                self.get_logger().info("Elbow homing complete from STM32 status: <x,x,3,x>")
        elif status == 4:  # STATUS_MOVING
            self.home_elbow_status = True
            self._homing_in_progress = False
        elif status == 5:  # STATUS_MOVE_SUCCESS
            self.home_elbow_status = True
            self._homing_in_progress = False
        elif status == 6:  # STATUS_MOVE_ERROR
            self._homing_in_progress = False
            self.get_logger().warn("STM32 elbow move error (status=6)")

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

    def _ensure_startup_idle_led(self) -> None:
        if self._startup_idle_led_applied:
            return

        if self._send_led_command(2):
            self._startup_idle_led_applied = True
            self.get_logger().info("Startup LED set to idle single-color mode")

    def close_serial(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def shutdown_leds(self) -> None:
        # Best-effort LED shutdown when this bridge node exits.
        if self._send_to_stm32("<L,0>"):
            self._last_led_payload = "<L,0>"
            self.get_logger().info("Sent LED shutdown command: <L,0>")

    def _parse_status_string(self, raw: str) -> Optional[ParsedStatus]:
        raw = raw.strip()
        match = re.search(r"<(.*?)>", raw)
        if match:
            try:
                parts = [part.strip() for part in match.group(1).split(',')]
                if len(parts) != 6:
                    return None

                switch2 = int(parts[0])
                switch3 = int(parts[1])
                moving_status = int(parts[2])
                power_status = int(parts[3])
                led_status = int(parts[4])
                loader_status = int(parts[5])

                return ParsedStatus(
                    sw2=switch2,
                    sw3=switch3,
                    elbow_moving_status=moving_status,
                    elbow_power_status=power_status,
                    led_status=led_status,
                    loader_status=loader_status
                )
            except (IndexError, ValueError) as e:
                self.get_logger().error(f"Error parsing status string: {e}")
                return None

        return None

    @staticmethod
    def _clamp_u8(value: int) -> int:
        return max(0, min(255, int(value)))

    def _send_led_command(self, led_value: int, r: Optional[int] = None, g: Optional[int] = None, b: Optional[int] = None) -> bool:
        led_value = int(led_value)
        if led_value == 2:
            rr = self._clamp_u8(self._idle_led_rgb[0] if r is None else r)
            gg = self._clamp_u8(self._idle_led_rgb[1] if g is None else g)
            bb = self._clamp_u8(self._idle_led_rgb[2] if b is None else b)
            payload = f"<L,2,{rr},{gg},{bb}>"
        else:
            payload = f"<L,{led_value}>"

        if payload == self._last_led_payload:
            return True

        if self._send_to_stm32(payload):
            self._last_led_payload = payload
            return True
        return False

    def _apply_led_request(self, msg: STM32Message) -> None:
        onset_state = int(getattr(msg, "onset_state", 0))
        if onset_state == 1:
            self._send_led_command(1)
            return

        if onset_state == 2:
            # RGB fields are optional in STM32Message; default to yellow when absent.
            red = int(getattr(msg, "led_r", self._idle_led_rgb[0]))
            green = int(getattr(msg, "led_g", self._idle_led_rgb[1]))
            blue = int(getattr(msg, "led_b", self._idle_led_rgb[2]))
            self._send_led_command(2, red, green, blue)
            return

        if onset_state == 0:
            self._send_led_command(0)

    def _apply_loader_request(self, msg: STM32Message) -> None:
        loader_request = int(getattr(msg, "loader_reqest", getattr(msg, "loader_request", 0)))
        if loader_request == 0:
            loader_request = int(getattr(msg, "stm32_state_request", 0))

        if loader_request == 0:
            self._last_loader_request_value = 0
            return

        if loader_request == self._last_loader_request_value:
            return

        if self._send_to_stm32("<B>"):
            self._last_loader_request_value = loader_request
            self.get_logger().info(f"Loader request pulse sent via <B> (request={loader_request})")

    def _on_command(self, msg: STM32Message) -> None:
        power_on = msg.power_on_status == 1
        home_requested = bool(getattr(msg, "home_elbow", 0) == 1 or getattr(msg, "home_elbow_request", 0) == 1)

        if self._last_power_on_status is None or power_on != self._last_power_on_status:
            self._send_to_stm32(f"<P,{1 if power_on else 0}>")
            self._last_power_on_status = power_on
            if power_on:
                self._send_led_command(2)
            else:
                self._send_led_command(0)
                self._last_angle_command_rad = None
                self._last_loader_request_value = 0

        if not power_on:
            self.home_elbow_status = False
            self._homing_in_progress = False
            self.get_logger().info("Power is OFF; ignoring motor command")
            return

        self._apply_led_request(msg)
        self._apply_loader_request(msg)

        if home_requested and not self._homing_in_progress:
            if self._send_to_stm32("<h>"):
                self._homing_in_progress = True
                self.get_logger().info("Homing command sent (<h>); waiting for STM32 status <x,x,3,x>")
            return

        if self.home_elbow_status:
            angle_rad = float(msg.angle_launch)
            should_send_angle = (
                self._last_angle_command_rad is None or
                abs(angle_rad - self._last_angle_command_rad) >= self._angle_command_epsilon_rad
            )
            if should_send_angle:
                self.message_to_stm32 = str(angle_rad)
                self.get_logger().info(f"Sending elbow angle: {angle_rad} rad")
                if self._send_to_stm32(f"<M,{angle_rad}>"):
                    self._last_angle_command_rad = angle_rad
        else:
            self.get_logger().info("Home not complete; ignoring elbow angle command")

    def _send_to_stm32(self, payload: str) -> bool:
        self.message_to_stm32 = payload
        try:
            if self.ser is None:
                self.ser = serial.Serial(self.usb_port, 115200, timeout=0.0, write_timeout=0.02)
            if self.ser.is_open:
                self.get_logger().info(f"Sending STM32 command: {payload}")
                self.ser.write(payload.encode("utf-8"))
                return True
        except Exception as e:
            self._log_serial_error_throttled(f"Serial write error: {e}")
            self.close_serial()
            self.ser = None
        return False


def main() -> None:
    rclpy.init()
    node = STM32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_leds()
        node.close_serial()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()