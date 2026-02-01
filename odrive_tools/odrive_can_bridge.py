#!/usr/bin/env python3
import threading
import time
import struct
from dataclasses import dataclass
from typing import Optional, Dict, List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

import can


# ---- ODrive CANSimple command IDs ---- #

CMD_HEARTBEAT = 0x01
CMD_SET_AXIS_STATE = 0x07
CMD_GET_ENCODER_ESTIMATES = 0x09
CMD_SET_CONTROLLER_MODE = 0x0B
CMD_SET_INPUT_VEL = 0x0D
CMD_GET_IQ = 0x14

AXIS_STATE_CLOSED_LOOP_CONTROL = 8
CONTROLLER_MODE_VELOCITY_CONTROL = 2

def make_arb_id(node_id: int, cmd_id: int) -> int:
    # CANSimple: (node_id << 5) | cmd_id - shifts node ID 5 left and ORs with command ID
    return (node_id << 5) | cmd_id

@dataclass
class ODriveAxis:
    # ODrive Axis State Data
    last_hb_time: float = 0.0
    error: int = 0
    state: int = 0
    result: int = 0
    traj_done: int = 0

    # Motion data
    pos_turns: float = 0.0
    vel_turns_per_sec: float = 0.0
    last_enc_time: float = 0.0

class ODriveCANBridge (Node):
    """ 
    Communicates with ODrive motor controllers over CAN bus using CANSimple protocol.

    Publishes: /odrive/joint_states (sensor_msgs/JointState)
    Subscribes: /odrive/command (sensor_msgs/JointState)

    - msg.name[i] selects the axis/joint
    - msg.velocity[i] sets the target velocity for that axis/joint
    - msg.position[i] sets the target position for that axis/joint

    """

    def __init__(self):
        super().__init__("odrive_can_bridge")

        # ---- Parameters ----
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("axis_ids", [0, 1, 2])
        self.declare_parameter("joint_names", ["axis0", "axis1", "axis2"])
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("auto_closed_loop", True)
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("topic_joint_states", "/odrive/joint_states")
        self.declare_parameter("topic_command", "/odrive/command")

        self.can_interface: str = self.get_parameter("can_interface").value
        self.axis_ids: List[int] = list(self.get_parameter("axis_ids").value)
        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.publish_rate_hz: float = float(self.get_parameter("publish_rate_hz").value)
        self.auto_closed_loop: bool = bool(self.get_parameter("auto_closed_loop").value)
        self.command_timeout_sec: float = float(self.get_parameter("command_timeout_sec").value)
        self.topic_joint_states: str = str(self.get_parameter("topic_joint_states").value)
        self.topic_command: str = str(self.get_parameter("topic_command").value)
        self._enc_rr_idx = 0


        if len(self.axis_ids) != len(self.joint_names):
            self.get_logger().error("Length of axis_ids and joint_names must be the same!")
            raise RuntimeError("Length of axis_ids and joint_names must be the same!")

        # Axis State Setup
        self.axes: Dict[int, ODriveAxis] = {axis_id: ODriveAxis() for axis_id in self.axis_ids}

        # Deadman Timer
        self.last_command_time: float = 0.0
        self.last_command_vel: Dict[int, float] = {axis_id: 0.0 for axis_id in self.axis_ids}
        self.last_command_pos: Dict[int, float] = {axis_id: 0.0 for axis_id in self.axis_ids}

        # Ros interfaces
        self.pub_js = self.create_publisher(JointState, self.topic_joint_states, 10)
        self.sub_cmd = self.create_subscription(JointState, self.topic_command, self.command_callback, 10)

        # CAN Bus Setup
        self.get_logger().info(f"Opening CAN interface: {self.can_interface}")
        self.can_bus = can.interface.Bus(channel=self.can_interface, bustype='socketcan')

        # Flush RX buffer so we don't parse stale frames
        while self.can_bus.recv(timeout=0.0) is not None:
            pass

        # Start CAN RX Thread
        self._rx_stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # Push all axes to closed loop on startup
        if self.auto_closed_loop:
            for axis_id in self.axis_ids:
                self._set_axis_state(axis_id, AXIS_STATE_CLOSED_LOOP_CONTROL)

        # Publish timer
        dt = 1.0 / max(self.publish_rate_hz, 1e-3)
        self._timer = self.create_timer(dt, self._publish_joint_states)
        self._enc_timer = self.create_timer(1.0 / 20.0, self._request_encoder_estimates)

        # Command watchdog timer (same cadence)
        self._watchdog_timer = self.create_timer(dt, self._command_watchdog)

        self.get_logger().info("odrive_can_bridge started")

    # ---- CAN TX functions ---- #
    def _send(self, arbitration_id: int, data: bytes):
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False
        )
        try:
            self.can_bus.send(msg)
        except can.CanError as e:
            self.get_logger().error(f"CAN send error (id=0x{arbitration_id:X}): {e}")

    def _set_axis_state(self, axis_id: int, requested_state: int):
        arb_id = make_arb_id(axis_id, CMD_SET_AXIS_STATE)
        data = struct.pack("<I", requested_state)
        self._send(arb_id, data)

    def _send_set_input_vel(self, axis_id: int, vel_turns_per_sec: float, torque_ff: float = 0.0):
        arb_id = make_arb_id(axis_id, CMD_SET_INPUT_VEL)
        data = struct.pack("<ff", vel_turns_per_sec, torque_ff)
        self._send(arb_id, data)

    # ---- CAN RX functions ---- #
    def _rx_loop(self):
        while not self._rx_stop.is_set():
            msg = self.can_bus.recv(timeout=0.1)
            if msg is None:
                continue
            if msg.is_extended_id:
                continue  # ignore non-CANSimple frames

            arb = msg.arbitration_id
            node_id = (arb >> 5) & 0x3F
            cmd_id = arb & 0x1F

            if node_id not in self.axes:
                continue

            now = time.time()

            if cmd_id == CMD_HEARTBEAT:
                # Guide example: error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if len(msg.data) >= 7:
                    error, state, result, traj_done = struct.unpack("<IBBB", bytes(msg.data[:7]))
                    st = self.axes[node_id]
                    st.error = int(error)
                    st.state = int(state)
                    st.result = int(result)
                    st.traj_done = int(traj_done)
                    st.last_hb_time = now

            elif cmd_id == CMD_GET_ENCODER_ESTIMATES:
                # Guide example: pos, vel = struct.unpack('<ff', bytes(msg.data))
                if len(msg.data) >= 8:
                    pos, vel = struct.unpack("<ff", bytes(msg.data[:8]))
                    st = self.axes[node_id]
                    st.pos_turns = float(pos)
                    st.vel_turns_per_sec = float(vel)
                    st.last_enc_time = now

    # ---- ROS Callbacks ---- #
    def command_callback(self, msg: JointState):
        """
        Expect:
          - msg.name: list of joint names (must match joint_names param)
          - msg.velocity: desired velocities in turns/s, aligned with msg.name
        """
        name_to_axis = {jn: aid for jn, aid in zip(self.joint_names, self.axis_ids)}

        if not msg.name or len(msg.velocity) != len(msg.name):
            self.get_logger().warn("Command JointState must include name[] and matching velocity[]")
            return

        for i, jname in enumerate(msg.name):
            if jname not in name_to_axis:
                continue
            aid = name_to_axis[jname]
            vel = float(msg.velocity[i])
            self.last_command_vel[aid] = vel
            self._send_set_input_vel(aid, vel, 0.0)

        self.last_command_time = time.time()

    def _command_watchdog(self):
        """
        If command stream stops, command all axes to 0 velocity.
        """
        if self.last_command_time <= 0.0:
            return
        if (time.time() - self.last_command_time) > self.command_timeout_sec:
            for aid in self.axis_ids:
                if self.last_command_vel.get(aid, 0.0) != 0.0:
                    self._send_set_input_vel(aid, 0.0, 0.0)
                    self.last_command_vel[aid] = 0.0

    def _request_encoder_estimates(self):
        if not self.axis_ids:
            return
        aid = self.axis_ids[self._enc_rr_idx % len(self.axis_ids)]
        self._enc_rr_idx += 1
        arb_id = make_arb_id(aid, CMD_GET_ENCODER_ESTIMATES)
        self._send(arb_id, b'')


    def _publish_joint_states(self):
        now_ros = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now_ros
        js.name = list(self.joint_names)
        js.position = []
        js.velocity = []
        js.effort = []  # leaving empty for now

        # Map axis states into the ordering of joint_names
        for jn, aid in zip(self.joint_names, self.axis_ids):
            st = self.axes[aid]
            js.position.append(st.pos_turns)     # turns
            js.velocity.append(st.vel_turns_per_sec)   # turns/s

        self.pub_js.publish(js)

    def destroy_node(self):
        self._rx_stop.set()
        try:
            if self._rx_thread.is_alive():
                self._rx_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.can_bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ODriveCANBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    

