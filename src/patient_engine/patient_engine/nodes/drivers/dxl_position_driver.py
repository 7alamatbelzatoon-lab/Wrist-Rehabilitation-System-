#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
)

# ---------------- DYNAMIXEL X-series control-table constants ----------------
# (Valid for XL430, XM540, etc., Protocol 2.0)
ADDR_OPERATING_MODE      = 11
ADDR_TORQUE_ENABLE       = 64
ADDR_PROFILE_ACCEL       = 108
ADDR_PROFILE_VELOCITY    = 112
ADDR_GOAL_POSITION       = 116
ADDR_PRESENT_POSITION    = 132

DXL_MIN_POS_TICK         = 0
DXL_MAX_POS_TICK         = 4095

TORQUE_DISABLE           = 0
TORQUE_ENABLE            = 1

OPERATING_MODE_POSITION  = 3     # Position Control Mode
PROTOCOL_VERSION         = 2.0

# 0–4095 ticks → 0–360°
TICKS_PER_REV            = 4096.0
DEG_PER_TICK             = 360.0 / TICKS_PER_REV
TICKS_PER_DEG            = TICKS_PER_REV / 360.0

# QoS similar to patient_node
qos_sensorData = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class DynamixelDriver(Node):
    """
    Motor driver for a single Dynamixel X-series joint (XL430 / XM540).
    - Subscribes: /patient/command_position  [deg]
    - Publishes:  /patient/joint_position    [deg]
    - Configures motor in Position Control Mode (3).
    """

    def __init__(self):
        super().__init__("patient_dxl_driver")

        # ---------------- Parameters ----------------
        self.declare_parameter("dxl_id", 1)
        self.declare_parameter("port_name", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 1_000_000)
        self.declare_parameter("loop_rate_hz", 100.0)

        # Center tick: 2048 → controller 0° corresponds to middle of 0..4095
        self.declare_parameter("center_position_tick", 2048)

        # Default -1 = "do not touch existing profile config"
        self.declare_parameter("profile_velocity", -1)
        self.declare_parameter("profile_acceleration", -1)

        self.dxl_id = int(self.get_parameter("dxl_id").value)
        self.port_name = str(self.get_parameter("port_name").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.loop_rate_hz = float(self.get_parameter("loop_rate_hz").value)
        self.center_tick = int(self.get_parameter("center_position_tick").value)
        self.profile_velocity = int(self.get_parameter("profile_velocity").value)
        self.profile_acceleration = int(self.get_parameter("profile_acceleration").value)

        self.get_logger().info(
            f"Starting Dynamixel driver on {self.port_name} "
            f"(ID={self.dxl_id}, baud={self.baudrate})"
        )

        # ---------------- DXL SDK: port + packet handler ----------------
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error(" Failed to open serial port. Check U2D2/USB and permissions.")
            raise RuntimeError("Failed to open Dynamixel port")

        if not self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().error(" Failed to set baudrate.")
            raise RuntimeError("Failed to set Dynamixel baudrate")

        # ---------------- Configure motor (Position Mode, profiles, torque) ----------------
        self._configure_dxl()

        # Quick sanity read
        present_tick = self._read_4byte(ADDR_PRESENT_POSITION)
        if present_tick is None:
            self.get_logger().error(
                " Could not read Present Position after config. "
                "Check ID/baudrate/wiring/power."
            )
        else:
            present_deg = self._tick_to_deg(present_tick)
            self.get_logger().info(
                f"DXL ID {self.dxl_id} initial position ≈ {present_deg:.2f}°"
            )

        # ---------------- ROS interfaces ----------------
        self.last_command_deg = None
        self.last_command_tick = None

        self.sub_cmd = self.create_subscription(
            Float32,
            "/patient/command_position",
            self.command_callback,
            qos_sensorData,
        )

        self.pub_joint = self.create_publisher(
            Float32,
            "/patient/joint_position",
            qos_sensorData,
        )

        period = 1.0 / self.loop_rate_hz if self.loop_rate_hz > 0.0 else 0.01
        self.timer = self.create_timer(period, self._update_loop)

        self.get_logger().info("✅ Dynamixel driver initialised (Position Control Mode 3).")

    # ---------------- DXL configuration ----------------

    def _configure_dxl(self):
        """Set Position Control Mode, optional profile, and enable torque."""
        # Torque OFF before changing EEPROM settings
        self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        # Set Operating Mode = Position Control (3)
        self._write_1byte(ADDR_OPERATING_MODE, OPERATING_MODE_POSITION)

        # Optional: set Profile Accel / Vel for smooth motion
        if self.profile_acceleration >= 0:
            self._write_4byte(ADDR_PROFILE_ACCEL, self.profile_acceleration)
        if self.profile_velocity >= 0:
            self._write_4byte(ADDR_PROFILE_VELOCITY, self.profile_velocity)

        # Torque ON
        self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        self.get_logger().info(
            f"DXL ID {self.dxl_id}: OperatingMode=POSITION(3), "
            f"ProfileVel={self.profile_velocity}, ProfileAcc={self.profile_acceleration}"
        )

    # ---------------- ROS callbacks ----------------

    def command_callback(self, msg: Float32):
        """Store the latest commanded position (deg) from controller."""
        cmd_deg = float(msg.data)
        self.last_command_deg = cmd_deg

    def _update_loop(self):
        """
        Periodic loop:
        - Send latest command (if any) → Goal Position
        - Read Present Position → publish /patient/joint_position
        """
        # 1) Write latest command to DXL (if available)
        if self.last_command_deg is not None:
            tick = self._deg_to_tick(self.last_command_deg)

            # Avoid spamming the bus if the tick hasn't really changed
            if tick != self.last_command_tick:
                self._write_4byte(ADDR_GOAL_POSITION, tick)
                self.last_command_tick = tick

        # 2) Read present position and publish
        present_tick = self._read_4byte(ADDR_PRESENT_POSITION)
        if present_tick is not None:
            present_deg = self._tick_to_deg(present_tick)
            self.pub_joint.publish(Float32(data=float(present_deg)))

    # ---------------- DXL helpers ----------------

    def _write_1byte(self, address: int, value: int):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, address, value
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(
                f"[DXL {self.dxl_id}] Write1Byte failed at 0x{address:02X}: "
                f"{self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )
        elif dxl_error != 0:
            self.get_logger().error(
                f"[DXL {self.dxl_id}] Write1Byte error at 0x{address:02X}: "
                f"{self.packet_handler.getRxPacketError(dxl_error)}"
            )

    def _write_4byte(self, address: int, value: int):
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.dxl_id, address, int(value)
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(
                f"[DXL {self.dxl_id}] Write4Byte failed at 0x{address:02X}: "
                f"{self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )
        elif dxl_error != 0:
            self.get_logger().error(
                f"[DXL {self.dxl_id}] Write4Byte error at 0x{address:02X}: "
                f"{self.packet_handler.getRxPacketError(dxl_error)}"
            )

    def _read_4byte(self, address: int):
        value, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.dxl_id, address
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(
                f"[DXL {self.dxl_id}] Read4Byte failed at 0x{address:02X}: "
                f"{self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )
            return None
        if dxl_error != 0:
            self.get_logger().error(
                f"[DXL {self.dxl_id}] Read4Byte error at 0x{address:02X}: "
                f"{self.packet_handler.getRxPacketError(dxl_error)}"
            )
            return None
        return value

    # ---------------- Angle / tick conversion ----------------

    def _deg_to_tick(self, deg: float) -> int:
        """
        Map controller angle [deg] to Dynamixel position tick.
        0 deg → center_tick (default 2048) so [-180, +180] maps into [0, 4095].
        """
        tick = self.center_tick + deg * TICKS_PER_DEG
        tick_int = int(round(tick))
        # Clamp to valid single-turn tick range
        tick_int = max(DXL_MIN_POS_TICK, min(DXL_MAX_POS_TICK, tick_int))
        return tick_int

    def _tick_to_deg(self, tick: int) -> float:
        """
        Map Dynamixel Present Position tick to controller angle [deg].
        center_tick → 0 deg.
        """
        return (int(tick) - self.center_tick) * DEG_PER_TICK

    # ---------------- Shutdown ----------------

    def shutdown(self):
        """Cleanly disable torque and close the port."""
        try:
            self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        except Exception:
            pass
        try:
            if self.port_handler is not None:
                self.port_handler.closePort()
        except Exception:
            pass
        self.get_logger().info("Dynamixel driver shut down (torque disabled, port closed).")


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
