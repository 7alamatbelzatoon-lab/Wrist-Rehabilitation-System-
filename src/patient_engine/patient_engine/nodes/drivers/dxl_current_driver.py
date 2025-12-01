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
# Valid for XM540, etc. (Protocol 2.0)
ADDR_OPERATING_MODE       = 11
ADDR_TORQUE_ENABLE        = 64
ADDR_TEMPERATURE_LIMIT    = 31
ADDR_CURRENT_LIMIT        = 38
ADDR_MAX_POSITION_LIMIT   = 48
ADDR_MIN_POSITION_LIMIT   = 52
ADDR_PROFILE_ACCEL        = 108
ADDR_PROFILE_VELOCITY     = 112
ADDR_GOAL_CURRENT         = 102
ADDR_GOAL_POSITION        = 116
ADDR_PRESENT_CURRENT      = 126
ADDR_PRESENT_POSITION     = 132

DXL_MIN_POS_TICK          = 0
DXL_MAX_POS_TICK          = 4095

TORQUE_DISABLE            = 0
TORQUE_ENABLE             = 1

OPERATING_MODE_CURRENT_BASED_POSITION = 5
PROTOCOL_VERSION          = 2.0

# 0–4095 ticks → 0–360°
TICKS_PER_REV             = 4096.0
DEG_PER_TICK              = 360.0 / TICKS_PER_REV
TICKS_PER_DEG             = TICKS_PER_REV / 360.0

# QoS similar to patient_node
qos_sensorData = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class DynamixelCurrentDriver(Node):
    """
    Motor driver for a single Dynamixel X-series joint (XL430 / XM540) in
    CURRENT-BASED POSITION MODE (5).

    - Subscribes: /patient/goal_current_mA   [Float32, mA]
    - Publishes:  /patient/joint_position    [Float32, deg]

    From the outside, /patient/joint_position is the same as with 
    position driver — the difference is only how we drive the motor.
    """

    def __init__(self):
        super().__init__("dxl_current_driver")

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

        # Safety / ROM / current limits (can be tuned via YAML)
        self.declare_parameter("temp_limit_degC", 80)       # °C
        self.declare_parameter("current_limit_mA", 1500)    # mA
        self.declare_parameter("rom_flexion_deg", -83.0)    # min wrist flexion
        self.declare_parameter("rom_extension_deg", 81.0)   # max wrist extension

        self.dxl_id = int(self.get_parameter("dxl_id").value)
        self.port_name = str(self.get_parameter("port_name").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.loop_rate_hz = float(self.get_parameter("loop_rate_hz").value)
        self.center_tick = int(self.get_parameter("center_position_tick").value)
        self.profile_velocity = int(self.get_parameter("profile_velocity").value)
        self.profile_acceleration = int(self.get_parameter("profile_acceleration").value)

        self.temp_limit_degC = int(self.get_parameter("temp_limit_degC").value)
        self.current_limit_mA = float(self.get_parameter("current_limit_mA").value)
        self.rom_flexion_deg = float(self.get_parameter("rom_flexion_deg").value)
        self.rom_extension_deg = float(self.get_parameter("rom_extension_deg").value)

        self.get_logger().info(
            f"Starting Dynamixel CURRENT driver on {self.port_name} "
            f"(ID={self.dxl_id}, baud={self.baudrate})"
        )

        # ---------------- DXL SDK: port + packet handler ----------------
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error(
                " Failed to open serial port. Check U2D2/USB and permissions."
            )
            raise RuntimeError("Failed to open Dynamixel port")

        if not self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().error(" Failed to set baudrate.")
            raise RuntimeError("Failed to set Dynamixel baudrate")

        # ---------------- Configure motor (Current-based Position Mode, profiles, torque) ----------------
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
        self.last_goal_current_mA = 0.0

        self.sub_current = self.create_subscription(
            Float32,
            "/patient/goal_current_mA",
            self.current_callback,
            qos_sensorData,
        )

        self.pub_joint = self.create_publisher(
            Float32,
            "/patient/joint_position",
            qos_sensorData,
        )

        period = 1.0 / self.loop_rate_hz if self.loop_rate_hz > 0.0 else 0.01
        self.timer = self.create_timer(period, self._update_loop)

        self.get_logger().info(
            " Dynamixel CURRENT driver initialised (Operating Mode 5: current-based position)."
        )

    # ---------------- DXL configuration ----------------

    def _configure_dxl(self):
        """
        Configure Dynamixel for Current-based Position Mode (5),
        set current & safety limits, ROM, motion profiles, and enable torque.
        """
        # Torque OFF before changing EEPROM settings
        self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        # 1) Operating mode = Current-based Position (5)
        self._write_1byte(ADDR_OPERATING_MODE, OPERATING_MODE_CURRENT_BASED_POSITION)

        # 2) Safety: temperature limit
        self._write_1byte(ADDR_TEMPERATURE_LIMIT, self.temp_limit_degC)

        # 3) Safety: current limit (mA -> raw ~ [0..2047] for ~[0..5.5A])
        raw_current_limit = int(self._mA_to_raw(self.current_limit_mA))
        self._write_2byte(ADDR_CURRENT_LIMIT, raw_current_limit)

        # 4) ROM position limits (based on center tick and flex/extension angles)
        pulses_per_degree = TICKS_PER_REV / 360.0
        max_pos_tick = int(self.center_tick + self.rom_extension_deg * pulses_per_degree)
        min_pos_tick = int(self.center_tick + self.rom_flexion_deg * pulses_per_degree)
        max_pos_tick = max(DXL_MIN_POS_TICK, min(DXL_MAX_POS_TICK, max_pos_tick))
        min_pos_tick = max(DXL_MIN_POS_TICK, min(DXL_MAX_POS_TICK, min_pos_tick))

        self._write_4byte(ADDR_MAX_POSITION_LIMIT, max_pos_tick)
        self._write_4byte(ADDR_MIN_POSITION_LIMIT, min_pos_tick)

        # 5) Motion profiles (optional, like before)
        if self.profile_acceleration >= 0:
            self._write_4byte(ADDR_PROFILE_ACCEL, self.profile_acceleration)
        if self.profile_velocity >= 0:
            self._write_4byte(ADDR_PROFILE_VELOCITY, self.profile_velocity)

        # 6) Torque ON
        self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        self.get_logger().info(
            f"DXL ID {self.dxl_id}: Mode=CURRENT_BASED_POSITION(5), "
            f"TempLimit={self.temp_limit_degC}°C, "
            f"CurrentLimit={self.current_limit_mA:.1f} mA, "
            f"ROM=[{self.rom_flexion_deg}, {self.rom_extension_deg}] deg, "
            f"ProfileVel={self.profile_velocity}, ProfileAcc={self.profile_acceleration}"
        )

    # ---------------- ROS callbacks ----------------

    def current_callback(self, msg: Float32):
        """Store the latest commanded motor current [mA] from controller."""
        self.last_goal_current_mA = float(msg.data)

    def _update_loop(self):
        """
        Periodic loop:
        - Send latest goal current (mA) → Goal Current
        - Read Present Position → publish /patient/joint_position
        """
        # 1) Write latest current command to DXL
        mA = self.last_goal_current_mA

        # Saturate according to configured limit
        if mA > self.current_limit_mA:
            mA = self.current_limit_mA
        elif mA < -self.current_limit_mA:
            mA = -self.current_limit_mA

        raw_current = int(self._mA_to_raw(mA))
        self._write_2byte(ADDR_GOAL_CURRENT, raw_current)

        # 2) Read present position and publish
        present_tick = self._read_4byte(ADDR_PRESENT_POSITION)
        if present_tick is not None:
            present_deg = self._tick_to_deg(present_tick)
            self.pub_joint.publish(Float32(data=float(present_deg)))

    # ---------------- DXL helpers ----------------

    def _write_1byte(self, address: int, value: int):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, address, int(value) & 0xFF                 #  clamp to 0..255 (unsigned 1 byte)
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

    def _write_2byte(self, address: int, value: int):
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, self.dxl_id, address, int(value) & 0xFFFF 
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(
                f"[DXL {self.dxl_id}] Write2Byte failed at 0x{address:02X}: "
                f"{self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )
        elif dxl_error != 0:
            self.get_logger().error(
                f"[DXL {self.dxl_id}] Write2Byte error at 0x{address:02X}: "
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

    # ---------------- Unit conversions ----------------

    @staticmethod
    def _mA_to_raw(mA: float) -> int:
        """
        Convert current in mA to Dynamixel raw units.
        For X-series, 1 unit ≈ 2.69 mA, 2047 ≈ 5.5 A.
        """
        # Clamp just in case
        mA_clamped = max(-5500.0, min(5500.0, mA))
        raw = int(round(mA_clamped * (2047.0 / 5500.0)))
        return raw

    def _deg_to_tick(self, deg: float) -> int:
        """
        Map controller angle [deg] to Dynamixel position tick.
        0 deg → center_tick (default 2048) so [-180, +180] maps into [0, 4095].
        (Not used in this node but kept for completeness.)
        """
        tick = self.center_tick + deg * TICKS_PER_DEG
        tick_int = int(round(tick))
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
        self.get_logger().info(
            "Dynamixel CURRENT driver shut down (torque disabled, port closed)."
        )


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelCurrentDriver()
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
