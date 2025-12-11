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
ADDR_VELOCITY_LIMIT       = 44
ADDR_PROFILE_ACCEL        = 108
ADDR_PROFILE_VELOCITY     = 112
ADDR_GOAL_VELOCITY        = 104
ADDR_PRESENT_VELOCITY     = 128
ADDR_PRESENT_POSITION     = 132

DXL_MIN_POS_TICK          = 0
DXL_MAX_POS_TICK          = 4095

TORQUE_DISABLE            = 0
TORQUE_ENABLE             = 1

OPERATING_MODE_VELOCITY   = 1
PROTOCOL_VERSION          = 2.0

# 0–4095 ticks → 0–360°
TICKS_PER_REV             = 4096.0
DEG_PER_TICK              = 360.0 / TICKS_PER_REV
TICKS_PER_DEG             = TICKS_PER_REV / 360.0

# Velocity units: 1 unit = 0.229 rpm  (standard X-series)
RPM_PER_UNIT              = 0.229

# QoS similar to patient_node
qos_sensorData = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class DynamixelVelocityDriver(Node):
    """
    Motor driver for a single Dynamixel X-series joint (XM540) in
    VELOCITY CONTROL MODE (1).

    - Subscribes: /patient/goal_velocity_deg_s  [Float32, deg/s]
    - Publishes:  /patient/joint_position       [Float32, deg]

    From the outside, /patient/joint_position is the same as with the
    position and current drivers — only the actuation law is different.
    """

    def __init__(self):
        super().__init__("dxl_velocity_driver")

        # ---------------- Parameters ----------------
        self.declare_parameter("dxl_id", 1)
        self.declare_parameter("port_name", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 1_000_000)
        self.declare_parameter("loop_rate_hz", 100.0)

        # Center tick: 2048 → controller 0° corresponds to middle of 0..4095
        self.declare_parameter("center_position_tick", 2048)

        # Optional profiles (mostly used in position modes; kept for consistency)
        self.declare_parameter("profile_velocity", -1)
        self.declare_parameter("profile_acceleration", -1)

        # Safety limits
        self.declare_parameter("temp_limit_degC", 80)         # °C
        self.declare_parameter("current_limit_mA", 1500.0)    # mA

        # Commanded velocity limit in controller units [deg/s]
        self.declare_parameter("velocity_limit_deg_s", 60.0)  

        self.dxl_id = int(self.get_parameter("dxl_id").value)
        self.port_name = str(self.get_parameter("port_name").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.loop_rate_hz = float(self.get_parameter("loop_rate_hz").value)
        self.center_tick = int(self.get_parameter("center_position_tick").value)
        self.profile_velocity = int(self.get_parameter("profile_velocity").value)
        self.profile_acceleration = int(self.get_parameter("profile_acceleration").value)

        self.temp_limit_degC = int(self.get_parameter("temp_limit_degC").value)
        self.current_limit_mA = float(self.get_parameter("current_limit_mA").value)
        self.velocity_limit_deg_s = float(self.get_parameter("velocity_limit_deg_s").value)

        # Pre-compute raw velocity limit for Goal Velocity / Velocity Limit registers
        self.velocity_limit_raw = abs(self._deg_s_to_raw(self.velocity_limit_deg_s))

        self.get_logger().info(
            f"Starting Dynamixel VELOCITY driver on {self.port_name} "
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

        # ---------------- Configure motor (Velocity Mode, safety, torque) ----------------
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
        self.last_goal_vel_deg_s = 0.0

        self.sub_vel = self.create_subscription(
            Float32,
            "/patient/goal_velocity_deg_s",
            self.velocity_callback,
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
            " Dynamixel VELOCITY driver initialised (Operating Mode 1: velocity control)."
        )

    # ---------------- DXL configuration ----------------

    def _configure_dxl(self):
        """
        Configure Dynamixel for Velocity Control Mode (1),
        set safety limits, optional profiles, and enable torque.
        """
        # Torque OFF before changing EEPROM settings
        self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        # 1) Operating mode = Velocity (1)
        self._write_1byte(ADDR_OPERATING_MODE, OPERATING_MODE_VELOCITY)

        # 2) Safety: temperature limit
        self._write_1byte(ADDR_TEMPERATURE_LIMIT, self.temp_limit_degC)

        # 3) Safety: current limit (mA -> raw)
        raw_current_limit = int(self._mA_to_raw(self.current_limit_mA))
        self._write_2byte(ADDR_CURRENT_LIMIT, raw_current_limit)

        # 4) Velocity limit (deg/s → raw goal velocity units)
        if self.velocity_limit_raw > 0:
            self._write_4byte(ADDR_VELOCITY_LIMIT, self.velocity_limit_raw)

        # 5) Optional motion profiles (not critical in pure velocity mode, keep for consistency)
        if self.profile_acceleration >= 0:
            self._write_4byte(ADDR_PROFILE_ACCEL, self.profile_acceleration)
        if self.profile_velocity >= 0:
            self._write_4byte(ADDR_PROFILE_VELOCITY, self.profile_velocity)

        # 6) Torque ON
        self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        self.get_logger().info(
            f"DXL ID {self.dxl_id}: Mode=VELOCITY(1), "
            f"TempLimit={self.temp_limit_degC}°C, "
            f"CurrentLimit={self.current_limit_mA:.1f} mA, "
            f"VelLimit={self.velocity_limit_deg_s:.1f} deg/s "
            f"(raw={self.velocity_limit_raw})"
        )

    # ---------------- ROS callbacks ----------------

    def velocity_callback(self, msg: Float32):
        """Store the latest commanded joint velocity [deg/s] from controller."""
        self.last_goal_vel_deg_s = float(msg.data)

    def _update_loop(self):
        """
        Periodic loop:
        - Send latest goal velocity (deg/s) → Goal Velocity
        - Read Present Position → publish /patient/joint_position
        """
        # 1) Write latest velocity command to DXL
        vel_deg_s = self.last_goal_vel_deg_s

        # Saturate in controller units
        v_max = abs(self.velocity_limit_deg_s)
        if vel_deg_s > v_max:
            vel_deg_s = v_max
        elif vel_deg_s < -v_max:
            vel_deg_s = -v_max

        raw_vel = self._deg_s_to_raw(vel_deg_s)

        # Also clamp to raw velocity limit
        if self.velocity_limit_raw > 0:
            if raw_vel > self.velocity_limit_raw:
                raw_vel = self.velocity_limit_raw
            elif raw_vel < -self.velocity_limit_raw:
                raw_vel = -self.velocity_limit_raw

        self._write_4byte(ADDR_GOAL_VELOCITY, raw_vel)

        # 2) Read present position and publish
        present_tick = self._read_4byte(ADDR_PRESENT_POSITION)
        if present_tick is not None:
            present_deg = self._tick_to_deg(present_tick)
            self.pub_joint.publish(Float32(data=float(present_deg)))

    # ---------------- DXL helpers ----------------

    def _write_1byte(self, address: int, value: int):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, address, int(value) & 0xFF
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
            self.port_handler, self.dxl_id, address, int(value) & 0xFFFFFFFF
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
        mA_clamped = max(-5500.0, min(5500.0, mA))
        raw = int(round(mA_clamped * (2047.0 / 5500.0)))
        return raw

    @staticmethod
    def _deg_s_to_raw(vel_deg_s: float) -> int:
        """
        Convert joint velocity [deg/s] to Dynamixel Goal Velocity raw units.

        - vel_deg_s → rpm  (rpm = deg/s * 60/360 = deg/s / 6)
        - rpm → raw unit   (1 unit = 0.229 rpm)
        """
        rpm = vel_deg_s / 6.0
        raw = int(round(rpm / RPM_PER_UNIT))
        # Clamp to signed 32-bit range just in case
        raw = max(-2**31, min(2**31 - 1, raw))
        return raw

    def _deg_to_tick(self, deg: float) -> int:
        """
        Map controller angle [deg] to Dynamixel position tick.
        0 deg → center_tick (default 2048) so [-180, +180] maps into [0, 4095].
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
        self.get_logger().info("Dynamixel VELOCITY driver shut down (torque disabled, port closed).")


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelVelocityDriver()
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
