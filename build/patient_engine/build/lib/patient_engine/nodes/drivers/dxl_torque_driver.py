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

# -------- DYNAMIXEL X-series control-table constants (XM540, Protocol 2.0) --------
ADDR_OPERATING_MODE     = 11
ADDR_TORQUE_ENABLE      = 64
ADDR_TEMPERATURE_LIMIT  = 31
ADDR_CURRENT_LIMIT      = 38
ADDR_GOAL_CURRENT       = 102
ADDR_PRESENT_CURRENT    = 126
ADDR_PRESENT_POSITION   = 132

TORQUE_DISABLE          = 0
TORQUE_ENABLE           = 1

OPERATING_MODE_CURRENT  = 0      # Pure current control
PROTOCOL_VERSION        = 2.0

# 0–4095 ticks → 0–360°
TICKS_PER_REV           = 4096.0
DEG_PER_TICK            = 360.0 / TICKS_PER_REV
TICKS_PER_DEG           = TICKS_PER_REV / 360.0

DXL_MIN_POS_TICK        = 0
DXL_MAX_POS_TICK        = 4095

# QoS similar to your other nodes
qos_sensorData = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class DynamixelTorqueDriver(Node):
    """
    Dynamixel driver for XM540 in CURRENT CONTROL MODE (0).

    - Subscribes:
        /patient/goal_current_mA  [Float32, mA]   (from PatientCurrentController)
    - Publishes:
        /patient/joint_position   [Float32, deg]  (to patient_node, controller, safety)
    """

    def __init__(self):
        super().__init__("dxl_torque_driver")

        # -------- Parameters --------
        self.declare_parameter("dxl_id", 1)
        self.declare_parameter("port_name", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 1_000_000)
        self.declare_parameter("loop_rate_hz", 100.0)

        # Center tick for angle mapping (same idea as other drivers)
        self.declare_parameter("center_position_tick", 2048)

        # Safety limits
        self.declare_parameter("temp_limit_degC", 80)          # °C
        self.declare_parameter("current_limit_mA", 1500.0)     # mA (hardware cap)

        self.dxl_id = int(self.get_parameter("dxl_id").value)
        self.port_name = str(self.get_parameter("port_name").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.loop_rate_hz = float(self.get_parameter("loop_rate_hz").value)
        self.center_tick = int(self.get_parameter("center_position_tick").value)
        self.temp_limit_degC = int(self.get_parameter("temp_limit_degC").value)
        self.current_limit_mA = float(self.get_parameter("current_limit_mA").value)

        self.get_logger().info(
            f"Starting Dynamixel TORQUE driver on {self.port_name} "
            f"(ID={self.dxl_id}, baud={self.baudrate})"
        )

        # -------- DXL SDK: port + packet handler --------
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error(" Failed to open serial port. Check U2D2/USB and permissions.")
            raise RuntimeError("Failed to open Dynamixel port")

        if not self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().error(" Failed to set baudrate.")
            raise RuntimeError("Failed to set Dynamixel baudrate")

        # -------- Configure motor (Mode 0, safety, torque) --------
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

        # -------- ROS interfaces --------
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
            " Dynamixel TORQUE driver initialised (Operating Mode 0: current control)."
        )

    # -------- DXL configuration --------

    def _configure_dxl(self):
        """
        Configure Dynamixel for Current Control Mode (0),
        set safety limits, and enable torque.
        """
        # Torque OFF before touching mode & limits
        self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        # 1) Operating mode = Current (0)
        self._write_1byte(ADDR_OPERATING_MODE, OPERATING_MODE_CURRENT)

        # 2) Safety: temperature limit
        self._write_1byte(ADDR_TEMPERATURE_LIMIT, self.temp_limit_degC)

        # 3) Safety: current limit (mA -> raw units)
        raw_current_limit = int(self._mA_to_raw(self.current_limit_mA))
        self._write_2byte(ADDR_CURRENT_LIMIT, raw_current_limit)

        # 4) Torque ON
        self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        self.get_logger().info(
            f"DXL ID {self.dxl_id}: Mode=CURRENT(0), "
            f"TempLimit={self.temp_limit_degC}°C, "
            f"CurrentLimit={self.current_limit_mA:.1f} mA"
        )

    # -------- ROS callbacks --------

    def current_callback(self, msg: Float32):
        """Store the latest commanded motor current [mA] from the controller."""
        self.last_goal_current_mA = float(msg.data)

    # -------- Main loop --------

    def _update_loop(self):
        """
        Periodic loop:
        - Send latest goal current (mA) → Goal Current
        - Read Present Position → publish /patient/joint_position
        """
        # 1) Write latest current command to DXL
        mA = self.last_goal_current_mA

        # Saturate according to configured hardware limit
        if mA > self.current_limit_mA:
            mA = self.current_limit_mA
        elif mA < -self.current_limit_mA:
            mA = -self.current_limit_mA

        raw_current = int(self._mA_to_raw(mA))
        self._write_2byte(ADDR_GOAL_CURRENT, raw_current)

        # 2) Read present position and publish (so patient_node & safety still work)
        present_tick = self._read_4byte(ADDR_PRESENT_POSITION)
        if present_tick is not None:
            present_deg = self._tick_to_deg(present_tick)
            self.pub_joint.publish(Float32(data=float(present_deg)))

    # -------- DXL helpers --------

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

    # -------- Unit conversions --------

    @staticmethod
    def _mA_to_raw(mA: float) -> int:
        """
        Convert current in mA to Dynamixel raw units.
        For X-series, 1 unit ≈ 2.69 mA, 2047 ≈ 5.5 A.
        """
        mA_clamped = max(-5500.0, min(5500.0, mA))
        raw = int(round(mA_clamped * (2047.0 / 5500.0)))
        return raw

    def _tick_to_deg(self, tick: int) -> float:
        """
        Map Dynamixel Present Position tick to controller angle [deg].
        center_tick → 0 deg.
        """
        return (int(tick) - self.center_tick) * DEG_PER_TICK

    # (deg_to_tick not strictly needed here, but you can add it if you like)

    # -------- Shutdown --------

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
        self.get_logger().info("Dynamixel TORQUE driver shut down (torque disabled, port closed).")


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelTorqueDriver()
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
