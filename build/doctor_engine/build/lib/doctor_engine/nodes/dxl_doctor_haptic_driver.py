#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


# ---------------- DYNAMIXEL X-series control-table constants ----------------
# XM540-W270, Protocol 2.0
ADDR_OPERATING_MODE      = 11
ADDR_TORQUE_ENABLE       = 64
ADDR_CURRENT_LIMIT       = 38      # 2 bytes (unsigned)
ADDR_GOAL_CURRENT        = 102     # 2 bytes (signed)
ADDR_PRESENT_POSITION    = 132     # 4 bytes

TORQUE_DISABLE           = 0
TORQUE_ENABLE            = 1

OPERATING_MODE_CURRENT   = 0       # Current (Torque) Control Mode
PROTOCOL_VERSION         = 2.0

# 0–4095 ticks → 0–360°
TICKS_PER_REV            = 4096.0
DEG_PER_TICK             = 360.0 / TICKS_PER_REV
TICKS_PER_DEG            = TICKS_PER_REV / 360.0

DXL_MIN_POS_TICK         = 0
DXL_MAX_POS_TICK         = 4095

# XM540 current unit ~ 2.69 mA per unit (good enough for scaling/clamping)
MA_PER_UNIT              = 2.69

# QoS (reliable sensor-ish)
qos_sensorData = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class DoctorHapticDriver(Node):
    """
    Doctor-side haptic driver for a single XM540:
    - Publishes:  /doctor/target_position   [deg]  (doctor handle angle -> patient target)
    - Subscribes: /doctor/haptic_ref        [deg]  (patient measured angle)
    - Computes PD on error = (haptic_ref - doctor_pos) -> commands Goal Current (resistance only)
    - Motor mode: Current Control (Operating Mode 0)
    """

    def __init__(self):
        super().__init__("dxl_doctor_haptic_driver")

        # ---------------- Parameters ----------------
        self.declare_parameter("dxl_id", 1)
        self.declare_parameter("port_name", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 1_000_000)
        self.declare_parameter("loop_rate_hz", 200.0)

        # Center tick: 2048 -> 0deg
        self.declare_parameter("center_position_tick", 2048)

        # Haptic PD gains (in mA/deg and mA/(deg/s))
        self.declare_parameter("kp_mA_per_deg", 10.0)
        self.declare_parameter("kd_mA_per_deg_s", 1.0)

        # Deadband to prevent buzzing around zero error
        self.declare_parameter("deadband_deg", 0.3)

        # Safety limits
        self.declare_parameter("current_limit_mA", 600.0)   # writes Current Limit(38)
        self.declare_parameter("imax_cmd_mA", 300.0)        # clamp command current magnitude

        self.dxl_id = int(self.get_parameter("dxl_id").value)
        self.port_name = str(self.get_parameter("port_name").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.loop_rate_hz = float(self.get_parameter("loop_rate_hz").value)
        self.center_tick = int(self.get_parameter("center_position_tick").value)

        self.kp = float(self.get_parameter("kp_mA_per_deg").value)
        self.kd = float(self.get_parameter("kd_mA_per_deg_s").value)
        self.deadband_deg = float(self.get_parameter("deadband_deg").value)

        self.current_limit_mA = float(self.get_parameter("current_limit_mA").value)
        self.imax_cmd_mA = float(self.get_parameter("imax_cmd_mA").value)

        self.get_logger().info(
            f"Starting DoctorHapticDriver on {self.port_name} "
            f"(ID={self.dxl_id}, baud={self.baudrate})"
        )

        # ---------------- DXL SDK: port + packet handler ----------------
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open serial port. Check U2D2/USB and permissions.")
            raise RuntimeError("Failed to open Dynamixel port")

        if not self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().error("Failed to set baudrate.")
            raise RuntimeError("Failed to set Dynamixel baudrate")

        # ---------------- Configure motor (Current Mode + Current Limit + Torque ON) ----------------
        self._configure_dxl()

        # ---------------- ROS interfaces ----------------
        # publish doctor handle angle
        self.pub_doctor_angle = self.create_publisher(
            Float32,
            "/doctor/target_position",
            qos_sensorData,
        )

        # subscribe patient reference angle
        self.sub_haptic_ref = self.create_subscription(
            Float32,
            "/doctor/haptic_ref",
            self.haptic_ref_callback,
            qos_sensorData,
        )

        self.haptic_ref_deg = None

        # PD state
        self.prev_err = 0.0
        self.prev_t = time.time()

        period = 1.0 / self.loop_rate_hz if self.loop_rate_hz > 0.0 else 0.01
        self.timer = self.create_timer(period, self._update_loop)

        self.get_logger().info("Doctor haptic driver initialised (Current Control Mode 0).")

    # ---------------- DXL configuration ----------------

    def _configure_dxl(self):
        # Torque OFF before changing mode
        self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        # Set Operating Mode = Current Control (0)
        self._write_1byte(ADDR_OPERATING_MODE, OPERATING_MODE_CURRENT)

        # Set Current Limit (38)
        lim_units = int(round(max(0.0, self.current_limit_mA) / MA_PER_UNIT))
        lim_units = max(0, min(2047, lim_units))
        self._write_2byte(ADDR_CURRENT_LIMIT, lim_units)

        # Torque ON
        self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        self.get_logger().info(
            f"DXL ID {self.dxl_id}: OperatingMode=CURRENT(0), "
            f"CurrentLimit={self.current_limit_mA:.1f} mA (~{lim_units} units)"
        )

    # ---------------- ROS callbacks ----------------

    def haptic_ref_callback(self, msg: Float32):
        self.haptic_ref_deg = float(msg.data)

    def _update_loop(self):
        """
        Periodic loop:
        - Read Present Position -> publish /doctor/target_position (deg)
        - If /doctor/haptic_ref available: compute PD current -> write Goal Current
        - Else: write 0 current
        """
        present_tick = self._read_4byte(ADDR_PRESENT_POSITION)
        if present_tick is None:
            return

        doctor_deg = self._tick_to_deg(present_tick)

        # 1) Publish doctor angle as target for patient side
        self.pub_doctor_angle.publish(Float32(data=float(doctor_deg)))

        # 2) Compute haptic current command based on error
        if self.haptic_ref_deg is None:
            self._write_goal_current_mA(0.0)
            return

        err = self.haptic_ref_deg - doctor_deg

        # deadband to avoid chatter
        if abs(err) < self.deadband_deg:
            err = 0.0

        now = time.time()
        dt = max(1e-3, now - self.prev_t)
        derr = (err - self.prev_err) / dt

        I_cmd = self.kp * err + self.kd * derr

        # clamp
        I_cmd = max(-self.imax_cmd_mA, min(self.imax_cmd_mA, I_cmd))

        # write
        self._write_goal_current_mA(I_cmd)

        self.prev_err = err
        self.prev_t = now

    # ---------------- DXL helpers ----------------

    def _write_goal_current_mA(self, mA: float):
        # respect current limit too
        mA = max(-self.current_limit_mA, min(self.current_limit_mA, mA))
        units = int(round(mA / MA_PER_UNIT))  # signed

        # convert to unsigned 16-bit two's complement
        raw = units & 0xFFFF
        self._write_2byte(ADDR_GOAL_CURRENT, raw)

    def _write_1byte(self, address: int, value: int):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, address, int(value)
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
            self.port_handler, self.dxl_id, address, int(value)
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

    def _tick_to_deg(self, tick: int) -> float:
        return (int(tick) - self.center_tick) * DEG_PER_TICK

    # ---------------- Shutdown ----------------

    def shutdown(self):
        try:
            self._write_goal_current_mA(0.0)
            self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        except Exception:
            pass
        try:
            if self.port_handler is not None:
                self.port_handler.closePort()
        except Exception:
            pass
        self.get_logger().info("Doctor haptic driver shut down (current=0, torque disabled, port closed).")


def main(args=None):
    rclpy.init(args=args)
    node = DoctorHapticDriver()
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
