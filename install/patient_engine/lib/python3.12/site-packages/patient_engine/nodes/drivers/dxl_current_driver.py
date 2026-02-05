#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# ================= XM540-W270 control table (Protocol 2.0) =================
ADDR_OPERATING_MODE       = 11
ADDR_TORQUE_ENABLE        = 64
ADDR_TEMPERATURE_LIMIT    = 31
ADDR_CURRENT_LIMIT        = 38
ADDR_PROFILE_ACCEL        = 108
ADDR_PROFILE_VELOCITY     = 112
ADDR_GOAL_CURRENT         = 102
ADDR_GOAL_POSITION        = 116
ADDR_PRESENT_CURRENT      = 126
ADDR_PRESENT_POSITION     = 132

TORQUE_DISABLE            = 0
TORQUE_ENABLE             = 1

OPERATING_MODE_MODE5_CURRENT_BASED_POSITION = 5
PROTOCOL_VERSION          = 2.0

# Position scaling for X-series single-turn ticks
DXL_MIN_POS_TICK          = 0
DXL_MAX_POS_TICK          = 4095
TICKS_PER_REV             = 4096.0
DEG_PER_TICK              = 360.0 / TICKS_PER_REV
TICKS_PER_DEG             = TICKS_PER_REV / 360.0

# XM540-W270 Current unit per eManual: about 2.69 mA per 1 raw unit
# Current Limit(38): 0..2047; Goal Current(102) must not exceed Current Limit
CURRENT_UNIT_mA_PER_RAW   = 2.69
CURRENT_RAW_MAX           = 2047

qos_sensorData = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

class DynamixelCurrentDriver(Node):
    """
    XM540-W270 Mode 5 (Current-based Position) driver:
      - Sub: /patient/command_position [deg]
      - Pub: /patient/joint_position   [deg]
      - (optional) Pub: /patient/present_current_mA [mA]

    Behavior:
      - Goal Position tracks command_position
      - Goal Current is used as a *cap* (non-negative), computed from tracking error
      - Anti-jitter (only write if changed + deadbands)
      - Software ROM clamp (because Min/Max Position Limit are not used in Mode 5)
    """

    def __init__(self):
        super().__init__("dxl_current_driver")

        # ---------------- Parameters ----------------
        self.declare_parameter("dxl_id", 1)
        self.declare_parameter("port_name", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 1_000_000)
        self.declare_parameter("loop_rate_hz", 100.0)

        # 0 deg -> center tick (typically around 2048 for mid-range)
        self.declare_parameter("center_position_tick", 2048)

        # Optional motion profile (leave -1 to not touch)
        self.declare_parameter("profile_velocity", -1)
        self.declare_parameter("profile_acceleration", -1)

        # Safety
        self.declare_parameter("temp_limit_degC", 80)
        self.declare_parameter("current_limit_mA", 1200.0)  # motor Current Limit (38)
        # Rehab ROM (software clamp)
        self.declare_parameter("rom_flexion_deg", -83.0)
        self.declare_parameter("rom_extension_deg", 81.0)

        # Error -> current cap mapping (telerehab choice B)
        self.declare_parameter("cap_min_mA", 120.0)          # compliant near target
        self.declare_parameter("cap_max_mA", 700.0)          # max assistance cap
        self.declare_parameter("cap_error_full_deg", 8.0)    # error where cap reaches max
        self.declare_parameter("cap_slew_mA_s", 3000.0)      # smooth cap changes

        # Anti-jitter deadbands
        self.declare_parameter("deadband_pos_tick", 1)       # write pos only if tick change >= this
        self.declare_parameter("deadband_current_raw", 2)    # write current only if raw change >= this

        # Optional debug publish
        self.declare_parameter("publish_present_current", True)

        # Read params
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

        self.cap_min_mA = float(self.get_parameter("cap_min_mA").value)
        self.cap_max_mA = float(self.get_parameter("cap_max_mA").value)
        self.cap_error_full_deg = float(self.get_parameter("cap_error_full_deg").value)
        self.cap_slew_mA_s = float(self.get_parameter("cap_slew_mA_s").value)

        self.deadband_pos_tick = int(self.get_parameter("deadband_pos_tick").value)
        self.deadband_current_raw = int(self.get_parameter("deadband_current_raw").value)

        self.publish_present_current = bool(self.get_parameter("publish_present_current").value)

        # Sanity clamps
        if self.rom_flexion_deg > self.rom_extension_deg:
            self.get_logger().warn("ROM bounds swapped; fixing automatically.")
            self.rom_flexion_deg, self.rom_extension_deg = self.rom_extension_deg, self.rom_flexion_deg

        self.cap_min_mA = max(0.0, self.cap_min_mA)
        self.cap_max_mA = max(self.cap_min_mA, self.cap_max_mA)
        # never allow cap to exceed motor Current Limit
        self.cap_max_mA = min(self.cap_max_mA, self.current_limit_mA)

        self.get_logger().info(
            f"XM540 Mode5 driver: ID={self.dxl_id}, port={self.port_name}, baud={self.baudrate}, "
            f"rate={self.loop_rate_hz}Hz, center_tick={self.center_tick}, "
            f"I_limit={self.current_limit_mA:.0f}mA, cap=[{self.cap_min_mA:.0f},{self.cap_max_mA:.0f}]mA"
        )

        # ---------------- DXL SDK setup ----------------
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            raise RuntimeError("Failed to open Dynamixel port")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError("Failed to set Dynamixel baudrate")

        self._configure_dxl_mode5()

        # ---------------- ROS I/O ----------------
        self.goal_deg = 0.0
        self.have_goal = False

        self.sub_goal = self.create_subscription(
            Float32, "/patient/command_position", self._goal_cb, qos_sensorData
        )
        self.pub_pos = self.create_publisher(
            Float32, "/patient/joint_position", qos_sensorData
        )
        self.pub_cur = self.create_publisher(
            Float32, "/patient/present_current_mA", qos_sensorData
        )

        # Anti-jitter state
        self.last_goal_tick = None
        self.last_goal_current_raw = None
        self.last_cap_mA = self.cap_min_mA

        self.dt = 1.0 / self.loop_rate_hz if self.loop_rate_hz > 0.0 else 0.01
        self.timer = self.create_timer(self.dt, self._loop)

    # ---------------- Configuration ----------------
    def _configure_dxl_mode5(self):
        # Torque off before changing mode/limits
        self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        # Operating mode 5
        self._write_1byte(ADDR_OPERATING_MODE, OPERATING_MODE_MODE5_CURRENT_BASED_POSITION)

        # Temperature limit
        self._write_1byte(ADDR_TEMPERATURE_LIMIT, self.temp_limit_degC)

        # Current limit (38): raw 0..2047
        raw_limit = self._mA_to_raw_unsigned(self.current_limit_mA)
        self._write_2byte(ADDR_CURRENT_LIMIT, raw_limit)

        # Optional profile
        if self.profile_acceleration >= 0:
            self._write_4byte(ADDR_PROFILE_ACCEL, self.profile_acceleration)
        if self.profile_velocity >= 0:
            self._write_4byte(ADDR_PROFILE_VELOCITY, self.profile_velocity)

        # Torque on
        self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        self.get_logger().info(
            "Configured XM540 for Mode 5. Note: Min/Max Position Limit are not used in Mode 5; "
            "ROM is enforced in software."
        )

    # ---------------- ROS callbacks ----------------
    def _goal_cb(self, msg: Float32):
        self.goal_deg = float(msg.data)
        self.have_goal = True

    # ---------------- Main loop ----------------
    def _loop(self):
        if not self.have_goal:
            return

        # 1) Read present position (and publish)
        present_tick = self._read_4byte(ADDR_PRESENT_POSITION)
        if present_tick is None:
            return

        present_deg = self._tick_to_deg(present_tick)
        self.pub_pos.publish(Float32(data=float(present_deg)))

        # Optional present current debug
        if self.publish_present_current:
            raw_i = self._read_2byte(ADDR_PRESENT_CURRENT)
            if raw_i is not None:
                # Present Current is signed in general; decode as int16
                raw_i_s = self._int16(raw_i)
                self.pub_cur.publish(Float32(data=float(raw_i_s * CURRENT_UNIT_mA_PER_RAW)))

        # 2) Clamp goal to rehab ROM (software clamp)
        goal_deg = self._cap(self.goal_deg, self.rom_flexion_deg, self.rom_extension_deg)
        goal_tick = self._deg_to_tick(goal_deg)

        # 3) Error-based current CAP (non-negative)
        err = abs(goal_deg - present_deg)
        alpha = self._cap(err / max(self.cap_error_full_deg, 1e-6), 0.0, 1.0)
        target_cap = self.cap_min_mA + alpha * (self.cap_max_mA - self.cap_min_mA)

        # Slew rate limit cap to avoid “kicks”
        max_step = self.cap_slew_mA_s * self.dt
        cap_mA = self._cap(target_cap, self.last_cap_mA - max_step, self.last_cap_mA + max_step)
        self.last_cap_mA = cap_mA

        # Respect motor Current Limit
        cap_mA = self._cap(cap_mA, 0.0, self.current_limit_mA)
        cap_raw = self._mA_to_raw_unsigned(cap_mA)

        # 4) Anti-jitter writes (deadbands)
        if (self.last_goal_current_raw is None) or (abs(cap_raw - self.last_goal_current_raw) >= self.deadband_current_raw):
            self._write_2byte(ADDR_GOAL_CURRENT, cap_raw)
            self.last_goal_current_raw = cap_raw

        if (self.last_goal_tick is None) or (abs(goal_tick - self.last_goal_tick) >= self.deadband_pos_tick):
            self._write_4byte(ADDR_GOAL_POSITION, goal_tick)
            self.last_goal_tick = goal_tick

    # ---------------- SDK helpers (with logging) ----------------
    def _write_1byte(self, addr: int, value: int):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, addr, int(value) & 0xFF
        )
        self._check_comm("write1", addr, dxl_comm_result, dxl_error)

    def _write_2byte(self, addr: int, value: int):
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
            self.port_handler, self.dxl_id, addr, int(value) & 0xFFFF
        )
        self._check_comm("write2", addr, dxl_comm_result, dxl_error)

    def _write_4byte(self, addr: int, value: int):
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.dxl_id, addr, int(value)
        )
        self._check_comm("write4", addr, dxl_comm_result, dxl_error)

    def _read_4byte(self, addr: int):
        value, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.dxl_id, addr
        )
        if not self._check_comm("read4", addr, dxl_comm_result, dxl_error, quiet=True):
            return None
        return value

    def _read_2byte(self, addr: int):
        value, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.dxl_id, addr
        )
        if not self._check_comm("read2", addr, dxl_comm_result, dxl_error, quiet=True):
            return None
        return value

    def _check_comm(self, op: str, addr: int, comm_result: int, dxl_error: int, quiet: bool=False) -> bool:
        ok = True
        if comm_result != COMM_SUCCESS:
            ok = False
            if not quiet:
                self.get_logger().error(
                    f"[DXL {self.dxl_id}] {op} failed at 0x{addr:02X}: {self.packet_handler.getTxRxResult(comm_result)}"
                )
        if dxl_error != 0:
            ok = False
            if not quiet:
                self.get_logger().error(
                    f"[DXL {self.dxl_id}] {op} error at 0x{addr:02X}: {self.packet_handler.getRxPacketError(dxl_error)}"
                )
        return ok

    # ---------------- Conversions ----------------
    def _deg_to_tick(self, deg: float) -> int:
        tick = self.center_tick + deg * TICKS_PER_DEG
        t = int(round(tick))
        return max(DXL_MIN_POS_TICK, min(DXL_MAX_POS_TICK, t))

    def _tick_to_deg(self, tick: int) -> float:
        return (int(tick) - self.center_tick) * DEG_PER_TICK

    @staticmethod
    def _mA_to_raw_unsigned(mA: float) -> int:
        # Goal Current used as cap => use non-negative raw
        raw = int(round(mA / CURRENT_UNIT_mA_PER_RAW))
        return max(0, min(CURRENT_RAW_MAX, raw))

    @staticmethod
    def _int16(u16: int) -> int:
        # interpret unsigned 16-bit as signed
        return u16 - 0x10000 if (u16 & 0x8000) else u16

    @staticmethod
    def _cap(x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    # ---------------- Shutdown ----------------
    def shutdown(self):
        try:
            self._write_1byte(ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        except Exception:
            pass
        try:
            if self.port_handler:
                self.port_handler.closePort()
        except Exception:
            pass

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
