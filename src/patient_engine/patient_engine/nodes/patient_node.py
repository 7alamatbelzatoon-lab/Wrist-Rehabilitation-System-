#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool, Empty
import os
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Patient locomotion CONTROLLER (was "dummy" executor)
# - CHANGED: This node is now a pure controller. It no longer simulates the plant/joint internally.
# - Modes: NORMAL | FREEZE | SLEEP | ZERO_POSITION
# - NORMAL: adaptive spring–damper based on real measured joint position
# - FREEZE: hold still around current angle + /patient/freeze_stable & /patient/freeze/done
# - SLEEP/ZERO: generate trajectory to 0° (SOFT vs CREEP) then auto FREEZE
# - Report-backs (patient is sole publisher):
#     /patient/freeze/done (Empty, edge)
#     /patient/zero_reached (Empty, edge)
#     /patient/freeze_stable (Bool, level)
#     /patient/at_zero (Bool, level)
# - CHANGED: Instead of publishing /patient/joint_position (simulated),
#            the node now PUBLISHES /patient/command_position (what the motor should do)
#            and SUBSCRIBES to /patient/joint_position (measured from hardware driver).

# 0) Sensor data
qos_sensorData = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# 1) Control command (SUB) — small, reliable, latest-only
qos_control_cmd_sub = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,   # publisher will be TL; reader can stay Volatile
)

# 2) Status "levels" (PUB) — latched so late joiners see current state
qos_status_level_pub = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,  # latch last True/False
)

# 3) One-shot "edges" (PUB) — do NOT latch (avoid replays)
qos_status_edge_pub = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,   # no replay
)


class PatientNode(Node):
    def __init__(self):
        super().__init__("patient_node")

        # Subscriptions (one publisher elsewhere)
        self.sub_target = self.create_subscription(
            Float32,
            '/patient/target_position',
            self.target_callback,
            qos_sensorData
        )
        self.sub_mode = self.create_subscription(
            String,
            '/patient/control_mode',
            self.mode_callback,
            qos_control_cmd_sub
        )

        # CHANGED: New subscription for measured joint position from hardware driver
        self.sub_joint_meas = self.create_subscription(
            Float32,
            '/patient/joint_position',           # same topic name, but now we LISTEN instead of publish
            self.joint_meas_callback,
            qos_sensorData
        )

        # Publishers (patient is sole publisher for status + command position)
        # CHANGED: We no longer publish /patient/joint_position here.
        #          Instead we publish /patient/command_position (controller output).
        self.pub_command = self.create_publisher(
            Float32,
            '/patient/command_position',         # NEW: command position for the motor driver
            qos_sensorData
        )
        self.pub_freeze_done = self.create_publisher(
            Empty,
            '/patient/freeze/done',
            qos_status_edge_pub
        )
        self.pub_zero_reached = self.create_publisher(
            Empty,
            '/patient/zero_reached',
            qos_status_edge_pub
        )
        self.pub_freeze_stable = self.create_publisher(
            Bool,
            '/patient/freeze_stable',
            qos_status_level_pub
        )
        self.pub_at_zero = self.create_publisher(
            Bool,
            '/patient/at_zero',
            qos_status_level_pub
        )

        # Loop timing — EXACTLY like the old node
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.update_tick)
        self.get_logger().info("Patient Node Initiated as CONTROLLER (NORMAL timing matches original).")

        # Core state (degrees & deg/s)
        self.mode = "FREEZE"           # safe default; orchestrator switches
        self.target_position = 0.0     # deg (from MQTT / doctor)

        # CHANGED: current_position is now MEASURED from hardware (via joint_meas_callback),
        #          not integrated internally.
        self.current_position = 0.0    # deg (measured joint angle)
        self.have_joint_measurement = False  # CHANGED: flag to wait for first sensor reading

        # CHANGED: velocity now represents INTERNAL COMMAND velocity (deg/s) for command_deg dynamics,
        #          not the physical joint velocity.
        self.velocity = 0.0            # deg/s (internal command velocity in NORMAL)

        # For SLEEP/ZERO: slewed command to 0°
        self.command_deg = 0.0         # CHANGED: treated as "commanded joint position" sent to motor
        self.command_vel_deg_s = 0.0   # internal velocity for SLEEP/ZERO trapezoid

        # Measured velocity (deg/s) for stability/zero checks (unit-consistent across modes)
        self.prev_position = self.current_position
        self.vel_meas_deg_s = 0.0      # CHANGED: this is derived from measured joint position only

        # CHANGED: FREEZE hold point (where to stop)
        self.freeze_origin_deg = 0.0   # NEW: remembers position at entry to FREEZE

        # === Adaptive stiffness (UNCHANGED) ===
        self.spring_k = 0.225
        self.damping_c = 0.4  # baseline damping (NORMAL)
        self.k_min = 0.05
        self.k_max = 0.5
        self.k_step = 0.01
        self.low_error_threshold  = 1.0   # deg
        self.high_error_threshold = 5.0   # deg

        # === Profiles / FREEZE parameters (tunable later) ===
        self.angle_min_deg = -65.0
        self.angle_max_deg = +65.0
        self.creep_threshold_deg = 40.0

        # SOFT (gentle) / CREEP (extra gentle) for SLEEP/ZERO
        self.vmax_soft_deg_s   = 40.0
        self.amax_soft_deg_s2  = 150.0
        self.vmax_creep_deg_s  = 15.0
        self.amax_creep_deg_s2 = 60.0

        # FREEZE uses higher damping to settle fast
        self.freeze_damping_c = 1.0

        # Stabilization / zero detection (debounced)
        self.zero_tol_deg     = 0.5
        self.vel_stable_deg_s = 2.0
        self.hold_time_s      = 0.5

        self._freeze_hold_elapsed = 0.0
        self._at_zero_hold_elapsed = 0.0
        self._freeze_stable_latch = False
        self._at_zero_latch = False

        # CSV logs (unchanged)
        self.log_path = os.path.expanduser('~/ros2_ws/adaptive_log.csv')
        with open(self.log_path, "w") as log_file:
            log_file.write("Time,Target,Current\n")
        self.k_log_path = os.path.expanduser('~/ros2_ws/stiffness_log.csv')
        with open(self.k_log_path, "w") as k_log_file:
            k_log_file.write("Time,Error,Stiffness_k\n")

    # ---------- Callbacks ----------

    def joint_meas_callback(self, msg: Float32):
        """CHANGED: callback for measured joint position from hardware driver."""
        pos = float(msg.data)
        if not self.have_joint_measurement:
            # CHANGED: initialize prev_position AND command_deg the first time to avoid jumps
            self.prev_position = pos
            self.command_deg = pos        # CHANGED: sync command with actual joint on first reading
            self.freeze_origin_deg = pos  # CHANGED: reasonable default for FREEZE
            self.have_joint_measurement = True
        self.current_position = pos  # CHANGED: measured joint angle replaces simulated state

    def mode_callback(self, msg: String):
        requested = (msg.data or "").strip().upper()
        if requested not in ("NORMAL", "FREEZE", "SLEEP", "ZERO_POSITION"):
            self.get_logger().warn(f"[Patient] Unknown mode '{requested}' ignored.")
            return
        if requested == self.mode:
            return

        # Hygiene at entry
        # CHANGED: we now use current measured position as the base for command_deg when changing modes.
        self.command_deg = self.current_position
        self.command_vel_deg_s = 0.0

        if requested == "FREEZE":
            self.damping_c = self.freeze_damping_c
            self._freeze_hold_elapsed = 0.0
            self._freeze_stable_latch = False
            self._publish_bool(self.pub_freeze_stable, False)
            self.freeze_origin_deg = self.current_position     # CHANGED: remember hold position
        elif requested in ("SLEEP", "ZERO_POSITION"):
            self.damping_c = 0.7
            self._at_zero_hold_elapsed = 0.0
            self._at_zero_latch = False
            self._publish_bool(self.pub_at_zero, False)
            # command_deg already set to current_position above
        else:  # NORMAL
            self.damping_c = 0.4  # original baseline

        # reset INTERNAL command velocity between changing modes
        self.velocity = 0.0              # CHANGED: interpreted as internal command velocity
        self.prev_position = self.current_position
        self.vel_meas_deg_s = 0.0

        self.mode = requested
        self.get_logger().info(f"[Patient] Mode → {self.mode}")

    def target_callback(self, msg: Float32):
        self.target_position = float(msg.data)
        self.get_logger().info(f'[Patient] Received target: {self.target_position:.2f}°')

    # ---------- Main loop ----------

    def update_tick(self):
        # CHANGED: Do nothing until we have at least one real joint measurement from driver
        if not self.have_joint_measurement:
            return

        if self.mode == "NORMAL":
            self._step_normal_controller()       # CHANGED: controller-only, updates command_deg
        elif self.mode == "FREEZE":
            self._step_freeze_controller()       # CHANGED: controller-only, holds freeze_origin_deg
        elif self.mode in ("SLEEP", "ZERO_POSITION"):
            self._step_sleep_like_controller()   # controller-only, moves command_deg → 0
        else:
            self._step_freeze_controller()

        # CHANGED: Soft clamp is now applied to COMMAND (what we ask the motor to do),
        #          not to the simulated current_position.
        if self.mode != "NORMAL":
            self.command_deg = self._cap(self.command_deg, self.angle_min_deg, self.angle_max_deg)

        # ---- measured velocity (deg/s) for stability checks ----
        # CHANGED: derived from measured joint position instead of internal simulation.
        self.vel_meas_deg_s = (self.current_position - self.prev_position) / self.dt
        self.prev_position = self.current_position

        # CHANGED: Publish COMMAND position instead of simulated joint position.
        self.pub_command.publish(Float32(data=float(self.command_deg)))

        # Report-backs (same logic, using measured position/velocity)
        self._update_freeze_status()
        self._update_zero_status()

        # CSV log (unchanged structure: target vs CURRENT MEASURED joint angle)
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        try:
            with open(self.log_path, "a") as log_file:
                log_file.write(f"{now},{self.target_position:.2f},{self.current_position:.2f}\n")
        except Exception:
            pass

    # ---------- Mode implementations (controller form) ----------

    def _step_normal_controller(self):
        """CHANGED: NORMAL as a pure controller (no internal plant integration)."""
        # Error in deg based on measured joint position
        error_signed = self.target_position - self.current_position
        error = abs(error_signed)

        # Adaptive stiffness (unchanged logic)
        if error > self.high_error_threshold:
            self.spring_k = min(self.spring_k + self.k_step, self.k_max)
            self.get_logger().info(
                f'[Adaptive] 🔺 High error → Increasing stiffness: k = {self.spring_k:.3f}'
            )
        elif error < self.low_error_threshold:
            self.spring_k = max(self.spring_k - self.k_step, self.k_min)
            self.get_logger().info(
                f'[Adaptive] 🔻 Low error → Decreasing stiffness: k = {self.spring_k:.3f}'
            )

        # Log k adaptation (unchanged)
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        try:
            with open(self.k_log_path, "a") as k_log_file:
                k_log_file.write(f"{now},{error:.2f},{self.spring_k:.3f}\n")
        except Exception:
            pass

        # Virtual spring–damper toward target (deg space), applied to COMMAND dynamics
        spring_term = -self.spring_k * (self.current_position - self.target_position)
        damping_term = -self.damping_c * self.velocity   # CHANGED: velocity is internal command velocity
        total = spring_term + damping_term

        # CHANGED: integrate internal command velocity & command position (same discrete form as original)
        self.velocity += total      # internal "command acceleration"
        self.command_deg += self.velocity  # new command angle

        # Console log (still shows measured position)
        self.get_logger().info(
            f'[Patient] 📐 Meas Pos: {self.current_position:.2f}° | Cmd: {self.command_deg:.2f}° | '
            f'Target: {self.target_position:.2f}° | Error: {error:.2f}°'
        )

    def _step_freeze_controller(self):
        """CHANGED: FREEZE as controller — hold around freeze_origin_deg."""
        # CHANGED: simply command the freeze_origin_deg and kill internal velocity.
        self.command_deg = self.freeze_origin_deg
        self.velocity = 0.0

    def _step_sleep_like_controller(self):
        """CHANGED: Return to 0° using SOFT or CREEP, via command trajectory only."""
        # CHANGED: decide SOFT/CREEP based on measured angle
        abs_ang = abs(self.current_position)
        if abs_ang >= self.creep_threshold_deg:
            vmax = self.vmax_creep_deg_s
            amax = self.amax_creep_deg_s2
        else:
            vmax = self.vmax_soft_deg_s
            amax = self.amax_soft_deg_s2

        goal = 0.0

        # Trapezoidal profile on the COMMAND setpoint (same idea as original)
        cmd_err = goal - self.command_deg
        v_des = self._cap(cmd_err / max(self.dt, 1e-6), -vmax, vmax)
        dv_max = amax * self.dt
        v_next = self._cap(
            v_des,
            self.command_vel_deg_s - dv_max,
            self.command_vel_deg_s + dv_max
        )
        self.command_vel_deg_s = v_next
        self.command_deg += self.command_vel_deg_s * self.dt

        # Prevent tiny overshoot around zero
        if (goal - self.command_deg) * self.command_vel_deg_s < 0.0:
            self.command_deg = goal
            self.command_vel_deg_s = 0.0

        # CHANGED: removed extra spring–damper integration on command_deg here.
        # The Dynamixel's internal position controller will track command_deg; we only need the trajectory.

    # ---------- Report-back helpers ----------

    def _update_freeze_status(self):
        if self.mode != "FREEZE":
            if self._freeze_stable_latch:
                self._freeze_stable_latch = False
                self._publish_bool(self.pub_freeze_stable, False)
            self._freeze_hold_elapsed = 0.0
            return

        # Use measured velocity in deg/s (consistent across modes)
        if abs(self.vel_meas_deg_s) < self.vel_stable_deg_s:
            self._freeze_hold_elapsed += self.dt
        else:
            self._freeze_hold_elapsed = 0.0

        if (not self._freeze_stable_latch) and (self._freeze_hold_elapsed >= self.hold_time_s):
            self._freeze_stable_latch = True
            self._publish_bool(self.pub_freeze_stable, True)
            self.pub_freeze_done.publish(Empty())

    def _update_zero_status(self):
        # Use measured velocity in deg/s (consistent across modes)
        at_zero_now = (
            abs(self.current_position) <= self.zero_tol_deg
        ) and (
            abs(self.vel_meas_deg_s) < self.vel_stable_deg_s
        )

        if self.mode in ("SLEEP", "ZERO_POSITION"):
            if at_zero_now:
                self._at_zero_hold_elapsed += self.dt
            else:
                self._at_zero_hold_elapsed = 0.0

            if (not self._at_zero_latch) and (self._at_zero_hold_elapsed >= self.hold_time_s):
                self._at_zero_latch = True
                self._publish_bool(self.pub_at_zero, True)
                self.pub_zero_reached.publish(Empty())
                # Auto transition to FREEZE for safe handoff
                self.mode_callback(String(data="FREEZE"))
        else:
            if self._at_zero_latch != at_zero_now:
                self._at_zero_latch = at_zero_now
                self._publish_bool(self.pub_at_zero, at_zero_now)
            self._at_zero_hold_elapsed = 0.0

    # ---------- Utils ----------

    @staticmethod
    def _cap(x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    def _publish_bool(self, pub, value: bool):
        pub.publish(Bool(data=bool(value)))


def main(args=None):
    rclpy.init(args=args)
    node = PatientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
