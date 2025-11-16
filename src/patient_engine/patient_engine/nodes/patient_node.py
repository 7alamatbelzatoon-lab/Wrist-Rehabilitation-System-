#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool, Empty
import os
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Patient locomotion "dummy" executor (NORMAL timing == original)
# - Modes: NORMAL | FREEZE | SLEEP | ZERO_POSITION
# - NORMAL: exact old behavior (10 Hz; no dt scaling; no caps; original adaptive stiffness)
# - FREEZE: hold still (higher damping) + /patient/freeze_stable (level) & /patient/freeze/done (edge)
# - SLEEP/ZERO: return to 0° with SOFT vs CREEP based on |angle| (node-local), then auto FREEZE
# - Report-backs (patient is sole publisher):
#     /patient/freeze/done (Empty, edge)
#     /patient/zero_reached (Empty, edge)
#     /patient/freeze_stable (Bool, level)
#     /patient/at_zero (Bool, level)
# - Kept from original: /patient/joint_position + CSV logs + adaptive stiffness logic

#Define all quality of service profiles we need first for proper topic behaviors
qos_sensorData = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )


class PatientNode(Node):
    def __init__(self):
        super().__init__("patient_node")

        # Subscriptions (one publisher elsewhere)
        self.sub_target = self.create_subscription(Float32, '/patient/target_position', self.target_callback, qos_sensorData)
        self.sub_mode   = self.create_subscription(String,  '/patient/control_mode',   self.mode_callback,   10)

        # Publishers (patient is sole publisher)
        self.pub_joint         = self.create_publisher(Float32, '/patient/joint_position', 10)
        self.pub_freeze_done   = self.create_publisher(Empty,   '/patient/freeze/done',    10)
        self.pub_zero_reached  = self.create_publisher(Empty,   '/patient/zero_reached',   10)
        self.pub_freeze_stable = self.create_publisher(Bool,    '/patient/freeze_stable',  10)
        self.pub_at_zero       = self.create_publisher(Bool,    '/patient/at_zero',        10)

        # Loop timing — EXACTLY like the old node 10Hz
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.update_tick)
        self.get_logger().info("Patient Node Initiated (NORMAL timing matches original).")

        # Core state (degrees & deg/s like the original)
        self.mode = "FREEZE"           # safe default; orchestrator switches
        self.target_position = 0.0     # deg (from MQTT)
        self.current_position = 0.0    # deg
        self.velocity = 0.0            # deg/s (in NORMAL this acts like deg-per-tick; kept as-is)

        # For SLEEP/ZERO: slewed command to 0°
        self.command_deg = 0.0
        self.command_vel_deg_s = 0.0

        # Measured velocity (deg/s) for stability/zero checks (unit-consistent across modes)
        self.prev_position = self.current_position
        self.vel_meas_deg_s = 0.0

        # === Adaptive stiffness (UNCHANGED) ===
        self.spring_k = 0.225
        self.damping_c = 0.4  # baseline damping (NORMAL)
        self.k_min = 0.05
        self.k_max = 0.5
        self.k_step = 0.01
        self.low_error_threshold  = 1.0   # deg
        self.high_error_threshold = 5.0   # deg

        # === Profiles / FREEZE parameters (tunable later) ===
        self.angle_min_deg = -60.0
        self.angle_max_deg = +60.0
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

    def mode_callback(self, msg: String):
        requested = (msg.data or "").strip().upper()
        if requested not in ("NORMAL", "FREEZE", "SLEEP", "ZERO_POSITION"):
            self.get_logger().warn(f"[Patient] Unknown mode '{requested}' ignored.")
            return
        if requested == self.mode:
            return

        # Hygiene at entry
        self.command_deg = self.current_position
        self.command_vel_deg_s = 0.0

        if requested == "FREEZE":
            self.damping_c = self.freeze_damping_c
            self._freeze_hold_elapsed = 0.0
            self._freeze_stable_latch = False
            self._publish_bool(self.pub_freeze_stable, False)
        elif requested in ("SLEEP", "ZERO_POSITION"):
            self.damping_c = 0.7
            self._at_zero_hold_elapsed = 0.0
            self._at_zero_latch = False
            self._publish_bool(self.pub_at_zero, False)
        else:  # NORMAL
            self.damping_c = 0.4  # original baseline

        self.mode = requested
        self.get_logger().info(f"[Patient] Mode → {self.mode}")

    def target_callback(self, msg: Float32):
        self.target_position = float(msg.data)
        self.get_logger().info(f'[Patient] Received target: {self.target_position:.2f}°')

    # ---------- Main loop ----------

    def update_tick(self):
        if self.mode == "NORMAL":
            self._step_normal_original()       # EXACT old behavior
        elif self.mode == "FREEZE":
            self._step_freeze()
        elif self.mode in ("SLEEP", "ZERO_POSITION"):
            self._step_sleep_like()
        else:
            self._step_freeze()

        # Soft clamp ONLY outside NORMAL (keep NORMAL identical to before)
        if self.mode != "NORMAL":
            self.current_position = self._cap(self.current_position, self.angle_min_deg, self.angle_max_deg)

        # ---- measured velocity (deg/s) for stability checks (mode-agnostic, harmless in NORMAL) ----
        self.vel_meas_deg_s = (self.current_position - self.prev_position) / self.dt
        self.prev_position = self.current_position

        # Publish joint angle
        self.pub_joint.publish(Float32(data=float(self.current_position)))

        # Report-backs
        self._update_freeze_status()
        self._update_zero_status()

        # CSV log (unchanged)
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        try:
            with open(self.log_path, "a") as log_file:
                log_file.write(f"{now},{self.target_position:.2f},{self.current_position:.2f}\n")
        except Exception:
            pass

    # ---------- Mode implementations ----------

    def _step_normal_original(self):
        """EXACT original NORMAL behavior (no dt scaling, no caps)."""
        # Error in deg
        error = abs(self.target_position - self.current_position)

        # Adaptive stiffness (unchanged)
        if error > self.high_error_threshold:
            self.spring_k = min(self.spring_k + self.k_step, self.k_max)
            self.get_logger().info(f'[Adaptive] 🔺 High error → Increasing stiffness: k = {self.spring_k:.3f}')
        elif error < self.low_error_threshold:
            self.spring_k = max(self.spring_k - self.k_step, self.k_min)
            self.get_logger().info(f'[Adaptive] 🔻 Low error → Decreasing stiffness: k = {self.spring_k:.3f}')

        # Log k adaptation (unchanged)
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        try:
            with open(self.k_log_path, "a") as k_log_file:
                k_log_file.write(f"{now},{error:.2f},{self.spring_k:.3f}\n")
        except Exception:
            pass

        # Virtual spring–damper toward target (deg space)
        spring_force = -self.spring_k * (self.current_position - self.target_position)
        damping_force = -self.damping_c * self.velocity
        total_force = spring_force + damping_force

        # *** ORIGINAL INTEGRATION (no dt scaling, no caps) ***
        self.velocity += total_force      # Assume unit mass
        self.current_position += self.velocity

        # Console log (unchanged)
        self.get_logger().info(
            f'[Patient] 📐 Position: {self.current_position:.2f}° | Target: {self.target_position:.2f}° | Error: {error:.2f}°'
        )

    def _step_freeze(self):
        """Hold still: higher damping to kill residual velocity (no adaptation)."""
        spring_force = 0.0
        damping_force = -self.damping_c * self.velocity
        total_accel = spring_force + damping_force
        # integrate with dt so it’s gentle (FREEZE is new, not part of original behavior)
        self.velocity += total_accel * self.dt
        self.current_position += self.velocity * self.dt

    def _step_sleep_like(self):
        """Return to 0° using SOFT or CREEP based solely on |current angle|."""
        abs_ang = abs(self.current_position)
        if abs_ang >= self.creep_threshold_deg:
            vmax = self.vmax_creep_deg_s
            amax = self.amax_creep_deg_s2
        else:
            vmax = self.vmax_soft_deg_s
            amax = self.amax_soft_deg_s2

        goal = 0.0

        # Trapezoid on the command setpoint
        cmd_err = goal - self.command_deg
        v_des = self._cap(cmd_err / max(self.dt, 1e-6), -vmax, vmax)
        dv_max = amax * self.dt
        v_next = self._cap(v_des, self.command_vel_deg_s - dv_max, self.command_vel_deg_s + dv_max)
        self.command_vel_deg_s = v_next
        self.command_deg += self.command_vel_deg_s * self.dt

        # Prevent tiny overshoot around zero
        if (goal - self.command_deg) * self.command_vel_deg_s < 0.0:
            self.command_deg = goal
            self.command_vel_deg_s = 0.0

        # Track the command with spring–damper (no adaptation here)
        spring_force = -self.spring_k * (self.current_position - self.command_deg)
        damping_force = -self.damping_c * self.velocity
        total_accel = spring_force + damping_force
        # integrate with dt & caps for gentle movement
        total_accel = self._cap(total_accel, -amax, amax)
        self.velocity = self._cap(self.velocity + total_accel * self.dt, -vmax, vmax)
        self.current_position += self.velocity * self.dt

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
        at_zero_now = (abs(self.current_position) <= self.zero_tol_deg) and (abs(self.vel_meas_deg_s) < self.vel_stable_deg_s)

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
