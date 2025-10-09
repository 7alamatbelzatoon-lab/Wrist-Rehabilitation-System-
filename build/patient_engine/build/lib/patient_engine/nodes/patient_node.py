#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool
import os
from datetime import datetime
from enum import Enum


class SafeState(Enum):
    IDLE = 0
    HOLD_SAFE = 1
    CREEP_TO_SAFE = 2   # very gentle nudge toward neutral
    SOFT_RETURN = 3     # smooth return to zero


class PatientNode(Node):
    def __init__(self):
        super().__init__("patient_node")

        # --- I/O ---
        self.sub_target = self.create_subscription(Float32, '/patient/target_position', self.target_callback, 10)
        self.pub_joint  = self.create_publisher(Float32, '/patient/joint_position', 10)

        # Control from anomaly handler
        self.sub_mode   = self.create_subscription(String, '/patient/control_mode', self.mode_callback, 10)
        self.sub_freeze = self.create_subscription(Bool,   '/patient/freeze',       self.freeze_callback, 10)

        # --- Plant state ---
        self.target_position  = 0.0
        self.current_position = 0.0
        self.velocity         = 0.0

        # --- Baseline (normal) impedance ---
        self.spring_k  = 0.225
        self.damping_c = 0.4

        # --- Adaptive stiffness (normal mode only) ---
        self.k_min = 0.05
        self.k_max = 0.5
        self.k_step = 0.01
        self.low_error_threshold  = 1.0   # deg
        self.high_error_threshold = 5.0   # deg

        # --- Safe-mode parameters ---
        self.neutral_deg = 0.0

        # HOLD_SAFE (Stage A)
        self.t_release_s = 2.5            # wait in HOLD before moving
        self.hold_d      = 0.8            # increase damping during HOLD

        # CREEP_TO_SAFE (Stage A+)
        self.creep_allow_window_deg = 90.0  # allow creep from anywhere within ±90°
        self.creep_k           = 0.02       # very soft spring toward neutral
        self.creep_vmax_deg_s  = 6.0        # comfortable but safe

        # SOFT_RETURN (Stage B)
        self.safe_zone_deg     = 40.0       # if |pos-0| ≤ 40°, go to soft return
        self.soft_k            = 0.08
        self.soft_vmax_deg_s   = 12.0
        self.soft_done_eps     = 2.5        # consider done within ±2.5°

        # Mode & FSM
        self.mode = 'normal'    # 'normal' | 'passive' | 'estop'  (from anomaly handler)
        self.freeze = False     # Bool (from anomaly handler)
        self.safe_state = SafeState.IDLE
        self.state_enter_time = None

        # --- Timer ---
        self.dt = 0.1
        self.create_timer(self.dt, self.update_position)

        # --- Logs ---
        self.get_logger().info("Patient Node Initiated.")
        self.log_path   = os.path.expanduser('~/ros2_ws/adaptive_log.csv')
        self.k_log_path = os.path.expanduser('~/ros2_ws/stiffness_log.csv')
        # richer adaptive log that matches actual control
        with open(self.log_path, "w") as f:
            f.write("Time,Target,EffTarget,Current,Mode,SafeState,k_used,c_used,vmax_s,ErrToDoctor,ErrToEff\n")
        # keep the stiffness log but include both error views + k_used
        with open(self.k_log_path, "w") as f:
            f.write("Time,ErrorToDoctor,ErrorToEff,k_used\n")

    # ------------------- Callbacks -------------------
    def target_callback(self, msg: Float32):
        self.target_position = float(msg.data)
        self.get_logger().info(f'[Patient] Received target: {self.target_position:.2f}°')

    def mode_callback(self, msg: String):
        new_mode = msg.data.strip()
        if new_mode != self.mode:
            self.get_logger().warn(f'[Patient] mode: {self.mode} -> {new_mode}')
            self.mode = new_mode
            # Reset FSM entry time so timers start fresh on mode changes
            self.state_enter_time = None

    def freeze_callback(self, msg: Bool):
        self.freeze = bool(msg.data)
        self.get_logger().info(f'[Patient] freeze={self.freeze}')

    # ------------------- FSM Helpers -------------------
    def _enter_state(self, st: SafeState):
        # idempotent entry
        if self.safe_state == st:
            return
        self.safe_state = st
        self.state_enter_time = self.get_clock().now()
        self.get_logger().warn(f'[SafeMode] -> {st.name}')
        # freeze velocity on HOLD to prevent drift
        if st == SafeState.HOLD_SAFE:
            self.velocity = 0.0

    def _time_in_state(self) -> float:
        if self.state_enter_time is None:
            return 0.0
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9

    # ------------------- Main control loop -------------------
    def update_position(self):
        # Decide whether we’re in safe mode or normal mode
        if self.mode == 'normal' and not self.freeze:
            # Leave safe FSM if active
            if self.safe_state != SafeState.IDLE:
                self._enter_state(SafeState.IDLE)

            # ----- NORMAL ADAPTIVE CONTROL -----
            error = abs(self.target_position - self.current_position)
            # adapt stiffness
            if error > self.high_error_threshold:
                self.spring_k = min(self.spring_k + self.k_step, self.k_max)
                self.get_logger().info(f'[Adaptive] 🔺 High error → Increasing stiffness: k = {self.spring_k:.3f}')
            elif error < self.low_error_threshold:
                self.spring_k = max(self.spring_k - self.k_step, self.k_min)
                self.get_logger().info(f'[Adaptive] 🔻 Low error → Decreasing stiffness: k = {self.spring_k:.3f}')

            # spring-damper control toward target
            k = self.spring_k
            c = self.damping_c
            eff_target = self.target_position
            vmax_s = None  # deg/s; no clamp in normal mode

        else:
            # ----- SAFE FSM -----
            k, c, eff_target, vmax_s = self._tick_safe_mode()  # vmax_s is in deg/s

        # ---- begin: logging locals (matches what controller uses) ----
        k_used = k
        c_used = c
        eff_tgt = eff_target
        vmax_s_used = vmax_s if vmax_s is not None else -1.0
        err_to_doctor = abs(self.target_position - self.current_position)
        err_to_eff    = abs(eff_tgt - self.current_position)
        # ---- end: logging locals ----

        # ------------- Integrate dynamics -------------
        spring_force  = -k * (self.current_position - eff_target)
        damping_force = -c * self.velocity
        total_force   = spring_force + damping_force

        # (Simple discrete integration; dt already in timer)
        self.velocity += total_force

        # clamp velocity if requested (safe modes)
        if vmax_s is not None:
            vcap_tick = vmax_s * self.dt  # convert deg/s → deg/tick
            if self.velocity > vcap_tick:  self.velocity = vcap_tick
            if self.velocity < -vcap_tick: self.velocity = -vcap_tick

        self.current_position += self.velocity

        # ------------- Publish & Log -------------
        out = Float32(); out.data = float(self.current_position)
        self.pub_joint.publish(out)

        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        # richer adaptive log
        with open(self.log_path, "a") as f:
            f.write(f"{now},{self.target_position:.2f},{eff_tgt:.2f},{self.current_position:.2f},"
                    f"{self.mode},{self.safe_state.name},{k_used:.3f},{c_used:.3f},{vmax_s_used:.2f},"
                    f"{err_to_doctor:.2f},{err_to_eff:.2f}\n")

        # keep stiffness log with clearer context
        with open(self.k_log_path, "a") as f:
            f.write(f"{now},{err_to_doctor:.2f},{err_to_eff:.2f},{k_used:.3f}\n")

        # Progression checks inside safe modes (done after integration)
        if self.safe_state == SafeState.SOFT_RETURN:
            if abs(self.current_position - self.neutral_deg) <= self.soft_done_eps:
                self.get_logger().info('[SafeMode] SOFT_RETURN complete → parked near neutral')

    # ------------------- Safe FSM core -------------------
    def _tick_safe_mode(self):
        """
        Returns (k, c, eff_target, vmax_s) where vmax_s is in deg/s
        """
        # Enter HOLD_SAFE if we just got here
        if self.safe_state == SafeState.IDLE:
            self._enter_state(SafeState.HOLD_SAFE)

        # HOLD_SAFE: freeze at last pose with higher damping, keep current k small
        if self.safe_state == SafeState.HOLD_SAFE:
            if self.state_enter_time is None:
                self.state_enter_time = self.get_clock().now()

            # Increase damping, soften spring a bit
            c = max(self.damping_c, self.hold_d)
            k = min(self.spring_k, 0.05)  # soften grip
            eff_target = self.current_position  # hold here
            vmax_s = 0.0  # deg/s: no motion (pure hold)

            # After timeout, decide next
            dt = self._time_in_state()
            # ADVANCE after the delay for BOTH passive (L2) and estop (L3)
            if dt >= self.t_release_s:
                dist = abs(self.current_position - self.neutral_deg)
                if dist <= self.safe_zone_deg:
                    self._enter_state(SafeState.SOFT_RETURN)
                elif dist <= self.creep_allow_window_deg:
                    self._enter_state(SafeState.CREEP_TO_SAFE)
                # else stay in HOLD if way outside
            return k, c, eff_target, vmax_s

        # CREEP_TO_SAFE: tiny bias toward neutral with strict low vmax
        if self.safe_state == SafeState.CREEP_TO_SAFE:
            c = max(self.damping_c, self.hold_d)
            k = min(self.spring_k, self.creep_k)
            eff_target = self.neutral_deg
            vmax_s = self.creep_vmax_deg_s      # deg/s
            # Transition to SOFT_RETURN once inside safe zone
            if abs(self.current_position - self.neutral_deg) <= self.safe_zone_deg:
                self._enter_state(SafeState.SOFT_RETURN)
            return k, c, eff_target, vmax_s

        # SOFT_RETURN: stronger but still gentle return to neutral
        if self.safe_state == SafeState.SOFT_RETURN:
            c = max(self.damping_c, self.hold_d)
            k = max(self.spring_k, self.soft_k)
            eff_target = self.neutral_deg
            vmax_s = self.soft_vmax_deg_s       # deg/s
            return k, c, eff_target, vmax_s

        # Fallback (shouldn’t happen)
        return self.spring_k, self.damping_c, self.target_position, None


def main(args=None):
    rclpy.init(args=args)
    node = PatientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
