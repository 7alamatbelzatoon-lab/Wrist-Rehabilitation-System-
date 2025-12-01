#!/usr/bin/env python3
# Motion anomaly detector (level-style) for wrist telerehab v1
# - Inputs (subscribe):
#     /patient/target_position : Float32 (deg, 1 Hz typical)
#     /patient/joint_position  : Float32 (deg, 10 Hz typical)
#     /patient/control_mode    : String
#     (optional) /patient/effort_meas : Float32 (N·m, signed)
# - Outputs (publish, TRANSIENT_LOCAL):
#     /safety/motion/advisory : Bool
#     /safety/motion/latched  : Bool
#     /safety/motion/cause    : String   -> "SOFT=a,b;HARD=X;INFO=flag1,flag2"
#
# Design notes:
#   • Level-style publishing (no internal latching). Orchestrator sequences safety behavior.
#   • Reality (joint/effort) is authoritative; target is contextual for tracking error.
#   • Symmetric thresholds for ± angles. Hard priority: LIMIT > SENSOR_INVALID > FORCE.
#   • 0.5 s grace after each target step (1 Hz target) before tracking-error evaluation.

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32, String, Bool

# ---------------- QoS Profiles ----------------
qos_sensor = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
qos_level_pub = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,  # latched level
)

# ---------------- Built-in constants (TUNE IN CODE) ----------------
# Limits (degrees)
WORKING_MAX_DEG = 60.0     # routine ops range (info only)
NEAR_LIMIT_DEG  = 65.0     # advisory band start
HARD_LIMIT_DEG  = 70.0     # immediate hard trip (first sample)
# Fast-trip on large target–joint mismatch (ALL MODES)
FAST_ERR_TRIP_DEG  = 15.0   # trip immediately if |target - joint| ≥ 20°
FAST_ERR_CLEAR_DEG = 12.0   # clear when error falls ≤ 14° (hysteresis)


# Drift in FREEZE (advisory-only)
DRIFT_WINDOW_S   = 0.5     # integrate |Δpos| over this window
DRIFT_ACCUM_DEG  = 1.0     # trigger if accumulated |Δpos| >= this
DRIFT_GRACE_S    = 0.3     # grace after entering FREEZE to avoid transient trips

# Tracking error in NORMAL (advisory)
TRACK_ERR_DEG    = 10.0    # |target - joint| threshold
TRACK_ERR_S      = 0.2     # must persist this long
TRACK_ERR_GRACE_AFTER_STEP_S = 0.3  # ignore ADV_TRACK_ERR for this long after a target change
TARGET_STEP_EPS_DEG = 0.25          # change >= this counts as a "step" (avoid jitter)

# Sensor health (joint stream)
SENSOR_SOFT_GAP_S = 0.3    # advisory if last joint sample older than this
SENSOR_HARD_GAP_S = 0.8    # hard if older than this (or NaN/inf on sample)

# Effort checks (activate only if /patient/effort_meas is alive)
# NOTE: Placeholder conservative thresholds — tune with hardware bench tests.
FORCE_SOFT_NM       = 1000.0  # effectively disabled until tuned
FORCE_HARD_NM       = 1500.0  # effectively disabled until tuned
FORCE_SOFT_MIN_S    = 0.25
FORCE_HARD_MIN_S    = 0.15
FORCE_NEAR_LIMIT_BIAS = 0.80  # lower thresholds by 20% when |pos| >= NEAR_LIMIT_DEG
FORCE_NEAR_LIMIT_WIN  = 0.50  # shorten duration by 50% near limit

# Timing
TICK_HZ       = 10.0
KEEPALIVE_HZ  = 0.0     # 0 disables periodic re-publish

# Cause tags
SOFT_ADV_NEAR_LIMIT   = 'ADV_NEAR_LIMIT'
SOFT_ADV_DRIFT        = 'ADV_DRIFT'
SOFT_ADV_TRACK_ERR    = 'ADV_TRACK_ERR'
SOFT_ADV_SENSOR_RATE  = 'ADV_SENSOR_RATE'
SOFT_ADV_FORCE        = 'ADV_FORCE'
SOFT_ADV_LARGE_ERR    = 'ADV_LARGE_ERR'


HARD_LIMIT            = 'HARD_LIMIT'
HARD_SENSOR_INVALID   = 'HARD_SENSOR_INVALID'
HARD_FORCE            = 'HARD_FORCE'

INFO_DATA_PROXY       = 'DATA_PROXY'  # effort absent: pure position proxy mode


class MotionDetector(Node):
    def __init__(self):
        super().__init__("motion_detector")

        # ---------------- Publishers ----------------
        self.pub_adv  = self.create_publisher(Bool,   '/safety/motion/advisory', qos_level_pub)
        self.pub_lat  = self.create_publisher(Bool,   '/safety/motion/latched',  qos_level_pub)
        self.pub_cause= self.create_publisher(String, '/safety/motion/cause',    qos_level_pub)

        # ---------------- Subscriptions ----------------
        self.create_subscription(Float32, '/patient/target_position', self._on_target, qos_sensor)
        self.create_subscription(Float32, '/patient/joint_position',  self._on_joint,  qos_sensor)
        self.create_subscription(String,  '/patient/control_mode',    self._on_mode,   qos_sensor)
        # Optional effort; if never received, we stay in proxy mode and add INFO flag.
        self.create_subscription(Float32, '/patient/effort_meas',     self._on_effort, qos_sensor)

        # ---------------- Internal state ----------------
        self._now = time.monotonic
        self.start_time = self._now()   # for clean startup gap handling
        self.tick_dt = 1.0 / TICK_HZ

        # Latest samples
        self.target_deg = None
        self.prev_target_deg = None
        self.last_target_change_time = None

        self.joint_deg  = None
        self.last_joint_time = None
        self.joint_invalid_this_tick = False

        self.mode = None  # 'NORMAL', 'FREEZE', 'SLEEP', 'ZERO_POSITION', ...

        # Drift accumulator (sum |Δpos| over sliding window)
        self.last_pos_for_drift = None
        self.drift_deque = deque()  # elements: (timestamp, abs_delta_deg)
        self.freeze_enter_time = None

        # Tracking error
        self.err_above_since = None
        self.fast_err_active = False


        # Effort
        self.effort_nm = None
        self.last_effort_time = None
        self.effort_soft_since = None
        self.effort_hard_since = None

        # Last published (avoid redundant pubs)
        self.last_pub_adv = None
        self.last_pub_lat = None
        self.last_pub_cause = None
        self.last_keepalive_time = self._now()

        # Kickoff: publish initial cleared state
        self._publish_levels_and_cause(
            advisory=False,
            latched=False,
            cause_str=self._format_cause([], "", self._info_flags()),
            force=True
        )

        # Main timer
        self.timer = self.create_timer(self.tick_dt, self._tick)

        self.get_logger().info('motion_detector started (level-style, 10 Hz)')

    # ---------------- Subscription callbacks ----------------
    def _on_target(self, msg: Float32):
        now = self._now()
        val = float(msg.data)
        # Detect meaningful step (avoid jitter)
        if (self.prev_target_deg is None) or (abs(val - self.prev_target_deg) >= TARGET_STEP_EPS_DEG):
            self.last_target_change_time = now
        self.prev_target_deg = val
        self.target_deg = val

    def _on_joint(self, msg: Float32):
        now = self._now()
        val = float(msg.data)

        # Validate on arrival
        if not math.isfinite(val):
            # mark invalid this tick; timer will convert to HARD_SENSOR_INVALID
            self.joint_invalid_this_tick = True
            # do not update last time/value to avoid hiding gaps
            return

        # Valid sample -> update time and drift accumulator
        if (self.joint_deg is not None) and (self.last_pos_for_drift is not None):
            delta = abs(val - self.last_pos_for_drift)
            if delta > 0.0:
                self.drift_deque.append((now, delta))
        self.last_pos_for_drift = val
        self.joint_deg = val
        self.last_joint_time = now

    def _on_mode(self, msg: String):
        new_mode = (msg.data or "").strip().upper()
        if new_mode != self.mode:
            # Entering FREEZE -> start grace
            if new_mode == 'FREEZE':
                self.freeze_enter_time = self._now()
                # Reset drift accumulator baseline on entry
                self.last_pos_for_drift = self.joint_deg
                self.drift_deque.clear()
            self.mode = new_mode

    def _on_effort(self, msg: Float32):
        now = self._now()
        val = float(msg.data)
        if math.isfinite(val):
            self.effort_nm = val
            self.last_effort_time = now
        else:
            # treat invalid effort sample as simply "no effort data" this tick
            self.effort_nm = None

    # ---------------- Main tick ----------------
    def _tick(self):
        now = self._now()

        # --------- Sensor health (joint stream) ---------
        soft_sensor_rate = False
        hard_sensor_invalid = False

        # Invalid sample seen this tick?
        if self.joint_invalid_this_tick:
            hard_sensor_invalid = True
            self.joint_invalid_this_tick = False  # consume

        # Gap-based checks (clean startup: measure from start_time until first joint arrives)
        if self.last_joint_time is None:
            gap = now - self.start_time
        else:
            gap = now - self.last_joint_time

        if gap >= SENSOR_SOFT_GAP_S:
            soft_sensor_rate = True
        if gap >= SENSOR_HARD_GAP_S:
            hard_sensor_invalid = True

        # --------- Kinematic checks ---------
        soft_causes = []
        hard_cause = ""

        pos = self.joint_deg
        tgt = self.target_deg
        # --- Fast-trip: large target–joint mismatch (ALL MODES, no grace) ---
        err = None
        if (pos is not None) and (tgt is not None) and math.isfinite(pos) and math.isfinite(tgt):
            err = abs(tgt - pos)
            if self.fast_err_active:
                if err <= FAST_ERR_CLEAR_DEG:
                    self.fast_err_active = False
            else:
                if err >= FAST_ERR_TRIP_DEG:
                    self.fast_err_active = True
            if self.fast_err_active:
                soft_causes.append(SOFT_ADV_LARGE_ERR)


        # Near-limit & hard-limit (if we have a position)
        if pos is not None and math.isfinite(pos):
            apos = abs(pos)
            if apos >= NEAR_LIMIT_DEG:
                soft_causes.append(SOFT_ADV_NEAR_LIMIT)
            if apos >= HARD_LIMIT_DEG:
                hard_cause = self._pick_hard(hard_cause, HARD_LIMIT)

        # Drift in FREEZE (advisory-only)
        if (self.mode == 'FREEZE') and (pos is not None):
            # grace after entering FREEZE
            in_grace = (self.freeze_enter_time is not None) and ((now - self.freeze_enter_time) < DRIFT_GRACE_S)
            if not in_grace:
                # prune drift window, compute accumulated |Δpos|
                while self.drift_deque and (now - self.drift_deque[0][0]) > DRIFT_WINDOW_S:
                    self.drift_deque.popleft()
                accum = sum(d for _, d in self.drift_deque)
                if accum >= DRIFT_ACCUM_DEG:
                    soft_causes.append(SOFT_ADV_DRIFT)

        # Tracking error in NORMAL (advisory) with grace after target steps
        # --- Target–tracking checks in NORMAL ---
        if (self.mode == 'NORMAL') and (pos is not None) and (tgt is not None):
            err = abs(tgt - pos)


            # 4.b Existing moderated tracking error (grace + persistence)
            in_grace = (
                self.last_target_change_time is not None and
                (now - self.last_target_change_time) < TRACK_ERR_GRACE_AFTER_STEP_S
            )
            if in_grace:
                self.err_above_since = None
            else:
                if err >= TRACK_ERR_DEG:
                    if self.err_above_since is None:
                        self.err_above_since = now
                    if (now - self.err_above_since) >= TRACK_ERR_S:
                        soft_causes.append(SOFT_ADV_TRACK_ERR)
                else:
                    self.err_above_since = None
        else:
            # not in NORMAL or missing data → clear tracking-related memory
            self.err_above_since = None
            #self.fast_err_active = False

        

        # Sensor health flags
        if soft_sensor_rate:
            soft_causes.append(SOFT_ADV_SENSOR_RATE)
        if hard_sensor_invalid:
            hard_cause = self._pick_hard(hard_cause, HARD_SENSOR_INVALID)

        # --------- Effort checks (optional) ---------
        info_flags = self._info_flags()

        # Only enable effort logic if effort is alive (recent)
        effort_alive = (self.last_effort_time is not None) and ((now - self.last_effort_time) < SENSOR_HARD_GAP_S)
        if effort_alive and (self.effort_nm is not None):
            eff = abs(self.effort_nm)

            # Near-limit biasing
            near_limit = (pos is not None) and (abs(pos) >= NEAR_LIMIT_DEG)
            thr_soft = FORCE_SOFT_NM * (FORCE_NEAR_LIMIT_BIAS if near_limit else 1.0)
            thr_hard = FORCE_HARD_NM * (FORCE_NEAR_LIMIT_BIAS if near_limit else 1.0)
            win_soft = FORCE_SOFT_MIN_S * (FORCE_NEAR_LIMIT_WIN if near_limit else 1.0)
            win_hard = FORCE_HARD_MIN_S * (FORCE_NEAR_LIMIT_WIN if near_limit else 1.0)

            # Soft effort
            if eff >= thr_soft:
                if self.effort_soft_since is None:
                    self.effort_soft_since = now
                if (now - self.effort_soft_since) >= win_soft:
                    soft_causes.append(SOFT_ADV_FORCE)
            else:
                self.effort_soft_since = None

            # Hard effort
            if eff >= thr_hard:
                if self.effort_hard_since is None:
                    self.effort_hard_since = now
                if (now - self.effort_hard_since) >= win_hard:
                    hard_cause = self._pick_hard(hard_cause, HARD_FORCE)
            else:
                self.effort_hard_since = None
        else:
            # effort absent -> INFO flag already handled by _info_flags()
            self.effort_soft_since = None
            self.effort_hard_since = None

        # --------- Consolidate & publish ---------
        advisory = len(soft_causes) > 0
        latched  = (hard_cause != "")

        cause_str = self._format_cause(sorted(set(soft_causes)), hard_cause, info_flags)
        self._publish_levels_and_cause(advisory, latched, cause_str)

    # ---------------- Helpers ----------------
    def _info_flags(self):
        """Return list of informational flags; INFO never affects levels."""
        flags = []
        # If we have NOT seen recent effort, expose proxy-mode info
        if (self.last_effort_time is None) or ((self._now() - self.last_effort_time) >= SENSOR_HARD_GAP_S):
            flags.append(INFO_DATA_PROXY)
        return flags

    @staticmethod
    def _pick_hard(current: str, candidate: str) -> str:
        """Choose highest-priority hard cause. Priority: LIMIT > SENSOR_INVALID > FORCE."""
        if current == candidate or candidate == "":
            return current
        if current == "":
            return candidate
        priority = {HARD_LIMIT: 3, HARD_SENSOR_INVALID: 2, HARD_FORCE: 1}
        return current if priority.get(current, 0) >= priority.get(candidate, 0) else candidate

    @staticmethod
    def _format_cause(soft_list, hard_code: str, info_list):
        soft_part = ",".join(soft_list)
        info_part = ",".join(info_list)
        return f"SOFT={soft_part};HARD={hard_code};INFO={info_part}"

    def _publish_levels_and_cause(self, advisory: bool, latched: bool, cause_str: str, force: bool = False):
        # Keepalive control
        now = self._now()
        time_for_keepalive = False
        if KEEPALIVE_HZ > 0.0:
            period = 1.0 / KEEPALIVE_HZ
            if (now - self.last_keepalive_time) >= period:
                time_for_keepalive = True

        changed = (
            force or
            advisory != self.last_pub_adv or
            latched  != self.last_pub_lat or
            cause_str != self.last_pub_cause
        )

        if changed or time_for_keepalive:
            self.pub_adv.publish(Bool(data=advisory))
            self.pub_lat.publish(Bool(data=latched))
            self.pub_cause.publish(String(data=cause_str))
            self.last_pub_adv = advisory
            self.last_pub_lat = latched
            self.last_pub_cause = cause_str
            if time_for_keepalive:
                self.last_keepalive_time = now


def main(args=None):
    rclpy.init(args=args)
    node = MotionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
