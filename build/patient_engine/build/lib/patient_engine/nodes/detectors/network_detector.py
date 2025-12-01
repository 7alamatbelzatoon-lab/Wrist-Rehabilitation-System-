#!/usr/bin/env python3
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Bool, Float32


class NetworkDetector(Node):
    """
    Level-style network anomaly detector for wrist rehab (1 Hz RTT input).
    - Subscribes: /patient_network/latency_ms (Float32, RTT)
    - Publishes:
        /safety/network/advisory : Bool         (True while any soft condition active)
        /safety/network/latched  : Bool         (True while any hard condition active)
        /safety/network/cause    : String       (SOFT=<comma,...>;HARD=<code>)
    """

    def __init__(self):
        super().__init__('network_detector')

        # ---------------- QoS ----------------
        self.qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.qos_level = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ---------------- Parameters (tunable defaults) ----------------
        gp = self.declare_parameter  # shortcut
        self.assume_input_is_rtt = gp('assume_input_is_rtt', True).value

        # Soft thresholds (advisory)
        self.soft_latency_ms = float(gp('soft_latency_ms', 120.0).value)           # S1
        self.soft_spike_ms = float(gp('soft_spike_ms', 200.0).value)               # S2
        self.soft_jitter_step_ms = float(gp('soft_jitter_step_ms', 60.0).value)    # S4
        self.soft_clear_consecutive = int(gp('soft_clear_consecutive', 2).value)   # clear after N healthy samples
        self.gap_soft_s = float(gp('gap_soft_s', 1.3).value)                       # S3 (rate degrade)

        # Hard thresholds (latched level)
        self.disconnect_timeout_s = float(gp('disconnect_timeout_s', 3.0).value)   # H1
        self.hard_latency_ms = float(gp('hard_latency_ms', 250.0).value)           # H2
        self.hard_latency_consecutive = int(gp('hard_latency_consecutive', 2).value)
        self.flap_gap_s = float(gp('flap_gap_s', 1.3).value)                       # H3
        self.flap_count = int(gp('flap_count', 2).value)
        self.flap_window_s = float(gp('flap_window_s', 10.0).value)
        self.fast_trip_single_hard_ms = float(gp('fast_trip_single_hard_ms', 350.0).value)
        self.fast_trip_enabled = bool(gp('fast_trip_enabled', False).value)

        # Keepalive re-publish rate (0 disables periodic re-publish)
        self.keepalive_hz = float(gp('keepalive_hz', 0.0).value)

        # ---------------- Publishers ----------------
        self.pub_adv = self.create_publisher(Bool, '/safety/network/advisory', self.qos_level)
        self.pub_lat = self.create_publisher(Bool, '/safety/network/latched', self.qos_level)
        self.pub_cause = self.create_publisher(String, '/safety/network/cause', self.qos_level)

        # ---------------- Subscriptions ----------------
        self.create_subscription(Float32, '/patient_network/latency_ms', self._on_latency, self.qos_sensor)

        # ---------------- Internal state ----------------
        self.last_sample_time = None       # time.monotonic() of last received sample
        self.prev_oneway_ms = None         # previous one-way latency for jitter step
        self.soft_healthy_consec = 0       # consecutive healthy samples (for clear)
        self.soft_latency_consec = 0       # S1 consecutive high-lat samples
        self.hard_latency_consec = 0       # H2 consecutive severe-lat samples
        self.soft_jitter_consec = 0        # S4 consecutive big step diffs

        # Soft flags (level)
        self.s_adv_soft_lat = False
        self.s_adv_spike = False
        self.s_adv_rate = False
        self.s_adv_jitter = False

        # Hard flags (level)
        self.h_disc = False
        self.h_lat = False
        self.h_flap = False
        self.gap_events = deque()  # timestamps of gaps > flap_gap_s within flap_window_s

        # Last-published values (to avoid redundant pubs)
        self.last_pub_adv = None
        self.last_pub_lat = None
        self.last_pub_cause = None
        self.last_keepalive_time = 0.0

        # Publish initial cleared state immediately (TRANSIENT_LOCAL helps late joiners)
        self._publish_levels_and_cause(advisory=False, latched=False, cause_str=self._format_cause([], ""))

        # Timer to monitor gaps & periodic publish
        self.timer = self.create_timer(0.1, self._tick)  # 10 Hz

        self.get_logger().info('network_detector started (level-style, 1 Hz friendly)')

    # ---------------- Helpers ----------------
    def _now(self) -> float:
        return time.monotonic()

    def _format_cause(self, soft_list, hard_code: str) -> str:
        # soft_list: list[str], e.g., ["ADV_SOFT_LAT","ADV_JITTER"]
        soft_part = ",".join(soft_list)
        return f"SOFT={soft_part};HARD={hard_code}"

    def _publish_levels_and_cause(self, advisory: bool, latched: bool, cause_str: str, force: bool = False):
        changed = (
            force or
            advisory != self.last_pub_adv or
            latched != self.last_pub_lat or
            cause_str != self.last_pub_cause
        )
        time_for_keepalive = False
        if self.keepalive_hz > 0.0:
            period = 1.0 / self.keepalive_hz
            if (self._now() - self.last_keepalive_time) >= period:
                time_for_keepalive = True

        if changed or time_for_keepalive:
            self.pub_adv.publish(Bool(data=advisory))
            self.pub_lat.publish(Bool(data=latched))
            self.pub_cause.publish(String(data=cause_str))
            self.last_pub_adv = advisory
            self.last_pub_lat = latched
            self.last_pub_cause = cause_str
            if time_for_keepalive:
                self.last_keepalive_time = self._now()

    # ---------------- Subscription callback ----------------
    def _on_latency(self, msg: Float32):
        now = self._now()
        rtt_ms = float(msg.data)

        # Validate input
        if not math.isfinite(rtt_ms) or rtt_ms < 0.0:
            # Treat invalid sample as severe latency (hard) until a valid one arrives.
            self.get_logger().warn(f"Invalid RTT sample: {rtt_ms} → treating as HARD_LAT this tick")
            self._mark_hard_latency_consec(now, force=True)
            # Do not update timing state to avoid hiding disconnects
            self._update_outputs(now)
            return

        # Compute effective one-way
        oneway_ms = rtt_ms * 0.5 if self.assume_input_is_rtt else rtt_ms

        # Compute gap (time since last message) and update gap-based flags
        gap_s = None
        if self.last_sample_time is not None:
            gap_s = now - self.last_sample_time
            # H3 flapping: record gap events > flap_gap_s
            if gap_s > self.flap_gap_s:
                self.gap_events.append(now)
                # S3 rate degrade: if gap exceeds soft but is below disconnect, set advisory rate
                if gap_s > self.gap_soft_s and gap_s < self.disconnect_timeout_s:
                    self.s_adv_rate = True
                    self.soft_healthy_consec = 0  # not healthy
        self.last_sample_time = now  # update last-sample timestamp

        # ---------- Soft rules ----------
        # S1: elevated latency consecutive
        if oneway_ms >= self.soft_latency_ms:
            self.soft_latency_consec += 1
        else:
            self.soft_latency_consec = 0

        # S2: spike (single-sample threshold) — stays active until N healthy samples
        if oneway_ms >= self.soft_spike_ms:
            self.s_adv_spike = True
            self.soft_healthy_consec = 0

        # S4: jitter step consecutive
        if self.prev_oneway_ms is not None and abs(oneway_ms - self.prev_oneway_ms) > self.soft_jitter_step_ms:
            self.soft_jitter_consec += 1
        else:
            self.soft_jitter_consec = 0

            

        # Evaluate soft flags from counters
        self.s_adv_soft_lat = self.soft_latency_consec >= 2
        self.s_adv_jitter = self.soft_jitter_consec >= 2

        # Healthy sample check (for clearing S1/S2/S4 and for S3 if gap now normal)
        healthy_now = (
            (oneway_ms < self.soft_latency_ms) and
            (self.prev_oneway_ms is None or abs(oneway_ms - self.prev_oneway_ms) <= self.soft_jitter_step_ms) and
            (gap_s is None or gap_s <= self.gap_soft_s)
        )
        self.prev_oneway_ms = oneway_ms

        if healthy_now:
            self.soft_healthy_consec += 1
        else:
            # Only reset if we actually had a non-healthy sample (avoid None at startup)
            if gap_s is not None:
                self.soft_healthy_consec = 0

        # Clear soft flags after N healthy consecutive samples
        if self.soft_healthy_consec >= self.soft_clear_consecutive:
            self.s_adv_spike = False
            self.s_adv_soft_lat = False
            self.s_adv_jitter = False
            # For rate degrade, also require that recent gap was normal
            if gap_s is None or gap_s <= self.gap_soft_s:
                self.s_adv_rate = False

        # ---------- Hard rules (H2 latency) ----------
        if oneway_ms >= self.hard_latency_ms:
            self.hard_latency_consec += 1
        else:
            self.hard_latency_consec = 0

        if self.fast_trip_enabled and (oneway_ms >= self.fast_trip_single_hard_ms):
            self.h_lat = True
        else:
            self.h_lat = self.hard_latency_consec >= self.hard_latency_consecutive

        # After processing this sample, update outputs (H1/H3 also handled in timer via gaps)
        self._update_outputs(now)

    # ---------------- Timer tick ----------------
    def _tick(self):
        now = self._now()

        # H1: disconnect if no samples for >= disconnect_timeout_s
        if self.last_sample_time is None:
            gap = None
        else:
            gap = now - self.last_sample_time

        prev_h_disc = self.h_disc
        self.h_disc = (gap is not None) and (gap >= self.disconnect_timeout_s)

        # While in a long-but-not-disconnect gap, keep S3 advisory true
        if (gap is not None) and (gap > self.gap_soft_s) and (gap < self.disconnect_timeout_s):
            self.s_adv_rate = True

        # H3: flapping — count recent large gaps within window
        # Prune old gap events
        while self.gap_events and (now - self.gap_events[0]) > self.flap_window_s:
            self.gap_events.popleft()
        self.h_flap = (len(self.gap_events) >= self.flap_count)

        # If we newly entered disconnect, it's a hard cause regardless of soft states
        if self.h_disc and not prev_h_disc:
            # When disconnect fires, soft rate is implied but the hard wins for behavior & label
            pass  # nothing else needed; outputs reflect it below

        self._update_outputs(now)

    # ---------------- Internal: consolidate & publish ----------------
    def _update_outputs(self, now: float):
        # Determine active soft causes
        soft_causes = []
        if self.s_adv_soft_lat:
            soft_causes.append('ADV_SOFT_LAT')
        if self.s_adv_jitter:
            soft_causes.append('ADV_JITTER')
        if self.s_adv_spike:
            soft_causes.append('ADV_SPIKE')
        if self.s_adv_rate:
            soft_causes.append('ADV_RATE')

        # Determine hard cause (one code; priority: DISC > LAT > FLAP)
        hard_code = ""
        if self.h_disc:
            hard_code = 'HARD_DISC'
        elif self.h_lat:
            hard_code = 'HARD_LAT'
        elif self.h_flap:
            hard_code = 'HARD_FLAP'

        advisory = len(soft_causes) > 0
        latched = (hard_code != "")

        cause_str = self._format_cause(soft_causes, hard_code)

        self._publish_levels_and_cause(advisory=advisory, latched=latched, cause_str=cause_str)

    # ---------------- Utility: mark hard latency on invalid sample ----------------
    def _mark_hard_latency_consec(self, now: float, force: bool = False):
        if force:
            self.h_lat = True
        else:
            self.hard_latency_consec += 1
            self.h_lat = self.hard_latency_consec >= self.hard_latency_consecutive


def main(args=None):
    rclpy.init(args=args)
    node = NetworkDetector()
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
