#!/usr/bin/env python3
# ROS 2 Jazzy — deterministic scenario tester for network_detector
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Bool, String

# --- tester param mirrors (keep in sync with detector @ 1 Hz) ---
GAP_SOFT_S = 1.3
DISCONNECT_TIMEOUT_S = 3.0
FLAP_WINDOW_S = 10.0

@dataclass
class Expect:
    advisory: Optional[bool] = None
    latched: Optional[bool] = None
    soft_contains: Optional[List[str]] = None
    hard_equals: Optional[str] = None

# SCENARIO tuple:
# (name, description, seq_list, gaps_list, expect, during_gap)
# NOTE: sequences end right at the condition (no trailing healthy samples)
SCENARIOS = [
    ("S1_soft_latency", "Two consecutive high RTT → ADV_SOFT_LAT",
     [70, 70, 70, 300, 300], [], Expect(advisory=True, latched=False, soft_contains=["ADV_SOFT_LAT"]), False),

    ("S2_spike", "Single spike RTT → ADV_SPIKE",
     [70, 70, 70, 420], [], Expect(advisory=True, latched=False, soft_contains=["ADV_SPIKE"]), False),

    # S3: rate degrade (gap a bit above soft, well below disconnect)
    ("S3_rate_degrade", f"Gap {GAP_SOFT_S+0.1:.1f}s (< disconnect) → ADV_RATE",
     [70, 70, 70], [GAP_SOFT_S + 0.1], Expect(advisory=True, latched=False, soft_contains=["ADV_RATE"]), False),

    ("S4_jitter_steps", "Two consecutive big step deltas → ADV_JITTER",
     [70, 300, 70, 300], [], Expect(advisory=True, latched=False, soft_contains=["ADV_JITTER"]), False),

    # H1: expect DURING the long gap
    #("H1_disconnect", f"No samples ≥ {DISCONNECT_TIMEOUT_S:.1f}s → HARD_DISC",
    #[70, 70, 70], [DISCONNECT_TIMEOUT_S + 0.5], Expect(latched=True, hard_equals="HARD_DISC"), True),

    ("H2_severe_latency", "Two consecutive severe RTT → HARD_LAT",
     [70, 70, 600, 600], [], Expect(latched=True, hard_equals="HARD_LAT"), False),

    ("H3_flapping", "Two gaps > soft threshold within window → HARD_FLAP",
     [70, 70, 70, 70, 70], [GAP_SOFT_S + 0.1, GAP_SOFT_S + 0.2], Expect(latched=True, hard_equals="HARD_FLAP"), False),
]

def parse_cause(s: str):
    # format: "SOFT=a,b;HARD=HARD_X"
    try:
        soft_part, hard_part = s.split(';')
        soft = soft_part.split('=', 1)[1]
        hard = hard_part.split('=', 1)[1]
        soft_list = [c for c in soft.split(',') if c] if soft else []
        return soft_list, hard
    except Exception:
        return [], ""

class ScenarioTester(Node):
    def __init__(self):
        super().__init__('network_detector_scenario_test')

        # QoS
        self.qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.qos_level = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # pubs/subs
        self.pub_rtt = self.create_publisher(Float32, '/patient_network/latency_ms', self.qos_sensor)
        self.create_subscription(Bool,   '/safety/network/advisory', self._on_adv,   self.qos_level)
        self.create_subscription(Bool,   '/safety/network/latched',  self._on_lat,   self.qos_level)
        self.create_subscription(String, '/safety/network/cause',    self._on_cause, self.qos_level)

        # observed outputs
        self.advisory = False
        self.latched = False
        self.soft: List[str] = []
        self.hard: str = ""

        # scenario state
        self.results = []
        self.scenario_idx = -1
        self._seq: List[float] = []
        self._gaps: List[float] = []
        self._expect: Expect = Expect()
        self._expect_during_gap: bool = False
        self._post_gap_sample = False

        self._phase = "idle"     # "idle" | "running"
        self._phase_deadline = time.monotonic() + 0.5
        self._next_tick = time.monotonic() + 0.5  # 1 Hz driver

        # 10 Hz loop
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info("Scenario tester started — make sure 'network_detector' is running.")

    # --- subs ---
    def _on_adv(self, m: Bool):   self.advisory = bool(m.data)
    def _on_lat(self, m: Bool):   self.latched = bool(m.data)
    def _on_cause(self, m: String):
        self.soft, self.hard = parse_cause(m.data)

    # --- helpers ---
    def _pub_rtt(self, val_ms: float):
        self.pub_rtt.publish(Float32(data=float(val_ms)))

    def _expect_until(self, expect: Expect, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            ok = True
            if expect.advisory is not None:
                ok = ok and (self.advisory == expect.advisory)
            if expect.latched is not None:
                ok = ok and (self.latched == expect.latched)
            if expect.soft_contains is not None:
                ok = ok and all(code in self.soft for code in expect.soft_contains)
            if expect.hard_equals is not None:
                ok = ok and (self.hard == expect.hard_equals)
            if ok:
                self.get_logger().info(
                    f"snapshot: adv={self.advisory} lat={self.latched} "
                    f"soft={','.join(self.soft)} hard={self.hard}"
                )
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().warn(
            f"timeout: expected {expect} | last snapshot adv={self.advisory} "
            f"lat={self.latched} soft={','.join(self.soft)} hard={self.hard}"
        )
        return False

    # --- main loop ---
    def _tick(self):
        now = time.monotonic()

        # 1 Hz driver
        if self._phase == "running" and now >= self._next_tick:
            self._next_tick += 1.0

            # If we just finished a scheduled gap, emit one nominal sample to close it.
            if self._post_gap_sample:
                self._pub_rtt(70.0)           # baseline RTT to mark the gap boundary
                self._post_gap_sample = False
                return

            if self._seq:
                v = self._seq.pop(0)
                self._pub_rtt(v)

            elif self._gaps:
                # schedule an intentional gap; optionally assert DURING the gap
                gap = self._gaps.pop(0)

                if self._expect_during_gap:
                    wait_s = max(0.1, gap - 0.05)  # leave a tiny margin
                    ok = self._expect_until(self._expect, timeout_s=wait_s)
                    name = SCENARIOS[self.scenario_idx][0]
                    self.results.append((name, ok))
                    self.get_logger().info(f"{'PASS' if ok else 'FAIL'}  {name}")
                    # resume: close the gap with one sample then cool down
                    self._next_tick = time.monotonic() + gap
                    self._post_gap_sample = True
                    self._phase = "idle"
                    self._phase_deadline = time.monotonic() + 2.0
                    return

                # default: just wait the gap; we'll assert after stimuli finish
                self._next_tick = time.monotonic() + gap
                self._post_gap_sample = True   # next tick after the gap, publish one sample

            else:
                # all stimuli sent → wait for expected condition to occur
                ok = self._expect_until(self._expect, timeout_s=5.0)
                name = SCENARIOS[self.scenario_idx][0]
                self.results.append((name, ok))
                self.get_logger().info(f"{'PASS' if ok else 'FAIL'}  {name}")
                # 2s cool-down before next scenario
                self._phase = "idle"
                self._phase_deadline = time.monotonic() + 2.0
                return

        # start next scenario
        if self._phase == "idle" and now >= self._phase_deadline:
            self._start_next_scenario()

        # finish
        if self.scenario_idx >= len(SCENARIOS):
            self._print_summary_and_exit()

    def _start_next_scenario(self):
        self.scenario_idx += 1
        if self.scenario_idx >= len(SCENARIOS):
            self._print_summary_and_exit()
            return

        name, desc, seq, gaps, expect, during_gap = SCENARIOS[self.scenario_idx]

        # If this scenario or the previous one uses gaps, cool down so old gaps expire
        uses_gaps = bool(gaps)
        prev_used_gaps = self.scenario_idx > 0 and bool(SCENARIOS[self.scenario_idx - 1][3])
        if uses_gaps or prev_used_gaps:
            # publish healthy samples for FLAP_WINDOW_S+1 seconds
            self.get_logger().info(f"(cooldown) publishing healthy for {FLAP_WINDOW_S+1:.1f}s to let gap window expire")
            end = time.monotonic() + (FLAP_WINDOW_S + 1.0)
            # drive healthy at 1 Hz without touching the main state machine
            while time.monotonic() < end:
                self.pub_rtt.publish(Float32(data=70.0))
                rclpy.spin_once(self, timeout_sec=0.2)
                time.sleep(0.8)

        self.get_logger().info(f"\n--- RUN {name} — {desc} ---")
        self._seq = list(seq)
        self._gaps = list(gaps)
        self._expect = expect
        self._expect_during_gap = during_gap
        self._phase = "running"
        self._next_tick = time.monotonic() + 0.01  # start immediately

    def _print_summary_and_exit(self):
        total = len(self.results)
        passed = sum(1 for _, ok in self.results if ok)
        self.get_logger().info("\n========== Scenario Test Summary ==========")
        for name, ok in self.results:
            self.get_logger().info(f"{'PASS' if ok else 'FAIL'}  {name}")
        self.get_logger().info(f"VERDICT: {passed}/{total} PASSED")

        # stop safely: cancel this timer, then destroy node & shutdown on a one-shot
        self.timer.cancel()
        def _stop():
            try:
                self.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
        self.create_timer(0.01, _stop)

def main(args=None):
    rclpy.init(args=args)
    node = ScenarioTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # if still running (e.g., Ctrl+C), clean up gracefully
        if rclpy.ok():
            try:
                node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()

if __name__ == "__main__":
    main()
