#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32, Bool, String


# ----------- Test definitions -----------

@dataclass
class TestCase:
    name: str
    description: str
    run_fn: str  # method name on the tester to execute this test


@dataclass
class Expect:
    advisory: Optional[bool] = None   # None means "don't-check"
    latched: Optional[bool] = None
    soft_contains: Optional[List[str]] = None  # list of substrings that must appear in SOFT=
    hard_equals: Optional[str] = None          # exact HARD= code (or "" for none)


def parse_cause(s: str):
    # Expected format: "SOFT=...;HARD=..."
    try:
        soft_part, hard_part = s.split(';')
        soft = soft_part.split('=', 1)[1]
        hard = hard_part.split('=', 1)[1]
        soft_list = [c for c in soft.split(',') if c] if soft else []
        return soft_list, hard
    except Exception:
        return [], ""


# ----------- The nano test runner -----------

class NetworkDetectorNanoTest(Node):
    """
    Drives /patient_network/latency_ms at 1 Hz, runs scenarios, and validates the detector:
      /safety/network/advisory (Bool),
      /safety/network/latched  (Bool),
      /safety/network/cause    (String, "SOFT=...;HARD=...")
    """

    def __init__(self):
        super().__init__('network_detector_nano_test')

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
        self.sub_adv = self.create_subscription(Bool,   '/safety/network/advisory', self._on_adv, self.qos_level)
        self.sub_lat = self.create_subscription(Bool,   '/safety/network/latched',  self._on_lat, self.qos_level)
        self.sub_cau = self.create_subscription(String, '/safety/network/cause',    self._on_cause, self.qos_level)

        # latest observed outputs
        self.advisory = False
        self.latched = False
        self.cause_soft: List[str] = []
        self.cause_hard: str = ""

        # 1 Hz publisher state
        self.base_rate_hz = 1.0
        self._next_tick = time.monotonic()
        self._bg_enabled = True            # when False, we simply don't publish (creates a gap)
        self._bg_value_ms = 70.0           # RTT
        self._inject_queue: List[float] = []  # precise sequence overrides (RTT values)

        # test bookkeeping
        self.results = []
        self.tests: List[TestCase] = [
            TestCase("S1_soft_latency",  "Two consecutive high RTT → ADV_SOFT_LAT", "test_s1_soft_latency"),
            TestCase("S2_spike",         "Single spike RTT → ADV_SPIKE",            "test_s2_spike"),
            TestCase("S3_rate_degrade",  "Gap 1.7s (< disconnect) → ADV_RATE",      "test_s3_rate_degrade"),
            TestCase("S4_jitter_steps",  "Two consecutive big step deltas → ADV_JITTER", "test_s4_jitter"),
            TestCase("H1_disconnect",    "No samples ≥ 2.5s → HARD_DISC",           "test_h1_disconnect"),
            TestCase("H2_severe_latency","Two consecutive severe RTT → HARD_LAT",    "test_h2_severe_latency"),
            TestCase("H3_flapping",      "Two soft gaps within 10s → HARD_FLAP",     "test_h3_flapping"),
            TestCase("HX_invalid",       "NaN sample → HARD_LAT (guard)",           "test_hx_invalid_sample"),
        ]

        # main timer (10 Hz loop to service 1 Hz publisher + test FSM)
        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info("nano test started — press Ctrl+C to abort")

        # kick off after a short settle
        self._stage = "idle"
        self._stage_deadline = time.monotonic() + 0.5
        self._current_test_index = -1

    # ---------- subs ----------
    def _on_adv(self, msg: Bool):
        self.advisory = bool(msg.data)

    def _on_lat(self, msg: Bool):
        self.latched = bool(msg.data)

    def _on_cause(self, msg: String):
        s = str(msg.data)
        self.cause_soft, self.cause_hard = parse_cause(s)

    # ---------- publisher helpers ----------
    def _pub_now(self, rtt_ms: float):
        self.pub_rtt.publish(Float32(data=float(rtt_ms)))

    def _enable_background(self, enabled: bool):
        self._bg_enabled = enabled

    def _inject(self, values: List[float]):
        """Queue exact RTT samples to push at 1 Hz ticks, pausing background."""
        self._inject_queue.extend(values)

    # ---------- timing loop ----------
    def _tick(self):
        now = time.monotonic()

        # 1 Hz publisher scheduling
        if now >= self._next_tick:
            self._next_tick += (1.0 / self.base_rate_hz)
            if self._inject_queue:
                # inject sample (background paused implicitly)
                val = self._inject_queue.pop(0)
                self._pub_now(val)
            elif self._bg_enabled:
                self._pub_now(self._bg_value_ms)
            else:
                # background paused: publish nothing this tick
                pass

        # test state machine
        if self._stage == "idle" and now >= self._stage_deadline:
            self._run_next_test()
            return

        # active test stages handled inside each test via waits/asserts

    # ---------- assertions with timeout ----------
    def _wait_until(self, condition_fn, timeout_s: float, desc: str) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if condition_fn():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().error(f"timeout waiting: {desc}")
        return False

    def _expect(self, expect: Expect, timeout_s: float = 3.5) -> bool:
        def cond():
            ok = True
            if expect.advisory is not None:
                ok = ok and (self.advisory == expect.advisory)
            if expect.latched is not None:
                ok = ok and (self.latched == expect.latched)
            if expect.soft_contains is not None:
                ok = ok and all(code in self.cause_soft for code in expect.soft_contains)
            if expect.hard_equals is not None:
                ok = ok and (self.cause_hard == expect.hard_equals)
            return ok

        return self._wait_until(cond, timeout_s, f"expect {expect}")

    # ---------- orchestrate the test list ----------
    def _run_next_test(self):
        self._current_test_index += 1
        if self._current_test_index >= len(self.tests):
            # print summary & exit
            total = len(self.results)
            passed = sum(1 for x in self.results if x[1])
            verdict = "ALL PASSED ✅" if passed == total else f"{passed}/{total} PASSED ❌"
            self.get_logger().info("\n========== Nano Test Summary ==========")
            for name, ok in self.results:
                self.get_logger().info(f"{'PASS' if ok else 'FAIL'}  {name}")
            self.get_logger().info(f"VERDICT: {verdict}\n")
            rclpy.shutdown()
            return

        t = self.tests[self._current_test_index]
        self.get_logger().info(f"\n--- RUN {t.name} — {t.description} ---")
        ok = False
        try:
            ok = getattr(self, t.run_fn)()
        except Exception as e:
            self.get_logger().error(f"Exception in {t.name}: {e}")
            ok = False
        self.results.append((t.name, ok))
        # small settle between tests
        self._stage = "idle"
        self._stage_deadline = time.monotonic() + 1.0

    # ---------- individual tests ----------
    def _settle_healthy(self, seconds=3):
        """Ensure detector reports healthy for a few ticks."""
        self._enable_background(True)
        self._inject_queue.clear()
        ok = self._expect(Expect(advisory=False, latched=False, soft_contains=[], hard_equals=""), timeout_s=seconds+2)
        return ok

    def test_s1_soft_latency(self) -> bool:
        if not self._settle_healthy():
            return False
        # Two consecutive high RTT samples (≥240 RTT → one-way ≥120)
        self._enable_background(False)
        self._inject([300.0, 300.0])
        ok1 = self._expect(Expect(advisory=True, latched=False, soft_contains=["ADV_SOFT_LAT"]), timeout_s=4.0)
        # Clear after two healthy
        self._enable_background(True)
        ok2 = self._expect(Expect(advisory=False, latched=False, soft_contains=[], hard_equals=""), timeout_s=4.0)
        return ok1 and ok2

    def test_s2_spike(self) -> bool:
        if not self._settle_healthy():
            return False
        # One spike (≥400 RTT → one-way ≥200)
        self._inject([420.0])
        ok1 = self._expect(Expect(advisory=True, latched=False, soft_contains=["ADV_SPIKE"]), timeout_s=3.5)
        ok2 = self._expect(Expect(advisory=False, latched=False, soft_contains=[], hard_equals=""), timeout_s=5.0)
        return ok1 and ok2

    def test_s3_rate_degrade(self) -> bool:
        if not self._settle_healthy():
            return False
        # Create a ~1.7s soft gap (<2.5s disconnect)
        self._enable_background(False)
        start = time.monotonic()
        while time.monotonic() - start < 1.7:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._enable_background(True)
        ok1 = self._expect(Expect(advisory=True, latched=False, soft_contains=["ADV_RATE"]), timeout_s=3.5)
        ok2 = self._expect(Expect(advisory=False, latched=False, soft_contains=[], hard_equals=""), timeout_s=5.0)
        return ok1 and ok2

    def test_s4_jitter(self) -> bool:
        if not self._settle_healthy():
            return False
        # Two consecutive big step changes: 70→300→70→300
        self._enable_background(False)
        self._inject([70.0, 300.0, 70.0, 300.0])
        ok1 = self._expect(Expect(advisory=True, latched=False, soft_contains=["ADV_JITTER"]), timeout_s=5.0)
        self._enable_background(True)
        ok2 = self._expect(Expect(advisory=False, latched=False, soft_contains=[], hard_equals=""), timeout_s=5.0)
        return ok1 and ok2

    def test_h1_disconnect(self) -> bool:
        if not self._settle_healthy():
            return False
        # No samples for ≥2.5s
        self._enable_background(False)
        start = time.monotonic()
        while time.monotonic() - start < 2.7:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._enable_background(True)
        ok1 = self._expect(Expect(latched=True, hard_equals="HARD_DISC"), timeout_s=1.0)
        ok2 = self._expect(Expect(latched=False), timeout_s=4.0)
        return ok1 and ok2

    def test_h2_severe_latency(self) -> bool:
        if not self._settle_healthy():
            return False
        # Two consecutive severe RTT (≥500 → one-way ≥250)
        self._enable_background(False)
        self._inject([600.0, 600.0])
        ok1 = self._expect(Expect(latched=True, hard_equals="HARD_LAT"), timeout_s=4.0)
        self._enable_background(True)
        ok2 = self._expect(Expect(latched=False), timeout_s=4.0)
        return ok1 and ok2

    def test_h3_flapping(self) -> bool:
        if not self._settle_healthy():
            return False
        # Two soft gaps >1.6s within 10s window
        self._enable_background(False)
        start = time.monotonic()
        while time.monotonic() - start < 1.7:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._enable_background(True)
        # wait ~2s with healthy in between
        self._wait_until(lambda: True, 2.0, "inter-gap wait")
        # second soft gap
        self._enable_background(False)
        start = time.monotonic()
        while time.monotonic() - start < 1.7:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._enable_background(True)

        ok1 = self._expect(Expect(latched=True, hard_equals="HARD_FLAP"), timeout_s=3.0)
        ok2 = self._expect(Expect(latched=False), timeout_s=5.0)
        return ok1 and ok2

    def test_hx_invalid_sample(self) -> bool:
        if not self._settle_healthy():
            return False
        # Publish NaN once
        self._enable_background(False)
        msg = Float32()
        msg.data = float('nan')
        self.pub_rtt.publish(msg)
        self._enable_background(True)
        ok1 = self._expect(Expect(latched=True, hard_equals="HARD_LAT"), timeout_s=2.0)
        ok2 = self._expect(Expect(latched=False), timeout_s=4.0)
        return ok1 and ok2


def main(args=None):
    rclpy.init(args=args)
    node = NetworkDetectorNanoTest()
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
