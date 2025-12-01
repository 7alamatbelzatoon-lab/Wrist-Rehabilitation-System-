#!/usr/bin/env python3
import time
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time

from std_msgs.msg import String, Bool, Empty, UInt8


# ---------- QoS (agreed) ----------
qos_edge_sub = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
qos_level_sub = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
qos_control_pub = QoSProfile(   # publish-on-change
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
qos_state_pub = QoSProfile(     # latched for late joiners
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
qos_event_pub = QoSProfile(     # breadcrumbs (no replay)
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class OrchestratorState(IntEnum):
    FREEZE = 0
    NORMAL = 1
    SLEEP = 2
    ZERO_POSITION = 3


class PatientOrchestrator(Node):
    def __init__(self):
        super().__init__('patient_orchestrator')

        # ---------- Params (tunable) ----------
        self.freeze_hold_s = self.declare_parameter('freeze_hold_s', 2.5).get_parameter_value().double_value
        self.quiet_window_s = self.declare_parameter('quiet_window_s', 1.5).get_parameter_value().double_value
        self.freeze_done_timeout_s = self.declare_parameter('freeze_done_timeout_s', 3.0).get_parameter_value().double_value
        self.zero_reached_timeout_s = self.declare_parameter('zero_reached_timeout_s', 10.0).get_parameter_value().double_value
        self.startup_auto_normal = self.declare_parameter('startup_auto_normal', True).get_parameter_value().bool_value

        # ---------- Publishers ----------
        self.pub_mode = self.create_publisher(String, '/patient/control_mode', qos_control_pub)
        self.pub_state = self.create_publisher(UInt8, '/orchestrator/state', qos_state_pub)
        self.pub_event = self.create_publisher(String, '/orchestrator/event', qos_event_pub)

        # ---------- Subscriptions (patient feedback) ----------
        self.create_subscription(Empty, '/patient/freeze/done', self._on_freeze_done, qos_edge_sub)
        self.create_subscription(Empty, '/patient/zero_reached', self._on_zero_reached, qos_edge_sub)
        self.create_subscription(Bool, '/patient/freeze_stable', self._on_freeze_stable, qos_level_sub)
        self.create_subscription(Bool, '/patient/at_zero', self._on_at_zero, qos_level_sub)

        # ---------- Subscriptions (detectors) ----------
        self.create_subscription(Bool, '/safety/motion/advisory', self._on_motion_adv, qos_level_sub)
        self.create_subscription(Bool, '/safety/motion/latched', self._on_motion_lat, qos_level_sub)
        self.create_subscription(Bool, '/safety/network/advisory', self._on_network_adv, qos_level_sub)
        self.create_subscription(Bool, '/safety/network/latched', self._on_network_lat, qos_level_sub)

        # ---------- Subscriptions (doctor commands; edge) ----------
        self.create_subscription(Empty, '/doctor/manual_estop', self._on_doctor_estop, qos_edge_sub)
        self.create_subscription(Empty, '/doctor/manual_sleep', self._on_doctor_sleep, qos_edge_sub)
        self.create_subscription(Empty, '/doctor/clear',        self._on_doctor_clear, qos_edge_sub)
        self.create_subscription(Empty, '/doctor/resume',       self._on_doctor_resume, qos_edge_sub)  # for ISO alignment

        # ---------- Internal state ----------
        self.state = OrchestratorState.FREEZE
        self.current_mode_cmd = ""  # last /patient/control_mode published

        self.latched_seq_started = False

        self.freeze_stable = False
        self.at_zero = False

        self.adv_motion = False
        self.adv_network = False
        self.lat_motion = False
        self.lat_network = False

        self.hard_gate_active = False
        self._prev_lat_motion = False
        self._prev_lat_network = False

        self.doctor_cleared_override = False  # ignore detector latched until they drop to False once
        self.manual_latched_freeze_only = False  # manual_estop
        self.manual_latched_sleep = False        # manual_sleep
        self.resume_requested = False            # set by /doctor/resume
        self.require_resume = False              # gate auto-NORMAL until resume (set on /doctor/clear)

        # timers / deadlines
        self.freeze_edge_deadline = None
        self.zero_edge_deadline = None
        self.advisory_freeze_until = None
        self.ok_since = None  # quiet window timer

        # ---------- Kickoff ----------
        self._enter_state(OrchestratorState.FREEZE)
        self._cmd_mode('FREEZE')  # explicit at start

        # main tick (deterministic single-thread loop)
        self.timer = self.create_timer(0.1, self._tick)  # 10 Hz

        self._event('orchestrator_start')

    # ---------- Helpers ----------
    def _now(self) -> float:
        # steady clock seconds
        return self.get_clock().now().nanoseconds / 1e9

    def _event(self, text: str):
        self.pub_event.publish(String(data=text))

    def _publish_state(self):
        self.pub_state.publish(UInt8(data=int(self.state)))

    def _cmd_mode(self, mode: str):
        mode = mode.upper()
        if mode != self.current_mode_cmd:
            self.pub_mode.publish(String(data=mode))
            self.current_mode_cmd = mode
            self._event(f'cmd_mode:{mode}')

    def _enter_state(self, new_state: OrchestratorState):
        if self.state == new_state:
            return
        self.state = new_state
        self._publish_state()
        # set associated timeouts
        if new_state == OrchestratorState.FREEZE:
            self.freeze_edge_deadline = self._now() + self.freeze_done_timeout_s
            # (quiet window restarts once we see freeze_stable True)
            self.ok_since = None
        elif new_state == OrchestratorState.ZERO_POSITION:
            self.zero_edge_deadline = self._now() + self.zero_reached_timeout_s
        else:
            self.freeze_edge_deadline = None
            self.zero_edge_deadline = None

    def _any_advisory(self) -> bool:
        return self.adv_motion or self.adv_network

    def _any_latched(self) -> bool:
        # detector latched ignored while doctor override is active
        external_latched = (self.lat_motion or self.lat_network)
        effective_external = (not self.doctor_cleared_override) and external_latched
        return (
            effective_external
            or self.manual_latched_freeze_only
            or self.manual_latched_sleep
            or self.hard_gate_active
        )

    # ---------- Callbacks ----------
    def _on_freeze_done(self, _):
        # confirmed stable; start quiet window if safe
        if self.state == OrchestratorState.FREEZE:
            if not self._any_advisory() and not self._any_latched():
                self.ok_since = self._now()

    def _on_zero_reached(self, _):
        # patient node will auto FREEZE afterward
        pass

    def _on_freeze_stable(self, msg: Bool):
        self.freeze_stable = bool(msg.data)
        if self.freeze_stable and self.state == OrchestratorState.FREEZE:
            if not self._any_advisory() and not self._any_latched():
                # (re)start quiet window
                self.ok_since = self._now()

    def _on_at_zero(self, msg: Bool):
        self.at_zero = bool(msg.data)

    def _on_motion_adv(self, msg: Bool):    self.adv_motion = bool(msg.data)
    def _on_motion_lat(self, msg: Bool):    self.lat_motion = bool(msg.data)
    def _on_network_adv(self, msg: Bool):   self.adv_network = bool(msg.data)
    def _on_network_lat(self, msg: Bool):   self.lat_network = bool(msg.data)

    def _on_doctor_estop(self, _):
        self.manual_latched_freeze_only = True
        self._event('doctor_estop')

    def _on_doctor_sleep(self, _):
        self.manual_latched_sleep = True
        self._event('doctor_sleep')

    def _on_doctor_resume(self, _):
        self.resume_requested = True
        self._event('doctor_resume')

    def _on_doctor_clear(self, _):
        # clear all manual latches; detector latched topics remain authoritative
        self.manual_latched_freeze_only = False
        self.manual_latched_sleep = False
        self.latched_seq_started = False
        self.hard_gate_active = False
        self._event('hard_gate_end')
        self.doctor_cleared_override = True   # <-- activate override
        # require an explicit /doctor/resume before leaving FREEZE
        self.require_resume = True
        self._event('doctor_clear')

    # ---------- Main tick ----------
    def _tick(self):
        now = self._now()

        # auto-drop doctor override once both detector-latched are False
        if self.doctor_cleared_override and not (self.lat_motion or self.lat_network):
            self.doctor_cleared_override = False
            self._event('doctor_override_off')

        # Arm sticky gate on RISING EDGE of any detector-latched, unless doctor override is active
        if (not self.doctor_cleared_override):
            if (self.lat_motion and not self._prev_lat_motion) or (self.lat_network and not self._prev_lat_network):
                if not self.hard_gate_active:
                    self.hard_gate_active = True
                    self._event('hard_gate_start')

        # PRIORITY: Doctor > Latched (detectors) > Advisory > Normal/Startup

        # 1) Doctor/Latched handling
        if self._any_latched():
            # Doctor e-stop: hold FREEZE; don't oscillate; also don't start the latched sequence
            if self.manual_latched_freeze_only:
                self.latched_seq_started = False
                if self.state != OrchestratorState.FREEZE:
                    self._enter_state(OrchestratorState.FREEZE)
                self._cmd_mode('FREEZE')
                return

            # Detector-latched path: target SLEEP once, then accept patient auto-FREEZE
            if not self.latched_seq_started:
                # First time we see latched: go FREEZE -> SLEEP
                if self.state != OrchestratorState.FREEZE:
                    self._enter_state(OrchestratorState.FREEZE)
                    self._cmd_mode('FREEZE')
                    return  # next tick we'll be in FREEZE, then proceed to SLEEP
                # We are in FREEZE: proceed to SLEEP once
                self._enter_state(OrchestratorState.SLEEP)
                self._cmd_mode('SLEEP')
                self.latched_seq_started = True
                return

            # Sequence already started: stay in SLEEP or (after patient auto-freeze) stay in FREEZE
            return
        else:
            # No latched active -> allow a future latched to start the sequence again
            self.latched_seq_started = False

        # 2) Advisory handling
        if self._any_advisory():
            if self.state != OrchestratorState.FREEZE and self.state != OrchestratorState.SLEEP:
                self._enter_state(OrchestratorState.FREEZE)
                self._cmd_mode('FREEZE')
                self.advisory_freeze_until = now + self.freeze_hold_s
                return

            if self.state == OrchestratorState.FREEZE:
                # ensure timer exists (new advisory or overlapping)
                if self.advisory_freeze_until is None:
                    self.advisory_freeze_until = now + self.freeze_hold_s
                # escalate to SLEEP after hold if still active
                if now >= self.advisory_freeze_until:
                    self._enter_state(OrchestratorState.SLEEP)
                    self._cmd_mode('SLEEP')
                    return

            elif self.state == OrchestratorState.SLEEP:
                # remain in SLEEP until advisory clears (preempt on clear handled below)
                pass

        # 3) Advisory cleared while in SLEEP → preempt to FREEZE then NORMAL after quiet window
        if self.state == OrchestratorState.SLEEP and not self._any_advisory():
            self.advisory_freeze_until = None   # clear hold window
            self._enter_state(OrchestratorState.FREEZE)
            self._cmd_mode('FREEZE')
            # quiet window starts when freeze_stable/edge arrives
            return

        # 4) Startup / returns: go to NORMAL automatically when all green
        all_green = (not self._any_latched()) and (not self._any_advisory()) and self.freeze_stable
        if self.state == OrchestratorState.FREEZE and all_green:
            # if a doctor_clear happened, require /doctor/resume before leaving FREEZE
            if self.require_resume and not self.resume_requested:
                # still allow quiet-window timer to (re)start so resume is instant later
                if self.ok_since is None:
                    self.ok_since = now
                return

            # start quiet window if not already
            if self.ok_since is None:
                self.ok_since = now
            if (now - self.ok_since) >= self.quiet_window_s:
                self._enter_state(OrchestratorState.NORMAL)
                self._cmd_mode('NORMAL')
                # consume the resume & clear the requirement
                self.resume_requested = False
                self.require_resume = False
                return

        # 5) Optional startup ZERO ritual (kept available if needed)
        if self.state == OrchestratorState.ZERO_POSITION:
            # rely on patient auto FREEZE after zero_reached; we just wait
            pass

        # 6) Safety timeouts (soft holds; no flapping)
        if self.state == OrchestratorState.FREEZE and self.freeze_edge_deadline:
            if now > self.freeze_edge_deadline:
                # stay in FREEZE and log once; reset deadline to avoid spam
                self._event('freeze_done_timeout')
                self.freeze_edge_deadline = None
        if self.state == OrchestratorState.ZERO_POSITION and self.zero_edge_deadline:
            if now > self.zero_edge_deadline:
                self._event('zero_reached_timeout')
                # abort to FREEZE
                self._enter_state(OrchestratorState.FREEZE)
                self._cmd_mode('FREEZE')

        self._prev_lat_motion = self.lat_motion
        self._prev_lat_network = self.lat_network


def main(args=None):
    rclpy.init(args=args)
    node = PatientOrchestrator()
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
