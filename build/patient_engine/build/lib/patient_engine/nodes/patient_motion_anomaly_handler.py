#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
import json
import time
from collections import deque


class PatientMotionAnomalyHandler(Node):
    def __init__(self):
        super().__init__('patient_motion_anomaly_handler')

        # ---- Parameters (YAML-friendly defaults) ----
        self.declare_parameters(
            namespace='motion_detector',
            parameters=[
                ('thresholds.max_angle', 80.0),          # deg
                ('thresholds.max_velocity', 25.0),       # deg/s
                ('thresholds.max_acceleration', 8.0),    # deg/s^2
                ('thresholds.max_torque', 10.0),         # unit depends on your sensor
                ('thresholds.stuck_time', 2.0),          # s (near-zero vel while torque high)
                ('thresholds.drift_angle', 12.0),        # deg (target-actual)
                ('thresholds.drift_time', 2.0),          # s (persistent)
                ('thresholds.oscillation_zcr_rate', 10.0),  # zero-crossings per second
                ('hysteresis_time', 2.0),                # s to clear L2 after stable
            ],
        )

        self.th = {
            'max_angle'           : self.get_parameter('motion_detector.thresholds.max_angle').value,
            'max_velocity'        : self.get_parameter('motion_detector.thresholds.max_velocity').value,
            'max_acceleration'    : self.get_parameter('motion_detector.thresholds.max_acceleration').value,
            'max_torque'          : self.get_parameter('motion_detector.thresholds.max_torque').value,
            'stuck_time'          : self.get_parameter('motion_detector.thresholds.stuck_time').value,
            'drift_angle'         : self.get_parameter('motion_detector.thresholds.drift_angle').value,
            'drift_time'          : self.get_parameter('motion_detector.thresholds.drift_time').value,
            'oscillation_zcr_rate': self.get_parameter('motion_detector.thresholds.oscillation_zcr_rate').value,
        }
        self.hysteresis_time = self.get_parameter('motion_detector.hysteresis_time').value

        # ---- ROS I/O ----
        self.sub_angle  = self.create_subscription(Float32, '/patient/joint_position',  self.on_angle,  10)
        self.sub_target = self.create_subscription(Float32, '/patient/target_position', self.on_target, 10)
        # optional: if you don’t publish effort yet, this will just stay 0
        self.sub_torque = self.create_subscription(Float32, '/patient/effort',          self.on_torque, 10)

        self.pub_fault  = self.create_publisher(Bool,   '/safety/motion_fault',  10)  # L2
        self.pub_estop  = self.create_publisher(Bool,   '/safety/motion_estop',  10)  # L3
        self.pub_event  = self.create_publisher(String, '/safety/event',         10)  # JSON

        # ---- State ----
        self.prev_angle     = None
        self.prev_velocity  = 0.0
        self.prev_t         = None
        self.last_move_t    = time.time()
        self.target         = 0.0
        self.torque         = 0.0
        self.vel_buffer     = deque(maxlen=50)   # ~1s at 50 Hz
        self.last_fault_t   = None
        self.last_estop_t   = None
        self.fault_active   = False
        self.estop_active   = False

        self.get_logger().info('[patient_motion_anomaly_handler] ready')

    # ----------------- Subscribers -----------------
    def on_angle(self, msg: Float32):
        now   = time.time()
        angle = float(msg.data)

        if self.prev_angle is None:
            self.prev_angle = angle
            self.prev_t = now
            return

        dt = now - self.prev_t
        # guard against bad timing (too fast or too stale)
        if dt <= 0.001 or dt > 0.5:
            self.prev_t = now
            self.prev_angle = angle
            return

        velocity     = (angle - self.prev_angle) / dt
        acceleration = (velocity - self.prev_velocity) / dt if self.prev_t else 0.0
        self.vel_buffer.append(velocity)

        # zero-crossing rate for oscillation (approx)
        zc = 0
        for i in range(1, len(self.vel_buffer)):
            if (self.vel_buffer[i - 1] * self.vel_buffer[i]) < 0:
                zc += 1
        zcr = (zc / max(dt * len(self.vel_buffer), 1e-6)) if len(self.vel_buffer) > 2 else 0.0

        drift = abs(self.target - angle)

        # movement seen?
        if abs(velocity) > 1e-3:
            self.last_move_t = now

        # ----------------- Detection -----------------
        fault  = False
        estop  = False
        reason = None

        # Joint limits (soft → L2, hard → L3 if far beyond)
        if abs(angle) > self.th['max_angle']:
            fault = True
            reason = 'ANGLE_LIMIT'
            if abs(angle) > self.th['max_angle'] + 10.0:  # hard margin
                estop = True
                reason = 'HARD_LIMIT'

        # Overspeed
        elif abs(velocity) > self.th['max_velocity']:
            fault = True
            reason = 'OVERSPEED'

        # Acceleration spike
        elif abs(acceleration) > self.th['max_acceleration']:
            fault = True
            reason = 'ACCEL_SPIKE'

        # Oscillation (instability)
        elif zcr > self.th['oscillation_zcr_rate']:
            fault = True
            reason = 'OSCILLATION'

        # Persistent drift / mismatch
        elif drift > self.th['drift_angle'] and (now - self.last_move_t) > self.th['drift_time']:
            fault = True
            reason = 'DRIFT'

        # Stuck with force (if effort provided)
        elif abs(velocity) < 0.5 and (self.torque is not None) and (self.torque > self.th['max_torque']) \
                and (now - self.last_move_t) > self.th['stuck_time']:
            estop = True
            reason = 'STUCK_FORCE'

        # ----------------- Latch / Clear logic -----------------
        self._update_flags(fault, estop, reason, velocity, acceleration, zcr, drift)

        # ----------------- Bookkeeping -----------------
        self.prev_angle    = angle
        self.prev_velocity = velocity
        self.prev_t        = now

    def on_target(self, msg: Float32):
        self.target = float(msg.data)

    def on_torque(self, msg: Float32):
        self.torque = float(msg.data)

    # ----------------- Helpers -----------------
    def _update_flags(self, fault, estop, reason, v, a, zcr, drift):
        now = time.time()

        if estop:
            if not self.estop_active:
                self._publish_event('MOTION_' + (reason or 'UNKNOWN'), 'L3', v, a, zcr, drift)
                self._pub_bool(self.pub_estop, True)
                self.estop_active = True
                self.get_logger().warn(f'[patient_motion_anomaly_handler] L3 ESTOP → {reason}')
            self.last_estop_t = now
            return

        if fault:
            if not self.fault_active:
                self._publish_event('MOTION_' + (reason or 'UNKNOWN'), 'L2', v, a, zcr, drift)
                self._pub_bool(self.pub_fault, True)
                self.fault_active = True
                self.get_logger().warn(f'[patient_motion_anomaly_handler] L2 FAULT → {reason}')
            self.last_fault_t = now
        else:
            # L2 clear after stable window
            if self.fault_active and self.last_fault_t and (now - self.last_fault_t) > self.hysteresis_time:
                self._pub_bool(self.pub_fault, False)
                self.fault_active = False
                self.get_logger().info('[patient_motion_anomaly_handler] CLEAR → normal')

            # L3 clear policy (software): only de-assert if quiet for some time
            # (network L3 clear still follows your existing handshake)
            if self.estop_active and self.last_estop_t and (now - self.last_estop_t) > 5.0:
                self._pub_bool(self.pub_estop, False)
                self.estop_active = False
                self.get_logger().info('[patient_motion_anomaly_handler] CLEAR ESTOP → normal')

    def _publish_event(self, code, level, v, a, zcr, drift):
        event = {
            'code': code,
            'level': level,
            'metrics': {'v': round(v, 2), 'a': round(a, 2), 'zcr': round(zcr, 2), 'drift': round(drift, 2)},
            't': time.time(),
        }
        msg = String()
        msg.data = json.dumps(event)
        self.pub_event.publish(msg)

    def _pub_bool(self, pub, val):
        m = Bool(); m.data = bool(val)
        pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PatientMotionAnomalyHandler()
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


if __name__ == '__main__':
    main()
