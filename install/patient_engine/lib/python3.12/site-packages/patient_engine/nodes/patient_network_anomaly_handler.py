#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time

class PatientNetworkAnomalyHandler(Node):
    """
    Consolidates anomaly flags into a single patient-facing mode:

      Inputs:
        Network:
          /network/warning           (L1)
          /safety/network_fault      (L2)
          /safety/network_estop      (L3)
        Motion (NEW):
          /safety/motion_fault       (L2)
          /safety/motion_estop       (L3)

      Outputs:
        /patient/control_mode : 'normal' | 'passive' | 'estop'
        /patient/freeze       : Bool

    Policy:
      - Any L3 (network or motion):   mode='estop',  freeze=True
      - Else any L2 (network/motion): mode='passive',freeze=False
      - Clear to normal only after all L2/L3 false for >= clear_delay_s.
    """

    def __init__(self):
        super().__init__('patient_network_anomaly_handler')

        # ---- Inputs (network) ----
        self.sub_l1 = self.create_subscription(Bool, '/network/warning',      self.on_l1, 10)
        self.sub_l2 = self.create_subscription(Bool, '/safety/network_fault', self.on_l2, 10)
        self.sub_l3 = self.create_subscription(Bool, '/safety/network_estop', self.on_l3, 10)

        # ---- Inputs (motion) [NEW] ----
        self.sub_m2 = self.create_subscription(Bool, '/safety/motion_fault',  self.on_m2, 10)
        self.sub_m3 = self.create_subscription(Bool, '/safety/motion_estop',  self.on_m3, 10)

        # ---- Outputs ----
        self.pub_mode   = self.create_publisher(String, '/patient/control_mode', 10)
        self.pub_freeze = self.create_publisher(Bool,   '/patient/freeze',       10)

        # ---- State ----
        self.l1 = False
        self.l2 = False
        self.l3 = False
        self.m2 = False   # NEW: motion L2
        self.m3 = False   # NEW: motion L3

        # Hysteresis (seconds) before clearing to normal
        self.clear_delay_s = 2.0
        self.last_l2l3_on_time = None  # last time we saw any L2 or L3 True

        # De-dup publishing
        self.current_mode = None
        self.current_freeze = None

        # Initial
        self._publish_if_changed('normal', False)
        self.get_logger().info('[patient_network_anomaly_handler] ready (hysteresis=2.0s)')

    # ---- subscriptions ----
    def on_l1(self, m: Bool):
        self.l1 = bool(m.data)
        self._decide_and_publish()

    def on_l2(self, m: Bool):
        self.l2 = bool(m.data)
        if self.l2:
            self.last_l2l3_on_time = time.time()
        self._decide_and_publish()

    def on_l3(self, m: Bool):
        self.l3 = bool(m.data)
        if self.l3:
            self.last_l2l3_on_time = time.time()
        self._decide_and_publish()

    # NEW: motion callbacks
    def on_m2(self, m: Bool):
        self.m2 = bool(m.data)
        if self.m2:
            self.last_l2l3_on_time = time.time()
        self._decide_and_publish()

    def on_m3(self, m: Bool):
        self.m3 = bool(m.data)
        if self.m3:
            self.last_l2l3_on_time = time.time()
        self._decide_and_publish()

    # ---- decision logic with hysteresis ----
    def _decide_and_publish(self):
        now = time.time()

        # Priority handling
        # Any L3 (network OR motion) → ESTOP
        if self.l3 or self.m3:
            self._publish_if_changed('estop', True)
            return

        # Else any L2 (network OR motion) → PASSIVE
        if self.l2 or self.m2:
            self._publish_if_changed('passive', False)
            return

        # Here: no L2/L3 active
        if self.last_l2l3_on_time is not None:
            elapsed = now - self.last_l2l3_on_time
            if elapsed < self.clear_delay_s:
                # Still in the post-fault window: hold passive a bit longer (no freeze)
                self._publish_if_changed('passive', False)
                return
            else:
                # Aggressive clear after stable window
                self._publish_if_changed('normal', False)
                self.get_logger().info('[patient_network_anomaly_handler] CLEAR → mode=normal (after restore)')
                # Reset the marker so we don't repeatedly log clears after long stable periods
                self.last_l2l3_on_time = None
                return

        # No L2/L3 events recorded, or already cleared for a while
        self._publish_if_changed('normal', False)

    def _publish_if_changed(self, mode: str, freeze: bool):
        if mode == self.current_mode and freeze == self.current_freeze:
            return
        self.current_mode = mode
        self.current_freeze = freeze

        sm = String(); sm.data = mode
        sb = Bool();   sb.data = freeze

        self.pub_mode.publish(sm)
        self.pub_freeze.publish(sb)

        self.get_logger().info(f'[patient_network_anomaly_handler] mode={mode} freeze={freeze}')

def main(args=None):
    rclpy.init(args=args)
    node = PatientNetworkAnomalyHandler()
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
