#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

class DoctorNetworkAnomalyHandler(Node):
    def __init__(self):
        super().__init__('doctor_network_anomaly_handler')

        # Inputs (same flags)
        self.sub_l1 = self.create_subscription(Bool, '/network/warning',      self.on_l1, 10)
        self.sub_l2 = self.create_subscription(Bool, '/safety/network_fault', self.on_l2, 10)
        self.sub_l3 = self.create_subscription(Bool, '/safety/network_estop', self.on_l3, 10)

        # Outputs (to doctor-side UI + gating)
        self.pub_gate   = self.create_publisher(Bool,   '/doctor/allow_targets', 10)
        self.pub_banner = self.create_publisher(String, '/doctor/ui_banner',     10)

        self.l1 = self.l2 = self.l3 = False
        self.update_outputs()
        self.get_logger().info('[doctor_network_anomaly_handler] ready (I/O wired)')

    def on_l1(self, m: Bool): self.l1 = bool(m.data); self.update_outputs()
    def on_l2(self, m: Bool): self.l2 = bool(m.data); self.update_outputs()
    def on_l3(self, m: Bool): self.l3 = bool(m.data); self.update_outputs()

    def update_outputs(self):
        # allow targets only if NOT L2/L3
        allow = not (self.l2 or self.l3)
        banner = 'OK'
        if self.l3:   banner = 'CRITICAL network — E-STOP'
        elif self.l2: banner = 'Network fault — targets paused'
        elif self.l1: banner = 'Network degraded — latency high'

        b = Bool();   b.data = allow
        s = String(); s.data = banner
        self.pub_gate.publish(b)
        self.pub_banner.publish(s)
        self.get_logger().info(f'[doctor_network_anomaly_handler] allow_targets={allow} banner="{banner}"')

def main(args=None):
    rclpy.init(args=args)
    node = DoctorNetworkAnomalyHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # silent Ctrl+C
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
