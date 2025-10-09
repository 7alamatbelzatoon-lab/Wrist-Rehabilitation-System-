#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import paho.mqtt.client as mqtt
import json, time, threading


class DoctorMQTTBridge(Node):
    """
    Doctor side bridge:
      ROS → MQTT:
        /doctor/target_position   -> rehab/target_position
      MQTT → ROS:
        rehab/joint_position      -> /doctor/haptic_ref

      Health:
        - Ping / Pong RTT, publish /network/latency_ms
        - Publish L1/L2/L3 and mirror to MQTT: rehab/safety/net_*

      Reconnect handshake:
        - MQTT Last Will:  rehab/doctor_alive="0" (retained)
        - On connect:      rehab/doctor_alive="1" (retained)
        - After healthy window: clear local flags and
          send a short, non-blocking burst of rehab/safety/clear_estop="1"
    """

    def __init__(self):
        super().__init__('doctor_mqtt_bridge')

        # ---- ROS wiring
        self.sub_target = self.create_subscription(
            Float32, '/doctor/target_position', self.ros_target_to_mqtt, 10
        )
        self.pub_haptic_ref = self.create_publisher(Float32, '/doctor/haptic_ref', 10)
        self.pub_latency_ms = self.create_publisher(Float32, '/network/latency_ms', 10)
        self.pub_warning    = self.create_publisher(Bool, '/network/warning', 10)          # L1
        self.pub_net_fault  = self.create_publisher(Bool, '/safety/network_fault', 10)     # L2
        self.pub_net_estop  = self.create_publisher(Bool, '/safety/network_estop', 10)     # L3

        # ---- HiveMQ Cloud (hard-coded for thesis)
        host   = "bea7b081570a4e03a77248e8b07072d9.s1.eu.hivemq.cloud"
        port   = 8883
        user   = "doctor"
        passwd = "Doctor123"

        self.mqtt = mqtt.Client()
        self.mqtt.username_pw_set(user, passwd)
        self.mqtt.tls_set()  # TLS for 8883

        # Last Will published by broker if we drop unexpectedly
        self.mqtt.will_set("rehab/doctor_alive", payload="0", qos=0, retain=True)

        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = self.on_mqtt_message
        self.mqtt.connect(host, port, keepalive=60)
        self.mqtt.loop_start()
        self.get_logger().info(f"[DoctorMQTTBridge] Connected securely to HiveMQ Cloud {host}:{port}")

        # ---- Heartbeat + thresholds
        self.ping_period_s     = 0.75
        self.warn_rtt_ms       = 300.0
        self.fault_rtt_ms      = 800.0
        self.no_pong_fault_s   = 1.5
        self.no_pong_estop_s   = 3.0
        self.good_required_s   = 2.0

        # ---- State
        self.ping_id       = 0
        self.outstanding   = {}  # id -> t0 ns
        self.last_ok_ns    = time.monotonic_ns()
        self.health_ok_since_ns = self.last_ok_ns

        self.consec_warn  = 0
        self.consec_fault = 0

        self.l1_warning = False
        self.l2_fault   = False
        self.l3_estop   = False

        self.estop_clear_sent = False  # one-shot per healthy cycle

        # Timers
        self.create_timer(self.ping_period_s, self.send_ping)
        self.create_timer(0.25, self.watchdog_tick)

    # ---------- ROS → MQTT ----------
    def ros_target_to_mqtt(self, msg: Float32):
        try:
            val = float(msg.data)
        except Exception:
            self.get_logger().error("Target publish received non-float data")
            return
        self.mqtt.publish("rehab/target_position", str(val))
        self.get_logger().info(f"[DoctorMQTTBridge] Sent target → MQTT: {val:.2f}°")

    # ---------- MQTT callbacks ----------
    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"[DoctorMQTTBridge] MQTT connected rc={rc}")
        client.subscribe("rehab/joint_position")
        client.subscribe("rehab/pong")
        # Mark alive (retained)
        client.publish("rehab/doctor_alive", "1", qos=0, retain=True)
        # Reset handshake on reconnect
        self.estop_clear_sent = False
        self.health_ok_since_ns = time.monotonic_ns()

    def on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic

        if topic == "rehab/joint_position":
            try:
                value = float(msg.payload.decode())
                out = Float32(); out.data = value
                self.pub_haptic_ref.publish(out)
                self.get_logger().info(f"[DoctorMQTTBridge] Haptic ref from MQTT: {value:.2f}°")
            except ValueError:
                self.get_logger().warn("[DoctorMQTTBridge] Invalid payload on rehab/joint_position")

        elif topic == "rehab/pong":
            try:
                payload = json.loads(msg.payload.decode())
                pid = int(payload.get("id", -1))
                t0  = int(payload.get("t0", 0))
            except Exception:
                self.get_logger().warn("[DoctorMQTTBridge] Bad pong JSON")
                return

            tnow = time.monotonic_ns()
            t0_sent = self.outstanding.pop(pid, None)
            if t0_sent is None or t0_sent != t0:
                return

            # RTT
            rtt_ms = (tnow - t0) / 1e6
            self.last_ok_ns = tnow

            lat = Float32(); lat.data = float(rtt_ms)
            self.pub_latency_ms.publish(lat)

            self.consec_warn  = self.consec_warn + 1 if rtt_ms > self.warn_rtt_ms  else 0
            self.consec_fault = self.consec_fault + 1 if rtt_ms >= self.fault_rtt_ms else 0

            rtt_warning = (self.consec_warn >= 3) and not self.l2_fault and not self.l3_estop
            rtt_fault   = (self.consec_fault >= 2)

            if not rtt_fault:
                self.health_ok_since_ns = tnow

            self.set_states(warning=rtt_warning, fault=rtt_fault, estop=False)

    # ---------- Ping / Watchdog ----------
    def send_ping(self):
        cutoff = time.monotonic_ns() - int(10 * 1e9)
        self.outstanding = {pid: t0 for pid, t0 in self.outstanding.items() if t0 >= cutoff}

        self.ping_id += 1
        t0 = time.monotonic_ns()
        self.outstanding[self.ping_id] = t0
        self.mqtt.publish("rehab/ping", json.dumps({"id": self.ping_id, "t0": t0}))

    def watchdog_tick(self):
        now_ns = time.monotonic_ns()
        age_s = (now_ns - self.last_ok_ns) / 1e9

        # timeouts dominate
        if age_s > self.no_pong_estop_s:
            self.set_states(False, False, True)
            self.estop_clear_sent = False
            return
        if age_s > self.no_pong_fault_s:
            self.set_states(False, True, False)
            self.estop_clear_sent = False
            return

        # Healthy long enough? clear + send handshake (once)
        healthy_s = (now_ns - self.health_ok_since_ns) / 1e9
        if healthy_s >= self.good_required_s:
            self.set_states(False, False, False)
            if not self.estop_clear_sent:
                # send clear_estop burst in a small thread to avoid blocking timers
                def _burst():
                    for _ in range(3):
                        self.mqtt.publish("rehab/safety/clear_estop", "1", qos=0, retain=False)
                        time.sleep(0.2)
                    self.get_logger().info("[DoctorMQTTBridge] Sent clear_estop burst after healthy window.")
                threading.Thread(target=_burst, daemon=True).start()
                self.estop_clear_sent = True

    # ---------- Helpers ----------
    def set_states(self, warning: bool, fault: bool, estop: bool):
        if estop:
            warning = False; fault = False
        elif fault:
            warning = False

        changed = (warning != self.l1_warning) or (fault != self.l2_fault) or (estop != self.l3_estop)
        self.l1_warning, self.l2_fault, self.l3_estop = warning, fault, estop

        if changed:
            b = Bool()
            b.data = self.l1_warning; self.pub_warning.publish(b)
            b.data = self.l2_fault;   self.pub_net_fault.publish(b)
            b.data = self.l3_estop;   self.pub_net_estop.publish(b)

            self.mqtt.publish("rehab/safety/net_warning", "1" if self.l1_warning else "0")
            self.mqtt.publish("rehab/safety/net_fault",   "1" if self.l2_fault   else "0")
            self.mqtt.publish("rehab/safety/net_estop",   "1" if self.l3_estop   else "0")

            self.get_logger().info(f"[DoctorMQTTBridge] L1={self.l1_warning} L2={self.l2_fault} L3={self.l3_estop}")


def main(args=None):
    rclpy.init(args=args)
    node = DoctorMQTTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down (Ctrl+C)")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
