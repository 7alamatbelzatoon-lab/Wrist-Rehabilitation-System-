#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import paho.mqtt.client as mqtt
import json, time


class PatientMQTTBridge(Node):
    """
    Patient side bridge:
      MQTT → ROS:
        rehab/target_position  -> /patient/target_position
        rehab/safety/net_*     -> /network/warning, /safety/network_fault, /safety/network_estop
        rehab/doctor_alive     -> reconnection awareness
        rehab/safety/clear_estop -> authorize clearing when healthy
      ROS → MQTT:
        /patient/joint_position -> rehab/joint_position

      RTT helper:
        - listens to rehab/ping and echoes rehab/pong (doctor computes RTT)

      Local safety:
        - watchdog: if no 'rehab/ping' seen for > thresholds, publish local L2/L3
    """

    def __init__(self):
        super().__init__('patient_mqtt_bridge')

        # ROS wiring
        self.pub_target = self.create_publisher(Float32, '/patient/target_position', 10)
        self.sub_joint  = self.create_subscription(Float32, '/patient/joint_position',
                                                   self.ros_joint_to_mqtt, 10)

        self.pub_warning   = self.create_publisher(Bool, '/network/warning', 10)
        self.pub_net_fault = self.create_publisher(Bool, '/safety/network_fault', 10)
        self.pub_net_estop = self.create_publisher(Bool, '/safety/network_estop', 10)

        # Local watchdog / health
        self.last_ping_ns       = time.monotonic_ns()
        self.no_ping_fault_s    = 1.5
        self.no_ping_estop_s    = 3.0
        self.good_required_s    = 2.0
        self.health_ok_since_ns = self.last_ping_ns

        self.debounce_needed = 2
        self.debounce_fault = 0
        self.debounce_estop = 0
        self.debounce_ok    = 0

        self.l1_warning = False
        self.l2_fault   = False
        self.l3_estop   = False

        self.doctor_alive = False

        self.create_timer(0.25, self.local_watchdog_tick)

        # HiveMQ (hard-coded)
        host   = "bea7b081570a4e03a77248e8b07072d9.s1.eu.hivemq.cloud"
        port   = 8883
        user   = "doctor"
        passwd = "Doctor123"

        self.mqtt = mqtt.Client()
        self.mqtt.username_pw_set(user, passwd)
        self.mqtt.tls_set()
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = self.on_mqtt_message
        self.mqtt.connect(host, port, keepalive=60)
        self.mqtt.loop_start()
        self.get_logger().info(f"[PatientMQTTBridge] Connected securely to HiveMQ Cloud {host}:{port}")

    # ROS → MQTT
    def ros_joint_to_mqtt(self, msg: Float32):
        try:
            val = float(msg.data)
        except Exception:
            return
        self.mqtt.publish("rehab/joint_position", str(val))
        self.get_logger().info(f"[PatientMQTTBridge] Sent joint: {val:.2f}°")

    # MQTT subscriptions
    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"[PatientMQTTBridge] MQTT connected rc={rc}")
        client.subscribe("rehab/target_position")
        client.subscribe("rehab/ping")
        client.subscribe("rehab/safety/net_warning")
        client.subscribe("rehab/safety/net_fault")
        client.subscribe("rehab/safety/net_estop")
        client.subscribe("rehab/doctor_alive")
        client.subscribe("rehab/safety/clear_estop")

    # MQTT → ROS
    def on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode().strip()

        if topic == "rehab/target_position":
            try:
                value = float(payload)
                out = Float32(); out.data = value
                self.pub_target.publish(out)
                self.get_logger().info(f"[PatientMQTTBridge] Received target: {value:.2f}°")
            except ValueError:
                self.get_logger().warn("[PatientMQTTBridge] Invalid target payload")

        elif topic == "rehab/ping":
            try:
                data = json.loads(payload)
                now = time.monotonic_ns()
                self.last_ping_ns = now
                self.health_ok_since_ns = now
                self.mqtt.publish("rehab/pong", json.dumps(data))
            except Exception:
                pass

        elif topic == "rehab/doctor_alive":
            self.doctor_alive = (payload == "1" or payload.lower() == "true")
            self.get_logger().info(f"[PatientMQTTBridge] doctor_alive={self.doctor_alive}")

        elif topic == "rehab/safety/clear_estop":
            # Only honor when locally healthy and doctor is alive
            now = time.monotonic_ns()
            healthy_s = (now - self.health_ok_since_ns) / 1e9
            if healthy_s >= self.good_required_s and self.doctor_alive:
                self._set_states(False, False, False, note="CLEAR via doctor handshake")
            else:
                self.get_logger().info("[PatientMQTTBridge] Ignored clear_estop (not healthy/doctor not alive)")

        elif topic == "rehab/safety/net_warning":
            is_on = (payload == "1" or payload.lower() == "true")
            self._set_states(warning=is_on, fault=self.l2_fault, estop=self.l3_estop, note=f"net_warning={is_on}")

        elif topic == "rehab/safety/net_fault":
            is_on = (payload == "1" or payload.lower() == "true")
            # fault suppresses warning
            self._set_states(warning=False if is_on else self.l1_warning, fault=is_on, estop=self.l3_estop,
                             note=f"net_fault={is_on}")

        elif topic == "rehab/safety/net_estop":
            is_on = (payload == "1" or payload.lower() == "true")
            if is_on:
                self._set_states(False, False, True, note="net_estop=1")
            else:
                # allow a clear via mirrored flag too, if locally healthy + doctor alive
                now = time.monotonic_ns()
                healthy_s = (now - self.health_ok_since_ns) / 1e9
                if healthy_s >= self.good_required_s and self.doctor_alive:
                    self._set_states(False, False, False, note="CLEAR via net_estop=0")
                else:
                    self.get_logger().info("[PatientMQTTBridge] net_estop=0 seen (not healthy yet)")

    # ---- Local watchdog (backup)
    def local_watchdog_tick(self):
        now = time.monotonic_ns()
        age_s = (now - self.last_ping_ns) / 1e9

        want_estop = age_s > self.no_ping_estop_s
        want_fault = (age_s > self.no_ping_fault_s) and not want_estop

        if want_estop:
            self.debounce_estop += 1; self.debounce_fault = 0; self.debounce_ok = 0
        elif want_fault:
            self.debounce_fault += 1; self.debounce_estop = 0; self.debounce_ok = 0
        else:
            self.debounce_ok += 1; self.debounce_fault = 0; self.debounce_estop = 0

        raise_estop = (self.debounce_estop >= self.debounce_needed)
        raise_fault = (self.debounce_fault >= self.debounce_needed)

        if raise_estop and not self.l3_estop:
            self._set_states(False, False, True, note=f"LOCAL ESTOP (no ping {age_s:.2f}s)")
            return
        if raise_fault and not self.l2_fault and not self.l3_estop:
            self._set_states(False, True, False, note=f"LOCAL FAULT (no ping {age_s:.2f}s)")
            return

        if not want_fault and not want_estop:
            healthy_s = (now - self.health_ok_since_ns) / 1e9
            if healthy_s >= self.good_required_s:
                if self.l3_estop or self.l2_fault or self.l1_warning:
                    self._set_states(False, False, False, note="LOCAL CLEAR (healthy)")

    def _set_states(self, warning: bool, fault: bool, estop: bool, note: str = ""):
        # Priority: L3 > L2 > L1
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
            if note:
                self.get_logger().warn(f"[PatientMQTTBridge] {note}")
            self.get_logger().info(f"[PatientMQTTBridge] L1={self.l1_warning} L2={self.l2_fault} L3={self.l3_estop}")


def main(args=None):
    rclpy.init(args=args)
    node = PatientMQTTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down (Ctrl+C)")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
