#!/usr/bin/env python3
import os, json, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import paho.mqtt.client as mqtt
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from builtin_interfaces.msg import Duration

MQTT_TOPIC_TARGET   = "rehab/target_position"   # doctor -> patient (JSON)
MQTT_TOPIC_JOINT    = "rehab/joint_position"    # patient -> doctor (float str)
MQTT_TOPIC_PING     = "rehab/ping"              # doctor -> patient (JSON)
MQTT_TOPIC_PONG     = "rehab/pong"              # patient -> doctor (JSON echo)
MQTT_TOPIC_LAT_MS   = "rehab/latency_ms"        # doctor -> patient (float str mirror)

qos_sensorData = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,     # no latching for commands
)

class DoctorMQTTBridge(Node):
    """
    Clean, dumb bridge:
      ROS → MQTT:
        /doctor/target_position  -> rehab/target_position  (JSON, QoS1, retain=False)
        RTT mirror               -> rehab/latency_ms       (float str, QoS1, retain=False)
      MQTT → ROS:
        rehab/joint_position     -> /doctor/haptic_ref     (Float32)
      RTT:
        send rehab/ping, expect rehab/pong, compute RTT → /doctor_network/latency_ms
    """

    def __init__(self):
        super().__init__('doctor_mqtt_bridge')

        # ROS wiring
        self.sub_target  = self.create_subscription(Float32, '/doctor/target_position', self._ros_target_to_mqtt, qos_sensorData)

        # this variable we use for GUI doictor to display current patient angle recieved no subscribers yet!
        self.pub_haptic  = self.create_publisher(Float32, '/doctor/haptic_ref', qos_sensorData) 

        # what we will use for network anomaly detection on doctor side                     
        self.pub_lat_doc = self.create_publisher(Float32, '/doctor_network/latency_ms', qos_sensorData)

        # MQTT config (TLS; env overrides)
        host   = os.getenv("MQTT_HOST", "bea7b081570a4e03a77248e8b07072d9.s1.eu.hivemq.cloud")
        port   = int(os.getenv("MQTT_PORT", "8883"))
        user   = os.getenv("MQTT_USER", "doctor")
        passwd = os.getenv("MQTT_PASS", "Doctor123")
        client_id = os.getenv("MQTT_CLIENT_ID", f"doctor-bridge-{int(time.time())}")

        self.mqtt = mqtt.Client(client_id=client_id, clean_session=True)
        self.mqtt.username_pw_set(user, passwd)
        self.mqtt.tls_set()
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_message = self._on_mqtt_msg
        self.mqtt.connect(host, port, keepalive=60)
        self.mqtt.loop_start()
        self.get_logger().info(f"[DoctorMQTTBridge] MQTT TLS connected {host}:{port} as {client_id}")

        # RTT state
        self.ping_id     = 0
        self.outstanding = {}  # id -> t0_ns
        self.max_outstanding_ids = 5  # keep only the most recent 5 pings * memory opitimization *
        self.create_timer(1.0, self._send_ping)

    # ROS → MQTT
    def _ros_target_to_mqtt(self, msg: Float32):
        payload = json.dumps({"deg": float(msg.data), "t0_ns": time.monotonic_ns()})
        # CHANGED: retain=False for fresh-only commands (no stale replay)
        self.mqtt.publish(MQTT_TOPIC_TARGET, payload, qos=1, retain=False)
        self.get_logger().debug(f"[DoctorMQTTBridge] target→MQTT q1,no-retain: {msg.data:.2f}°")

    # MQTT callbacks
    def _on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"[DoctorMQTTBridge] MQTT connected rc={rc}")
        client.subscribe(MQTT_TOPIC_JOINT, qos=0)
        client.subscribe(MQTT_TOPIC_PONG,  qos=1)

    def _on_mqtt_msg(self, client, userdata, msg):
        if msg.topic == MQTT_TOPIC_JOINT:
            try:
                v = float(msg.payload.decode())
                self.pub_haptic.publish(Float32(data=v))
                self.get_logger().debug(f"[DoctorMQTTBridge] haptic_ref: {v:.2f}°")
            except ValueError:
                self.get_logger().warning("[DoctorMQTTBridge] invalid rehab/joint_position payload")
        elif msg.topic == MQTT_TOPIC_PONG:
            try:
                data = json.loads(msg.payload.decode())
                pid = int(data.get("id", -1)); t0 = int(data.get("t0", 0))
            except Exception:
                return
            tnow = time.monotonic_ns()
            t0_sent = self.outstanding.pop(pid, None)
            if t0_sent is None or t0_sent != t0:
                return
            rtt_ms = (tnow - t0) / 1e6
            self.pub_lat_doc.publish(Float32(data=float(rtt_ms)))
            # mirror to MQTT so patient can also display/republish it
            self.mqtt.publish(MQTT_TOPIC_LAT_MS, f"{rtt_ms:.3f}", qos=1, retain=False)
            self.get_logger().debug(f"[DoctorMQTTBridge] RTT={rtt_ms:.2f} ms")


    #  this method is to clear memory
    def _prune_outstanding(self):
        """Keep only the most recent N ping ids to avoid unbounded growth if pongs are missed."""
    # Fast path: nothing to prune
        if len(self.outstanding) <= self.max_outstanding_ids:
            return
    # Remove oldest ids, keep the newest N
    # ids are monotonically increasing; sorting is fine at this tiny size
        keep_from = sorted(self.outstanding.keys())[-self.max_outstanding_ids:]
        keep_set = set(keep_from)
    # Rebuild dict with only the ids we keep (avoids mutating while iterating)
        self.outstanding = {k: self.outstanding[k] for k in keep_from}

    # Ping (1 Hz)
    def _send_ping(self):
        self.ping_id += 1
        t0 = time.monotonic_ns()
        self.outstanding[self.ping_id] = t0
        self._prune_outstanding()
        self.mqtt.publish(MQTT_TOPIC_PING, json.dumps({"id": self.ping_id, "t0": t0}), qos=1, retain=False)

def main(args=None):
    rclpy.init(args=args)
    node = DoctorMQTTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt.loop_stop()
        node.mqtt.disconnect()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
