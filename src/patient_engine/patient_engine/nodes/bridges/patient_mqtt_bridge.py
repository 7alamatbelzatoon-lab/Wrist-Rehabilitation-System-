#!/usr/bin/env python3
import os, json, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import paho.mqtt.client as mqtt
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Duration  # CHANGED: for deadline

MQTT_TOPIC_TARGET   = "rehab/target_position"   # doctor -> patient
MQTT_TOPIC_JOINT    = "rehab/joint_position"    # patient -> doctor
MQTT_TOPIC_PING     = "rehab/ping"              # doctor -> patient
MQTT_TOPIC_PONG     = "rehab/pong"              # patient -> doctor (echo)
MQTT_TOPIC_LAT_MS   = "rehab/latency_ms"        # doctor -> patient (float str mirror)   #   Final new part for duability

# CHANGED: fresh-only command QoS (RELIABLE + VOLATILE), larger depth, add deadline
qos_sensorData= QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )


class PatientMQTTBridge(Node):
    """
    Clean, dumb bridge:
      MQTT → ROS:
        rehab/target_position  -> /patient/target_position   (Float32)
        rehab/latency_ms       -> /patient_network/latency_ms(Float32)
      ROS → MQTT:
        /patient/joint_position-> rehab/joint_position       (float str, QoS0)
      RTT:
        echo rehab/ping back to rehab/pong (no local calc)
    """

    def __init__(self):
        super().__init__('patient_mqtt_bridge')

        # ROS wiring
        
        self.pub_target   = self.create_publisher(Float32, '/patient/target_position', qos_sensorData)
        self.pub_lat_pati = self.create_publisher(Float32, '/patient_network/latency_ms', qos_sensorData)
        
        self.sub_joint    = self.create_subscription(Float32, '/patient/joint_position', self._ros_joint_to_mqtt, qos_sensorData)

        # MQTT config (TLS; env overrides)
        host   = os.getenv("MQTT_HOST", "bea7b081570a4e03a77248e8b07072d9.s1.eu.hivemq.cloud")
        port   = int(os.getenv("MQTT_PORT", "8883"))
        user   = os.getenv("MQTT_USER", "doctor")
        passwd = os.getenv("MQTT_PASS", "Doctor123")
        # unique per run; still overridable via env
        client_id = os.getenv("MQTT_CLIENT_ID", f"patient-bridge-{os.getpid()}-{int(time.time())}")

        #client_id = os.getenv("MQTT_CLIENT_ID", "patient-bridge")    # OLD ID

        self.mqtt = mqtt.Client(client_id=client_id, clean_session=True)
        self.mqtt.username_pw_set(user, passwd)
        self.mqtt.tls_set()
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_message = self._on_mqtt_msg
        self.mqtt.connect(host, port, keepalive=60)
        self.mqtt.loop_start()
        self.get_logger().info(f"[PatientMQTTBridge] MQTT TLS connected {host}:{port} as {client_id}")

    # ROS → MQTT (patient joint upstream)
    def _ros_joint_to_mqtt(self, msg: Float32):
        self.mqtt.publish(MQTT_TOPIC_JOINT, str(float(msg.data)), qos=0, retain=False)
        self.get_logger().debug(f"[PatientMQTTBridge] joint→MQTT q0: {msg.data:.2f}°")

    # MQTT subscriptions
    def _on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"[PatientMQTTBridge] MQTT connected rc={rc}")
        client.subscribe(MQTT_TOPIC_TARGET, qos=1)
        client.subscribe(MQTT_TOPIC_PING,   qos=1)
        client.subscribe(MQTT_TOPIC_LAT_MS, qos=1)

    # MQTT → ROS
    def _on_mqtt_msg(self, client, userdata, msg):
        if msg.topic == MQTT_TOPIC_TARGET:
            raw = msg.payload.decode().strip()
            val = None
            try:
                data = json.loads(raw)
                val = float(data["deg"]) if isinstance(data, dict) and "deg" in data else float(data)
            except Exception:
                try:
                    val = float(raw)
                except Exception:
                    self.get_logger().warning("[PatientMQTTBridge] invalid target payload")
                    return
            self.pub_target.publish(Float32(data=val))
            self.get_logger().debug(f"[PatientMQTTBridge] target←MQTT q1: {val:.2f}°")

        elif msg.topic == MQTT_TOPIC_PING:
            # echo back for RTT calc on doctor side
            try:
                payload = json.loads(msg.payload.decode())
                self.mqtt.publish(MQTT_TOPIC_PONG, json.dumps(payload), qos=1, retain=False)
            except Exception:
                pass

        elif msg.topic == MQTT_TOPIC_LAT_MS:
            try:
                v = float(msg.payload.decode())
                self.pub_lat_pati.publish(Float32(data=v))
                self.get_logger().debug(f"[PatientMQTTBridge] RTT mirror: {v:.2f} ms")
            except ValueError:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = PatientMQTTBridge()
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
