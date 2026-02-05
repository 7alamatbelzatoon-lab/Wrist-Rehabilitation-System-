#!/usr/bin/env python3
import os, json, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import paho.mqtt.client as mqtt
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Duration  # CHANGED: for deadline
from std_msgs.msg import Empty      #for buttons
from std_msgs.msg import String     #for events



MQTT_TOPIC_TARGET   = "rehab/target_position"           # doctor -> patient
MQTT_TOPIC_JOINT    = "rehab/joint_position"            # patient -> doctor
MQTT_TOPIC_PING     = "rehab/ping"                      # doctor -> patient
MQTT_TOPIC_PONG     = "rehab/pong"                      # patient -> doctor (echo)
MQTT_TOPIC_LAT_MS   = "rehab/latency_ms"                # doctor -> patient (float str mirror)   #   Final new part for duability
MQTT_TOPIC_ESTOP  = "rehab/cmd/manual_estop"            # doctor -> patient
MQTT_TOPIC_SLEEP  = "rehab/cmd/manual_sleep"            # doctor -> patient
MQTT_TOPIC_CLEAR  = "rehab/cmd/clear"                   # doctor -> patient
MQTT_TOPIC_RESUME = "rehab/cmd/resume"                  # doctor -> patient
MQTT_TOPIC_ORCH_EVENT    = "rehab/orchestrator/event"   # patient -> doctor
MQTT_TOPIC_MOTION_CAUSE  = "rehab/status/motion"        # patient -> doctor
MQTT_TOPIC_NETWORK_CAUSE = "rehab/status/network"       # patient -> doctor



# CHANGED: fresh-only command QoS (RELIABLE + VOLATILE), larger depth, add deadline
qos_sensorData= QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
qos_edge = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
qos_level_sub = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
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
        self._closing = False


        # ROS wiring
        
        self.pub_target   = self.create_publisher(Float32, '/patient/target_position', qos_sensorData)
        self.pub_lat_pati = self.create_publisher(Float32, '/patient_network/latency_ms', qos_sensorData)
        self.pub_estop  = self.create_publisher(Empty, '/doctor/manual_estop', qos_edge)
        self.pub_sleep  = self.create_publisher(Empty, '/doctor/manual_sleep', qos_edge)
        self.pub_clear  = self.create_publisher(Empty, '/doctor/clear',        qos_edge)
        self.pub_resume = self.create_publisher(Empty, '/doctor/resume',       qos_edge)
        self.create_subscription(String, '/orchestrator/event',    self._ros_orch_event_to_mqtt,   qos_edge)
        self.create_subscription(String, '/safety/motion/cause',   self._ros_motion_cause_to_mqtt, qos_level_sub)
        self.create_subscription(String, '/safety/network/cause',  self._ros_network_cause_to_mqtt,qos_level_sub)

        
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

    def _ros_orch_event_to_mqtt(self, msg: String):
        self.mqtt.publish(MQTT_TOPIC_ORCH_EVENT, msg.data, qos=1, retain=False)

    def _ros_motion_cause_to_mqtt(self, msg: String):
        self.mqtt.publish(MQTT_TOPIC_MOTION_CAUSE, msg.data, qos=1, retain=True)

    def _ros_network_cause_to_mqtt(self, msg: String):
        self.mqtt.publish(MQTT_TOPIC_NETWORK_CAUSE, msg.data, qos=1, retain=True)


    # MQTT subscriptions
    def _on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"[PatientMQTTBridge] MQTT connected rc={rc}")
        client.subscribe(MQTT_TOPIC_TARGET, qos=1)
        client.subscribe(MQTT_TOPIC_PING,   qos=1)
        client.subscribe(MQTT_TOPIC_LAT_MS, qos=1)
        client.subscribe(MQTT_TOPIC_ESTOP,  qos=2)
        client.subscribe(MQTT_TOPIC_SLEEP,  qos=1)
        client.subscribe(MQTT_TOPIC_CLEAR,  qos=1)
        client.subscribe(MQTT_TOPIC_RESUME, qos=1)


    # MQTT → ROS
    def _on_mqtt_msg(self, client, userdata, msg):
        if self._closing or (not rclpy.ok()):
            return
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

        elif msg.topic == MQTT_TOPIC_ESTOP:
            self.pub_estop.publish(Empty())

        elif msg.topic == MQTT_TOPIC_SLEEP:
            self.pub_sleep.publish(Empty())

        elif msg.topic == MQTT_TOPIC_CLEAR:
            self.pub_clear.publish(Empty())

        elif msg.topic == MQTT_TOPIC_RESUME:
            self.pub_resume.publish(Empty())
    

def main(args=None):
    rclpy.init(args=args)
    node = PatientMQTTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._closing = True

    # stop MQTT callbacks first
        try:
            node.mqtt.on_message = None
            node.mqtt.on_connect = None
        except Exception:
            pass

        try:
            node.mqtt.loop_stop()
        except Exception:
            pass

        try:
            node.mqtt.disconnect()
        except Exception:
            pass

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
