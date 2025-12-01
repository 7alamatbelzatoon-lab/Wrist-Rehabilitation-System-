#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32


qos_sensorData = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class PatientCurrentController(Node):
    """
    PD controller that takes a desired joint angle (from patient_node)
    and the measured angle (from DynamixelDriver) and outputs motor
    current in mA, similar to Controller.set_target_position() in the
    old system but running continuously as a ROS node.
    """

    def __init__(self):
        super().__init__("patient_current_controller")

        # PD gains – start with the same defaults as his Controller
        self.kp = float(self.declare_parameter("kp", 1.0).value)        #for tuning start with a low value while kd=0 then handle overshoots with kd step increase
        self.kd = float(self.declare_parameter("kd", 0.1).value)        # tune with outer loop on!

        # Safety: limit commanded current
        self.max_current_mA = float(self.declare_parameter("max_current_mA", 1500.0).value)

        # Control loop rate
        self.rate_hz = float(self.declare_parameter("rate_hz", 100.0).value)    # 100Hz the loop period is 10 ms, which is more in line with the USB latency (16 ms)
        self.dt = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.01

        # State
        self.command_deg = 0.0      # from patient_node
        self.measured_deg = 0.0     # from driver
        self.have_measurement = False
        self.prev_error = 0.0

        # Subscribers
        self.sub_cmd = self.create_subscription(
            Float32,
            "/patient/command_position",
            self._command_cb,
            qos_sensorData,
        )

        self.sub_meas = self.create_subscription(
            Float32,
            "/patient/joint_position",
            self._meas_cb,
            qos_sensorData,
        )

        # Publisher: desired motor current (mA)
        self.pub_current = self.create_publisher(
            Float32,
            "/patient/goal_current_mA",
            qos_sensorData,
        )

        # Timer loop
        self.timer = self.create_timer(self.dt, self._control_step)

        self.get_logger().info(
            f"PatientCurrentController started "
            f"(kp={self.kp}, kd={self.kd}, max_current={self.max_current_mA} mA, rate={self.rate_hz} Hz)"
        )

    # ---------- Callbacks ----------

    def _command_cb(self, msg: Float32):
        # Desired joint angle from patient_node (deg)
        self.command_deg = float(msg.data)

    def _meas_cb(self, msg: Float32):
        # Measured joint angle from DynamixelDriver (deg)
        self.measured_deg = float(msg.data)
        self.have_measurement = True

    # ---------- Control loop ----------

    def _control_step(self):
        if not self.have_measurement:
            return

        error = self.command_deg - self.measured_deg       # deg
        derr = (error - self.prev_error) / self.dt         # deg/s

        # PD law -> current in mA (like his Controller, but in ROS)
        current_mA = self.kp * error + self.kd * derr

        # Saturate to safe range
        if current_mA > self.max_current_mA:
            current_mA = self.max_current_mA
        elif current_mA < -self.max_current_mA:
            current_mA = -self.max_current_mA

        self.pub_current.publish(Float32(data=float(current_mA)))
        self.prev_error = error


def main(args=None):
    rclpy.init(args=args)
    node = PatientCurrentController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
