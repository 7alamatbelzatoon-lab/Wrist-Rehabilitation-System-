#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32

# QoS similar to patient_node / existing controllers
qos_sensorData = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class PatientVelocityController(Node):
    """
    Simple PD controller on joint position that outputs a desired joint velocity [deg/s].

    - Subscribes:
        /patient/command_position  [Float32, deg]   (from patient_node)
        /patient/joint_position    [Float32, deg]   (from hardware driver)
    - Publishes:
        /patient/goal_velocity_deg_s  [Float32, deg/s] (to dxl_velocity_driver)
    """

    def __init__(self):
        super().__init__("patient_velocity_controller")

        # -------- Parameters --------
        # PD gains mapping position error → velocity (deg/s)
        self.declare_parameter("kp", 1.0)         # deg/s per deg of error
        self.declare_parameter("kd", 0.1)         # deg/s per (deg/s) of error derivative

        # Max absolute velocity to command (safety + smoothness)
        self.declare_parameter("max_velocity_deg_s", 60.0)   # e.g. 120 deg/s ≈ 20 rpm

        # Control loop rate
        self.declare_parameter("rate_hz", 100.0)

        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.max_velocity_deg_s = float(self.get_parameter("max_velocity_deg_s").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.dt = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 0.01

        # -------- Internal state --------
        self.command_deg = 0.0
        self.measured_deg = 0.0
        self.prev_error = 0.0
        self.have_measurement = False

        # -------- ROS interfaces --------

        # Commanded joint angle from patient_node
        self.sub_cmd = self.create_subscription(
            Float32,
            "/patient/command_position",
            self._command_cb,
            qos_sensorData,
        )

        # Measured joint angle from hardware driver
        self.sub_meas = self.create_subscription(
            Float32,
            "/patient/joint_position",
            self._meas_cb,
            qos_sensorData,
        )

        # Desired joint velocity [deg/s] to velocity driver
        self.pub_vel = self.create_publisher(
            Float32,
            "/patient/goal_velocity_deg_s",
            qos_sensorData,
        )

        # Timer loop
        self.timer = self.create_timer(self.dt, self._control_step)

        self.get_logger().info(
            f"PatientVelocityController started "
            f"(kp={self.kp}, kd={self.kd}, "
            f"v_max={self.max_velocity_deg_s} deg/s, rate={self.rate_hz} Hz)"
        )

    # ---------- Callbacks ----------

    def _command_cb(self, msg: Float32):
        # Desired joint angle from patient_node (deg)
        self.command_deg = float(msg.data)

    def _meas_cb(self, msg: Float32):
        # Measured joint angle from Dynamixel driver (deg)
        self.measured_deg = float(msg.data)
        self.have_measurement = True

    # ---------- Control loop ----------

    def _control_step(self):
        if not self.have_measurement:
            return

        error = self.command_deg - self.measured_deg       # deg
        derr = (error - self.prev_error) / self.dt         # deg/s

        # PD law -> desired joint velocity [deg/s]
        vel_cmd = self.kp * error + self.kd * derr

        # Saturate for safety / smoothness
        v_max = abs(self.max_velocity_deg_s)
        if vel_cmd > v_max:
            vel_cmd = v_max
        elif vel_cmd < -v_max:
            vel_cmd = -v_max

        self.pub_vel.publish(Float32(data=float(vel_cmd)))
        self.prev_error = error


def main(args=None):
    rclpy.init(args=args)
    node = PatientVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
