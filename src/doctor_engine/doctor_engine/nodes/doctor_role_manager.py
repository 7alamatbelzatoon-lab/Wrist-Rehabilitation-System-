#This node is to be used with doctor dxl in case of role managing for safety protocol


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class DoctorRoleManager(Node):
    """
    Uses patient-side /patient/control_mode to decide:
      - If patient mode != NORMAL: doctor follows patient (publish /doctor/goal_position = q_p)
      - If patient mode == NORMAL: doctor is master (publish /patient/target_position = q_d + offset)
    Offset is updated once on transition follower->master to avoid jumps.
    """

    def __init__(self):
        super().__init__("doctor_role_manager")

        # Params: "feel" via current limit; start constant (low test burden)
        self.declare_parameter("current_limit_master_mA", 220.0)
        self.declare_parameter("current_limit_follow_mA", 150.0)
        self.I_master = float(self.get_parameter("current_limit_master_mA").value)
        self.I_follow = float(self.get_parameter("current_limit_follow_mA").value)

        self.mode_patient = "FREEZE"
        self.prev_is_master = False

        self.q_d = None
        self.q_p = None
        self.offset = 0.0

        # Subs
        self.create_subscription(String,  "/patient/control_mode",  self.cb_mode,   10)
        self.create_subscription(Float32, "/doctor/joint_position", self.cb_doctor, 10)
        self.create_subscription(Float32, "/patient/joint_position", self.cb_patient,10)

        # Pubs
        self.pub_patient_target = self.create_publisher(Float32, "/patient/target_position", 10)
        self.pub_doctor_goal    = self.create_publisher(Float32, "/doctor/goal_position", 10)
        self.pub_doctor_ilim    = self.create_publisher(Float32, "/doctor/current_limit_mA", 10)

        # 50 Hz decision loop
        self.timer = self.create_timer(0.02, self.tick)

    def cb_mode(self, msg: String):
        self.mode_patient = (msg.data or "").strip().upper()

    def cb_doctor(self, msg: Float32):
        self.q_d = float(msg.data)

    def cb_patient(self, msg: Float32):
        self.q_p = float(msg.data)

    def tick(self):
        if self.q_d is None or self.q_p is None:
            return

        is_master = (self.mode_patient == "NORMAL")

        if is_master:
            # follower->master transition: clutch offset so patient doesn't jump
            if not self.prev_is_master:
                self.offset = self.q_p - self.q_d

            self.pub_doctor_ilim.publish(Float32(data=float(self.I_master)))
            self.pub_patient_target.publish(Float32(data=float(self.q_d + self.offset)))
        else:
            # any non-NORMAL mode: doctor follows patient
            self.pub_doctor_ilim.publish(Float32(data=float(self.I_follow)))
            self.pub_doctor_goal.publish(Float32(data=float(self.q_p)))

        self.prev_is_master = is_master


def main(args=None):
    rclpy.init(args=args)
    node = DoctorRoleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
