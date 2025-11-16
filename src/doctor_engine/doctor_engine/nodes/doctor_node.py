#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from builtin_interfaces.msg import Duration

qos_sensorData = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,     # no latching for commands
)

class DoctorNode(Node):
    def __init__(self):
        super().__init__("doctor_node")
        
        # ROS Publisher: sending target position to the patient
        self.publisher_ = self.create_publisher(Float32, '/doctor/target_position', qos_sensorData)
        
        # Predefined sequence of angles to test the adaptive system
        self.angle_sequence = [
            0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 55.0, 50.0, 45.0, 40.0, 
            35.0, 30.0, 25.0, 20.0, 15.0, 10.0, 5.0, 0.0
       ]
        #self.angle_sequence = [
        #0.0, -5.0, -10.0, -15.0, -20.0, -25.0, -30.0, -35.0, -40.0, -45.0, -50.0, -55.0,
        #-60.0, -65.0, -66.0, -66.0, -64.0, -60.0, -55.0, -50.0, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0.0
       # ]           

        self.index = 0
        
        # Timer to periodically publish target position
        self.timer = self.create_timer(1.0, self.publish_target_position)
        
        self.get_logger().info("Doctor Node initiated!")

    # Method to publish target position from the sequence
    def publish_target_position(self):
        if self.index < len(self.angle_sequence):
            target = self.angle_sequence[self.index]
            msg = Float32()
            msg.data = target
            self.publisher_.publish(msg)
            self.get_logger().info(f'[Doctor] 🧠 Published target: {msg.data:.2f}°')
            self.index += 1
        else:
            self.get_logger().info("[Doctor] ✅ Finished publishing all target angles.")
            self.destroy_node()  # Stop the node after publishing all targets

def main(args=None):
    rclpy.init(args=args)
    node = DoctorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
