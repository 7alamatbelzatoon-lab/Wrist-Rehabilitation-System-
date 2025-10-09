#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64      #3amalna import lelmsg type le 3ayzeeno


class numberpublishernode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.number_publisher_ = self.create_publisher(Int64, "number", 10)
        self.timer_ = self.create_timer(1, self.publish_number)       #marra fel sanya beypublish
        self.get_logger().info("Numbers are now being sent")
       
    def publish_number(self):
        msg = Int64()
        msg.data = 2
        self.number_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = numberpublishernode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()