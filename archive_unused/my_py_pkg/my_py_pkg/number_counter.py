#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64  


class numbercounternode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.Current_=0
        self.subsciber_ = self.create_subscription(Int64,"number", self.callback_numberAdder, 10)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        #self.timer_ = self.create_timer(1, self.publish_data)   publish only when receive!
        self.get_logger().info("Number Counter Initiated.")
    
    def callback_numberAdder(self, msg: Int64):
        self.Current_ += msg.data
        out_msg = Int64()
        out_msg.data=self.Current_
        self.publisher_.publish(out_msg)
    
  #  def publish_data(self):
   #     msg = Int64()
   #     msg.data = self.Current_
    #    self.publisher_.publish(msg)




def main(args=None):
    rclpy.init(args=args)
    node = numbercounternode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()