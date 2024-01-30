#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
 
 
class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.declare_parameter("published_number",1)
        self.declare_parameter("publish_frequency",1.0)

        self.number = self.get_parameter("published_number").value
        self.publish_frequency = self.get_parameter("publish_frequency").value
        self.number_publisher = self.create_publisher(Int64, "number", 10)   
        self.number_timer = self.create_timer(1/self.publish_frequency, self.publish_number)
        self.get_logger().info("Enviando o número " + str(self.number))
 
    def publish_number(self):
        msg = Int64()
        msg.data = self.number
        self.number_publisher.publish(msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()