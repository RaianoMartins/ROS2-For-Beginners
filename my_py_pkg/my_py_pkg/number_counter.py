#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter = 0
        self.number_subscriber = self.create_subscription(
            Int64,"number",self.callback_number,10)   
        self.number_publisher = self.create_publisher(
            Int64, "number_count", 10)   
        self.server  = self.create_service(
            SetBool,"reset_number_counter",self.callback_reset)
        self.get_logger().info("Countador iniciado!")

    def callback_reset(self,request,response):
        if (request.data):
            self.counter = 0
            response.success = True
            response.message = "Contador zerado!"
        else:
            response.success = False
            response.message = "Continuar contagem!"
        return response
 
    def callback_number(self,msg):
        self.counter += msg.data
        self.get_logger().info("Contagem " + str(self.counter))
        new_msg = Int64()
        new_msg.data = self.counter
        self.number_publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()