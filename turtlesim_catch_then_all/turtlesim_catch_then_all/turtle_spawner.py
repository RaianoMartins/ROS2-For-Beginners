#!/usr/bin/env python3
import rclpy
import math
import random
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from functools import partial

 
class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner") 
        self.declare_parameter("spawn_frequency",1.0)
        self.declare_parameter("turtle_name_prefix","turtle")

        self.spawn_frequency = self.get_parameter("spawn_frequency").value
        self.turtle_name_prefix  = self.get_parameter("turtle_name_prefix").value
        self.turtle_name_counter = 1
        self.alive_turtles = []

        self.alive_turtles_publisher = self.create_publisher(TurtleArray,"alive_turtles",10)
        self.spawn_timer = self.create_timer(
            1.0/self.spawn_frequency ,self.spawn_new_turle)
        self.catch_turtle_service = self.create_service(
            CatchTurtle,"catch_turtle",self.callback_catch_turtle)
 
    def callback_catch_turtle(self, request,response):
        self.call_kill_server(request.name)
        response.success = True
        return response

    def spawn_new_turle(self):
        self.turtle_name_counter += 1
        name = self.turtle_name_prefix + str(self.turtle_name_counter)
        x = random.uniform(0.0,11.0)
        y = random.uniform(0.0,11.0)
        theta = random.uniform(0.0,2*math.pi)
        self.call_spawn_server(x,y,theta,name)

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles
        self.alive_turtles_publisher.publish(msg)
        
    def call_spawn_server(self,x,y,theta,turtle_name):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0): #Aguarda até que o server esteja pronto  
            self.get_logger().warn("Esperando pelo server...") #Envia o warning a uma taxa de 1hz
            
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn_server,x=x,y=y,theta=theta,turtle_name=turtle_name))
        #partial permite adicionar mais parametros, 
        #caso o contrario apenas future seria passo a função.
        
    def callback_call_spawn_server(self,future,x,y,theta,turtle_name):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info("Turtle " + response.name + " is alive!")
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x 
                new_turtle.y = y
                new_turtle.theta = theta
                self.alive_turtles.append(new_turtle)
                self.publish_alive_turtles()

        except Exception as erro:
            self.get_logger().error("Chamada de servico falhou! %r" % (erro,))

    def call_kill_server(self,turtle_name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0): #Aguarda até que o server esteja pronto  
            self.get_logger().warn("Esperando pelo server...") #Envia o warning a uma taxa de 1hz
            
        request = Kill.Request()
        request.name = turtle_name
    
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_kill_server,turtle_name=turtle_name))
        #partial permite adicionar mais parametros, 
        #caso o contrario apenas future seria passo a função.

    def callback_call_kill_server(self,future,turtle_name):
        try:
            response = future.result()
            for (i,turtle) in enumerate(self.alive_turtles):
               if turtle.name == turtle_name:
                   del self.alive_turtles[i]
                   self.publish_alive_turtles()
                   break 
        except Exception as erro:
            self.get_logger().error("Chamada de servico falhou! %r" % (erro,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()