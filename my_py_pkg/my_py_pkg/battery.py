import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from functools import partial
 
 
class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery")
        self.battery_sate = "full"
        self.last_battery_change = self.get_current_time_seconds()
        self.battery_timer = self.create_timer(0.1,self.check_battery_state)
        self.get_logger().info("Bateria iniciou")

    def get_current_time_seconds(self):
        seconds,nanosseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanosseconds/1000000000
    
    def check_battery_state(self):
        time = self.get_current_time_seconds()
        if self.battery_sate == "full":
            if (time - self.last_battery_change > 4):
                self.battery_sate = "empty"
                self.get_logger().info("Battery is empty! Charging ...")
                self.last_battery_change = time
                self.call_set_led_server(3,1)

        else:
            if (time - self.last_battery_change > 6):
                self.battery_sate = "full"
                self.get_logger().info("Battery full!")
                self.last_battery_change = time
                self.call_set_led_server(3,0)

    def call_set_led_server(self,led_number,state):
        client = self.create_client(SetLed, "set_led")

        while not client.wait_for_service(1.0): #Aguarda até que o server esteja pronto  
            self.get_logger().warn("Esperando pelo server") #Envia o warning a uma taxa de 1hz
            
        request = SetLed.Request()
        request.led_number = led_number
        request.state = state

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_set_led,led_number=led_number,state=state))
        #partial permite adicionar mais parametros, 
        #caso o contrario apenas future seria passo a função.

    def callback_set_led(self,future,led_number,state):
        try:
            response = future.result()
            self.get_logger().info(str(response.success))
        except Exception as erro:
            self.get_logger().error("Chamada de servico falhou! %f" % (erro,))
 
def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode() 
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()