import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
 
 
def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_no_opp")
    client = node.create_client(AddTwoInts, "add_two_ints")

    while not client.wait_for_service(1.0): #Aguarda até que o server esteja pronto  
        node.get_logger().warn("Esperando pelo server Add Two Ints") #Envia o warning a uma taxa de 1hz

    request = AddTwoInts.Request()
    request.a = 4
    request.b = 5

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    try:
        response = future.result()
        node.get_logger().info(str(request.a) + " + " + str(request.b) + 
                                                " = " + str(response.sum))
    except Exception as erro:
        node.get_logger().error("Chamada de servico falhou!" % (erro,))

    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()