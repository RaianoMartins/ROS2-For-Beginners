#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"
 
class HardwarePublisher : public rclcpp::Node 
{
public:
    HardwarePublisher(): Node("hardware_status_publisher")
    {
        publisher = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status",10);
        timer = this->create_wall_timer(std::chrono::milliseconds(1000),
                                        std::bind(&HardwarePublisher::publishNews,this));
        RCLCPP_INFO(this->get_logger(), "Hardware status publisher iniciou!");
    }
 
private:
    void publishNews()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = 57;
        msg.are_motors_ready = false;
        msg.debug_message = "Motores estão muito quentes!";
        publisher->publish(msg);
    }

    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwarePublisher>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}