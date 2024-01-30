#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"
 
class HardwarePublisher : public rclcpp::Node 
{
public:
    HardwarePublisher(): Node("hardware_status_publisher")
    {
        this->declare_parameter("temperature_to_publish",40);
        this->declare_parameter("publish_frequency",1.0);

        temperature = this->get_parameter("temperature_to_publish").as_int();
        double publish_frequency = this->get_parameter("publish_frequency").as_double();

        publisher = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status",10);
        timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/publish_frequency)),
                                        std::bind(&HardwarePublisher::publishNews,this));
        RCLCPP_INFO(this->get_logger(), "Hardware status publisher iniciou!");
    }
 
private:
    void publishNews()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = temperature;
        msg.are_motors_ready = false;
        msg.debug_message = "Motores estão muito quentes!";
        publisher->publish(msg);
    }

    int temperature;
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