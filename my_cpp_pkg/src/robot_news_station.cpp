#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"
 
class RobotNewsStationNode : public rclcpp::Node 
{
public:
    RobotNewsStationNode(): Node("robot_news_Station"),  robot_name("Bender")
    {
        publisher = this->create_publisher<example_interfaces::msg::String>("robot_news",10);
        timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                        std::bind(&RobotNewsStationNode::publishNews,this));

        RCLCPP_INFO(this->get_logger(), "Robot News Station iniciou!");
    }
 
private:
    void publishNews()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Olá, eu sou o " + std::string(robot_name)+ " da estação Robot News!");
        publisher->publish(msg);
    }

    std::string robot_name;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}