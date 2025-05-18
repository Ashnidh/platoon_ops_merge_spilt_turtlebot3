#include "rclcpp/rclcpp.hpp"
#include "turtlebot3/cpp_header.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
