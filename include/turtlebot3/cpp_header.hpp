/*
Header file for C++ Nodes
*/

#include "rclcpp/rclcpp.hpp"

class MyCustomNode : public rclcpp::Node 
{
    public: 
        MyCustomNode(): Node("my_node") {
            RCLCPP_INFO(this->get_logger(), "Test CPP Node");
        }
    private:
};