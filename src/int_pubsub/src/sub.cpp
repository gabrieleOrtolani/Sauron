#include <cstdio>
#include <iostream>
#include "../include/cpp_pubsub/sub.hpp"

Sub::Sub()
: Node("Subscriber_Node"){
    subscriber_ = this->create_subscription<std_msgs::msg::Int64>("esempi/gabriele", rclcpp::QoS(10), std::bind(&Sub::callback_int, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscriber initialized");
}

void Sub::callback_int(const std_msgs::msg::Int64::SharedPtr a){
    RCLCPP_INFO(this->get_logger(), "Subscribed: %d", a->data);
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    auto sub_node = std::make_shared<Sub>();
    rclcpp::spin(sub_node);
    
    rclcpp::shutdown();

    std::cout << "Sub terminated" << std::endl;
    exit(EXIT_SUCCESS);
}
