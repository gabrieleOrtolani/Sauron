#include <cstdio>
#include <iostream>
#include "../include/sub_position/sub.hpp"
using namespace geometry_msgs::msg;
Sub::Sub()
: Node("Subscriber_Node"){
    subscriber_ = this->create_subscription<Point>("/position", rclcpp::QoS(10), std::bind(&Sub::callback_int, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscriber initialized");
}

void Sub::callback_int(const Point::SharedPtr a){
    RCLCPP_INFO(this->get_logger(), "Subscribed: %f\t%f\t%f", a->x, a->y,a->z);
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    auto sub_node = std::make_shared<Sub>();
    rclcpp::spin(sub_node);
    
    rclcpp::shutdown();

    std::cout << "Sub terminated" << std::endl;
    exit(EXIT_SUCCESS);
}
