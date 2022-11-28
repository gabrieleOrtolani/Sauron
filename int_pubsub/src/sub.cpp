#include <cstdio>
#include <iostream>
#include "../include/int_pubsub/sub.hpp"
#include "../include/int_pubsub/pub.hpp"
int64_t num;
char *topic_name = "esempi/gabriele";


Sub::Sub()
: Node("Subscriber_Node"){
    subscriber_ = this->create_subscription<std_msgs::msg::Int64>("esempi/Test", rclcpp::QoS(10), std::bind(&Sub::callback_int, this, std::placeholders::_1));
    
    pub_timer_ =
        this->create_wall_timer(
        std::chrono::milliseconds(300),
        std::bind(&Sub::sub_timer_callback, this));
    
    publisher_ =  this -> create_publisher<std_msgs::msg::Int64>(topic_name, rclcpp::QoS(10));
    RCLCPP_INFO(this->get_logger(), "Subscriber initialized");
}


void Sub::callback_int(const std_msgs::msg::Int64::SharedPtr a){
    
    //RCLCPP_INFO(this->get_logger(), "Subscribed: %d", a->data);
    num = a -> data;
    //num = 999;
}

void Sub::sub_timer_callback(void){
    auto msgFw = num;
    auto message = std_msgs::msg::Int64();
    message.data = num;
    RCLCPP_INFO(this->get_logger(), "Topic: %s has puslished: %d", topic_name, msgFw);
    publisher_->publish(message);
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    auto sub_node = std::make_shared<Sub>();
    //auto pub_node = std::make_shared<Pub>();

    rclcpp::spin(sub_node);
    //rclcpp::spin(pub_node);
    
    //execl("/home/lorenzo/Scaricati/ros2-examples-galactic/src/int_pubsub/src", a.data)
    rclcpp::shutdown();

    std::cout << "Sub terminated" << std::endl;
    exit(EXIT_SUCCESS);
}