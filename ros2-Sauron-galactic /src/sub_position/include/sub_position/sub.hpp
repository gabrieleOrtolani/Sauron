#ifndef SUB_HPP
#define SUB_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

class Sub : public rclcpp::Node{
    public:
        Sub();

    private:
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_;
        void callback_int(const geometry_msgs::msg::Point::SharedPtr a);
};
#endif
