#ifndef SUB_HPP
#define SUB_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

#define PUB_PERIOD 300

class Sub : public rclcpp::Node{
    public:
        Sub();

    private:
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr pub_timer_;
        void sub_timer_callback(void);
        void callback_int(const std_msgs::msg::Int64::SharedPtr a);
};
#endif