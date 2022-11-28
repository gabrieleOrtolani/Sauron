#ifndef PUB_HPP
#define PUB_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

#define PUB_PERIOD 300

class Pub : public rclcpp::Node{
    public:
        Pub();
    private:
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr pub_timer_;
        void pub_timer_callback(void); // Come una write --> una funzione che riempie il campo sharedPoint

        unsigned long pub_cnt_;
};

#endif