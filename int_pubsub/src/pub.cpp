#include <iostream>
#include "../include/int_pubsub/pub.hpp"

Pub::Pub()
: Node("Publisher_node"),
pub_cnt_(0)
{
  //pub_cnt_ = 0;
  publisher_ = this->create_publisher<std_msgs::msg::Int64>("esempi/Test", rclcpp::QoS(10));

  pub_timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds(PUB_PERIOD),
    std::bind(&Pub::pub_timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Pub initialized");
}
//la classe mi instaura un tipo da dare all'oggetto
void Pub::pub_timer_callback(void)
{
  auto message = std_msgs::msg::Int64();
  message.data = pub_cnt_++;
  RCLCPP_INFO(this->get_logger(), "Puslished: %d ", message.data);
  publisher_->publish(message);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto pub_node = std::make_shared<Pub>();
  rclcpp::spin(pub_node);
  rclcpp::shutdown();
  std::cout << "Pub terminated" << std::endl;
  exit(EXIT_SUCCESS);
}
