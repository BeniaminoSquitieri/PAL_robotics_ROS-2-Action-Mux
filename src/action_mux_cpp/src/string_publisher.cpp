// string_publisher.cpp
// This file implements a simple publisher node in C++.
// It publishes std_msgs::msg::String messages on the "input_topic" every 2 seconds.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class StringPublisher : public rclcpp::Node
{
public:
  StringPublisher() : Node("string_publisher")
  {
    // Create a publisher on "input_topic" with a queue size of 10.
    publisher_ = this->create_publisher<std_msgs::msg::String>("input_topic", 10);

    // Create a timer that triggers every 2 seconds.
    timer_ = this->create_wall_timer(
      2s, std::bind(&StringPublisher::timer_callback, this)
    );
    RCLCPP_INFO(this->get_logger(), "String publisher initialized on topic 'input_topic'");
  }

private:
  // Timer callback to publish messages.
  void timer_callback()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello from ROS2 C++!";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StringPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
