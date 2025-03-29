// generic_subscriber.cpp
// This file implements a generic subscriber node using rclcpp's GenericSubscription API.
// It subscribes to a topic (specified via parameters) and prints out the size of each received serialized message.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// Include the header for GenericSubscription
#include "rclcpp/generic_subscription.hpp"

class GenericSubscriber : public rclcpp::Node
{
public:
  GenericSubscriber() : Node("generic_subscriber")
  {
    // Declare parameters: topic name and message type.
    this->declare_parameter("topic_name", "input_topic");
    this->declare_parameter("msg_type", "std_msgs/msg/String");

    // Retrieve the parameters using the get<T>() method.
    std::string topic_name = this->get_parameter("topic_name").get_value<std::string>();
    std::string msg_type = this->get_parameter("msg_type").get_value<std::string>();

    // Create a generic subscription.
    // The callback accepts a shared pointer to a SerializedMessage.
    generic_subscription_ = this->create_generic_subscription(
      topic_name,
      msg_type,
      rclcpp::QoS(10),
      [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        this->callback(msg);
      }
    );

    RCLCPP_INFO(this->get_logger(), "Generic subscription created on topic: %s with type: %s",
                topic_name.c_str(), msg_type.c_str());
  }

private:
  // The callback receives a serialized message.
  // We print out the size of the received message.
  void callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received serialized message of size: %zu bytes", msg->size());
  }

  rclcpp::GenericSubscription::SharedPtr generic_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GenericSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
