// action_client.cpp
// This file implements a ROS 2 Action Client in C++.
// The client subscribes to "input_topic" (of type std_msgs::msg::String)
// and sends a new goal for the custom action (DoWork) each time a message is received.
// If an active goal exists, it cancels that goal before sending a new one.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

// Include the custom action interface header.
#include "my_action_interfaces/action/do_work.hpp"

using namespace std::chrono_literals;
using DoWork = my_action_interfaces::action::DoWork;
using GoalHandleDoWork = rclcpp_action::ClientGoalHandle<DoWork>;

class ActionMuxClient : public rclcpp::Node
{
public:
  ActionMuxClient() : Node("action_mux_client")
  {
    // Create an action client for the "do_work" action.
    action_client_ = rclcpp_action::create_client<DoWork>(this, "do_work");

    // Create a subscription to "input_topic" of type std_msgs::msg::String.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "input_topic",
      10,
      std::bind(&ActionMuxClient::topic_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Action client and subscription initialized.");
  }

private:
  rclcpp_action::Client<DoWork>::SharedPtr action_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::shared_ptr<GoalHandleDoWork> current_goal_handle_;

  // Callback invoked when a new message is received on "input_topic".
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
    RCLCPP_INFO(this->get_logger(), "Message type: std_msgs::msg::String");

    // If a goal is already active, cancel it before sending a new one.
    if (current_goal_handle_) {
      RCLCPP_INFO(this->get_logger(), "Canceling current goal");
      action_client_->async_cancel_goal(current_goal_handle_);
    }

    // Create a new goal with a work duration of 5 seconds.
    DoWork::Goal goal;
    goal.work_duration = 5;
    send_goal(goal);
  }

  // Function to send a new goal to the action server.
  void send_goal(const DoWork::Goal & goal)
  {
    // Wait for up to 10 seconds for the action server to become available.
    if (!action_client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Sending new goal");
    auto options = rclcpp_action::Client<DoWork>::SendGoalOptions();
    options.result_callback = [this](const GoalHandleDoWork::WrappedResult & result) {
      this->result_callback(result);
    };

    // Call async_send_goal and get the future.
    auto goal_future = action_client_->async_send_goal(goal, options);
    // Spawn a thread to wait on the future.
    std::thread([this, goal_future]() mutable {
      auto goal_handle = goal_future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
      current_goal_handle_ = goal_handle;
    }).detach();
  }

  // Callback that is called when the result for the goal is received.
  void result_callback(const GoalHandleDoWork::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded with result: success=%s",
                    result.result->success ? "true" : "false");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(this->get_logger(), "Goal was aborted");
        break;
      default:
        RCLCPP_INFO(this->get_logger(), "Unknown result code");
        break;
    }
    // Reset the current goal handle after receiving the result.
    current_goal_handle_.reset();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionMuxClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
