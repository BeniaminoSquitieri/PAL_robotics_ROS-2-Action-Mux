// action_server.cpp
// This file implements a ROS 2 Action Server in C++
// It uses the custom action interface defined in the my_action_interfaces package.
// The server simulates work for 5 seconds and provides feedback every second.
// If a cancellation request is received during execution, the goal is aborted.

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Include the generated header for the custom action interface.
#include "my_action_interfaces/action/do_work.hpp"

// Use a namespace alias for convenience.
using DoWork = my_action_interfaces::action::DoWork;
using GoalHandleDoWork = rclcpp_action::ServerGoalHandle<DoWork>;

class DoWorkActionServer : public rclcpp::Node
{
public:
  explicit DoWorkActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("do_work_action_server", options)
  {
    // Create an action server for the 'do_work' action.
    action_server_ = rclcpp_action::create_server<DoWork>(
      this,
      "do_work", // Action name.
      std::bind(&DoWorkActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DoWorkActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&DoWorkActionServer::handle_accepted, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Action server has been started.");
  }

private:
  // Pointer to the action server.
  rclcpp_action::Server<DoWork>::SharedPtr action_server_;

  // Callback to handle incoming goal requests.
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const DoWork::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with work_duration: %d", goal->work_duration);
    // Accept all incoming goals.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Callback to handle cancellation requests.
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDoWork> /*goal_handle*/)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Once a goal is accepted, this callback is invoked.
  // It starts the goal execution in a separate thread.
  void handle_accepted(const std::shared_ptr<GoalHandleDoWork> goal_handle)
  {
    std::thread{[this, goal_handle]() { this->execute(goal_handle); }}.detach();
  }

  // This function simulates work for 5 seconds.
  // It publishes feedback every second and checks if the goal is canceled.
  void execute(const std::shared_ptr<GoalHandleDoWork> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto feedback = std::make_shared<DoWork::Feedback>();
    auto result = std::make_shared<DoWork::Result>();

    // Loop for 5 seconds.
    for (int i = 0; i < 5; ++i) {
      // If cancellation has been requested, abort the goal.
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        return;
      }
      // Simulate work by sleeping for 1 second.
      std::this_thread::sleep_for(std::chrono::seconds(1));

      // Publish feedback (progress as a fraction).
      feedback->progress = static_cast<float>(i + 1) / 5.0f;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Progress: %.0f%%", feedback->progress * 100);
    }

    // After completing the work, mark the goal as succeeded.
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal completed successfully");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DoWorkActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
