cmake_minimum_required(VERSION 3.5)
project(action_mux_cpp)

# Find dependencies.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_action_interfaces REQUIRED)

# Build executable for the action server.
add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp rclcpp_action my_action_interfaces)

# Build executable for the action client.
add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp rclcpp_action std_msgs my_action_interfaces)

# Build executable for the generic subscriber.
add_executable(generic_subscriber src/generic_subscriber.cpp)
ament_target_dependencies(generic_subscriber rclcpp)

# Build executable for the string publisher.
add_executable(string_publisher src/string_publisher.cpp)
ament_target_dependencies(string_publisher rclcpp std_msgs)

# Install the executables.
install(TARGETS
  action_server
  action_client
  generic_subscriber
  string_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
