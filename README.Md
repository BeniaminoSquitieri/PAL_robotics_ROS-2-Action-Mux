# PAL Robotics ROS 2 Action Mux

This repository contains a demonstration ROS 2 project that implements a custom action interface along with both Python and C++ implementations of an action server, action client, generic subscriber, and string publisher.

## Repository Overview

The repository is organized into three packages:

- **my_action_interfaces**  
  Contains the custom action interface (`DoWork.action`). This defines:
  - **Goal:** an integer field `work_duration`
  - **Result:** a boolean field `success`
  - **Feedback:** a float field `progress`

- **action_mux_py**  
  A Python package that implements:
  - An **action server** that simulates work for 5 seconds, provides feedback every second, and handles cancellations.
  - An **action client** that subscribes to a topic (`input_topic`) and sends a goal upon receiving a message. If a goal is already active, it cancels it before sending a new one.
  - A **generic subscriber** that subscribes to any specified topic and prints the received message and its type.
  - A **string publisher** that publishes test messages on `input_topic`.

- **action_mux_cpp**  
  A C++ package with the same functionality as the Python package:
  - An **action server**
  - An **action client**
  - A **generic subscriber**
  - A **string publisher**

## Prerequisites

- **ROS 2** (Humble or later)
- **colcon** build system
- A C++ compiler supporting C++11 (or later)
- **Python 3**

## Installation and Build Instructions

1. **Clone the Repository into Your ROS 2 Workspace**

   First, navigate to your ROS 2 workspace's `src` directory. For example:
   ```bash
   cd ~/ros2_ws/src
   ```
   Then, clone the repository:
   ```bash
   git clone https://github.com/BeniaminoSquitieri/PAL_robotics_ROS-2-Action-Mux.git
   ```

2. **Prepare the Workspace Structure**

   If the repository was cloned into a subfolder (e.g. PAL_robotics_ROS-2-Action-Mux), move the packages into your workspace's src folder:
   ```bash
   mv PAL_robotics_ROS-2-Action-Mux/src/* ./
   rm -rf PAL_robotics_ROS-2-Action-Mux
   ```
   After this, your `src` folder should contain:

   - `my_action_interfaces/`
   - `action_mux_py/`
   - `action_mux_cpp/`

3. **Build the Workspace**

   From the root of your workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

4. **Source the Workspace**

   After a successful build, source your workspace:
   ```bash
   source install/setup.bash
   ```

## Running the Demo

The demo requires launching several nodes in separate terminals. Be sure to source your workspace (`source install/setup.bash`) in each terminal before running commands.

### Python Implementation

- **Terminal 1: Action Server (Python)**
  ```bash
  ros2 run action_mux_py action_server
  ```
- **Terminal 2: Action Client (Python)**
  ```bash
  ros2 run action_mux_py action_client
  ```
- **Terminal 3: Generic Subscriber (Python)**
  ```bash
  ros2 run action_mux_py generic_subscriber --ros-args -p topic_name:=input_topic -p msg_type:="std_msgs/msg/String"
  ```
- **Terminal 4: String Publisher (Python)**
  ```bash
  ros2 run action_mux_py string_publisher
  ```

### C++ Implementation

- **Terminal 1: Action Server (C++)**
  ```bash
  ros2 run action_mux_cpp action_server
  ```
- **Terminal 2: Action Client (C++)**
  ```bash
  ros2 run action_mux_cpp action_client
  ```
- **Terminal 3: Generic Subscriber (C++)**
  ```bash
  ros2 run action_mux_cpp generic_subscriber --ros-args -p topic_name:=input_topic -p msg_type:="std_msgs/msg/String"
  ```
- **Terminal 4: String Publisher (C++)**
  ```bash
  ros2 run action_mux_cpp string_publisher
  ```

## Expected Output

- **Action Server:** Logs the reception of a goal, prints progress updates (20%, 40%, etc.), and indicates whether the goal was completed successfully or canceled.
- **Action Client:** Logs the message received from `input_topic`, cancels any ongoing goal if necessary, sends a new goal, and prints the goal result.
- **Generic Subscriber:** Prints the details (and size) of each message received on the subscribed topic.
- **String Publisher:** Logs the publication of a message on `input_topic` (e.g., "Hello from ROS2 C++!" or "Hello from ROS2 Python!").

