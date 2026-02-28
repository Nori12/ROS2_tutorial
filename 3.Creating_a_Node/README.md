# Creating a Node

## Creating a simple node

Go to the the source folder of the target package:

```bash
cd ~/ros2_ws/src/my_package/src
```

And use the template code below to create the `simple_node.cpp` file.

```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MySimpleNode : public rclcpp::Node
{
public:
  MySimpleNode() : Node("simple_node")
  {
    RCLCPP_INFO(this->get_logger(), "Starting the Simple Node with TF2 support!");

    // Example: Creating a point
    geometry_msgs::msg::Point p;
    p.x = 1.0; p.y = 2.0; p.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Point initialized at: x=%.2f, y=%.2f", p.x, p.y);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MySimpleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

In the `CMakeLists.txt` file, add the following lines:

```python
# 1. Create the executable
add_executable(my_node src/simple_node.cpp)

# 2. Link the dependencies (libraries)
ament_target_dependencies(my_node rclcpp tf2_geometry_msgs)

# 3. Install the executable so "ros2 run" can find it
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME}
)
```

Back to the root folder of the workspace, build the package:

```bash
cd ~/ros2_ws/
colcon build --packages-select my_package
```

Run the node:

```bash
ros2 run my_package my_node
```

OBS: note that `my_node` is the executable name you defined in the `add_executable` part of the `CMakeLists.txt`.


## Add a publisher to a topic

Replace the previous code in `simple_node.cpp` in the path `~/ros2_ws/src/my_package/src/` by the code below.

```cpp
#include <chrono> // Required for timing
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class MySimpleNode : public rclcpp::Node
{
public:
  MySimpleNode() : Node("simple_node"), count_(0)
  {
    // 1. Initialize the Publisher
    // Topic name: "robot_position", Queue size: 10
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("robot_position", 10);

    // 2. Initialize the Timer
    // Calls 'timer_callback' every 500ms (2Hz)
    timer_ = this->create_wall_timer(500ms, std::bind(&MySimpleNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Publisher node has started!");
  }

private:
  // 3. The Callback Function
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Point();
    message.x = static_cast<double>(count_);
    message.y = static_cast<double>(count_) * 2.0;
    message.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Publishing: x=%.2f, y=%.2f", message.x, message.y);

    // 4. Send the message
    publisher_->publish(message);
    count_++;
  }

  // Member variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MySimpleNode>());
  rclcpp::shutdown();
  return 0;
}
```

Build and execute the package:

```bash
cd ~/ros2_ws/
colcon build --packages-select my_package
ros2 run my_package my_node
```

Verify the creation of the topic by opening another terminal and running:

```bash

```