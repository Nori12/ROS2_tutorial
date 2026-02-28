# Creating a Node

To create a node, go to the the source folder of the target package:

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

Go back the root folder of the workspace and build the package:

```bash
cd ~/ros2_ws/
colcon build --packages-select my_package
```

Run the node:

```bash
ros2 run my_package my_node
```

OBS: note that `my_node` is the executable name you defined in the `add_executable` part of the `CMakeLists.txt`.
