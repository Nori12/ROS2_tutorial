# Creating a Package

## Create a package with dependencies

For the dependencies `rclcpp` and `tf2_geometry_msgs`, run:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_package --dependencies rclcpp tf2_geometry_msgs
```

### What happened?

ROS 2 generated a folder structure for you. The two most important files for managing your dependencies are `package.xml` and `CMakeLists.txt`.

* `package.xml`: This file is used by the ROS 2 ecosystem to know what your package needs to run. You will see these lines added:

```text
<depend>rclcpp</depend>
<depend>tf2_geometry_msgs</depend>
```

* `CMakeLists.txt`: This file tells the compiler how to build your code. Because you used the --dependencies flag, ROS 2 automatically added:
```text
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select my_package
source install/setup.bash
```




