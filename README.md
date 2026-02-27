# ROS2_tutorial
My owm tutorial to guide me through ROS2.

Version: Kilted Kaiju

### Essential : run setup.bash

Before working on a ROS2 workspace, it is important to run:
```bash
source install/setup.bash
```

With this command, it:
- enables autocompletion.
- adds your workspace's bin folder to the system's PATH.
- tells ROS 2 where to find the package.xml and metadata for your custom nodes.
- updates PYTHONPATH and LD_LIBRARY_PATH, since ROS 2 relies heavily on Python for launch files and specific node implementations.

### Compiling

Compile all packages:
```bash
colcon build
```

Compile specific packages:
```bash
colcon build --packages-select MY_PACKAGE
```

### Topics - General commands

To list all current topics:
```bash
ros2 topic list
```

To print the data going through a Topic:
```bash
ros2 topic echo /topic_name
```

Get more details about a Topic
```bash
ros2 topic info /topic_name
ros2 topic type /topic_name
```