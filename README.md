# ROS2_tutorial
My owm tutorial to guide me through ROS2.

Version: Kilted Kaiju

## Essential : run setup.bash

Before working on a ROS2 workspace, it is important to run:
```bash
source install/setup.bash
```

With this command, it:
- enables autocompletion.
- adds your workspace's bin folder to the system's PATH.
- tells ROS 2 where to find the package.xml and metadata for your custom nodes.
- updates PYTHONPATH and LD_LIBRARY_PATH, since ROS 2 relies heavily on Python for launch files and specific node implementations.

## Compiling

### Compile all packages:
```bash
colcon build
```

### Compile a specific package:
```bash
colcon build --packages-select MY_PACKAGE
```

## Topics - General commands

### List all current topics:
```bash
ros2 topic list
```

### Print the data going through a topic:
```bash
ros2 topic echo /topic_name
```

### Get more details about a topic:
```bash
ros2 topic info /topic_name
ros2 topic type /topic_name
```

### Publish to a topic from the terminal:

(At a rate of 10Hz)
```bash
ros2 topic pub -r 10 /topic_name example_interfaces/msg/String "{data: 'Hello from terminal'}"
```

(Only once)
```bash
ros2 topic pub -1 /topic_name example_interfaces/msg/String "{data: 'Hello from terminal'}"
```

### Check if your publishers/subscribers manage to follow the rhythm

```bash
ros2 topic hz /topic_name
```


### Check how much data is going through a topic

```bash
ros2 topic bw /topic_name
```

