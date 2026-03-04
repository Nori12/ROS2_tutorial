# Creating a Launcher

## Create a simple launcher

Inside a package, create a folder `launch` and add to it a file `mypackage_launch.py`:

```bash
mkdir launch
touch mypackage_launch.py
```

Use the following example for the :

```python
from launch import LaunchDescription
from launch_ros.actions import Node

# Function called by ros2 launch to get the list of nodes to launch
def generate_launch_description():
    node1 = Node(
        package='package_name', # package name
        namespace='',
        executable='exec_exe',  # executable name
        name='node_A'           # (Optional) Overrides the node name in the constructor
        parameters=[            # (Optional) Pass parameters here
            {'pub_frequency': 20.0}
        ],
        remappings=[            # (Optional) Rename topics
            ('/cmd_vel', '/boat/cmd_vel')
        ]
    )
    node2 = Node(
        package='package_name', # package name
        namespace='',
        executable='exec_exe',  # executable name
        name='node_B'           # (Optional) Overrides the node name in the constructor
    )

    return LaunchDescription([
            node1,
            node2
    ])
```

Update `CMakeLists.txt` by adding:

```cpp
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```