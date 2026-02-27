# Creating a Workspace

## Create the Workspace Directory

By convention, people use `_ws` at the end of the workspace folder name.
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## Build the Workspace

The simple way:
```bash
colcon build
```

Other possibility:
```bash
colcon build --symlink-install
```

`--symlink-install` allows the installed files to be changed by changing the files in the source space (e.g. Python files or other non-compiled resources) for faster iteration.
