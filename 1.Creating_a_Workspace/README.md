# Creating a Workspace


## Create the Workspace Directory

By convention, people use `_ws` at the end of the workspace folder name.
```bash
mkdir -p ~/MYPROJECT_ws/src
cd ~/MYPROJECT_ws
```

<!-- ## Source the main ROS2 installation

For Kilted Kaiju:

```bash
source /opt/ros/kilted/setup.bash
```

Alternatively, it is possible to add it permanently by modifying the `bashrc` file:
```bash
cd ~
vim .bashrc
```

Go to the end and add:
```bash
source /opt/ros/kilted/setup.bash
```

OBS: Since ROS2 is by nature distributed, if you are connected to the same network that is executing topics/nodes, you are susceptible to see them from your computer. To avoid this, also add to the `.bashrc`:
```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
```
`ROS_DOMAIN_ID` is an ID from 0 to 101 supposed to be different from other computers.
`ROS_LOCALHOST_ONLY` (0|1) Chose 1 for local

After modifying the network connection, it is recommended to restart de ROS2 daemon:

```bash
ros2 daemon stop
ros2 daemon start
``` -->




