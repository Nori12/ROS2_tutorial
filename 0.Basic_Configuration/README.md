# Basic Configuration


## Source the main ROS2 installation on bashrc

To avoid sourcing the ROS2 installation files every time you launch a new terminal, add it permanently by modifying the `bashrc` file:
```bash
cd ~
vim .bashrc
```

Go to the end and add:
```bash
source /opt/ros/kilted/setup.bash
```
It is necessary to restart the open terminals to validate the modification.

## Avoid seeing external topics/nodes on the same network

Since ROS2 is distributed by nature, if you are connected to the same network of another computer executing topics/nodes, you are susceptible to see them from yours. To avoid this, modify the `.bashrc` file:
```bash
cd ~
vim .bashrc
```

Add it to the end:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
```

`ROS_DOMAIN_ID` is an ID from 0 to 101 supposed to be different for each computer.
`ROS_LOCALHOST_ONLY` (0|1) Chose 1 for local

After modifying the network connection, it is recommended to restart de ROS2 daemon:

```bash
ros2 daemon stop
ros2 daemon start
```




