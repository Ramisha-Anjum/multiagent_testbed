# multiagent_testbed

### OS Distributor: Ubuntu 22.04

### ROS DISTRO: ROS2 Humble

## 1- Create a workspace

```sh
mkdir -p ~/multiagent_ws/src    ## name this directory folder the way you like
cd ~/multiagent_ws/src
```

## 2- Clone the multiagent_testbed package

```sh
git clone https://github.com/Ramisha-Anjum/multiagent_testbed.git
```
    
## 3- Build Package

```sh
colcon build --symlink-install       ## To build all the packages in the entire workspace
colcon build --packages-select turtlesim_vicon      ## To build only turtlesim_vicon (Package_name) package
source ~/multiagent_ws/install/setup.bash
```

## 4- Launch the nodes inside the turtlesim_vicon package

```sh
ros2 launch turtlesim_vicon multiagent.launch.py          ## To launch with turtlesim
ros2 launch turtlesim_vicon multiagent_vicon_pipe.launch.py    ## To launch with vicon
```
