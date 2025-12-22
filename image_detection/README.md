# multiagent_testbed

### OS Distributor: Ubuntu 22.04

### ROS DISTRO: ROS2 Humble

## 1- Create a workspace

```sh
mkdir -p ~/multiagent_ws/src    ## name this directory folder the way you like
cd ~/multiagent_ws/src
```

## 2- Clone the package

```sh
git clone https://github.com/Ramisha-Anjum/multiagent_testbed/image_detection.git
```
    
## 3- Download the dependencies

```sh
sudo apt install -y ros-humble-v4l2-camera	## v4l2_camera (recommended for USB webcams)
sudo apt install -y ros-humble-rviz2            ## if you want to see marker in RViz
```

## 3- Build the package

```sh
colcon build --symlink-install       ## To build all the packages in the entire workspace
colcon build --packages-select image_detection      ## To build only image_detection (Package_name) package
source ~/multiagent_ws/install/setup.bash
```

## 4- Run the node to get bearing angle of the target object with the webcam
The target object has an aruco marker set on it. And the camera (webcam) detects that marker to detect the target object.

```sh
ros2 run v4l2_camera v4l2_camera_node          ## In Terminal 1
ros2 run image_detection webcam_aruco_bearing --ros-args   -p image_topic:=/image_raw   -p marker_id:=2   -p hfov_deg:=60.0          ## Run your node in Terminal 2
```
