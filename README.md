# multiagent_testbed

clone multiagent_testbed

## 2- Run the Formation Controller

  - **Clone the formation controller package**

    ```sh
    git clone https://github.com/AhmedeFahim/GDLS-Multi-Vehicle-Formation.git
    ```
    
  - **Source the controller package and PX4 simulator**

    ```sh
    cd GDLS-Multi-Vehicle-Formation/formation_pkg
    source devel/setup.bash

    ## Enusre to navigate to the Px4-Autopilot folder directory
    cd multi-vehicle_Formation/PX4-Autopilot

    source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
  
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
    ```
  - **Now you are ready to launch the simulator that deploys a warthog and 3 Iris Drones**
  
    ```sh
    roslaunch formation_offb main.launch
    ```


Launch turtlesim

or 

Lunch vicon
