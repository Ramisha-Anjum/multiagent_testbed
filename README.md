# multiagent_testbed

## 1- Create a workspace

    mkdir -p ~/multiagent_ws/src    ## name this directory folder the way you like
    cd ~/multiagent_ws
    source /opt/ros/humble/setup.bash    ##If you keep it in your .bashrc file you don't have to run it again

    
    
## 2- Run the Formation Controller

  - **Clone the formation controller package**

    ```sh
    git clone https://github.com/Ramisha-Anjum/multiagent_testbed.git
    ```
    
  - **Build Package**

    ```sh
    colcon build --symlink-install       ## To build all the packages in the entire workspace
    colcon build --packages-select turtlesim_vicon      ## name this package the way you like
    source ~/multiagent_ws/install/setup.bash
    ```
  - **Now you are ready to launch the nodes inside this package**
  
    ```sh
    ros2 launch turtlesim_vicon multiagent.launch.py          ## To launch with turtlesim
    ros2 launch turtlesim_vicon multiagent_vicon_pipe.launch.py    ## To launch with vicon
    ```
