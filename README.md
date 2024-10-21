# Simulator 

## Description
Simulation Environment for the Bosch Future Mobility Challenge: https://boschfuturemobility.com/
Starter code from https://github.com/ECC-BFMC/Simulator

## Structure

- **example**: provide some example scripts on how to interact with the simulator.
- **models_pkg**: contains sdf files describing the car components and other objects in the simulator.
- **realsense_ros_gazebo**: provides plugin to simulate the realsense D435i camera.
- **plugins_pkgs**: code to simulate the components of the car (gps, motor and servo commands...)
- **sim_pkg**: provides launch files to launch the simulation world.
- **traffic_light_pkg**: manages the traffic lights 
- **utils**: provides msg and srv files needed in other pkgs.
- **localization_bridge**: simulate the gps system in the competition with a delay of 1 second and uniform noise of 0.2.

## Dependencies

### ROS
#### Installation:
http://www.autolabor.com.cn/book/ROSTutorials/chapter1/12-roskai-fa-gong-ju-an-zhuang/124-an-zhuang-ros.html

## Build

1. Open a terminal and clone the repository
    ```bash
    cd
    git clone https://github.com/slsecrets357/Simulator.git
    ```

2. Build the packages
    ```bash
    catkin_make --pkg utils
    catkin_make
    ```

3. (only after building/catkin_make) ```gedit devel/setup.bash``` and add these 2 lines to the file with your Simulator path:
    ```sh
    export GAZEBO_MODEL_PATH="/home/{YOUR_USER}/Documents/Simulator/src/models_pkg:$GAZEBO_MODEL_PATH"
    export ROS_PACKAGE_PATH="/home/{YOUR_USER}/Documents/Simulator/src:$ROS_PACKAGE_PATH"
    ```

## Usage
1. Start the gazebo simulation:
    ```bash
    source devel/setup.bash
    roslaunch sim_pkgs map2025objects.launch
    ```
2. To modify the objects you want to spawn and their locations, modify the spawn_signs.launch file in sim_pkg

