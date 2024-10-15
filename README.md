# Simulator 

Starter code from https://github.com/ECC-BFMC/Simulator

There are multiple packages/repos in our organisation for the car's functions that can be added in the repo for testing/development.
(ex:Advanced-Lane-Detection, Control...)

Build process:

1- Clone the repo and the other packages you need to use.

2- Add symbolic link to the packages in src with:

```sh
ln -s {path to package}/{package name}
```

ex:

```sh
ln -s /home/antoinedeng/Desktop/BFMC_Simulator_packages/nav_pkg
```

(When you add a symbolic link add it in the .gitignore as well)

3-Build the repo with 

```sh
catkin_make --pkg utils
catkin_make
```

(If there's an error about opencv2 such as ```<opencv2/imgproc.hpp>```, open /usr/include/opencv4 and move the opencv2 folder inside /usr/include/opencv4 to /usr/include)

## Requirements install:

1. open a terminal and cd to Simulator/src

2. ```pip install -r requirements.txt```

3. Other requirements: 

- openCV: https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html

- ncnn: https://github.com/Tencent/ncnn/wiki/how-to-build#build-for-linux

- YoloFastestV2: https://github.com/xs2445/Yolo-FastestV2-NCNN-RasPi4B

- raspicam: https://github.com/cedricve/raspicam

## Usage:

1. open a terminal and cd to Simulator

2. (only after building/catkin_make) ```gedit devel/setup.bash``` and add these 2 lines to the file with your Simulator path:

```sh
export GAZEBO_MODEL_PATH="/home/{YOUR_USER}/Documents/Simulator/src/models_pkg:$GAZEBO_MODEL_PATH"
export ROS_PACKAGE_PATH="/home/{YOUR_USER}/Documents/Simulator/src:$ROS_PACKAGE_PATH"
```

3. ```source devel/setup.bash```

## Run simulator:

```roslaunch sim_pkg map_with_car.launch```
(you can replace "map_with_car.launch" with different launch files in sim_pkg/launch and/or modify them to add objects in the simulator)

## Run controller or function nodes:

open new terminal with "Usage" (cd to Simulator and ```source devel/setup.bash```)

All controller nodes: ```roslaunch control car_control.launch``` (you can edit the launch file to add/remove ros nodes)

Any other functions: ```rosrun control *.py``` (any python file in control/scripts or any c++ files in control/src without .cpp)

(ex: ```rosrun control lane.py```
ex: ```rosrun control lane```)

## Advanced stuff:

```car_control.launch``` file arguments:

```xml
<arg name="show_sign"  default="False" doc="display sign detection window (True or False)"/>
<arg name="show_lane"  default="False" doc="display lane detection window (True or False)"/>
<arg name="print_sign"  default="False" doc="print sign detection (True or False)"/>
<arg name="print_lane"  default="False" doc="print lane detection (True or False)"/>
<!-- <arg name="method"  default="histogram" doc="method for lane detection (histogram or houghlines)"/> -->
<arg name="simulation"  default="True" doc="Simulation mode (True or False)"/>
<arg name="path"  default="/paths/path.json" doc="the relative json file path to the planned path"/>
<arg name="custom"  default="False" doc="user input map (True or False)"/>
```

(add them by adding ```(arg name):=(arg value)``` after ```roslaunch control car_control.launch```)

ex: ```roslaunch control car_control.launch show_sign:=True path:=/paths/path1.json```

## Files structures in src:

control: our controller package

- launch: launch files for controller function

- models: yolo models for object detection

- msg: custom ros messages used by controller functions

- scripts: controller functions in python

- src: controller functions in c++

- srv: custom ros services used by controller functions

example: given demo code
- its src has control.py and camera.py to demo keyboard control and camera

models_pkg: sdf models used in the simulator

plugins_pkg: code to simulate the components of the car (gps, motor and servo commands...)

sim_pkg: contains simulator worlds and launch files for them

traffic_light_pkg: idk

utils: custom ros messages and services

