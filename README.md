# TurtleBot3 Walker

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Overview

Turtlebot3 implementation to avoid obstacles in an unknown environment mimicking the working of a roomba vaccum cleaning robot.

## Assumptions

* OS: Ubuntu Jammy Jellyfish (22.04) 64-bit
* ROS2 Distro: Humble Hawksbill (binary installation)
* ROS2 Workspace name: ros2_ws
* ROS2 Installation Directory: ros2_humble [if installed through source]
* Robot Used: Turtlebot3
* Physics Engine: Gazebo 11

## ROS2 Dependencies

* ```ament_cmake```
* ```rclcpp```
* ```geometry_msgs```
* ```sensor_msgs```
* ```gazebo_ros```
* ```turtlebot3```
* ```turtlebot3_msgs```
* ```dynamixel-sdk```
* ```turtlebot3_simulations```

## Instructions to Build the Package

``` cd <path-to-ROS2-workspace>/ros2_ws/src
git clone https://github.com/bharadwaj-chukkala/turtlebot_roomba.git
cd ..  
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install --packages-select turtlebot_roomba
```

## Instructions to Run the Package

### Simulation and Visualisation

Open a new terminal and source it to local install files after building
```
cd turtlebot_roomba
. install/setup.bash
```

Before launching the node, we need to export the turtlebot3 model and its path for it to spawn in the gazebo environment
```
# Model export
export TURTLEBOT3_MODEL=waffle_pi

#Path export
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg prefix turtlebot3_gazebo `/share/turtlebot3_gazebo/models/
```

Now run the launch file to visualize the obstacle detction implementation
```
cd turtlebot_roomba
. install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

In another terminal
```
cd turtlebot_roomba
. install/setup.bash
ros2 run turtlebot3_roomba walker
```

You can also directly use a launch file to open gazebo with spawned turtlebot in it and also run the walker node at the same time.
```
cd turtlebot_roomba
. install/setup.bash
ros2 launch turtlebot_roomba walker.launch.py
```

### Bag File recording

The launch file is set to record a bag file in the directory where it is run, the bag file will be saved as ```package_bag```.
To view bag information
```
cd package_bag
ros2 bag info package_bag
```

To play the recorded bag file (recorded for more than 20 seconds of simulation):
```
cd package_bag
ros2 bag play package_bag
```

To not record the bag file, you can manually set the ```enable_recording``` parameter to false
```
ros2 launch turtlebot_roomba walker.launch.py enable_recoring:=false
```

## Results

### Static Code Analysis

#### cpplint

Change to the root directory of the package, ```/turtlebot3_roomba```, and run:
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp ./include/beginner_tutorials/*.hpp > ./results/cpplint.txt
```

The results of running ```cpplint``` can be found in ```/results/cpplint.txt```.

#### cppcheck

Change to the root directory of the package, ```/turtlebot3_roomba```, and run:
```
cppcheck --enable=all --std=c++17 ./src/*.cpp ./include/beginner_tutorials/*.hpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
The results of running ```cppcheck``` can be found in ```/results/cppcheck.txt```.