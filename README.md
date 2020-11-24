# Turtlebot Walker Simulation Package
## About
This is a ROS package which simulates turtlebot implementing a walker algorithm. The package is simulated in Gazebo. We subscribe to Laser Scans from the turtlebot and publish the messages to /cmd_vel topic. An obstacle detection and avoidance technique is set to avoid the obstacles and move in straight lines.

### Author
Aditya Khopkar, akhopkar@terpmail.umd.edu

## Dependencies
The package has the dependencies: 
1. ROS Melodic
2. [Gazebo](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros).
3. [Turtlebot3](https://answers.ros.org/question/293514/turtlebot-installation-on-ros-melodic/)

### Package Dependencies
The package is built on the following dependencies:
* roscpp
* rospy
* std_msgs
* sensor_msgs
* geometry_msgs

## Instructions
The package uses a custom gazebo world to spawn the turtlebot and perform the walker algorithm. A custom gazebo world can be checked in /worlds directory. You may add your own custom world to /worlds. You may have to change the inclusion of the world line in ```turtlebot_world.launch```. 

### Running the package
To run the package do the following:
Make sure you have catkin workspace
```
cd catkin_ws/src
git clone https://github.com/akhopkar01/turtlebot-walker.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch turtlebot-walker turtlebot_simulation.launch
```

### Running with ```rosbag``` enabled
The package has a rosbag functionality which can be enabled, which will record the topics. In order to do so execute the following command
```
roslaunch turtlebot-walker turtlebot_simulation.launch record:=enable
```

#### Inspecting bag file
The bag file can be found in the /results directory. The rosbag file can be inspected using the command : ```rosbag info results/turtlebot_walker.bag```. The expected output is as follows:
```
path:         results/turtlebot_walker.bag
version:      2.0
duration:     28.7s
start:        Dec 31 1969 20:54:26.14 (6866.14)
end:          Dec 31 1969 20:54:54.84 (6894.84)
size:         11.7 MB
messages:     94327
compression:  bz2 [143/143 chunks; 9.85%]
uncompressed: 107.0 MB @   3.7 MB/s
compressed:    10.5 MB @ 375.8 KB/s (9.85%)
types:        dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
              dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
              gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
              gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
              geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
              nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
              rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
              rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
              sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
              sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
              sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
              tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:       /clock                           28702 msgs    : rosgraph_msgs/Clock                  
              /cmd_vel                            58 msgs    : geometry_msgs/Twist                  
              /gazebo/link_states              28645 msgs    : gazebo_msgs/LinkStates               
              /gazebo/model_states             28640 msgs    : gazebo_msgs/ModelStates              
              /gazebo/parameter_descriptions       1 msg     : dynamic_reconfigure/ConfigDescription
              /gazebo/parameter_updates            1 msg     : dynamic_reconfigure/Config           
              /imu                              5506 msgs    : sensor_msgs/Imu                      
              /joint_states                      826 msgs    : sensor_msgs/JointState               
              /odom                              825 msgs    : nav_msgs/Odometry                    
              /rosout                             85 msgs    : rosgraph_msgs/Log                    
              /rosout_agg                         74 msgs    : rosgraph_msgs/Log                    
              /scan                              138 msgs    : sensor_msgs/LaserScan                
              /tf                                826 msgs    : tf2_msgs/TFMessage

```

#### Playing rosbag file
Since we have the recorded bag file, it can be played as follows - 
Open a new terminal and type the command:
```
$ roscore
```

In a new terminal,
```
$ cd catkin_ws
$ source devel/setup.bash
$ roscd turtlebot-walker
$ rosbag play results/turtlebot_walker.bag
```

In a new terminal simultaneously,
run ```$ rqt_console```, you will see all the ROS messages printed in the console suggesting the status and the action taken by the turtlebot.