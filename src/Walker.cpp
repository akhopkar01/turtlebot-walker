/**
 * MIT License

Copyright (c) 2020 Aditya Khopkar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 * @file: Walker class implementation 
 * @brief: This file contains the implementation of the walker class
 * */

#include "Walker.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>

#define _ANGLE_ 60

/**
 * @brief: Constructor implementation
 * */
turtlebot::Walker::Walker(ros::NodeHandle n)
    : angularVel_{0.6}, linearVel_{0.2}, nh_{n}, obstacleInRange_{false} {
}

/**
 * @brief: Callback for LaserScan implementation
 * */
void turtlebot::Walker::sensorCallback(const sensor_msgs::
                                        LaserScan::ConstPtr& sense) {
  auto sensorRanges = sense->ranges;
  for (size_t i = 0; i <= _ANGLE_; ++i) {
    // If obstacle in range [-60, 60]
    if (sensorRanges[i] < 0.5 && sensorRanges[i + 299] < 0.5) {
      obstacleInRange_ = true;
      return;
    }
  }
  obstacleInRange_ = false;
  return;
}

/**
 * @brief: reset method implementation
 * */
void turtlebot::Walker::reset(geometry_msgs::Twist& twist) {
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;
}

/**
 * @brief: Implementation of ROS node publisher-subscriber
 * */
void turtlebot::Walker::pubROSNode() {
  // Initialize publisher subscriber node
  pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1000,
                          &turtlebot::Walker::sensorCallback, this);
  ros::Rate loopRate(2);
  while (ros::ok()) {
    geometry_msgs::Twist twist;

    // Reset twist to 0
    this->reset(twist);
    if (obstacleInRange_) {
      ROS_INFO_STREAM("Obstacle detected.. (turning left)");
      twist.angular.z = angularVel_;
    } else {
      ROS_INFO_STREAM("Path Clear");
      twist.linear.x = linearVel_;
    }

    // Publish twist message
    pub_.publish(twist);
    ros::spinOnce();
    loopRate.sleep();
  }
}

/**
 * @brief: Destructor call
 * */
turtlebot::Walker::~Walker() {}
