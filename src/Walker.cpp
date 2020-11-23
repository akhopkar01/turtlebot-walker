#include "Walker.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

turtlebot::Walker::Walker(float angularVel=0.1, float linearVel=0.2):angularVel{angularVel}, linearVel{linearVel} {
}

void turtlebot::Walker::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& sense) {
  auto sensorRanges = sense->ranges;
  for (size_t i = 0; i < sensorRanges.size(); ++i) {
    if (sense->range_min < sensorRanges[i] < 0.3) {
      obstacleInRange = true;
      return;
    }
  }
  obstacleInRange = false;
  return;
}


void turtlebot::Walker::reset(geometry_msgs::Twist& twist) {
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;
}

void turtlebot::Walker::pubROSNode() {
  pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &turtlebot::Walker::sensorCallback, this);
  ros::Rate loopRate(2);
  while (ros::ok()) {
    geometry_msgs::Twist twist;
    this->reset(twist);
    if (obstacleInRange == true) {
      ROS_INFO_STREAM("OBSTACLE DETECTED (in sensor range)");
      twist.angular.z = angularVel;
    } else {
      ROS_INFO_STREAM("PATH CLEAR (no obstacle)");
      twist.linear.x = linearVel;
    }
    pub_.publish(twist);
    ros::spinOnce();
    loopRate.sleep();
  }
}

turtlebot::Walker::~Walker() {}
