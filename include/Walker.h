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

 * @file: Walker class declaration 
 * @brief: This file contains the declaration of the walker class
 * */

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>

namespace turtlebot {
    class Walker {
        public:
        /**
         * @brief: Callback for sensor messages
         * @param: Constant pointer to reference from sensor_msgs
         * @return: None
         * */
         void sensorCallback(const sensor_msgs::LaserScan::ConstPtr&);

         /**
          * @brief: Publisher to ROS
          * @param: None
          * @return: None
          * */
         void pubROSNode();

         /**
         * @brief: Constructor for walker class
         * @param: ros node
         * */
         explicit Walker(ros::NodeHandle);
         
         /**
         * @brief: Destructor for walker class
         * @param: None
         * */
         ~Walker();

        private:
         bool obstacleInRange_;
         ros::NodeHandle nh_;
         ros::Publisher pub_;
         ros::Subscriber sub_;
         float angularVel_;
         float linearVel_;

        protected:
        /**
         * @brief: resetting the twist message
         * @param: reference to Twist message geometry_msgs
         * @return: None
         * */
         void reset(geometry_msgs::Twist&);
    };
    }  // namespace turtlebot
