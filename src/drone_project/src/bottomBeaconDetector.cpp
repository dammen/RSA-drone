/*
 * mapRelay.cpp
 *
 *  Created on: 16/09/2016
 *      Author: Ben Faul (z3422539)
 */

#include <ros/ros.h>

#include <bottomBeaconDetector.hpp>
//#include <std_msgs/String.h>

#define BASE_FRAME  "base_link"

namespace drone {

BottomBeaconDetector::BottomBeaconDetector(){}

void BottomBeaconDetector::configure() {}

void BottomBeaconDetector::startup() {
    ros::NodeHandle nh;
    bottomCameraSub = nh.subscribe<sensor_msgs::Image>(
        "ardrone/bottom_raw",
        1,
        &BottomBeaconDetector::callbackControl,
         this
    );
}

void BottomBeaconDetector::shutdown(){}

void BottomBeaconDetector::callbackControl(const sensor_msgs::ImageConstPtr& frame) {
    ROS_INFO("BottomBeaconDetector::callbackControl - Image received");
}


} // namespace comp3431
