/*
 * mapRelay.cpp
 *
 *  Created on: 23/08/2016
 *      Author: Ben Faul (z3422539)
 */

#include <ros/ros.h>

#include <cameraToggle.hpp>
#include <std_msgs/String.h>

namespace drone {

CameraToggle::CameraToggle(){}

void CameraToggle::configure() {}

void CameraToggle::startup() {
    ros::NodeHandle nh;
    //client = nh.serviceClient<ardrone::togglecam>("ardrone/togglecam");
    commandSub = nh.subscribe<std_msgs::String>("cmd", 1, &CameraToggle::callbackControl, this);
}

void CameraToggle::shutdown(){}

void CameraToggle::callbackControl(const std_msgs::StringConstPtr& cmd) {
    if (strcmp(cmd->data.c_str(), "toggle_camera") == 0){
        //toggle camera
        //client.call(null);
        system("rosservice call ardrone/togglecam");
    }
}


} // namespace drone
