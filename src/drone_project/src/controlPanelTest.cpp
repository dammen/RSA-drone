/*
 * mapRelay.cpp
 *
 *  Created on: 23/08/2016
 *      Author: Ben Faul (z3422539)
 */

#include <ros/ros.h>

#include <controlPanelTest.hpp>
#include <std_msgs/String.h>

#define BASE_FRAME  "base_link"

namespace drone {

ControlPanelTest::ControlPanelTest(){}

void ControlPanelTest::configure() {}

void ControlPanelTest::startup() {
    ros::NodeHandle nh;
    commandSub = nh.subscribe<std_msgs::String>("cmd", 1, &ControlPanelTest::callbackControl, this);
    takeoffPub = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000);
    resetPub = nh.advertise<std_msgs::Empty>("/ardrone/reset", 1000);
    landPub = nh.advertise<std_msgs::Empty>("/ardrone/land", 1000);
}

void ControlPanelTest::shutdown(){}

void ControlPanelTest::callbackControl(const std_msgs::StringConstPtr& cmd) {
    std::string msgData = cmd->data.c_str();
    if (msgData == "emergency_stop")  {
        resetPub.publish(std_msgs::Empty());
            ROS_INFO("ControlPanelTest::callbackControl - %s pressed", cmd->data.c_str());
    }
    else if (msgData == "take_off") {
        takeoffPub.publish(std_msgs::Empty());
            ROS_INFO("ControlPanelTest::callbackControl - %s pressed", cmd->data.c_str());
    }
    else if (msgData== "land") {
        landPub.publish(std_msgs::Empty());
            ROS_INFO("ControlPanelTest::callbackControl - %s pressed", cmd->data.c_str());
    }

}


} // namespace comp3431
