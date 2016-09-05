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
}

void ControlPanelTest::shutdown(){}

void ControlPanelTest::callbackControl(const std_msgs::StringConstPtr& cmd) {
    ROS_INFO("ControlPanelTest::callbackControl - %s pressed", cmd->data.c_str());
}


} // namespace comp3431
