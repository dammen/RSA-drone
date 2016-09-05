/*
 *
 *  Created on: 23/08/2016
 *      Author: Ben Faul (z3422539)
 */

#include <ros/ros.h>

#include <controlPanelTest.hpp>

#include <cstdio>
#include <cstdlib>

#define LOG_START    "ControlPanelTest ::"

namespace drone {

int main_controlPanelTest(int argc, char** argv) {
    ros::init(argc, argv, "drone_controlPanelTest");

    // Node Handle - Use '~' when loading config parameters
    // Use without '~' when publish/subscribe/service
    ros::NodeHandle nh("~");

    ControlPanelTest test;
    test.configure();
    test.startup();

    while (ros::ok()) {
        ROS_INFO("%s Spinning", LOG_START);

        ros::spin();
    }

    // Disconnect
    test.shutdown();

    return 0;
}

} // namespace comp3431

// Actual main method outside of namespace
int main(int argc, char** argv) {
    drone::main_controlPanelTest(argc, argv);
}
