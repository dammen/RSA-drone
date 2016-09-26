/*
 *
 *  Created on: 23/08/2016
 *      Author: Ben Faul (z3422539)
 */

#include <ros/ros.h>

#include <cameraToggle.hpp>

#include <cstdio>
#include <cstdlib>

#define LOG_START    "CameraToggle ::"

namespace drone {

int main_cameraToggle(int argc, char** argv) {
    ros::init(argc, argv, "drone_cameraToggle");

    // Node Handle - Use '~' when loading config parameters
    // Use without '~' when publish/subscribe/service
    ros::NodeHandle nh("~");

    CameraToggle ct;
    ct.configure();
    ct.startup();

    while (ros::ok()) {
        ROS_INFO("%s Spinning", LOG_START);

        ros::spin();
    }

    // Disconnect
    ct.shutdown();

    return 0;
}

} // namespace drone

// Actual main method outside of namespace
int main(int argc, char** argv) {
    drone::main_cameraToggle(argc, argv);
}
