/*
 *
 *  Created on: 22/10/2016
 *      Author: Ben Faul (z3422539)
 */

#include <ros/ros.h>

#include <altitudeControl.hpp>

#include <cstdio>
#include <cstdlib>

#define LOG_START    "AltitudeControl ::"

namespace drone {

int main_altitudeControl(int argc, char** argv) {
    ros::init(argc, argv, "drone_altitudeControl");

    // Node Handle - Use '~' when loading config parameters
    // Use without '~' when publish/subscribe/service
    ros::NodeHandle nh("~");

    AltitudeControl alt;
    alt.configure();
    alt.startup();

    while (ros::ok()) {
        ROS_INFO("%s Spinning", LOG_START);

        ros::spin();
    }

    // Disconnect
    alt.shutdown();

    return 0;
}

} // namespace drone

// Actual main method outside of namespace
int main(int argc, char** argv) {
    drone::main_altitudeControl(argc, argv);
}
