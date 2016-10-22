/*
 * altitudeControl.cpp
 *
 *  Created on: 22/10/2016
 *      Author: Ben Faul (z3422539)
 */

#include <ros/ros.h>

#include <altitudeControl.hpp>
//#include <std_msgs/String.h>
#define ALTITUDE 1500
#define ALT_TOLERANCE 30

namespace drone {

AltitudeControl::AltitudeControl(){}

void AltitudeControl::configure() {}

void AltitudeControl::startup() {
    ros::NodeHandle nh;
    navSub = nh.subscribe<ardrone_autonomy::Navdata>("ardrone/navdata", 1, &AltitudeControl::callbackControl, this);
    altPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void AltitudeControl::shutdown(){}

void AltitudeControl::callbackControl(const ardrone_autonomy::NavdataConstPtr& data) {
    ROS_INFO("altitude = %d mm\n", data->altd);
    int current_alt = data->altd;
    geometry_msgs::Twist t;
    t.angular.x = t.angular.y = t.angular.z = 0;
    t.linear.x = t.linear.y = 0;
    if (current_alt < ALTITUDE-ALT_TOLERANCE){
        // too low, go up
        t.linear.z = 1;
    } else if (current_alt > ALTITUDE+ALT_TOLERANCE){
        //too high come down
        t.linear.z = -1;
    } else {}
    
    altPub.publish(t);

}


} // namespace drone
