/*
 * altitudeControl.hpp
 *
 *  Created on: 22/10/2016
 *      Author: Ben Faul (z3422539)
 */

#ifndef AR_DRONE_ALTITUDECONTROL_HPP_
#define AR_DRONE_ALTITUDECONTROL_HPP_


#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

namespace drone {

class AltitudeControl {
private:

    ros::Subscriber navSub;
    ros::Publisher  altPub;

public:
    AltitudeControl();
    virtual ~AltitudeControl() {};

    void configure();
    void startup();
    void shutdown();

    void callbackControl(const ardrone_autonomy::NavdataConstPtr& data);
};

} // namespace drone

#endif
