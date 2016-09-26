/*
 * mapRelay.hpp
 *
 *  Created on: 23/08/2016
 *      Author: Ben Faul (z3422539)
 */

#ifndef AR_DRONE_CAMERATOGGLE_HPP_
#define AR_DRONE_CAMERATOGGLE_HPP_


#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <std_msgs/String.h>
#include <sstream>

namespace drone {

class CameraToggle {
private:

    ros::Subscriber commandSub;
    ros::ServiceClient client; //not currently used

public:
    CameraToggle();
    virtual ~CameraToggle() {};

    void configure();
    void startup();
    void shutdown();

    void callbackControl(const std_msgs::StringConstPtr& command);
};

} // namespace drone

#endif
