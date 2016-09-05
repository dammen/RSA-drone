/*
 * mapRelay.hpp
 *
 *  Created on: 23/08/2016
 *      Author: Ben Faul (z3422539)
 */

#ifndef AR_DRONE_CONTROLPANELTEST_HPP_
#define AR_DRONE_CONTROLPANELTEST_HPP_


#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <std_msgs/String.h>
#include <sstream>

namespace drone {

class ControlPanelTest {
private:

    ros::Subscriber commandSub;

public:
    ControlPanelTest();
    virtual ~ControlPanelTest() {};

    void configure();
    void startup();
    void shutdown();

    void callbackControl(const std_msgs::StringConstPtr& command);
};

} // namespace comp3431

#endif
