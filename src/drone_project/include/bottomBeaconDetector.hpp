/*
 * mapRelay.hpp
 *
 *  Created on: 23/08/2016
 *      Author: Ben Faul (z3422539)
 */

#ifndef AR_DRONE_BOTTOMBEACONDETECTOR_HPP_
#define AR_DRONE_BOTTOMBEACONDETECTOR_HPP_


#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <sstream>

namespace drone {

class BottomBeaconDetector {
private:

    ros::Subscriber bottomCameraSub;

public:
    BottomBeaconDetector();
    virtual ~BottomBeaconDetector() {};

    void configure();
    void startup();
    void shutdown();

    void callbackControl(const sensor_msgs::ImageConstPtr& frame);
};

} // namespace comp3431

#endif
