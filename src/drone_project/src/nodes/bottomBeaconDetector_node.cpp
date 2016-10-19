/*
 *
 *  Created on: 16/09/2016
 *      Author: Ben Faul (z3422539)
 */

#include <ros/ros.h>

#include <bottomBeaconDetector.hpp>


#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <vector>
#include <drone/beaconGeometry.h>

#define LOG_START    "BottomBeaconDetector_node ::"

namespace drone {
using namespace cv;
using namespace std;

int main_bottomBeaconDetector(int argc, char** argv) {

    ros::init(argc, argv, "drone_bottomBeaconDetector");

    // Node Handle - Use '~' when loading config parameters
    // Use without '~' when publish/subscribe/service
   
    ros::NodeHandle nh();
   
    BottomBeaconDetector bbd;
 
    bbd.startup();
    bbd.configure();
    
    while (ros::ok()) {
        ROS_INFO("%s Spinning", LOG_START);
        ros::spin();
    }

    return 0;
}

} // namespace comp3431

// Actual main method outside of namespace
int main(int argc, char** argv) {
    drone::main_bottomBeaconDetector(argc, argv);
}
