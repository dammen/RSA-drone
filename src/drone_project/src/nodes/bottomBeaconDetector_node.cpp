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
cv::Mat bgr_image;
cv::Mat hsv_image;
cv::Mat mask;
cv::Mat helperMask;
cv::Mat blueFilterRange;
cv::Mat blackFilterRange;

void trackBarblack(int, void*) {
    
    int h1 = 100;
    int s1 = 100;
    int v1 = 100;
    int h = 100;
    int s = 100;
    int v = 100;
    h1 = getTrackbarPos("h1", "Black Filter");
    s1 = getTrackbarPos("s1", "Black Filter");
    v1 = getTrackbarPos("v1", "Black Filter");
    h = getTrackbarPos("h", "Black Filter");
    s = getTrackbarPos("s", "Black Filter");
    v = getTrackbarPos("v", "Black Filter");
    cv::inRange(hsv_image, cv::Scalar(h, s, v), cv::Scalar(h1, s1, v1), blackFilterRange);
    imshow("Black Filter", blackFilterRange);
}

void trackBarblue(int, void*) {
    int h1 = 100;
    int s1 = 100;
    int v1 = 100;
    int h = 100;
    int s = 100;
    int v = 100;
    
    h = getTrackbarPos("h", "Blue Filter");
    s = getTrackbarPos("s", "Blue Filter");
    v = getTrackbarPos("v", "Blue Filter");
    h1 = getTrackbarPos("h1", "Blue Filter");
    s1 = getTrackbarPos("s1", "Blue Filter");
    v1 = getTrackbarPos("v1", "Blue Filter");
    
    cv::inRange(hsv_image, cv::Scalar(h, s, v), cv::Scalar(h1, s1, v1), blueFilterRange);
    cv::imshow("Blue Filter", blueFilterRange);
}



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
