/*
 * mapRelay.cpp
 *
 *  Created on: 16/09/2016
 *      Author: Ben Faul (z3422539)
 */

#include <ros/ros.h>

#include <bottomBeaconDetector.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/String.h>


namespace drone {
using namespace cv;

BottomBeaconDetector::BottomBeaconDetector() : it_(nh) {}

void BottomBeaconDetector::configure() {}

void BottomBeaconDetector::startup() {
    
    bottomCameraSub = nh.subscribe<sensor_msgs::Image>(
        "/ardrone/bottom_raw",
        1,
        &BottomBeaconDetector::callbackControl,
         this
    );
    
    // Make custom message type later
    anglePub = nh.advertise<std_msgs::Float32>("/ardrone/beaconAngleRelative", 1);
    locationPub = nh.advertise<std_msgs::Int32MultiArrayLayout>("/ardrone/beaconLocation", 1);
    
    cv::namedWindow(OPENCV_WINDOW);
}

void BottomBeaconDetector::shutdown() {
     cv::destroyWindow(OPENCV_WINDOW);
}

void BottomBeaconDetector::callbackControl(const sensor_msgs::ImageConstPtr& frame) {
    ROS_INFO("BottomBeaconDetector::callbackControl - Image received");
    cv_bridge::CvImagePtr cv_ptr;
    
    try {
        cv_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    analysePicture(cv_ptr);
    
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    image_pub_.publish(cv_ptr->toImageMsg());
}

void BottomBeaconDetector::analyseImage(cv_bridge::CvImagePtr cv_ptr) {
    cv::Mat bgr_image = cv_ptr->image;
    cv::medianBlur(bgr_image, bgr_image, 3);

	// Convert input image to HSV
	cv::Mat hsv_image;
	cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
	
	// Have to find better values
	cv::Mat blackFilterRange;
	cv::Mat blueFilterRange;
	cv::inRange(hsv_image, cv::Scalar(0, 0, 0, 0), cv::Scalar(180, 255, 30, 0), blackFilterRange);
	cv::inRange(hsv_image, cv::Scalar(120, 255, 255), cv::Scalar(120, 255, 255), blueFilterRange);
	
	// Find appropriate threshold in testing
	if (cv::countNonZero(blackFilterRange) < 200) {
        return;
    }
    
    ROS_INFO("Beacon Detected");
    
    vector<Vec2f> lines;
    vector<Vec3f> circles;
    
    HoughLines(blackFilterRange, lines, 1, CV_PI/180, 100, 0, 0);
    HoughCircles(blueFilterRange, circles, CV_HOUGH_GRADIENT, 1, blueFilterRange.rows/8, 200, 100, 0, 0);
    
    anglePub.publish(lines[0][1]);
    locationPub.publish([circles[0][0], circles[0][1]]);
    
}

} // namespace drone







