/*
 * mapRelay.cpp
 *
 *  Created on: 16/09/2016
 *      Author: Ben Faul (z3422539)
 */

#include <ros/ros.h>

#include <bottomBeaconDetector.hpp>
#include <vector>
#include <drone/beaconGeometry.h>


namespace drone {
using namespace cv;

BottomBeaconDetector::BottomBeaconDetector() : it_(nh) {}

void BottomBeaconDetector::configure() {}

void BottomBeaconDetector::startup() {
    
    bottomCameraSub = nh.subscribe<sensor_msgs::Image>(
        "/ardrone/bottom/image_raw",
        1,
        &BottomBeaconDetector::callbackControl,
         this
    );
    
    // Make custom message type later
    beaconPub = nh.advertise<drone::beaconGeometry>("/ardrone/beaconGeometry", 1);

    
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
    
    analyseImage(cv_ptr);
    
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //image_pub_.publish(cv_ptr->toImageMsg());
}
/*
void BottomBeaconDetector::trackBarblack(int, void*) {
    
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

void BottomBeaconDetector::trackBarblue(int, void*) {
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
*/
void BottomBeaconDetector::analyseImage(cv_bridge::CvImagePtr cv_ptr) {
    cv::Mat bgr_image = cv_ptr->image;
    cv::Mat hsv_image;
    cv::Mat blueFilterRange;
    cv::Mat blackFilterRange;
    drone::beaconGeometry msg;
    
    cv::medianBlur(bgr_image, bgr_image, 3);

	// Convert input image to HSV   
	cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
	
	cv::inRange(hsv_image, cv::Scalar(105, 84, 70), cv::Scalar(180, 130, 139), blackFilterRange);
	cv::inRange(hsv_image, cv::Scalar(100, 130, 57), cv::Scalar(115, 255, 255), blueFilterRange);
	
	// Find appropriate threshold in testing
	if (cv::countNonZero(blackFilterRange) < 500) {
	    msg.canSeeBeacon = false;
	    msg.positionX = -1;
	    msg.positionY = -1;
	    msg.angle = -1;
	    beaconPub.publish(msg);
        return;
    }
    
    ROS_INFO("Beacon Detected");
    
    msg.canSeeBeacon = true;
    
    std::vector<Vec3f> circles;
    vector<Vec4i> linesP;

    GaussianBlur(blueFilterRange, blueFilterRange, Size(9,9), 2, 2);
    HoughCircles(blueFilterRange, circles, CV_HOUGH_GRADIENT, 2, blueFilterRange.rows/4, 100, 50);
    
    Canny(blackFilterRange, blackFilterRange, 50, 200);
    HoughLinesP(blackFilterRange, linesP, 1, CV_PI/90, 20, 30, 10);
    
    if (!circles.empty()) {
        ROS_INFO("Beacon Location: X:%d Y:%d, Radius: %d", (int)circles[0][0], (int)circles[0][1], (int)circles[0][2]);
        
        msg.positionX = (int)circles[0][0];
        msg.positionY = (int)circles[0][1];
        
        //Drawing Circles
        Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
        int radius = cvRound(circles[0][2]);
        circle(bgr_image, center, radius, Scalar(0,255,0), -3, 8, 0);
    }
    
    if (!linesP.empty()) {
        Vec4i max_l;
        double max_dist = 100;

        for( size_t i = 0; i < linesP.size(); i++ )
        {
            Vec4i l = linesP[i];
            double theta1,theta2, hyp, result;

            theta1 = (l[3]-l[1]);
            theta2 = (l[2]-l[0]);
            hyp = hypot(theta1,theta2);

            if (max_dist >  hyp) {
                max_l = l;
                max_dist = hyp;
                break;
            }           
        }
        /*
        Draw all matching lines
        for( size_t i = 0; i < linesP.size(); i++ )
        {
          Vec4i l = linesP[i];
          line( bgr_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
        }
        */
        line( bgr_image, Point(max_l[0], max_l[1]), Point(max_l[2], max_l[3]), Scalar(255,0,0), 3, CV_AA);
        
        float angle = atan2(max_l[1] - max_l[3], max_l[0] - max_l[2]);   
        ROS_INFO("Beacon Angle: %f, %d", angle + 90, (int)linesP.size());
        msg.angle = angle + 90; // offset
    }

    // Display filtered Images

    cv::imshow("Blue Filter", blueFilterRange);

    cv::imshow("Black Filter", blackFilterRange);
    
    cv::imshow("Normal Image", bgr_image);
    /*
    createTrackbar("h", "Black Filter", 0, 179, trackBarblack);
    createTrackbar("s", "Black Filter", 0, 255,trackBarblack);
    createTrackbar("v", "Black Filter", 0, 255,trackBarblack);
    createTrackbar("h1", "Black Filter", 0, 179, trackBarblack);
    createTrackbar("s1", "Black Filter", 0, 255,trackBarblack);
    createTrackbar("v1", "Black Filter", 0, 255,trackBarblack);
    
    createTrackbar("h", "Blue Filter", 0, 179,trackBarblue);
    createTrackbar("s", "Blue Filter", 0, 255, trackBarblue);
    createTrackbar("v", "Blue Filter", 0, 255, trackBarblue);
    createTrackbar("h1", "Blue Filter", 0, 179,trackBarblue);
    createTrackbar("s1", "Blue Filter", 0, 255, trackBarblue);
    createTrackbar("v1", "Blue Filter", 0, 255, trackBarblue);
    waitKey(2);
    */
    beaconPub.publish(msg);
}

} // namespace drone







